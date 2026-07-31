#!/usr/bin/env python3
"""給建圖用的鍵盤遙控，修掉原廠 teleop_twist_keyboard 的三個坑。

## 坑一：按 q/w/e/z/x/c 調速度，車子會動

原廠的主迴圈長這樣：

    elif key in speedBindings.keys():
        speed = speed * 1.1
        turn  = turn  * 1.1
        print(...)
        # x, y, th 完全沒有被重設

    twist.linear.x = x * speed     # <- 上一次的方向 x 新的速度
    pub.publish(twist_msg)         # <- 無條件發布

調速分支不會把方向歸零，而底部的發布是無條件的。所以按 `q` 等於
「把上一次的方向用新速度重送一次」——上次按過 `i`，按 `q` 就往前衝，
而且是用剛剛調高 10% 的速度。這裡改成：**調速時一律送停止**，
要動就重新按方向鍵。

## 坑二：按一次就一直開

原廠的 `getKey()` 是 `sys.stdin.read(1)`，完全阻塞、沒有逾時。
按下 `i` 之後放開，程式停在那裡等下一個按鍵，什麼都不會發；
底盤只是把最後收到的指令送去序列埠，ROS 這端沒有逾時保護，
於是車子一直往前開到你按下一個鍵為止。

這裡改成有逾時的讀取：超過 `key_timeout` 沒有新按鍵就送停止。
終端機的自動重複會在你按著不放時持續送鍵，所以行為變成
**按著才動、放開就停**，跟 HMI 的看門狗一致。

## 坑三：QoS 對不上底盤

底盤 `wheeltec_robot` 訂閱 /cmd_vel 用 RELIABLE。BEST_EFFORT 的發布端
連線建立不起來，指令一則都送不到（2026-07-31 在 HMI 上踩過）。
這裡明確用 RELIABLE + depth=1。

## 按鍵

    u i o       前左 / 前 / 前右
    j k l       左轉 / 停 / 右轉
    m , .       後左 / 後 / 後右
    q/z         全部加速 / 減速 10%（不會移動）
    w/x         只調線速度
    e/c         只調角速度
    Ctrl-C      離開（會補送停止）
"""
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

MOVE = {
    "u": (1.0, 1.0), "i": (1.0, 0.0), "o": (1.0, -1.0),
    "j": (0.0, 1.0), "k": (0.0, 0.0), "l": (0.0, -1.0),
    "m": (-1.0, 1.0), ",": (-1.0, 0.0), ".": (-1.0, -1.0),
}
SPEED = {
    "q": (1.1, 1.1), "z": (0.9, 0.9),
    "w": (1.1, 1.0), "x": (0.9, 1.0),
    "e": (1.0, 1.1), "c": (1.0, 0.9),
}

BANNER = """
建圖用鍵盤遙控（teleop_keyboard_cc）
────────────────────────────────────
   u    i    o        i/,  前進 / 後退
   j    k    l        j/l  左轉 / 右轉
   m    ,    .        k    停止

   q/z  全部加減速 10%   w/x  只調線速度   e/c  只調角速度
   ★ 調速鍵不會讓車子移動（原廠版會，這裡修掉了）
   ★ 放開按鍵 {:.1f} 秒後自動停止

   Ctrl-C 離開
"""


def main():
    rclpy.init()
    node = rclpy.create_node("teleop_keyboard_cc_node")
    node.declare_parameter("speed", 0.06)      # 建圖用的慢速，不是原廠的 0.5
    node.declare_parameter("turn", 0.4)
    node.declare_parameter("key_timeout", 0.4)
    node.declare_parameter("cmd_topic", "cmd_vel")

    speed = float(node.get_parameter("speed").value)
    turn = float(node.get_parameter("turn").value)
    key_timeout = float(node.get_parameter("key_timeout").value)

    pub = node.create_publisher(
        Twist, node.get_parameter("cmd_topic").value,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST))

    settings = termios.tcgetattr(sys.stdin) if os.isatty(sys.stdin.fileno()) else None

    def get_key():
        """有逾時的讀取：沒按鍵就回 None，讓呼叫端送停止"""
        if settings is None:
            return None
        tty.setraw(sys.stdin.fileno())
        r, _, _ = select.select([sys.stdin], [], [], key_timeout)
        key = sys.stdin.read(1) if r else None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def send(lin, ang):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        pub.publish(t)

    def vels():
        return f"目前速度：{speed:.3f} m/s   轉向 {turn:.3f} rad/s"

    print(BANNER.format(key_timeout))
    print(vels())

    try:
        while rclpy.ok():
            key = get_key()

            if key == "\x03":          # Ctrl-C
                break

            if key in MOVE:
                lin, ang = MOVE[key]
                send(lin * speed, ang * turn)
            elif key in SPEED:
                speed *= SPEED[key][0]
                turn *= SPEED[key][1]
                # ★ 這裡是跟原廠最大的差別：調速一律送停止，不重送舊方向
                send(0.0, 0.0)
                print(vels())
            else:
                # 沒按鍵（逾時）或按到沒定義的鍵 -> 停
                send(0.0, 0.0)
    except KeyboardInterrupt:
        pass
    finally:
        send(0.0, 0.0)
        if settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\n已停車並離開")


if __name__ == "__main__":
    main()
