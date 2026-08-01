#!/usr/bin/env python3
"""給建圖用的鍵盤遙控。

坑一~三是原廠 `teleop_twist_keyboard` 的問題，坑四是本檔前一版自己的問題。

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

## 坑四：方向舵一直回正又打回去（2026-08-01 修）

前一版把「讀到按鍵」直接當成「發布指令」，逾時就立刻送停止：

    key = get_key()          # select 逾時 0.4 秒
    if key in MOVE: send(...)
    else:           send(0, 0)      # <- 逾時 = 馬上回正

問題是 **`select` 逾時代表「這段時間沒有新字元到達」，不代表
「使用者放開了按鍵」**。按著鍵不放時字元並不是連續來的：

    t=0.00  按下          -> 1 個字元  -> send(轉向)
    t=0.40  select 逾時               -> send(0,0)    <- 回正！
    t=0.66  OS 自動重複開始 -> 字元     -> send(轉向)   <- 又打方向

X11 的鍵盤自動重複**初始延遲預設 660 ms**，比 0.4 秒的逾時還長，
所以按下後必然先來一次回正。透過 SSH 操作更糟：字元被 TCP 打包，
網路抖動讓字元流不斷出現超過 0.4 秒的空隙，每個空隙都是一次回正。
使用者看到的就是舵機持續抽動（也會磨損舵機、吃電流）。

修法跟 HMI 那邊一樣：**把「設定值」與「發布」拆開**。

    主執行緒：讀鍵盤 -> 只更新設定值與時間戳
    發布執行緒：固定 20 Hz 送出目前設定值；超過 key_timeout 才歸零

這樣字元流的抖動完全不會傳導到舵機，而且底盤永遠收得到穩定的指令流
（韌體有 1.0 秒逾時，指令流不能斷）。key_timeout 同時放寬到 0.8 秒，
容納自動重複的初始延遲。

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
import threading
import time
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
    # 0.8 秒而不是 0.4：X11 鍵盤自動重複的初始延遲預設 660 ms，
    # 逾時比它短的話「按下 -> 還沒開始重複」就會先被判成放開（見坑四）。
    node.declare_parameter("key_timeout", 0.8)
    node.declare_parameter("cmd_topic", "cmd_vel")
    # 發布頻率。底盤韌體有 1.0 秒指令逾時，指令流不能斷。
    node.declare_parameter("publish_rate", 20.0)

    speed = float(node.get_parameter("speed").value)
    turn = float(node.get_parameter("turn").value)
    key_timeout = float(node.get_parameter("key_timeout").value)
    publish_period = 1.0 / max(1.0, float(node.get_parameter("publish_rate").value))

    pub = node.create_publisher(
        Twist, node.get_parameter("cmd_topic").value,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST))

    settings = termios.tcgetattr(sys.stdin) if os.isatty(sys.stdin.fileno()) else None

    # ── 設定值（由鍵盤更新）與發布（固定頻率）分離 ────────────────
    # 這是坑四的修法：字元流的抖動不可以傳導到舵機。
    state_lock = threading.Lock()
    state = {"lin": 0.0, "ang": 0.0, "stamp": 0.0}
    running = threading.Event()
    running.set()

    def set_cmd(lin, ang):
        with state_lock:
            state["lin"] = lin
            state["ang"] = ang
            # 只有「有方向」才更新時間戳。送停止時把時間戳歸零，
            # 讓發布迴圈不用再等 key_timeout 才停。
            state["stamp"] = time.monotonic() if (lin or ang) else 0.0

    def publish_loop():
        """固定頻率送出目前設定值；太久沒按鍵才歸零

        用獨立執行緒而不是 ROS timer：這支程式的主執行緒阻塞在
        select 讀鍵盤上，timer 根本不會被執行。
        """
        next_t = time.monotonic()
        while running.is_set():
            with state_lock:
                lin, ang, stamp = state["lin"], state["ang"], state["stamp"]
                if stamp and time.monotonic() - stamp > key_timeout:
                    # 逾時：放開按鍵了（或 SSH 斷線）
                    state["lin"] = state["ang"] = 0.0
                    state["stamp"] = 0.0
                    lin = ang = 0.0
            t = Twist()
            t.linear.x = lin
            t.angular.z = ang
            pub.publish(t)
            next_t += publish_period
            time.sleep(max(0.0, next_t - time.monotonic()))

    def get_key():
        """有逾時的讀取。逾時只是讓迴圈有機會檢查 Ctrl-C，不代表放開按鍵"""
        if settings is None:
            return None
        # 逾時取短一點（0.1 秒）純粹是為了離開時反應快；
        # 「放開按鍵」的判定交給發布迴圈的 key_timeout，兩者是不同的事。
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        return sys.stdin.read(1) if r else None

    def vels():
        return f"目前速度：{speed:.3f} m/s   轉向 {turn:.3f} rad/s"

    print(BANNER.format(key_timeout))
    print(vels())

    pub_thread = threading.Thread(target=publish_loop, name="teleop_pub", daemon=True)
    pub_thread.start()

    try:
        # raw 模式只設定一次。原本每讀一個字元就 setraw + tcsetattr(TCSADRAIN)
        # 來回切換，TCSADRAIN 還會等輸出排空，在 SSH 上特別慢。
        if settings is not None:
            tty.setraw(sys.stdin.fileno())

        while rclpy.ok():
            key = get_key()

            if key is None:
                continue               # 只是輪詢逾時，不是放開按鍵

            if key == "\x03":          # Ctrl-C
                break

            if key in MOVE:
                lin, ang = MOVE[key]
                set_cmd(lin * speed, ang * turn)
            elif key in SPEED:
                speed *= SPEED[key][0]
                turn *= SPEED[key][1]
                # ★ 跟原廠最大的差別：調速一律停車，不重送舊方向
                set_cmd(0.0, 0.0)
                if settings is not None:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
                print(vels())
                if settings is not None:
                    tty.setraw(sys.stdin.fileno())
            else:
                set_cmd(0.0, 0.0)      # 按到沒定義的鍵 -> 停
    except KeyboardInterrupt:
        pass
    finally:
        set_cmd(0.0, 0.0)
        # 讓發布迴圈至少再送幾則零速出去，DDS 掉一則不能變成「車子繼續跑」
        time.sleep(publish_period * 3)
        running.clear()
        pub_thread.join(timeout=1.0)
        pub.publish(Twist())
        if settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\n已停車並離開")


if __name__ == "__main__":
    main()
