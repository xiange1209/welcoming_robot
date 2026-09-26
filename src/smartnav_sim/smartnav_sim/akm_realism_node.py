#!/usr/bin/env python3

"""實車不對稱模擬 —— 把 Gazebo 裡的「理想阿克曼車」變成「我們那台車」

Gazebo 的 AckermannSteering 外掛是一台完美的車：沒有死區、左右對稱、
下 angular.z = 0 就走直線。實車三樣都不是（數字都有實測出處）：

    線速度死區      0.085 m/s 以下馬達不轉        韌體 PID 克服不了靜摩擦；8/24 逐筆 1583 筆：
                                                    前進平均指令 +0.080、平均實速 −0.001 m/s
    最小迴轉半徑    左 1.030 m／右 1.183 m        8/29 實測（韌體設計是對稱 0.750，車子/README.md）
    機械零位偏移    +0.235 rad/m（往左偏）         8/29 steer_asym_check：前進 +13.46 度/m、
                                                    倒車 −15.70 度/m → 等效舵角偏 +4.32 度

模擬車如果沒有這些，參數在模擬裡驗過了、上車照樣失敗 —— 等於沒驗。
所以這個節點插在「導航鏈的最後一個輸出」與 Gazebo 之間，扮演底盤：

    … → cmd_vel_floor_cc --cmd_vel--> [本節點] --cmd_vel_sim--> ros_gz_bridge --> AckermannSteering

★ 機械偏移為什麼是 +0.235：導航鏈裡的 steering_trim_cc 預設補 −0.235 rad/m，
  兩者相加才會回到「下 0 走直線」。realism:=false 時**偏移照樣保留**，只關死區
  與不對稱半徑 —— 不然 steering_trim 會把理想車補成往右偏，反而更不像。

換算都用曲率 κ = ω / v（阿克曼的舵角只由 κ 決定）：
    1. |v| < 死區           → v = ω = 0（車不動；舵機會轉，但對位姿沒有影響）
    2. 韌體截斷             → |κ| ≤ 1 / 0.750（對稱，韌體 robot_select_init.h）
    3. 加機械偏移           → κ += 0.235
    4. 機構極限（不對稱）   → −1/1.183 ≤ κ ≤ 1/1.030
    5. ω = κ · v            → 倒車時 v < 0，ω 自動反號（符合實測「倒車反號」）
    6. 1 秒沒收到指令就停   → 跟韌體命令看門狗一樣（底盤與韌體分析.md）
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def shape_command(v, w, deadzone, r_fw, bias, r_left, r_right):
    """把導航鏈的指令 (v, ω) 換成實車真的會做出來的 (v, ω)

    回傳 (v_out, w_out, 事件)，事件是 "deadzone" / "clamp_left" / "clamp_right" / None，
    給統計用。純函式、不碰 ROS，方便單元測試。
    """
    if abs(v) < deadzone or v == 0.0:
        # 死區：馬達不轉。v == 0 另外列出來是因為 deadzone 可以設 0（理想模式）
        return 0.0, 0.0, ("deadzone" if v != 0.0 else None)

    k = w / v
    k_fw = 1.0 / r_fw
    k = max(-k_fw, min(k_fw, k))          # 2. 韌體截斷（對稱）
    k += bias                              # 3. 機械零位偏移
    event = None
    k_left = 1.0 / r_left
    k_right = 1.0 / r_right
    if k > k_left:                         # 4. 機構極限（不對稱）
        k, event = k_left, "clamp_left"
    elif k < -k_right:
        k, event = -k_right, "clamp_right"
    return v, k * v, event


class AkmRealismNode(Node):
    """扮演 senior_akm 底盤：死區、不對稱迴轉半徑、機械偏移、命令看門狗"""

    def __init__(self):
        super().__init__("akm_realism_node")

        self.declare_parameter("input_topic", "cmd_vel")
        self.declare_parameter("output_topic", "cmd_vel_sim")
        # false = 關掉死區與不對稱半徑（當成韌體設計值的對稱 0.750），機械偏移保留
        self.declare_parameter("realism", True)
        self.declare_parameter("deadzone_mps", 0.085)
        self.declare_parameter("r_min_firmware", 0.750)
        self.declare_parameter("r_min_left", 1.030)
        self.declare_parameter("r_min_right", 1.183)
        self.declare_parameter("mech_bias_rad_per_m", 0.235)
        self.declare_parameter("cmd_timeout_sec", 1.0)
        # Gazebo 外掛會一直沿用最後一筆指令，所以要定時重發（逾時就發 0）
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("report_sec", 10.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.realism = bool(p("realism"))
        self.r_fw = float(p("r_min_firmware"))
        self.bias = float(p("mech_bias_rad_per_m"))
        if self.realism:
            self.deadzone = float(p("deadzone_mps"))
            self.r_left = float(p("r_min_left"))
            self.r_right = float(p("r_min_right"))
        else:
            self.deadzone = 0.0
            self.r_left = self.r_right = self.r_fw
        self.timeout = float(p("cmd_timeout_sec"))

        self._out = Twist()
        self._last_cmd = None       # 最後一次收到指令的時間（節點時鐘；模擬時是 /clock）
        self._timed_out = True
        self._stats = {"cmd": 0, "deadzone": 0, "clamp_left": 0, "clamp_right": 0}

        self.pub = self.create_publisher(Twist, p("output_topic"), 10)
        self.create_subscription(Twist, p("input_topic"), self._cmd_cb, 10)
        self.create_timer(1.0 / float(p("publish_rate_hz")), self._tick)
        report = float(p("report_sec"))
        if report > 0.0:
            self.create_timer(report, self._report)

        self.get_logger().info(
            f"實車不對稱模擬 {'開' if self.realism else '關（理想車，只留機械偏移）'}："
            f"死區 {self.deadzone:.3f} m/s、最小迴轉半徑 左 {self.r_left:.3f}／右 {self.r_right:.3f} m、"
            f"機械偏移 {self.bias:+.3f} rad/m（{math.degrees(self.bias):+.1f} 度/m）、"
            f"{p('input_topic')} -> {p('output_topic')}")

    def _cmd_cb(self, msg: Twist) -> None:
        v, w, event = shape_command(
            msg.linear.x, msg.angular.z,
            self.deadzone, self.r_fw, self.bias, self.r_left, self.r_right)
        out = Twist()
        out.linear.x = v
        out.angular.z = w
        self._out = out
        self._last_cmd = self.get_clock().now()
        self._timed_out = False
        self._stats["cmd"] += 1
        if event:
            self._stats[event] += 1
        # 收到就立刻轉發，不等計時器：少一拍延遲，跟實車序列埠「隨指令觸發」一致
        self.pub.publish(out)

    def _tick(self) -> None:
        if self._last_cmd is None:
            return
        age = (self.get_clock().now() - self._last_cmd).nanoseconds / 1e9
        if age > self.timeout:
            if not self._timed_out:
                self._timed_out = True
                self.get_logger().debug(f"{self.timeout:.1f} 秒沒收到指令，停車（韌體看門狗）")
            self.pub.publish(Twist())
        else:
            self.pub.publish(self._out)

    def _report(self) -> None:
        s = self._stats
        if s["deadzone"] or s["clamp_left"] or s["clamp_right"]:
            self.get_logger().info(
                f"近 {self.get_parameter('report_sec').value:.0f} 秒 {s['cmd']} 筆指令："
                f"死區吃掉 {s['deadzone']}、舵角到底 左 {s['clamp_left']}／右 {s['clamp_right']}")
        for k in s:
            s[k] = 0


def main(args=None):
    rclpy.init(args=args)
    node = AkmRealismNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
