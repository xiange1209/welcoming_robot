#!/usr/bin/env python3
"""速度下限守門員 —— 擋住「下了指令但馬達不轉」這整類失敗

## 為什麼需要這支（2026-08-24 實機證據）

底盤韌體的線速度死區是 **0.085 m/s**：低於它馬達根本不轉。
而速度指令鏈上有**三個**會壓低速度的機制，它們互相不知道對方存在：

    path_teach `_lift_deadband()`   把非零速度抬到 min_move_speed 0.10  ← 唯一知道死區的
    steering_trim_cc                不動 linear.x（只改 angular.z）
    collision_monitor PolygonSlow   × slowdown_ratio                    ← 8/24 已改 0.5→0.85
    collision_monitor FootprintApproach  按碰撞剩餘時間**連續縮放，沒有下限** ← 這支要擋的

`_lift_deadband` 抬完之後，下游還會再乘兩次。8/24 逐筆分析 1,583 筆：

    前進 463 筆，平均指令 +0.080 m/s，**平均實速 −0.001 m/s**
    另有 442 筆落在 0.050~0.070，其中 **31.4% 輪速為 0**
    死區以下的比例：順利那趟 1.1%、窄空間那趟 **26.3%**、倒車失敗那趟 18.0%

★★ **後果不是報錯，是誤診** ★★

    馬達不轉 → path_teach 判定「有下速度指令但車子沒動」→ 脫困
    → 脫困轉向把 AMCL 也轉丟（實測事後吻合度剩 23.4%、yaw 差約 250 度）

現場看起來像定位壞掉或控制器沒調好，真因是**兩個安全機制的算術**。

## 為什麼放在鏈的最後

改 `slowdown_ratio` 只解決了三個機制裡的第二個，`FootprintApproach` 仍然會壓。
放在 `collision_monitor` **之後**，一次擋住**所有**下游來源，包括以後才加的。

    velocity_smoother → steering_trim_cc → collision_monitor → **本節點** → 底盤

## ★ 這是取捨，不是「安全被拿掉」

抬速度確實部分抵銷了減速。但「0.05 m/s」這個指令在**物理上不可執行**——
馬達能走的最慢速度就是死區，所以 0.10 反而是對「請你慢慢走」最接近的實現，
停下來才是更大的偏離。而且：

  - **停止指令永遠不抬**。`linear.x == 0` 原樣通過。
    這是「減速」與「停止」的分界，也是本節點唯一不能妥協的地方。
  - `PolygonStop` 與 `FootprintApproach` 判定要停時送的就是 0，照樣會停。
  - path_teach 自己還有 `_body_margin` 硬停與 `_arc_clearance_robust` 弧線預測。

## ★ 曲率必須一起等比例放大

阿克曼的舵角由曲率 κ = ω / v 決定。只抬 v 而不動 ω，κ 會**變小**，
車子會走出比指令更直的線——在 0.99 m 走廊裡那等於直接撞牆。
所以兩個一起乘同一個倍率，κ 保持不變。
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class CmdVelFloorNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_floor_cc_node")

        self.declare_parameter(
            "in_topic", "cmd_vel_prefloor",
            ParameterDescriptor(description="輸入（collision_monitor 的輸出）"))
        self.declare_parameter(
            "out_topic", "cmd_vel",
            ParameterDescriptor(description="輸出（底盤實際吃的）"))
        # ★ 死區 0.085 是 23.7 V 時量的，而**死區會隨電壓變化**。
        #   0.095 留了約 12% 餘裕。低電量時若仍出現「不轉」，先調這個而不是別處。
        self.declare_parameter(
            "min_speed", 0.095,
            ParameterDescriptor(description="線速度下限 m/s（0 = 停用本節點）"))
        # 角速度也要有下限：原地打舵時 linear.x 是 0，這條不適用；
        # 但 |v| 很小配很大的 ω 是脫困那種動作，那時 v 會被抬起來，ω 跟著等比例放大。
        self.declare_parameter(
            "max_scale", 3.0,
            ParameterDescriptor(description="等比例放大的倍率上限（避免 ω 爆掉）"))
        self.declare_parameter(
            "log_every", 40,
            ParameterDescriptor(description="每幾次介入印一行（0 = 不印）"))

        self.in_topic = self.get_parameter("in_topic").get_parameter_value().string_value
        self.out_topic = self.get_parameter("out_topic").get_parameter_value().string_value
        self.min_speed = abs(self.get_parameter("min_speed").get_parameter_value().double_value)
        self.max_scale = abs(self.get_parameter("max_scale").get_parameter_value().double_value)
        self.log_every = int(self.get_parameter("log_every").get_parameter_value().integer_value)

        self._lifts = 0          # 抬過幾次
        self._passes = 0         # 總共處理幾則
        self._max_scale_hits = 0  # 撞到倍率上限幾次

        # 指令話題要 RELIABLE：漏一則停止指令的代價太高。
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Twist, self.out_topic, qos)
        self.create_subscription(Twist, self.in_topic, self._cb, qos)

        self.get_logger().info(
            f"✓ 速度下限守門員：{self.in_topic} → {self.out_topic}，"
            f"下限 {self.min_speed:.3f} m/s（底盤死區實測 0.085）")
        if self.min_speed <= 0.0:
            self.get_logger().warning(
                "⚠ min_speed = 0，本節點只做轉送。要停用請直接不要啟動它，"
                "留著轉送只會多一跳延遲。")

    def _cb(self, msg: Twist) -> None:
        self._passes += 1
        v = msg.linear.x

        # ★★ 停止指令永遠原樣通過 —— 這是本節點唯一不能妥協的地方 ★★
        #   PolygonStop 與 FootprintApproach 判定要停時送的就是 0。
        #   把 0 抬起來等於把安全系統的「停」變成「慢慢往前」。
        if self.min_speed <= 0.0 or v == 0.0 or abs(v) >= self.min_speed:
            self.pub.publish(msg)
            return

        # 到這裡代表：有非零速度指令，但小到馬達不會轉。
        scale = self.min_speed / abs(v)
        if scale > self.max_scale:
            # 指令小到要放大三倍以上 —— 那已經不是「慢慢走」而是「幾乎停住」，
            # 抬上去會與原始意圖差太多。這種情況照原樣送出去，
            # 讓上游的卡住偵測自己處理（它現在也擋得住，見 path_teach 的硬停分支）。
            self._max_scale_hits += 1
            if self.log_every and self._max_scale_hits % self.log_every == 1:
                self.get_logger().warning(
                    f"指令 {v:+.4f} m/s 要放大 {scale:.1f} 倍才過死區，"
                    f"超過上限 {self.max_scale:.1f} —— 原樣送出（第 "
                    f"{self._max_scale_hits} 次）")
            self.pub.publish(msg)
            return

        out = Twist()
        out.linear.x = math.copysign(self.min_speed, v)
        # ★ 曲率 κ = ω / v 必須保持不變：阿克曼的舵角由 κ 決定，
        #   只抬 v 會讓車子走出比指令更直的線。
        out.angular.z = msg.angular.z * scale
        # 其餘軸原樣帶過（阿克曼用不到，但不要靜默丟掉）
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        self.pub.publish(out)

        self._lifts += 1
        if self.log_every and self._lifts % self.log_every == 1:
            pct = 100.0 * self._lifts / max(1, self._passes)
            self.get_logger().info(
                f"抬過死區：{v:+.4f} → {out.linear.x:+.3f} m/s"
                f"（ω {msg.angular.z:+.3f} → {out.angular.z:+.3f}，×{scale:.2f}）"
                f"　累計 {self._lifts}/{self._passes} = {pct:.1f}%")

    def destroy_node(self) -> None:
        if self._passes:
            pct = 100.0 * self._lifts / self._passes
            self.get_logger().info(
                f"總結：{self._passes} 則指令，抬過死區 {self._lifts} 則（{pct:.1f}%），"
                f"超過倍率上限 {self._max_scale_hits} 則")
            # ★ 這個百分比是判斷「這條路徑好不好走」的直接指標：
            #   8/24 實測 順利 1.1% / 窄空間 26.3% / 倒車失敗 18.0%
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelFloorNode()
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
