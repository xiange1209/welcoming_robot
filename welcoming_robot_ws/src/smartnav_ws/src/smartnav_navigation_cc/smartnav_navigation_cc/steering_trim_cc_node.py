#!/usr/bin/env python3

"""轉向零位補償節點 (_cc)

這台車下 angular.z = 0 (要求直行) 時並不會走直線。實測：

    以 0.15 m/s 直線前進 0.464 m，航向變化 +3.03 度
    扣掉 IMU 零偏在同一段時間的貢獻 (+0.23 度)
    => 真正的機械偏移約 +6.0 度/公尺，車子持續往左偏

原因是前輪轉向的機械零位沒對正 (連桿/舵機中位偏了)，
而 wheeltec 的 wheeltec_param.yaml 沒有提供轉向零位校準參數，
所以只能在指令層補回來。

補償方式：偏轉量與行進距離成正比，所以修正量要跟線速度成正比

    angular.z += trim_rad_per_m * linear.x

speed 為 0 時修正量也是 0 —— 阿克曼車靜止時轉方向盤沒有意義，
而且會讓 collision_monitor 誤判成有旋轉意圖。

## 接線位置

    velocity_smoother  --cmd_vel_smoothed-->  [本節點]
                       --cmd_vel_trimmed-->   collision_monitor
                       --cmd_vel-->           底盤

刻意插在 collision_monitor **之前**：補償後的指令仍然要通過碰撞檢查。
如果插在後面，等於繞過安全層，補償量一旦設錯就直接撞牆。

## 重新校準

    ros2 run smartnav_navigation_cc steering_trim_cc --ros-args -p measure_mode:=true

會以固定速度直行一段距離，量出偏轉率並印出建議的 trim_rad_per_m。
車子會移動約 0.5 公尺，請確認前方淨空。
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class SteeringTrimCcNode(Node):
    """把「下 0 卻會轉」的機械偏移補回來"""

    def __init__(self):
        super().__init__("steering_trim_cc_node")

        # 每公尺要補多少弧度。實測車子往左偏 +6.0 度/公尺，
        # 所以補一個負值把它拉回來：-6.0 度/m = -0.105 rad/m
        # -0.072 rad/m = -4.12 度/m (2026-07-28 重新校準)
        #
        # 校準歷程值得記錄：第一次量到 +6.54 度/m、補了 -0.105，結果補過頭
        # (原本偏左 0.076 m 變成偏右 0.123 m)。原因是當時 IMU 零偏還沒修，
        # 量到的偏轉裡混了 +5 度/分鐘的感測器漂移，不是真的機械偏移。
        #
        # 修好 IMU 零偏融合之後重測兩次：+4.25 與 +4.00 度/m，
        # 一致性很好，平均 +4.12 度/m 才是前輪轉向零位的真實偏差。
        #
        # 要重新校準：ros2 service call /measure_steering_trim std_srvs/srv/Trigger
        # (務必在 IMU 零偏已校準的狀態下量，否則會再次量錯)
        self.declare_parameter("trim_rad_per_m", -0.072)
        self.declare_parameter("input_topic", "cmd_vel_smoothed")
        self.declare_parameter("output_topic", "cmd_vel_trimmed")
        # 補償量上限，避免參數設錯時整台車繞圈
        self.declare_parameter("max_trim_rad_s", 0.15)
        self.declare_parameter("measure_mode", False)
        self.declare_parameter("measure_speed", 0.15)
        self.declare_parameter("measure_duration_sec", 4.0)

        # ── 轉向平滑（2026-08-01 加）──────────────────────────────
        #
        # 症狀：導航中方向舵持續抽動、不停地回正又打回去。
        #
        # 成因不是這個節點，是上游 MPPI：它每個週期從 400 條取樣軌跡做
        # softmax 加權，`wz_std: 0.4` 的取樣噪聲讓輸出的 angular.z 本來就在
        # 小幅震盪；而 velocity_smoother 的 deadband_velocity 是 [0,0,0]，
        # 等於把每一個微小變化原封不動送到舵機。舵機以 10~20 Hz 抽動，
        # 既磨損機構也吃電流。
        #
        # 這裡做三件事（都可以個別關掉）：
        #   1. 一階低通 —— 主要手段。震盪頻率(5~10 Hz)遠高於真正的轉向動作
        #      (1~3 秒完成)，低通能大幅衰減前者而幾乎不影響後者。
        #   2. 遲滯死區 —— 變化小於死區就維持原輸出，殺掉殘餘的微抖。
        #   3. 停車不強制回正 —— 車子停下時把輪子扳回中間是純粹多餘的動作。
        #      阿克曼車靜止時前輪角度不影響任何事，下次起步再轉即可。
        #
        # ⚠ 低通會在轉向迴路裡引入延遲，而 MPPI 的模型並不知道這個延遲。
        #   time_constant 0.15 秒 @ 0.25 m/s 約等於 3.7 公分的行程落後。
        #   調大更平順但可能讓 MPPI 過衝，**必須實機驗證**。設 0 可完全停用。
        self.declare_parameter("steer_filter_tau", 0.15)
        self.declare_parameter("steer_deadband_rad_s", 0.02)
        self.declare_parameter("hold_steer_on_stop", True)
        # 抖動量測：每隔這麼久印一次「每秒轉向反轉次數」，用來驗證有沒有改善
        self.declare_parameter("chatter_report_sec", 0.0)   # 0 = 不印

        self.trim = float(self.get_parameter("trim_rad_per_m").value)
        self.max_trim = float(self.get_parameter("max_trim_rad_s").value)
        self.steer_tau = float(self.get_parameter("steer_filter_tau").value)
        self.steer_deadband = float(self.get_parameter("steer_deadband_rad_s").value)
        self.hold_on_stop = bool(self.get_parameter("hold_steer_on_stop").value)
        self.chatter_report = float(self.get_parameter("chatter_report_sec").value)
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        self._steer_out = 0.0        # 濾波後的轉向輸出（狀態）
        self._steer_last_t = time.monotonic()
        # 抖動統計：反轉次數 = 轉向指令變號的次數，直接反映舵機來回動的次數
        self._chatter_reversals = 0
        self._chatter_samples = 0
        self._chatter_prev_delta = 0.0
        self._chatter_t0 = time.monotonic()

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, in_topic, self._cmd_cb, 10)
        self.create_service(Trigger, "measure_steering_trim", self._measure_cb)

        self._odom = None
        self.create_subscription(Odometry, "odom_combined", self._odom_cb, 10)

        self.get_logger().info(
            f"轉向補償啟動: {in_topic} -> {out_topic}, trim={self.trim:.4f} rad/m "
            f"({math.degrees(self.trim):+.1f} 度/m)"
        )

        if bool(self.get_parameter("measure_mode").value):
            threading.Thread(target=self._measure_sequence, daemon=True).start()

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def _smooth_steer(self, target: float, moving: bool) -> float:
        """低通 + 遲滯死區 + 停車不回正。回傳實際要送出的角速度"""
        now = time.monotonic()
        dt = now - self._steer_last_t
        self._steer_last_t = now
        # dt 異常（第一次呼叫、或節點被卡住很久）時不要讓濾波器一次跳到底
        dt = min(0.5, max(1e-3, dt))

        # 車子沒在動就維持目前輪角。回正是多餘動作，阿克曼車靜止時輪角不影響任何事。
        if not moving and self.hold_on_stop:
            return self._steer_out

        # 遲滯：變化太小就不動舵機
        if abs(target - self._steer_out) < self.steer_deadband:
            return self._steer_out

        if self.steer_tau <= 0.0:
            self._steer_out = target          # 停用濾波
        else:
            alpha = dt / (self.steer_tau + dt)
            self._steer_out += alpha * (target - self._steer_out)
        return self._steer_out

    def _tally_chatter(self, delta: float) -> None:
        """統計轉向指令的反轉次數，用來量化抖動有沒有改善"""
        if self.chatter_report <= 0.0:
            return
        self._chatter_samples += 1
        if delta * self._chatter_prev_delta < 0.0:     # 變號 = 舵機掉頭
            self._chatter_reversals += 1
        if abs(delta) > 1e-6:
            self._chatter_prev_delta = delta

        elapsed = time.monotonic() - self._chatter_t0
        if elapsed >= self.chatter_report:
            self.get_logger().info(
                f"轉向抖動：{self._chatter_reversals / elapsed:.1f} 次反轉/秒"
                f"（{self._chatter_samples} 筆指令 / {elapsed:.0f} 秒）"
            )
            self._chatter_reversals = 0
            self._chatter_samples = 0
            self._chatter_t0 = time.monotonic()

    def _cmd_cb(self, msg: Twist) -> None:
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y

        target = msg.angular.z
        # 修正量與線速度成正比。靜止時不補 —— 阿克曼車原地打方向盤沒有意義。
        moving = abs(msg.linear.x) > 1e-3
        if moving:
            corr = self.trim * msg.linear.x
            corr = max(-self.max_trim, min(self.max_trim, corr))
            target += corr

        prev = self._steer_out
        out.angular.z = self._smooth_steer(target, moving)
        self._tally_chatter(out.angular.z - prev)

        self.pub.publish(out)

    # ==================================================================
    @staticmethod
    def _yaw(msg: Odometry) -> float:
        q = msg.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _measure_sequence(self):
        """直行一段距離，量出每公尺的偏轉率"""
        time.sleep(3.0)
        speed = float(self.get_parameter("measure_speed").value)
        duration = float(self.get_parameter("measure_duration_sec").value)

        if self._odom is None:
            self.get_logger().error("收不到 odom，無法量測")
            return

        start = self._odom
        x0, y0 = start.pose.pose.position.x, start.pose.pose.position.y
        yaw0 = self._yaw(start)

        self.get_logger().warn(f"開始量測：直行 {duration:.1f} 秒 @ {speed:.2f} m/s，請確認前方淨空")
        cmd = Twist()
        cmd.linear.x = speed
        # 量測時不套用補償，才量得到原始偏移
        t0 = time.time()
        while time.time() - t0 < duration and rclpy.ok():
            self.pub.publish(cmd)
            time.sleep(0.05)
        self.pub.publish(Twist())
        time.sleep(1.5)

        end = self._odom
        x1, y1 = end.pose.pose.position.x, end.pose.pose.position.y
        dist = math.hypot(x1 - x0, y1 - y0)
        dyaw = math.atan2(math.sin(self._yaw(end) - yaw0), math.cos(self._yaw(end) - yaw0))

        if dist < 0.05:
            self.get_logger().error(f"位移只有 {dist:.3f} m，量測無效")
            return

        rate = dyaw / dist
        suggested = -rate
        self.get_logger().warn(
            f"量測結果：位移 {dist:.3f} m、偏轉 {math.degrees(dyaw):+.2f} 度\n"
            f"  偏轉率 {math.degrees(rate):+.2f} 度/m\n"
            f"  建議 trim_rad_per_m: {suggested:.4f}  ({math.degrees(suggested):+.2f} 度/m)\n"
            f"  注意：這個值包含了 IMU 零偏的貢獻，車速越慢、零偏佔比越大。"
        )

    def _measure_cb(self, _req, res):
        threading.Thread(target=self._measure_sequence, daemon=True).start()
        res.success = True
        res.message = "已開始量測，結果會印在 log (車子會前進約 0.5 m)"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = SteeringTrimCcNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
