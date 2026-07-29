#!/usr/bin/env python3

"""IMU 陀螺儀零偏補償節點 (_cc)

## 為什麼需要

這台車的 IMU 靜止時 z 軸角速度不是 0，實測：

    車子完全靜止，angular_velocity.z 平均 = +0.00132 rad/s
                                          = +4.54 度/分鐘

這個零偏會被 robot_localization 的 EKF 一路積分進 odom_combined 的 yaw：

    odom->base_footprint 的 yaw   60 秒漂 +4.74 度
    map->odom                     完全不動 (AMCL 靜止時不更新)
    => map->base_footprint        整個跟著漂

後果不是「顯示不準」而已。導航目標常以車頭方向推算，靜置五分鐘就偏 23 度，
「前方一公尺」會被算到牆裡去，規劃器直接回 no valid path。
在長直走廊裡更糟：兩側平行牆讓 AMCL 幾乎無法從雷射觀測修正角度，
零偏推多少就偏多少，沒有東西拉得回來。

## 做法

開機時車子必須靜止，取一段樣本算出各軸的角速度平均值當作零偏，
之後每一則 IMU 訊息都扣掉它，發布到新話題供 EKF 使用。

    /imu/data_raw  --[本節點扣零偏]-->  /imu/data_unbiased  --> EKF

EKF 讀哪個話題是由 ekf.yaml 的 imu0 參數決定，可以在 launch 層覆寫，
不需要改動 turn_on_wheeltec_robot 的原廠設定檔。

## 為什麼光有開機校準不夠（2026-07-29 實測）

零偏會隨溫度漂移，而且**在同一次開機內就會變**。實測：

    開機時校準得到     z = -2.68 度/分
    暖機約 1.5 小時後   殘餘漂移 = +2.46 度/分

方向相反、大小相近 —— 典型的「開機值變成過度補償」。後果很嚴重：
車子靜止 10 分鐘，估計朝向就轉了 25 度，而 costmap 是在這個
**持續旋轉的座標系**裡累積牆面觀測，結果把走廊兩側的牆抹到車頭正前方，
規劃器直接回 `no valid path found`（實測就是這樣卡住的）。

在長直走廊裡沒有東西救得回來：兩側平行牆讓 scan matching 幾乎無法修正角度，
而 slam_toolbox 的 `minimum_travel_distance` 讓車子不動時根本不處理新掃描，
map->odom 一直是單位矩陣，零偏推多少就偏多少。

所以改成**車子靜止時持續重估零偏**（慣性導航裡的 ZUPT，zero-velocity update）：
用**輪速里程計**（獨立於 IMU，不會被零偏污染）判斷車子確實沒動，
確認靜止一段時間後才開始取樣，用指數移動平均慢慢修正零偏估計值。

## 注意

- 校準期間車子**必須完全靜止**，否則會把真實旋轉當成零偏記下來。
  節點會檢查樣本的標準差，變動太大就拒絕校準並沿用預設值。
- 自動重估只在靜止時進行，而且每次更新有幅度上限，
  避免某次異常取樣把零偏一次拉歪。
- 仍然保留 /recalibrate_imu_bias 服務，可以強制從頭重新校準。
"""

import math
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger


class ImuBiasCorrectorCcNode(Node):
    """扣掉陀螺儀零偏後轉發 IMU 資料"""

    def __init__(self):
        super().__init__("imu_bias_corrector_cc_node")

        self.declare_parameter("input_topic", "/imu/data_raw")
        self.declare_parameter("output_topic", "/imu/data_unbiased")
        # 校準取樣數與最長等待時間
        self.declare_parameter("calibration_samples", 300)
        self.declare_parameter("calibration_timeout_sec", 30.0)
        # 樣本標準差超過這個值就判定車子不是靜止的，拒絕校準
        self.declare_parameter("max_stddev_rad_s", 0.02)
        # 手動指定零偏 (三軸)，非零時跳過自動校準
        self.declare_parameter("manual_bias", [0.0, 0.0, 0.0])

        # --- 靜止時自動重估零偏 (ZUPT) ---
        self.declare_parameter("auto_recalibrate", True)
        # 用輪速里程計判斷靜止：它不經過 IMU，不會被零偏污染
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("stationary_linear_thresh", 0.01)   # m/s
        self.declare_parameter("stationary_angular_thresh", 0.01)  # rad/s
        # 剛停下來時底盤還在晃，要等安定
        self.declare_parameter("stationary_settle_sec", 3.0)
        # 累積這麼長的靜止取樣才更新一次
        self.declare_parameter("refine_window_sec", 20.0)
        # 指數移動平均權重。取小一點，讓估計值慢慢靠過去而不是跳動
        self.declare_parameter("refine_alpha", 0.3)
        # 單次更新的幅度上限，避免異常取樣一次把零偏拉歪
        # 0.0005 rad/s = 1.7 度/分，正常的溫漂遠比這慢
        self.declare_parameter("refine_max_step_rad_s", 0.0005)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self.n_samples = int(self.get_parameter("calibration_samples").value)
        self.calib_timeout = float(self.get_parameter("calibration_timeout_sec").value)
        self.max_stddev = float(self.get_parameter("max_stddev_rad_s").value)

        self.auto_recal = bool(self.get_parameter("auto_recalibrate").value)
        self.lin_thresh = float(self.get_parameter("stationary_linear_thresh").value)
        self.ang_thresh = float(self.get_parameter("stationary_angular_thresh").value)
        self.settle_sec = float(self.get_parameter("stationary_settle_sec").value)
        self.refine_window = float(self.get_parameter("refine_window_sec").value)
        self.refine_alpha = float(self.get_parameter("refine_alpha").value)
        self.refine_max_step = float(self.get_parameter("refine_max_step_rad_s").value)

        manual = list(self.get_parameter("manual_bias").value)
        self.bias = list(manual)
        self.calibrated = any(abs(v) > 1e-9 for v in manual)

        self._samples = []
        self._collecting = not self.calibrated
        self._lock = threading.Lock()

        # ZUPT 狀態
        self._moving_until = 0.0     # 在這個時刻之前都當作「還在動/還沒安定」
        self._refine_samples = []
        self._refine_start = None
        self._n_refines = 0

        self.pub = self.create_publisher(Imu, out_topic, qos_profile_sensor_data)
        self.create_subscription(Imu, in_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_service(Trigger, "recalibrate_imu_bias", self._recal_cb)

        if self.auto_recal:
            self.create_subscription(
                Odometry, self.get_parameter("odom_topic").value, self._odom_cb, 20
            )

        if self.calibrated:
            self.get_logger().info(f"使用手動指定的零偏: {self.bias}")
        else:
            self.get_logger().warn(
                f"IMU 零偏校準中，請保持車子完全靜止 ({self.n_samples} 筆樣本)..."
            )
        self.get_logger().info(f"IMU 補償: {in_topic} -> {out_topic}")

        self._start_time = self.get_clock().now()

    # ==================================================================
    def _odom_cb(self, msg: Odometry) -> None:
        """用輪速判斷車子有沒有在動。

        刻意不用 /odom_combined（EKF 輸出）——那是 IMU 融合過的結果，
        零偏本身就會讓它顯示出不存在的旋轉，拿它判斷靜止會自我循環。
        """
        v = msg.twist.twist
        moving = (
            abs(v.linear.x) > self.lin_thresh
            or abs(v.linear.y) > self.lin_thresh
            or abs(v.angular.z) > self.ang_thresh
        )
        if moving:
            now = self.get_clock().now().nanoseconds * 1e-9
            with self._lock:
                # 動了就把安定倒數重新拉長，並丟掉已經收集的樣本
                self._moving_until = now + self.settle_sec
                self._refine_samples = []
                self._refine_start = None

    def _maybe_refine(self, now: float) -> None:
        """呼叫時必須已經持有 _lock。累積夠了就更新零偏估計。"""
        if self._refine_start is None or now - self._refine_start < self.refine_window:
            return

        n = len(self._refine_samples)
        self._refine_start = now
        samples = self._refine_samples
        self._refine_samples = []
        if n < 50:
            return

        means = [sum(s[i] for s in samples) / n for i in range(3)]
        var_z = sum((s[2] - means[2]) ** 2 for s in samples) / n
        if var_z**0.5 > self.max_stddev:
            # 輪速說沒動，但陀螺儀在抖 —— 可能有人推車或地面震動，這批不要
            return

        old_z = self.bias[2]
        new = []
        for i in range(3):
            target = (1.0 - self.refine_alpha) * self.bias[i] + self.refine_alpha * means[i]
            step = max(-self.refine_max_step, min(self.refine_max_step, target - self.bias[i]))
            new.append(self.bias[i] + step)
        self.bias = new
        self._n_refines += 1

        # 只有變化夠大才記錄，否則穩定之後會洗版
        if abs(self.bias[2] - old_z) > 1e-5:
            self.get_logger().info(
                f"零偏重估 #{self._n_refines}：z {math.degrees(old_z) * 60:+.2f} -> "
                f"{math.degrees(self.bias[2]) * 60:+.2f} 度/分 "
                f"(本批靜止觀測 {math.degrees(means[2]) * 60:+.2f} 度/分, n={n})"
            )

    def _imu_cb(self, msg: Imu) -> None:
        with self._lock:
            if self._collecting:
                self._samples.append(
                    (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
                )
                if len(self._samples) >= self.n_samples:
                    self._finish_calibration()
                # 校準期間仍然轉發原始資料，避免 EKF 斷流
            elif self.auto_recal and self.calibrated:
                now = self.get_clock().now().nanoseconds * 1e-9
                if now >= self._moving_until:
                    # 已經靜止且安定，這一則可以拿來重估
                    if self._refine_start is None:
                        self._refine_start = now
                    self._refine_samples.append(
                        (
                            msg.angular_velocity.x,
                            msg.angular_velocity.y,
                            msg.angular_velocity.z,
                        )
                    )
                    self._maybe_refine(now)
            bias = tuple(self.bias)

        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity.x = msg.angular_velocity.x - bias[0]
        out.angular_velocity.y = msg.angular_velocity.y - bias[1]
        out.angular_velocity.z = msg.angular_velocity.z - bias[2]
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(out)

    def _finish_calibration(self) -> None:
        """呼叫時必須已經持有 _lock"""
        n = len(self._samples)
        means = [sum(s[i] for s in self._samples) / n for i in range(3)]
        variances = [
            sum((s[i] - means[i]) ** 2 for s in self._samples) / n for i in range(3)
        ]
        stddevs = [v**0.5 for v in variances]

        self._collecting = False
        self._samples = []

        # 標準差太大代表取樣期間車子有在動，那樣算出來的「零偏」會把
        # 真實旋轉也記進去，之後每次轉彎都會被錯誤補償，比不補還糟。
        if stddevs[2] > self.max_stddev:
            self.get_logger().error(
                f"校準失敗：z 軸角速度標準差 {stddevs[2]:.5f} rad/s 過大 "
                f"(上限 {self.max_stddev})，車子在校準期間可能有移動。"
                "零偏維持 0，請靜止後呼叫 /recalibrate_imu_bias 重試。"
            )
            return

        self.bias = means
        self.calibrated = True

        self.get_logger().warn(
            f"IMU 零偏校準完成 (n={n}):\n"
            f"  x {means[0]:+.6f}  y {means[1]:+.6f}  z {means[2]:+.6f} rad/s\n"
            f"  z 軸 = {math.degrees(means[2]) * 60:+.2f} 度/分鐘 —— 這就是 yaw 一直漂的來源\n"
            f"  標準差 z={stddevs[2]:.6f} rad/s\n"
            f"  自動重估(ZUPT)：{'開啟' if self.auto_recal else '關閉'} —— "
            f"零偏會隨溫度變，實測暖機 1.6 小時可以從 -2.68 漂到 +7.06 度/分"
        )

    def _recal_cb(self, _req, res):
        with self._lock:
            self._samples = []
            self._collecting = True
            # 重新校準期間不要讓 ZUPT 同時動手，兩者會互相干擾
            self._refine_samples = []
            self._refine_start = None
        self.get_logger().warn("重新校準 IMU 零偏，請保持車子靜止...")
        res.success = True
        res.message = f"開始重新校準 ({self.n_samples} 筆樣本)，請勿移動車子"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ImuBiasCorrectorCcNode()
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
