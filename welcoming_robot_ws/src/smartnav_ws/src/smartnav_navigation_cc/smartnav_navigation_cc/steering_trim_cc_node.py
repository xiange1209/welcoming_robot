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


def _yaw_from_quat(q) -> float:
    """四元數取 yaw。只需要繞 z 的分量，不必引進 tf_transformations。"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _norm_angle(a: float) -> float:
    """正規化到 (-pi, pi]。跨 ±pi 時若不做這件事，朝向差會算成 2pi 的誤差。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


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
        # 0.02 -> 0.005（2026-08-01 二次修正）。
        # 死區是**遲滯**：它容許的穩態誤差上限就等於死區本身。0.02 rad/s
        # 在 0.15 m/s 下相當於 0.133 rad/m = 7.6 度/m，比 steering_trim
        # 要修的 4.12 度/m 還大 —— 抑制抖動的工具不該比病症更嚴重。
        # 主要手段是低通（steer_filter_tau），死區只用來殺掉量化級的雜訊。
        self.declare_parameter("steer_deadband_rad_s", 0.005)
        self.declare_parameter("hold_steer_on_stop", True)
        # 抖動量測：每隔這麼久印一次「每秒轉向反轉次數」，用來驗證有沒有改善
        self.declare_parameter("chatter_report_sec", 0.0)   # 0 = 不印

        # ── 線上估測轉向偏移（2026-08-06 加）──────────────────────
        #
        # 為什麼需要：`trim_rad_per_m` 假設被補償的量是常數，但實測它不是。
        # 同一個 session 內量到兩次，差了一個數量級：
        #     稍早（trim -0.072 生效、經完整鏈路）  -1.87 度/m  往右
        #     稍後（繞過 trim、直接發底盤）         +12.7 度/m  往左
        # 中間發生的事：車子被搬動、楔住、人工救車、遙控來回多次。
        # 使用者的觀察「轉向補償感覺每次都不太相同」因此從感覺變成資料。
        #
        # 而且「左右打滿反推中位」得到 −2.19°（偏右），
        # 直接量中位卻是 +4.08°（偏左）——**方向相反**，
        # 所以它也不是單純的零位偏移，反推模型不成立。
        #
        # 一個寫死的常數修不了一個會漂移的量。改成邊開邊估：
        # 車子直行時（補償**之前**的指令角速度 ≈ 0），odom 量到的偏轉率
        # 除以速度就是當下的殘餘，把它慢慢補進 trim。
        # 輪子再被撞歪也會自己跟上，不需要專門的校正動作。
        self.declare_parameter("auto_trim", True)
        # 每次更新只走殘餘的這個比例。取小值：寧可收斂慢，
        # 也不要在轉彎剛結束、yaw 還在安定時被一兩筆壞資料帶歪。
        self.declare_parameter("auto_trim_gain", 0.02)
        self.declare_parameter("auto_trim_min_speed", 0.03)      # 太慢時 yaw 訊噪比差
        self.declare_parameter("auto_trim_straight_dz", 0.02)    # 指令 |ω| 小於這個才算直行
        self.declare_parameter("auto_trim_settle_sec", 1.5)      # 直行滿這麼久才開始採信
        self.declare_parameter("auto_trim_limit", 0.30)          # trim 的絕對值上限
        # ── 時間平均視窗（2026-08-06）★ 這是把震盪變成收斂的關鍵 ──
        #
        # 原本逐筆 odom（20 Hz）算 residual = wz / v 就更新一次。實機 log：
        #     殘餘  +3.49 度/m -> trim -0.0732    1 次
        #     殘餘 -10.23 度/m -> trim -0.1009   29 次
        #     殘餘  -2.32 度/m -> trim -0.0524   80 次
        # 殘餘在 +3.49 / −10.23 / −2.32 之間跳，trim 跟著 −0.073 / −0.101 /
        # −0.052 來回 —— 這是**震盪不是收斂**。單筆 odom 的 yaw 速率雜訊
        # 遠大於我們要估的量（trim 等效約 1.3 度，而雜訊有好幾度）。
        #
        # 改成累積一段時間的**位移**與**朝向變化**再相除：
        #     殘餘 = Δyaw / Δs
        # 位置與朝向是積分量，雜訊會互相抵消；速率是微分量，雜訊被放大。
        # 同樣的資料，換個算法訊噪比差一個數量級。
        self.declare_parameter("auto_trim_window_sec", 2.0)

        self.trim = float(self.get_parameter("trim_rad_per_m").value)
        self.max_trim = float(self.get_parameter("max_trim_rad_s").value)
        self.steer_tau = float(self.get_parameter("steer_filter_tau").value)
        self.steer_deadband = float(self.get_parameter("steer_deadband_rad_s").value)
        self.hold_on_stop = bool(self.get_parameter("hold_steer_on_stop").value)
        self.chatter_report = float(self.get_parameter("chatter_report_sec").value)
        self.auto_trim = bool(self.get_parameter("auto_trim").value)
        self.at_gain = float(self.get_parameter("auto_trim_gain").value)
        self.at_min_speed = float(self.get_parameter("auto_trim_min_speed").value)
        self.at_dz = float(self.get_parameter("auto_trim_straight_dz").value)
        self.at_settle = float(self.get_parameter("auto_trim_settle_sec").value)
        self.at_limit = float(self.get_parameter("auto_trim_limit").value)
        self.at_window = float(self.get_parameter("auto_trim_window_sec").value)
        self._straight_since = 0.0     # 指令保持直行的起始時刻（0 = 目前不是直行）
        self._at_updates = 0
        self._at_last_log = 0.0
        # 時間平均視窗的起點：(x, y, yaw, t, 行進方向)。None = 尚未開始累積。
        self._at_win = None
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        # ★ 2026-08-04：讓 `ros2 param set trim_rad_per_m` 真的生效 ★
        #
        # 原本 self.trim 只在這裡讀一次。`ros2 param set` 會回報成功、
        # `ros2 param get` 也讀得到新值，但 _cmd_cb 用的是快取，行為完全不變。
        # 同一個坑今天在 path_teach_cc 的 merge_min_segment_m 也踩過一次
        # （代價是白錄一趟路徑）。
        # ★ `ros2 param get` 讀回新值只證明參數伺服器存了它，不證明節點會用它。
        #
        # 這條在熱路徑上（10~20 Hz），所以用回呼更新快取，不在 _cmd_cb 裡現讀。
        #
        # 為什麼需要**執行時**可調：轉向偏移不是固定的機械常數。
        # 實測（2026-08-04）：白天三次導航都往左偏，收工前在同一條走廊量到的
        # 卻是往右偏 1.87 度/m。中間差別是車子被人搬動過很多次——
        # 停車時 hold_steer_on_stop=True 不會把前輪扳正，搬動時輪子是自由的，
        # 舵機的指令位置與實際位置很容易脫節。**一個寫死的常數不可能一直對。**
        def _on_param(params):
            from rcl_interfaces.msg import SetParametersResult
            for p in params:
                if p.name == "trim_rad_per_m":
                    self.trim = float(p.value)
                    self.get_logger().info(
                        f"轉向補償更新為 {self.trim:.4f} rad/m "
                        f"({math.degrees(self.trim):+.2f} 度/m)")
                elif p.name == "max_trim_rad_s":
                    self.max_trim = float(p.value)
            return SetParametersResult(successful=True)
        self.add_on_set_parameters_callback(_on_param)

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
        if self.auto_trim:
            self._update_auto_trim(msg)

    def _update_auto_trim(self, msg: Odometry) -> None:
        """直行時把量到的殘餘偏轉率補進 trim（見參數宣告處的說明）

        閘門有四道，任何一道不過就丟掉整個累積視窗：
          1. 目前指令是直行（補償前的 |ω| < at_dz）
          2. 已經直行滿 at_settle 秒（避免吃到轉彎後 yaw 還在安定的資料）
          3. 車速夠快（太慢時 yaw 的訊噪比很差）
          4. 行進方向在視窗中途沒有變號

        ★ 2026-08-06：改成時間平均，不再逐筆更新。

        殘餘用「一段時間走過的距離」與「同一段時間的朝向變化」相除，
        而不是單筆的 `wz / v`：

            residual = Δyaw / Δs        （帶號，倒車時 Δs 為負）

        位置與朝向是**積分量**，感測雜訊會互相抵消；角速度是**微分量**，
        雜訊被放大。同一批資料換個算法，訊噪比差一個數量級。
        逐筆版本的實測是 +3.49 / −10.23 / −2.32 度/m 反覆跳（見參數處註解）。

        閘門不過時**必須清掉視窗**，否則會把轉彎前後兩段接起來算，
        得到一個完全不存在的「殘餘」。
        """
        now = time.monotonic()

        # ── 四道閘門：任何一道不過，累積作廢 ──
        if self._straight_since == 0.0 or now - self._straight_since < self.at_settle:
            self._at_win = None
            return
        v = msg.twist.twist.linear.x
        if abs(v) < self.at_min_speed:
            self._at_win = None
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        cur_dir = 1 if v > 0 else -1

        if self._at_win is None:
            self._at_win = (x, y, yaw, now, cur_dir)
            return
        x0, y0, yaw0, t0, dir0 = self._at_win

        # 視窗中途變換行進方向 -> 前進段與倒車段的殘餘不能混算
        if cur_dir != dir0:
            self._at_win = (x, y, yaw, now, cur_dir)
            return
        if now - t0 < self.at_window:
            return

        dist = math.hypot(x - x0, y - y0)
        # 走得比「最低速 × 視窗長度的一半」還少 -> 打滑或被擋住，這段不可信。
        # （這是第 4 道閘門在時間平均版本下的等價物：原本比對 odom 與指令
        #   同號，現在直接看整段到底有沒有移動。）
        if dist < self.at_min_speed * self.at_window * 0.5:
            self._at_win = None
            return

        dyaw = _norm_angle(yaw - yaw0)
        residual = dyaw / (dist * dir0)      # 每公尺的偏轉；倒車時距離帶負號
        new = self.trim - self.at_gain * residual
        new = max(-self.at_limit, min(self.at_limit, new))

        # 視窗用掉就重開，下一段從現在的位姿重新累積
        self._at_win = (x, y, yaw, now, cur_dir)

        if abs(new - self.trim) < 1e-6:
            return
        self.trim = new
        self._at_updates += 1
        if now - self._at_last_log > 5.0:
            self._at_last_log = now
            self.get_logger().info(
                f"線上估測（{self.at_window:.0f} 秒平均，走 {dist:.2f} m 轉 "
                f"{math.degrees(dyaw):+.2f} 度）：殘餘 {math.degrees(residual):+.2f} 度/m -> "
                f"trim {self.trim:+.4f} ({math.degrees(math.atan(self.trim * 0.322)):+.2f} 度)"
                f"  已更新 {self._at_updates} 次")

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
        """先平滑上游指令，**再**加補償 —— 順序很重要

        === 2026-08-01 修：原本順序反了，補償量被死區整個吃掉 ===

        原本是「先加 trim -> 再過死區/低通」：

            target = msg.angular.z + trim * linear.x
            out.angular.z = self._smooth_steer(target, moving)

        直走廊裡 MPPI 輸出穩定接近 0 時，target 與上一次輸出的差就只有
        trim 那一點點，而 trim 在各速度產生的量是：

            0.05 m/s -> 0.0036    0.10 -> 0.0072
            0.15 m/s -> 0.0108    0.25 -> 0.0180

        **全部小於 0.02 的死區**，於是 _smooth_steer 每次都 return 舊值，
        補償量 100% 被丟棄，車子照 4.12 度/m 往左漂 —— 為了治舵機抖動
        加的東西，反而製造了比原病症更大的誤差源。

        正解是分清楚兩者的性質：
          - MPPI 的 angular.z 含高頻抖動 -> 該被低通與死區抑制
          - trim 是常態 DC 偏置        -> 不該受抖動抑制邏輯管轄
        所以平滑只作用在上游指令上，trim 在平滑之後才加。
        """
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y

        moving = abs(msg.linear.x) > 1e-3
        prev = self._steer_out

        # 只平滑上游指令
        smoothed = self._smooth_steer(msg.angular.z, moving)

        # 補償量與線速度成正比。
        # 機械零位偏移本身是固定的轉向角，但 /cmd_vel 的介面是**角速度**
        # （韌體 usartx.c:394 的 Vz_to_Akm_Angle 才把角速度換成轉向角），
        # 所以在指令層要抵銷它，補償量必須是 -v*tan(delta_off)/L，正比於 v。
        #   delta_off = atan(0.0719 * 0.322) = 0.02315 rad = 1.33 度
        #   反查 trim = -tan(0.02315)/0.322 = -0.0719  <-> 參數 -0.072，吻合
        # 靜止時不補 —— 阿克曼車原地打方向盤沒有意義。
        corr = 0.0
        if moving:
            corr = self.trim * msg.linear.x
            corr = max(-self.max_trim, min(self.max_trim, corr))

        out.angular.z = smoothed + corr
        self._tally_chatter(smoothed - prev)

        # 記錄「補償之前的指令是不是直行」——線上估測的閘門之一。
        # 用 msg.angular.z（上游要求的）而不是 out.angular.z（含補償），
        # 否則補償量本身會被當成轉彎指令，估測永遠不會啟動。
        now = time.monotonic()
        if moving and abs(msg.angular.z) < self.at_dz:
            if self._straight_since == 0.0:
                self._straight_since = now
        else:
            self._straight_since = 0.0

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
        # ★ 2026-08-25：牆鐘改單調鐘。這是一個**會讓車子直線衝出去**的迴圈，
        #   而 RPi4 沒有 RTC 電池，NTP 在連上熱點後會把系統時間階躍校正。
        #   往後跳的話 time.time() - t0 變負，這個迴圈**不會因逾時停下**，
        #   而它全程沒有任何淨空檢查（只有上面那句「請確認前方淨空」的提醒）。
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration and rclpy.ok():
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
