#!/usr/bin/env python3

"""卡住偵測節點 (_cc)

## 為什麼需要

雷達裝在離地約 0.11 m 的高度，**比它矮的東西完全掃不到**：地上的雜物、
電線、地毯捲邊、椅腳橫桿。這些東西卡得住輪子，但 costmap 上一片空白，
Nav2 會持續下前進指令，馬達空轉、輪子空滑，系統卻以為一切正常。

這台車的超音波話題 (`/ultrasonic_data_A~F`) 雖然存在，但原廠
`ranger_avoid_flag` 與 `ultrasonic_avoid` 都是 false、實測收不到任何資料，
所以沒有辦法從感測器端偵測這類障礙。

## 做法

改成偵測「結果」而不是「原因」：比對**下出去的速度指令**與**實際里程計位移**。

    指令要求在動 (|v| > cmd_threshold)
    但實際幾乎沒動 (實測速度 < motion_threshold)
    持續超過 stuck_time_sec
    => 判定卡住

這種偵測方式不管障礙物是什麼、有沒有被掃到，只要輪子沒有把車子推動就會發現。

判定卡住後：
  1. 發布 /robot_stuck (std_msgs/Bool)
  2. 取消目前的 navigate_to_pose 目標 —— 讓 Nav2 停止硬推
     ★ 自動探索進行中（map_service_cc 發的 /exploration_active 為 true）時**不取消**，
       見下方「探索期間讓步」
  3. 同一次卡住每 cooldown_sec 秒重複提醒一次（再發一次 /robot_stuck）；新的一次卡住照常判定

## 探索期間讓步（2026-09-24）

舊版這裡寫「取消目標可以讓 frontier explorer 把它記為失敗而換下一個」——
**這個假設是錯的**。explorer 收到 STATUS_CANCELED 時，只有在取消是它**自己的**
無進展看門狗發起的才記失敗（上游 frontier_explorer_core_dispatch.cpp:1093-1102）；
外部取消一律當成正常搶佔，1 秒沉澱後重選，多半又選回同一個到不了的點：

    開過去 -> 卡住 -> 4 秒後這裡取消 -> explorer 不記失敗 -> 又選同一點 -> 開過去 -> ...

explorer 的控制服務只有 START/STOP，沒有「把這個點標成到不了」的介面，
所以唯一不改上游的修法是：探索期間**不在這裡取消**，交給 explorer 自己的看門狗
（frontier_explore_cc.yaml 的 frontier_suppression_no_progress_timeout_s）。
它取消時會記失敗、把該區域抑制起來，才會真的換目標。
代價是真的被矮障礙卡住時，要推到那個逾時才停，所以那邊同步縮短了。

★ 例外（2026-09-25）：探索開始後的前 defer_grace_sec 秒**照舊取消**。
那段是 explorer 的啟動寬限期，它的看門狗不動作 —— 這裡再讓步就沒有人會停車。

★ 後備（2026-09-25）：從第一次讓步起，車子 defer_max_sec 秒都沒真的動起來，就不再等，照舊取消
（實際約在讓步後 20 秒、卡住後約 24 秒生效，原因見 defer_max_sec 的註解）。
讓步的前提是「explorer 的看門狗會在約 16 秒內接手」，explorer 當掉或卡在長計算時這個前提不成立。

★ 看門狗本身也修過（2026-09-25）：上游會被 nav2（Jazzy）每個新目標第一筆 distance_remaining=0.0
鎖死，變成「每個開超過 15 秒的目標都判失敗」，跟有沒有在前進無關。修補已 commit 進子模組指向的
fork（說明在 patches/frontier_exploration_ros2/），讓步的設計是建立在修補過的看門狗上。

## 注意

倒車脫困、窄處來回修正時，車子本來就會短暫停頓，所以 stuck_time_sec
不能設太短，否則會把正常的恢復動作誤判成卡住。
"""

import math
import threading
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class StuckDetectorCcNode(Node):
    """指令在動但車子沒動 -> 判定卡住"""

    def __init__(self):
        super().__init__("stuck_detector_cc_node")

        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("odom_topic", "odom_combined")
        # 指令速度超過這個值才視為「要求車子移動」
        #
        # ★★ 2026-08-31：0.05 -> 0.10，因為 0.05 落在底盤的低速死區裡面 ★★
        #
        #   底盤實測死區約 **0.085 m/s**：指令低於它，馬達根本不轉
        #   （2026-08-17 逐段實測，0.05/0.07/0.08 的位移都是 0.000 m，
        #    見 交接_20260817_車子端現況.md:61）。
        #
        #   而本節點的判定是「指令 > cmd_threshold 而且 實測 < motion_threshold」，
        #   所以 **0.05 ~ 0.085 這一段是保證誤判區**：
        #       指令 0.06  -> moving_requested = True
        #       底盤在死區 -> 輪子不轉 -> actual ~ 0 < 0.02 -> actually_moving = False
        #       持續 4 秒  -> 判定卡住 -> **取消導航目標**
        #   車子並沒有被卡住，只是被要求做一件硬體做不到的事。
        #   nav2 很容易下這個區間的指令：接近終點減速、障礙旁限速、窄處轉彎。
        #
        #   ★ 時間線：這個門檻是 2026-07-29（commit 5c61be8）定的，
        #     而死區是 8/17 才量出來的 —— 門檻比量測早三週，之後沒人回頭改。
        #     README 記錄過「卡住偵測器記的是**指令值**，於是報成像被東西卡住」，
        #     但那次只修正了對症狀的理解，沒有修這個數字。
        #
        #   0.10 的依據：與 path_teach_cc 的 `min_move_speed`（0.10）一致 ——
        #   那是本專案已經確立的「低於此值就別指望車子會動」的地板。
        #   語意變成「**指令快到應該要會動了，卻沒動**，才算卡住」。
        #   ⚠ 死區會隨電壓上移（0.085 是 23.7 V 量的），低電量時即使 0.10
        #     也可能不動 —— 那是「電量 >= 23 V 才開始實驗」這條規則的另一個理由。
        self.declare_parameter("cmd_threshold", 0.10)
        # 實際速度低於這個值視為「幾乎沒動」
        self.declare_parameter("motion_threshold", 0.02)
        # 上述狀態持續多久才判定卡住。
        # 不能太短：阿克曼車在窄處來回修正、倒車脫困時本來就會短暫停頓。
        self.declare_parameter("stuck_time_sec", 4.0)
        # 「同一次卡住」重複提醒的間隔。★ 2026-09-25：10 -> 5，而且只管同一次。
        #   舊版是全域冷卻：取消後 explorer 1 秒就重選、車又頂上同一個障礙時，
        #   第二次卡住要等滿 10 秒才觸發 —— 實際推了約 8 秒而不是 4 秒。
        #   現在新的一次卡住（中間有恢復過）一律照 stuck_time_sec 判定。
        self.declare_parameter("cooldown_sec", 5.0)
        self.declare_parameter("cancel_navigation", True)
        # 自動探索進行中就不取消，交給 frontier explorer 自己的看門狗（原因見檔頭）
        self.declare_parameter("defer_to_explorer", True)
        # 探索開始後多久才讓步。★ 要跟 frontier_explore_cc.yaml 的
        # frontier_suppression_startup_grace_period_s 同值（2026-09-25）：
        # 上游每次 START 都重新起算這段寬限期，期間它的無進展看門狗不動作、失敗也不記
        # （core_suppression.cpp:79-85、:123-128）。這段時間若這裡也讓步，就沒有人會停車，
        # 真卡住時要硬推到寬限期結束再加 15 秒。寬限期內取消造成的重選迴圈，
        # 最多也只持續到寬限期結束，比硬推安全。
        self.declare_parameter("defer_grace_sec", 30.0)
        # 讓步的後備上限（2026-09-25，審查 safety#1）：從第一次讓步起算，車子一直沒真的動起來
        # 超過這麼久，就不再等 explorer，照舊取消。
        #   正常情況 explorer 的看門狗在卡住後約 16 秒取消（frontier_explore_cc.yaml 的
        #   no_progress_timeout 15 秒＋1 秒檢查週期），第一次讓步是卡住後 4 秒，所以約 12 秒就解決，
        #   18 秒碰不到。會走到後備的是：explorer 當掉或卡在長計算（Pi 滿載時單執行緒 executor），
        #   或 explorer 換了目標卻又頂上同一個障礙。以前這些情況要推到 240 秒停滯才停。
        #   只有「車子實測真的動了」才重新起算 —— 指令中斷（nav2 恢復、換目標）不算，
        #   不然 nav2 每 20 秒一次的恢復會讓它永遠碰不到上限。
        self.declare_parameter("defer_max_sec", 18.0)
        # ★ 實際生效點（2026-09-25 審查更正）：只在「觸發」時檢查，同一次卡住每 cooldown_sec（5 秒）才觸發一次，
        #   所以是讓步後 ceil(18/5)×5 = **20 秒**、卡住後約 24 秒。若 nav2 的 progress_checker（20 秒）先放棄、
        #   清 costmap 重試，就在重試後第一次觸發（再 4 秒）—— 兩條路都落在卡住後約 24~25 秒。
        #   不要為了對齊 18 秒去改成 15：那會把對 explorer 看門狗（約 16 秒）的餘裕從 8 秒壓到 3 秒。
        # 「脫困」要連續幾筆（0.5 秒一筆）才算，見 _check
        self.declare_parameter("recover_samples", 2)
        self.declare_parameter("exploration_active_topic", "exploration_active")

        self.cmd_threshold = float(self.get_parameter("cmd_threshold").value)
        self.motion_threshold = float(self.get_parameter("motion_threshold").value)
        self.stuck_time_sec = float(self.get_parameter("stuck_time_sec").value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.cancel_navigation = bool(self.get_parameter("cancel_navigation").value)
        self.defer_to_explorer = bool(self.get_parameter("defer_to_explorer").value)
        self.defer_grace_sec = float(self.get_parameter("defer_grace_sec").value)
        self.defer_max_sec = float(self.get_parameter("defer_max_sec").value)
        self.recover_samples = max(1, int(self.get_parameter("recover_samples").value))
        # map_service_cc 用 latched 發布，這裡也要 transient_local 才收得到晚起來之前的狀態
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        cb = ReentrantCallbackGroup()
        self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self._cmd_cb, 10, callback_group=cb
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._odom_cb, 10, callback_group=cb
        )
        self.create_subscription(
            Bool,
            self.get_parameter("exploration_active_topic").value,
            self._exploring_cb,
            latched,
            callback_group=cb,
        )
        self.stuck_pub = self.create_publisher(Bool, "robot_stuck", 10)
        self.create_service(Trigger, "get_stuck_status", self._status_cb, callback_group=cb)

        # ★ 2026-09-25：取消用的 client 在這裡建一次。舊版每次取消都新建 client 再 wait_for_service ——
        #   新 client 的端點可能還沒配對好，第一個請求有機會遺失，還多一段 discovery 延遲，
        #   而這正好是停車的路徑。
        self._cancel_client = self.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal", callback_group=cb
        )

        self._lock = threading.Lock()
        self._cmd_speed = 0.0
        self._cmd_time = 0.0
        self._actual_speed = 0.0
        # ★ 2026-09-06：odom 的新鮮度。0.0 代表「從來沒收到過」。
        self._odom_time = 0.0
        self._suspect_since = None
        self._last_trigger = 0.0
        self._episode_triggered = False  # 這一次卡住已經觸發過（之後只是重複提醒）
        self._defer_since = None         # 第一次讓步的時間；車子真的動了才清掉
        self._recover_streak = 0         # 連續幾筆「不像卡住」（見 recover_samples）
        self._moving_streak = 0          # 連續幾筆實測在動
        self._stuck = False
        # 沒收到過就當作「沒在探索」—— 維持舊行為（會取消），不會因為少一個話題就失去保護
        self._exploring = False
        self._explore_since = 0.0  # 收到 exploration_active=True 的時間（monotonic）

        self.create_timer(0.5, self._check, callback_group=cb)
        self.stuck_pub.publish(Bool(data=False))
        self.get_logger().info(
            f"卡住偵測啟動：指令>{self.cmd_threshold} 而實測<{self.motion_threshold} "
            f"持續 {self.stuck_time_sec}s 即判定"
        )

    # ==================================================================
    def _cmd_cb(self, msg: Twist) -> None:
        with self._lock:
            self._cmd_speed = abs(msg.linear.x)
            self._cmd_time = time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            self._actual_speed = abs(msg.twist.twist.linear.x)
            self._odom_time = time.monotonic()

    def _exploring_cb(self, msg: Bool) -> None:
        with self._lock:
            changed = self._exploring != msg.data
            self._exploring = msg.data
            if changed and msg.data:
                self._explore_since = time.monotonic()
            if changed:
                self._defer_since = None
        if changed:
            self.get_logger().info(
                "自動探索開始：卡住時不取消目標，交給 frontier explorer 的看門狗"
                if msg.data
                else "自動探索結束：恢復卡住時取消導航目標"
            )

    def _check(self) -> None:
        now = time.monotonic()
        with self._lock:
            cmd = self._cmd_speed
            cmd_age = now - self._cmd_time
            actual = self._actual_speed
            odom_age = (now - self._odom_time) if self._odom_time else None

        # ★★ 2026-09-06：收不到 odom 就**不准判定卡住** ★★
        #
        #   `_actual_speed` 初值是 0.0，而唯一的寫入者是 _odom_cb。所以只要
        #   odom 沒來，它就永遠是 0 -> actually_moving 恆為 False ->
        #   導航中指令是連續的（cmd_age 那道守衛擋不住）-> 4 秒後判定卡住
        #   -> **取消每一個導航目標**，而且症狀是「導航一直被莫名取消」。
        #
        #   這不是假想：本節點的 odom_topic 預設是 `odom_combined`，那是
        #   `wheeltec_ekf.launch.py:43` 把 robot_localization 的
        #   `/odometry/filtered` remap 出來的。**只跑 base_serial 而沒跑 EKF 時
        #   這個話題根本不存在**（底盤驅動自己發的是 `odom`）。
        #
        #   判定「卡住」必須基於**觀測到車子沒動**，不能基於**沒有觀測**。
        if odom_age is None or odom_age > 2.0:
            if odom_age is None:
                self.get_logger().warn(
                    f"⚠ 從未收到 {self.get_parameter('odom_topic').value} —— "
                    f"卡住偵測停用（沒有觀測就不能判定卡住）。"
                    f"檢查 robot_localization 的 ekf_node 有沒有起來",
                    throttle_duration_sec=10.0)
            else:
                self.get_logger().warn(
                    f"⚠ odom 已 {odom_age:.1f} 秒沒更新 —— 卡住偵測暫停",
                    throttle_duration_sec=10.0)
            self._moving_streak = 0
            self._reset()
            return

        # 指令太舊代表現在根本沒有人在下命令 (例如導航已結束)
        if cmd_age > 1.0:
            self._reset()
            return

        moving_requested = cmd > self.cmd_threshold
        actually_moving = actual > self.motion_threshold

        # ★ 2026-09-25（審查）：「脫困」要連續 recover_samples 筆才算。堵轉時輪速編碼器會抖、
        #   MPPI 輸出也會短暫掉到門檻下，單筆就重置的話，一次實體卡住會被切成好幾次 ——
        #   map_service 的卡住預算被灌大、後備計時也永遠湊不滿。
        #   但「明確停車」（指令歸零：nav2 取消、目標結束、換目標）是目標邊界，照舊立刻結束。
        self._moving_streak = self._moving_streak + 1 if actually_moving else 0
        if self._moving_streak >= self.recover_samples:
            # 車子真的動了：讓步的後備計時重新起算（指令中斷不算，見 defer_max_sec）
            self._defer_since = None
        if cmd <= 1e-3:
            self._reset()
            return
        if not moving_requested or actually_moving:
            self._recover_streak += 1
            if self._suspect_since is None or self._recover_streak >= self.recover_samples:
                self._reset()
            return
        self._recover_streak = 0

        # 要求動、但沒在動
        if self._suspect_since is None:
            self._suspect_since = now
            return

        if now - self._suspect_since < self.stuck_time_sec:
            return

        # 冷卻只管同一次卡住的重複提醒（見 cooldown_sec）
        if self._episode_triggered and now - self._last_trigger < self.cooldown_sec:
            return

        repeat = self._episode_triggered
        self._episode_triggered = True
        self._last_trigger = now
        self._stuck = True
        stuck_for = now - self._suspect_since
        self.stuck_pub.publish(Bool(data=True))
        if repeat:
            # 刻意不用「偵測到卡住」開頭：verify_5 與 map_service 只數新的一次卡住
            self.get_logger().error(f"仍然卡住（同一次，已 {stuck_for:.1f} 秒）：指令 {cmd:.3f} m/s 但實測 {actual:.3f} m/s")
        else:
            self.get_logger().error(
                f"偵測到卡住：指令 {cmd:.3f} m/s 但實測 {actual:.3f} m/s，"
                f"持續 {stuck_for:.1f} 秒。"
                "可能是低於雷達高度 (0.11 m) 的雜物卡住輪子 —— costmap 上看不到這種障礙。"
            )
        if not self.cancel_navigation:
            return
        with self._lock:
            exploring = self._exploring
            explored_for = now - self._explore_since
        if exploring and self.defer_to_explorer:
            if explored_for < self.defer_grace_sec:
                self.get_logger().warn(
                    f"自動探索才開始 {explored_for:.0f} 秒（explorer 看門狗寬限期 "
                    f"{self.defer_grace_sec:.0f} 秒內不動作）：照舊取消，先讓車停下來"
                )
            else:
                if self._defer_since is None:
                    self._defer_since = now
                deferred_for = now - self._defer_since
                if deferred_for < self.defer_max_sec:
                    # 仍然發了 /robot_stuck 並記 log（上面），只是不取消 —— 取消了 explorer
                    # 不會記失敗，只會又選回這一點（見檔頭「探索期間讓步」）
                    self.get_logger().warn(
                        "自動探索中：不取消目標，交給 frontier explorer 的無進展看門狗處理"
                        "（它取消時才會把這一點記成到不了、換下一個目標）"
                    )
                    return
                self.get_logger().warn(
                    f"讓步已 {deferred_for:.0f} 秒車子仍沒動起來（explorer 可能當掉、卡在長計算，"
                    f"或換了目標又頂上同一個障礙）：後備取消"
                )
        threading.Thread(target=self._cancel_nav, daemon=True).start()

    def _reset(self) -> None:
        self._suspect_since = None
        self._episode_triggered = False
        self._recover_streak = 0
        if self._stuck:
            self._stuck = False
            self.stuck_pub.publish(Bool(data=False))

    def _cancel_nav(self) -> None:
        """取消目前的導航目標

        不直接對 cmd_vel 發零速 —— 那會跟 collision_monitor 搶同一個話題。
        取消目標讓 Nav2 自己停下來。

        ★ 2026-09-24 更正：這裡原本寫「順帶讓 frontier explorer 收到失敗、把這個
          到不了的地方記起來」—— 錯的。explorer 不把外部取消當失敗（見檔頭），
          所以自動探索期間只有寬限期內與後備上限才會走到這裡。
        """
        try:
            if not self._cancel_client.service_is_ready() and not self._cancel_client.wait_for_service(
                timeout_sec=1.0
            ):
                self.get_logger().warn("找不到 cancel_goal 服務，無法取消導航")
                return
            self._cancel_client.call_async(CancelGoal.Request())  # 空的 goal_info = 取消全部
            self.get_logger().warn("已請求取消目前的導航目標")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"取消導航失敗: {exc}")

    def _status_cb(self, _req, res):
        res.success = True
        res.message = "stuck" if self._stuck else "ok"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = StuckDetectorCcNode()
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
