#!/usr/bin/env python3

"""導航動作節點 (_cc 重寫版)

對外介面沿用舊名稱 (/navigate)，HMI 與 smartnav_brain 不需修改。

相對舊版的修正：

  1. 定位品質檢查不再「還沒收到 amcl_pose 就直接拒絕」。
     舊版 current_covariance_norm 初始值是 inf，而 AMCL 只有在機器人移動超過
     update_min_d 之後才會發第一則 /amcl_pose，於是開機後第一個導航請求
     必定被擋下來，回「當前定位不夠準確，請嘗試全域定位」。
     現在先呼叫 AMCL 的 /request_nomotion_update 逼它算一次並發布，
     真的等不到才報錯，而且訊息會說清楚是「還沒有定位資料」還是「協方差過大」。

  2. 導航逾時從寫死的 100 秒改成參數 (預設 300 秒)。
     阿克曼車在室內要繞路又要倒車，100 秒常常只夠走到一半。

  3. 逾時判斷改看 Nav2 回報的 distance_remaining：有在推進就不算逾時，
     真正卡住 (progress_stall_timeout_sec 內都沒有縮短距離) 才取消。
"""

import math
import threading
import time
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import Trigger

from smartnav_msgs.action import FollowTaughtPath, Navigate
from smartnav_msgs.srv import DeleteTaughtPath, GetWaypoint, PlanTaughtPath

from smartnav_navigation_cc import depart_gate

# 內層導航被別人取消（平板急停、卡住偵測），外層卻沒收到取消時的結果訊息。
# ★ 「請勿自動重試」是寫給 LLM 看的：它的提示詞要求失敗先自動重試，
#   把急停回報成一般失敗，車子會在急停之後又被叫出去（2026-09-25 審查）。
EXTERNAL_STOP_MSG = "導航被外部停止（緊急停止或卡住偵測），請勿自動重試"
# path_teach 重播被「地圖上沒有的障礙」擋住而交回時，失敗訊息的開頭。
# ★ 與 path_teach_cc_node.py 的 NEW_OBSTACLE_TAG 同一個字串（改那邊要一起改這裡）
NEW_OBSTACLE_TAG = "[新障礙]"
# 呼叫 /unwedge 之前，離導航總期限至少要剩這麼多秒。
# ★ 2026-10-09（獨立審查）：/unwedge 最久約 37 秒（最多 8 段 × (0.12 m ÷ 0.10 m/s + 3 秒逾時) ＋ 每段 0.3 秒等雷射），
#   而且不可中途收手（只能靠 unwedge_abort 叫它停）。剩不到這麼多時間還去推車，只會把總時間推過
#   LLM／HMI 等結果的上限（340 秒），車子在使用者以為已經結束之後還在動。
UNWEDGE_MIN_REMAINING_S = 45.0


def wait_for_future(future, timeout_sec: float) -> Any:
    """阻塞等待 future 完成"""
    event = threading.Event()
    future.add_done_callback(lambda _f: event.set())
    if not event.wait(timeout=timeout_sec):
        raise TimeoutError("服務請求超時")
    return future.result()


class NavigationActionCcNode(Node):
    """導航動作節點"""

    def __init__(self):
        super().__init__("navigation_action_cc_node")

        self.declare_parameter("max_covariance_norm", 0.15)
        self.declare_parameter("navigation_timeout_sec", 300.0)
        # 連續這麼久都沒有縮短 distance_remaining 就視為卡死
        self.declare_parameter("progress_stall_timeout_sec", 60.0)
        self.declare_parameter("progress_epsilon_m", 0.15)
        # 開機後等第一則 /amcl_pose 的時間
        self.declare_parameter("initial_pose_wait_sec", 5.0)
        # 每次導航前是否先重新對齊位姿
        self.declare_parameter("align_before_navigate", True)
        # 導航前清一次全域成本地圖的 obstacle 層（2026-08-06 補上）。
        #
        # 為什麼要有：`path_teach_cc` 的 `_plan_cb` 從 8/05 起就會清，而這裡
        # 不會 —— 同一台車、兩個導航入口、行為不一致。8/04 量到的污染是
        # 「13 個取樣點中 11 個致命」，那種狀態下規劃器眼中的走廊是實心牆。
        #
        # 8/06 已驗證污染的根因修好了（obstacle_layer 改吃 /scan_slam，
        # 未清空即規劃成功），所以這裡是**第二道保險**而不是主要手段。
        # 靜態層有完整地圖，obstacle 層在一次任務開始時沒有值得保留的東西。
        #
        # 可關：清空要多一次服務往返，若之後量到它拖慢起步就設 false。
        self.declare_parameter("clear_costmap_before_navigate", True)

        # ★★ 2026-08-10：導航預設改走「nav2 規劃 + 純追蹤執行」★★
        #
        # 這個節點原本一律送 nav2 的 /navigate_to_pose，控制器是 MPPI。
        # 但 MPPI 在這台車的走廊裡**已證實走不通**：
        #     預測視野 = 30 x 0.1 x 0.25 = 0.75 m
        #     90 度轉彎需要 (pi/2) x 0.80 = 1.26 m      <- 只看得到所需的 60%
        # 2026-08-03 正式判定，2026-08-06 再次實測確認：
        # 直接叫 /compute_path_to_pose **規劃兩個目標都成功**
        # （起點 8 點 0.92 m、門口 30 點 4.46 m）——失敗在 MPPI 這一層。
        #
        # 同一天，教導-重現的 follow_taught_path 是 **escaping 0%、success x3**。
        # 也就是說系統裡同時存在「已證實過得去」與「已證實過不去」兩條路，
        # 而 HMI 的導航按鈕接的是後者。這不是還沒調好，是接錯線。
        #
        # 改法就是 PlanTaughtPath.srv 自己寫在檔頭的那句話：
        #   「規劃器沒有問題，出問題的是 MPPI 控制器，
        #     所以『用 nav2 規劃、用純追蹤執行』是最省事也最可靠的組合。」
        #
        # 流程：/plan_taught_path（內部逐段呼叫 nav2 規劃器）-> path_id
        #       -> /follow_taught_path（純追蹤執行，含貼牆排斥與脫困）
        # 任何一步不成就**自動退回 nav2/MPPI**，不會比原本更糟。
        self.declare_parameter("prefer_taught_path", True)
        self.declare_parameter("taught_plan_timeout_sec", 30.0)
        # 成功後把這條臨時路徑刪掉，否則每按一次導航就多一條，paths.json 會長爆。
        # ★ 失敗時**故意保留**——那正是要拿去查為什麼跑不完的東西。
        self.declare_parameter("delete_temp_taught_path", True)

        # ★★ 2026-08-14：失敗自動重新規劃。這是「能自己跑」與「跑不完就放棄」的分界 ★★
        #
        # 重播失敗的原因大多是**暫時性**的：有人走過去把車擋停超過 wait_timeout、
        # 成本地圖殘留一格障礙、或這次剛好規劃出多個折返點（8/10 實測
        # 「0 折返 6 勝／多折返 5 敗」）。這三種「從現在的位置重新規劃一次」
        # 都會得到明顯不同的結果 —— 而舊版連試都不試。
        #
        # 3 次是折衷：多數暫時性障礙一次就過，而真正的死結試三次也不會通，
        # 再多只是把電量與時間燒在同一個地方。
        self.declare_parameter("max_replan_attempts", 3)
        # 兩次嘗試之間車子至少要移動這麼多，才算「這次重試有意義」。
        # 低於它連續兩輪就判定卡死並放棄 —— 沒有這道檢查，車子會在窄轉角
        # 原地磨到總逾時（300 秒）才停，現場看起來就像當機。
        #
        # ★ 0.20 m 是從**車輛物理**推出來的，不是隨手取的：
        #   最小轉彎半徑 0.80 m，一段有意義的脫困轉向至少轉 20 度
        #   -> 弧長 0.80 × (20°×π/180) = **0.28 m**。
        #   門檻取在它下面，才不會把「有在動、只是慢」誤判成卡死。
        #   （若日後 min_turning_radius 改了，這個值要跟著重算。）
        #
        # ★ 重試之間的空檔（清成本地圖 + 重新規劃，最多約 35 秒）**沒有人在發
        #   cmd_vel**。這是安全的：廠商韌體的指令逾時剛好 1.0 秒
        #   （`command_lost_count > RATE_100_HZ`，見 韌體限制_不可違反.md），
        #   超時底盤會自己停住。車子在空檔期間是靜止的，不會滑行。
        self.declare_parameter("replan_min_progress_m", 0.20)
        # ★★ 2026-10-20：重新規劃不出路徑時，還有重試次數就等一下再試，不要第一次就放棄 ★★
        #
        # 10/20 14:53:58（nav_20261020_144647.log）：第 1 次重播止損 -> 重新規劃第 2/3 次回
        # 「Start occupied」-> /unwedge 推不開 -> 直接「導航失敗：重新規劃不出路徑」，第 3 次根本沒用到。
        # 起點被占用常是暫時的（跟車的人站在車旁、剛走開的人還在障礙層上），而 /unwedge 一次只推
        # 一小段 —— 下一輪再清一次成本地圖、再拉開一次、再規劃一次，才有機會出來。
        # 每輪照樣用掉一次重試、受總期限與取消約束。0 = 回到舊行為（第一次重新規劃失敗就放棄）。
        self.declare_parameter("replan_fail_retry_sec", 4.0)
        # ★★ 2026-10-01：被「地圖上沒有的障礙」擋住（人、剛放的東西）—— 繞過去，或等通道淨空 ★★
        #
        # path_teach 判斷擋路的是新東西時不脫困、停車等 new_obstacle_wait_sec，
        # 還在就用「[新障礙]」開頭的訊息交回來。這裡接手：
        #   1. 重新規劃（path_teach 會先清殘影、再等障礙層把**現在**看得到的東西標回去）
        #      —— 旁邊有空間，規劃器就會給一條繞過去的路
        #   2. 規劃不出來 = 通道整個被擋住 -> 原地等，每 blocked_poll_sec 重新規劃一次，
        #      一通就走；同一處最多等 blocked_patience_sec
        # 這種失敗**不算**重試次數、也不算「原地失敗」—— 人走開之前車子本來就不該動。
        # 模擬 DYN_BASE（改之前）：人站在路中 45 秒 -> 3 次重試用完、車子停在家具旁，
        # 下一個目標 34 秒就失敗。
        self.declare_parameter("blocked_patience_sec", 180.0)
        self.declare_parameter("blocked_poll_sec", 4.0)
        self.declare_parameter("blocked_speech_wait", "前面的路被擋住了，我在這裡等，通了就繼續")
        self.declare_parameter("blocked_speech_resume", "路通了，繼續前進")
        # ★ 2026-10-27：出發前確認第一段行進方向淨空（客人講完話常站在車前；路徑三成先倒車）。見 depart_gate.py。
        #   depart_gate_enable:=false＝舊行為（規劃完直接起跑）。
        self.declare_parameter("depart_gate_enable", True)
        self.declare_parameter("depart_clear_m", 0.90)       # 雷射到行進方向最近的東西至少這麼遠才出發
        self.declare_parameter("depart_sector_deg", 25.0)    # 行進方向左右各幾度（0.45 m 寬的人在 0.9 m 約 ±14°）
        self.declare_parameter("depart_wait_max_sec", 20.0)  # 最多等多久，之後照常出發

        self.max_covariance_norm = float(self.get_parameter("max_covariance_norm").value)
        self.navigation_timeout_sec = float(self.get_parameter("navigation_timeout_sec").value)
        self.progress_stall_timeout_sec = float(self.get_parameter("progress_stall_timeout_sec").value)
        self.progress_epsilon_m = float(self.get_parameter("progress_epsilon_m").value)
        self.initial_pose_wait_sec = float(self.get_parameter("initial_pose_wait_sec").value)
        self.align_before_navigate = bool(self.get_parameter("align_before_navigate").value)
        self.clear_costmap_before_navigate = bool(
            self.get_parameter("clear_costmap_before_navigate").value
        )
        self.prefer_taught_path = bool(self.get_parameter("prefer_taught_path").value)
        self.taught_plan_timeout = float(self.get_parameter("taught_plan_timeout_sec").value)
        self.delete_temp_taught = bool(self.get_parameter("delete_temp_taught_path").value)
        self.max_replan_attempts = max(1, int(self.get_parameter("max_replan_attempts").value))
        self.replan_min_progress_m = float(self.get_parameter("replan_min_progress_m").value)
        self.replan_fail_retry_sec = max(0.0, float(self.get_parameter("replan_fail_retry_sec").value))
        self.blocked_patience_sec = float(self.get_parameter("blocked_patience_sec").value)
        self.blocked_poll_sec = max(1.0, float(self.get_parameter("blocked_poll_sec").value))
        self.blocked_speech_wait = str(self.get_parameter("blocked_speech_wait").value)
        self.blocked_speech_resume = str(self.get_parameter("blocked_speech_resume").value)
        self._taught_seq = 0
        # 重試迴圈用來判斷「上一輪有沒有真的移動」。由 /amcl_pose 更新。
        self._pose_lock = threading.Lock()
        self._last_xy = None

        self.server_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = ReentrantCallbackGroup()

        self.navigate_action = ActionServer(
            self,
            Navigate,
            "navigate",
            execute_callback=self._navigate_callback,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self.server_cb_group,
        )

        self.nav2_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.client_cb_group
        )
        self.get_waypoint_client = self.create_client(
            GetWaypoint, "get_waypoint", callback_group=self.client_cb_group
        )
        # AMCL 的「不移動也強制更新一次」服務，用來逼它發布第一則 /amcl_pose
        self.nomotion_update_client = self.create_client(
            EmptySrv, "/request_nomotion_update", callback_group=self.client_cb_group
        )
        # 導航前請 map_service_cc 用雷射重新對齊位姿 (IMU 零偏會讓靜置中的車子
        # yaw 一直漂，帶著錯的角度起步會直接把目標算到牆裡)
        self.align_pose_client = self.create_client(
            Trigger, "/align_pose", callback_group=self.client_cb_group
        )
        # ★ 2026-08-17：規劃器回 "Start occupied" 時的唯一自救管道，
        #   見 _call_unwedge 與 path_teach_cc._unwedge_cb 的說明。
        self.unwedge_client = self.create_client(
            Trigger, "/unwedge", callback_group=self.client_cb_group
        )
        # ★ 2026-10-09（獨立審查）：/unwedge 是 path_teach 的同步服務，旁路直發 cmd_vel、
        #   自己沒有任何取消檢查（最久約 40 秒）。急停或取消 action 碰不到它，
        #   所以等服務結果的期間偵測到取消，就對 unwedge_abort 發一則 Empty
        #   （話題合約：Empty、相對名稱、depth 10；訂閱者是 path_teach_cc，HMI 急停與
        #   map_service 取消時也發同一個話題）。
        self.unwedge_abort_pub = self.create_publisher(Empty, "unwedge_abort", 10)
        self._unwedge_goal = None        # 目前這次 /unwedge 呼叫所屬的 goal_handle（偵測取消用）
        self._announce_start = True      # 這一次起跑要不要播「開始導航到 X」（見 _try_taught_path）
        self._last_plan_fail_msg = ""    # 最近一次規劃失敗的原因（_wait_for_passage 用來認「起點被占用」）
        # 導航前清全域成本地圖的 obstacle 層（見參數宣告處的說明）
        self.clear_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
            callback_group=self.client_cb_group,
        )
        # 「nav2 規劃 + 純追蹤執行」用的兩個入口（見 prefer_taught_path 的說明）
        self.plan_taught_client = self.create_client(
            PlanTaughtPath, "plan_taught_path", callback_group=self.client_cb_group
        )
        self.delete_taught_client = self.create_client(
            DeleteTaughtPath, "delete_taught_path", callback_group=self.client_cb_group
        )
        self.follow_taught_client = ActionClient(
            self, FollowTaughtPath, "follow_taught_path", callback_group=self.client_cb_group
        )

        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "amcl_pose",
            self._amcl_pose_callback,
            10,
            callback_group=self.client_cb_group,
        )
        self.speech_text_pub = self.create_publisher(String, "speech_text", 10)
        # 導航進行中的旗標。camera_manager_cc 訂閱它來決定相機要不要待機 ——
        # 相機相關行程實測吃掉約 110% CPU，導航期間關掉才跑得動。
        # latched：訂閱者晚啟動也能立刻知道目前狀態。
        # ★ 2026-08-27 晚間：話題名改成**絕對名** "/navigation_active"。
        #   原本是相對名，而兩個訂閱端（face_embedding_node.py、
        #   camera_manager_cc_node.py）都寫死絕對名。目前沒有人用 namespace，
        #   所以相對名剛好解析成同一個而連得上 —— 但**只要有人加了 namespace，
        #   兩個訂閱端會同時靜默斷線**：人臉全速跑、相機管理失效，兩個都不報錯。
        #   統一成絕對名就沒有這個地雷。
        self.nav_active_pub = self.create_publisher(
            Bool, "/navigation_active",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )
        self.nav_active_pub.publish(Bool(data=False))

        self.pose_received = False
        self.current_covariance_norm = float("inf")

        # 進度追蹤 (在 __init__ 就建好，feedback 回呼可能比動作主體先跑到)
        self._feedback_lock = threading.Lock()
        self._last_distance = float("inf")
        self._last_progress_time = time.monotonic()

        self.get_logger().info("導航動作節點 (_cc) 已啟動")

    # ==================================================================
    # 定位品質
    # ==================================================================
    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        cov_xx = msg.pose.covariance[0]
        cov_yy = msg.pose.covariance[7]
        self.current_covariance_norm = math.sqrt(cov_xx**2 + cov_yy**2)
        self.pose_received = True
        # 給重試迴圈判斷「這一輪有沒有真的移動」用（見 _try_taught_path）
        with self._pose_lock:
            self._last_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _check_localization(self):
        """回傳 (ok, message)"""
        if not self.pose_received:
            # AMCL 要移動超過 update_min_d 才會主動發 /amcl_pose，
            # 剛開機或剛切完地圖時一定還沒有。先請它就地更新一次。
            if self.nomotion_update_client.wait_for_service(timeout_sec=2.0):
                try:
                    wait_for_future(self.nomotion_update_client.call_async(EmptySrv.Request()), 5.0)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"request_nomotion_update 失敗: {exc}")
            else:
                return False, "定位系統尚未就緒 (AMCL 不在線上)，請確認已載入地圖"

            deadline = time.monotonic() + self.initial_pose_wait_sec
            while not self.pose_received and time.monotonic() < deadline:
                time.sleep(0.1)

        if not self.pose_received:
            return False, "尚未取得定位資料，請先執行全域定位"

        if self.current_covariance_norm > self.max_covariance_norm:
            return False, "當前定位不夠準確，請嘗試全域定位"

        return True, ""

    # ==================================================================
    # 導航動作
    # ==================================================================
    def _navigate_callback(self, goal_handle):
        request = goal_handle.request
        result = Navigate.Result()

        self.get_logger().info("收到導航請求")

        try:
            if not request.use_target_pose and not request.waypoint_id:
                return self._abort(goal_handle, result, "必須提供地點ID或目標座標以進行導航")

            ok, message = self._check_localization()
            if not ok:
                return self._abort(goal_handle, result, message)

            # 起步前把位姿對正。走廊裡角度本來就難觀測，加上 IMU 零偏，
            # 靜置越久偏得越多；不先對齊就可能照著錯了十幾度的方向規劃。
            if self.align_before_navigate and self.align_pose_client.service_is_ready():
                try:
                    res = wait_for_future(
                        self.align_pose_client.call_async(Trigger.Request()), timeout_sec=20.0
                    )
                    self.get_logger().info(f"導航前位姿對齊: {res.message}")
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"導航前位姿對齊失敗 (繼續導航): {exc}")

            # 清一次全域成本地圖的 obstacle 層（見參數宣告處）。
            # 失敗不擋下導航 —— 這是保險不是前提，讓 nav2 自己回報真正的問題。
            if self.clear_costmap_before_navigate and self.clear_costmap_client.service_is_ready():
                try:
                    wait_for_future(
                        self.clear_costmap_client.call_async(ClearEntireCostmap.Request()),
                        timeout_sec=5.0,
                    )
                    self.get_logger().info("已清空全域成本地圖的 obstacle 層")
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"清空成本地圖失敗 (繼續導航): {exc}")

            if request.use_target_pose:
                # HMI 直接點地圖導航：座標已經是 map frame，不必查地點資料庫
                target_pose = request.target_pose
                waypoint_name = request.target_name or "指定位置"
            else:
                req = GetWaypoint.Request()
                req.waypoint_id = request.waypoint_id
                try:
                    response = wait_for_future(self.get_waypoint_client.call_async(req), timeout_sec=5.0)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(f"查詢地點座標失敗: {exc}")
                    return self._abort(goal_handle, result, "無法查詢地點座標，導航失敗")

                if not response.success:
                    return self._abort(goal_handle, result, f"查詢地點失敗: {response.message}")

                target_pose = response.waypoint_info.pose
                waypoint_name = response.waypoint_info.waypoint_name

            # 全零四元數是無效旋轉，會讓 Nav2 的朝向計算爆掉
            q = target_pose.orientation
            if abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) < 1e-6:
                target_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            # ★ 先試「nav2 規劃 + 純追蹤執行」，不成才退回 nav2/MPPI（見參數說明）
            if self.prefer_taught_path:
                taught = self._try_taught_path(goal_handle, target_pose, waypoint_name, result)
                if taught is not None:
                    return taught
                self.get_logger().warn("教導-重現路線不可用，退回 nav2/MPPI")

            if not self.nav2_client.wait_for_server(timeout_sec=10.0):
                return self._abort(goal_handle, result, "Nav2 導航伺服器不在線上，導航失敗")

            goal = NavigateToPose.Goal()
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.header.frame_id = "map"
            goal.pose.pose = target_pose

            with self._feedback_lock:
                self._last_distance = float("inf")
                self._last_progress_time = time.monotonic()

            try:
                nav2_goal_handle = wait_for_future(
                    self.nav2_client.send_goal_async(goal, feedback_callback=self._nav2_feedback),
                    timeout_sec=10.0,
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"發送導航請求失敗: {exc}")
                return self._abort(goal_handle, result, "無法發送導航請求，導航失敗")

            if not nav2_goal_handle.accepted:
                return self._abort(goal_handle, result, "導航請求被拒絕，導航失敗")

            self.speech_text_pub.publish(String(data=f"開始導航到 {waypoint_name}"))
            self.get_logger().info(f"開始導航到 {waypoint_name}")
            self.nav_active_pub.publish(Bool(data=True))

            try:
                outcome = self._monitor_navigation(goal_handle, nav2_goal_handle)
            finally:
                # 不論成功、失敗或取消都要放開，否則相機再也不會恢復
                self.nav_active_pub.publish(Bool(data=False))

            if outcome == "cancel":
                result.success = False
                result.message = "導航請求已被系統或使用者取消"
                goal_handle.canceled()
                self.get_logger().info(result.message)
                return result

            if outcome == "timeout":
                return self._abort(goal_handle, result, "導航逾時，已自動取消")

            if outcome == "stalled":
                return self._abort(goal_handle, result, "導航停滯無法前進，已自動取消")

            if outcome == "external_cancel":
                # 外層沒收到取消，狀態還是 EXECUTING —— 只能 abort，不能 canceled()
                return self._abort(goal_handle, result, EXTERNAL_STOP_MSG)

            if outcome != "succeeded":
                return self._abort(goal_handle, result, "導航過程出現異常，導航失敗")

            result.success = True
            result.message = "導航成功"
            goal_handle.succeed()
            self.get_logger().info("✓ 導航成功")
            return result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"導航發生例外: {exc}")
            self.nav_active_pub.publish(Bool(data=False))
            return self._abort(goal_handle, result, "系統出現異常，導航失敗")

    def _try_taught_path(self, goal_handle, target_pose, waypoint_name, result):
        """用「nav2 規劃 + 純追蹤執行」導航，**失敗會自動重新規劃再試**。

        回傳 `Navigate.Result` 代表這條路已經處理完（成功或失敗都算數）；
        回傳 `None` 代表**第一次就根本沒能起跑**，呼叫端應該退回 nav2/MPPI。

        ★ 這個區分很重要：規劃不出路徑（服務沒開、目標不可達）要讓 MPPI 有機會試；
          但**路徑跑到一半失敗**不該再讓 MPPI 跑一次——同一段路它只會更糟，
          而且車子已經不在起點了，再送一次會從奇怪的位置重新規劃。

        ## ★★ 2026-08-14：加入自動重新規劃（這是「能自己跑」的關鍵）★★

        在這之前，只要重播失敗一次就直接 abort，整個導航任務結束。
        但實際跑失敗的原因**大多是暫時性的**：

          - 有人走過去，`obstacle_stop` 讓車停下等待，等超過 `wait_timeout` 就失敗
          - 成本地圖上有一格殘留障礙（清一次就沒了）
          - 這條路徑剛好排到多個折返點（8/10 實測「0 折返 6 勝／多折返 5 敗」），
            **而重新規劃很可能得到不同的折返數** —— 起點變了，解就變了

        以上三種，「從現在的位置重新規劃一條」都會有明顯不同的結果，
        而原本的行為是連試都不試。這正是使用者說的「要能自行運作」缺的那一塊。

        重試迴圈做四件事，缺一不可：
          1. 清一次全域成本地圖的 obstacle 層（把暫時性障礙掃掉）
          2. **從車子當下的位置**重新規劃（不是重跑同一條路徑）
          3. 檢查兩次嘗試之間車子有沒有真的移動 —— 沒動就是卡在同一個地方，
             再規劃幾次也一樣，直接放棄（否則會在原地磨到逾時）
          4. 全程受總逾時與取消旗標約束
        """
        if not self.plan_taught_client.service_is_ready():
            self.get_logger().info("plan_taught_path 服務未就緒")
            return None
        if not self.follow_taught_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().info("follow_taught_path 動作伺服器未就緒")
            return None

        deadline = time.monotonic() + self.navigation_timeout_sec
        # ★ 2026-10-20：每次導航重讀（ros2 param set 不必重啟就能 A/B）
        # ★ 2026-10-21：讀不到參數（例如沒有參數伺服器的替身測試）就沿用現值，沒有現值＝0＝舊行為；
        #   讀參數失敗不該讓整趟導航丟例外。
        try:
            self.replan_fail_retry_sec = max(
                0.0, float(self.get_parameter("replan_fail_retry_sec").value))
        except Exception:  # noqa: BLE001
            self.replan_fail_retry_sec = float(getattr(self, "replan_fail_retry_sec", 0.0))
        last_why = ""
        stuck_rounds = 0
        prev_xy = self._robot_xy()
        attempt = 0              # 一般失敗用掉的重試次數（被新障礙擋住的不算）
        runs = 0                 # 實際起跑過幾次
        last_dyn = False         # 上一次失敗是不是「[新障礙]」
        blocked_since = None     # 這一處被新障礙擋住的起始時間（等待耐心從這裡起算）
        blocked_xy = None

        while True:
            if goal_handle.is_cancel_requested:
                return self._canceled(goal_handle, result)
            if time.monotonic() > deadline:
                return self._abort(goal_handle, result, "導航逾時，已自動取消")
            if not last_dyn:
                attempt += 1
                if attempt > self.max_replan_attempts:
                    return self._abort(
                        goal_handle, result,
                        f"導航失敗：重試 {self.max_replan_attempts} 次仍未抵達（最後一次：{last_why}）")

            if runs > 0:
                # 重試前先清障礙層：暫時性障礙（人走過去）是最常見的失敗原因，
                # 不清的話重新規劃看到的還是同一張被污染的地圖，等於白試。
                # ★ 2026-10-01：path_teach 的規劃服務清完會等障礙層把「現在」看得到的
                #   東西標回去（plan_clear_settle_sec），站在路上的人不會被清成透明。
                self._clear_costmap_quiet()
                if last_dyn:
                    self.get_logger().warn(f"被新障礙擋住，重新規劃看能不能繞過去（{last_why}）")
                else:
                    self.get_logger().warn(
                        f"重新規劃第 {attempt}/{self.max_replan_attempts} 次"
                        f"（上一次失敗：{last_why}）")

            plan = self._plan_taught_once(waypoint_name, target_pose)
            if plan is None and last_dyn:
                # 繞不過去 = 通道整個被擋住：原地等它淨空（不推車、不脫困）
                plan, why = self._wait_for_passage(
                    goal_handle, waypoint_name, target_pose, deadline, blocked_since)
                if plan is None:
                    if why == "cancel":
                        return self._canceled(goal_handle, result)
                    if why == "timeout":
                        return self._abort(goal_handle, result, "導航逾時，已自動取消")
                    return self._abort(
                        goal_handle, result,
                        f"導航失敗：通道被擋住超過 {self.blocked_patience_sec:.0f} 秒仍未淨空"
                        f"（{last_why}）")
                # ★ 2026-10-09（獨立審查 [8]）：通道淨空 = 這一處的擋路結束，等待計時歸零。
                #   原本只在「換了地方（離上次被擋處 > 0.5 m）」才重算，通道通了之後車子只前進一小段、
                #   在同處附近又被擋，會直接繼承先前已等的時間（例如已等 175 秒、這次只擋了 8 秒就判
                #   「同一處被擋住超過 180 秒」放棄）。總逾時（navigation_timeout_sec）仍然有效，不會無限等。
                blocked_since = None
                blocked_xy = None
            if plan is None:
                # ★ 2026-10-09（審查 [6]）：規劃服務最長會阻塞 30 秒（含清成本地圖後的 2 秒等待），
                #   這段期間按急停，回來之後要先看取消再決定要不要推車、要不要退回 nav2/MPPI。
                if goal_handle.is_cancel_requested:
                    return self._canceled(goal_handle, result)
                # ★★ 2026-08-17：規劃不出來時先試著把車子推開再規劃一次 ★★
                #
                # 8/17 實測到的死結：車子跑完一趟停在牆邊，車體壓在成本地圖的
                # 內切格上，規劃器對**每一個目標**都回 "Start occupied"。
                # 這時候「重新規劃」再多次也沒用 —— 起點本身就是不合法的，
                # 而 nav2 內建的 backup / drive_on_heading 一樣要做碰撞檢查，
                # 實測也跟著失敗（`backup failed`）。當天使用者手動救了三次車。
                #
                # `/unwedge` 繞過規劃器直接打舵推開，是唯一有效的一條路。
                # ★ 只試一次：推不開就是真的推不開，多試只是拖時間又耗電。
                # ★ 2026-10-20：「一次」是指這一輪重試裡一次；下面加了「還有次數就再試一輪」，
                #   /unwedge 可能被叫不只一次，所以推車前也守總期限（同「車身外緣持續不足」那條的守門）。
                self._unwedge_goal = goal_handle
                if deadline - time.monotonic() < UNWEDGE_MIN_REMAINING_S:
                    self.get_logger().info(
                        f"規劃不出路徑，但離總期限不到 {UNWEDGE_MIN_REMAINING_S:.0f} 秒，不請 /unwedge 推車")
                elif self._call_unwedge():
                    # 脫困（最久約 37 秒）期間才按急停：不再規劃、不再起跑
                    if goal_handle.is_cancel_requested:
                        return self._canceled(goal_handle, result)
                    plan = self._plan_taught_once(waypoint_name, target_pose)
                elif goal_handle.is_cancel_requested:
                    return self._canceled(goal_handle, result)
            if plan is None:
                if runs == 0:
                    # 一次都沒跑過 -> 車子還在原地，讓 MPPI 有機會試
                    return None
                # ★ 2026-10-20：還有重試次數 -> 等一下、從當下位置再規劃一輪（見 replan_fail_retry_sec）。
                #   回到迴圈開頭會 attempt += 1、清成本地圖，次數用完就由那裡的出口結束。
                if (self.replan_fail_retry_sec > 0.0
                        and attempt < self.max_replan_attempts
                        and time.monotonic() + self.replan_fail_retry_sec < deadline):
                    self.get_logger().warn(
                        f"重新規劃不出路徑（第 {attempt}/{self.max_replan_attempts} 次）—— "
                        f"{self.replan_fail_retry_sec:.0f} 秒後從當下位置再試一輪"
                        f"（前次失敗：{last_why}）")
                    t_end = time.monotonic() + self.replan_fail_retry_sec
                    while time.monotonic() < t_end:
                        if goal_handle.is_cancel_requested:
                            return self._canceled(goal_handle, result)
                        time.sleep(0.2)
                    continue
                return self._abort(
                    goal_handle, result,
                    f"導航失敗：重新規劃不出路徑（已試 {attempt}/{self.max_replan_attempts} 次；"
                    f"前次失敗：{last_why}）")

            runs += 1
            # ★ 2026-10-09（審查 [9]）：「開始導航到 X」只在第一次起跑與一般重試時播報。
            #   被新障礙擋住後重新起跑，一輪只有 10~12 秒，耐心（180 秒）內可以重複十幾次，
            #   平板每輪都重複播同一句。用屬性傳遞而不是參數：_follow_taught_once 的簽名不變。
            #   （/navigation_active 的 True／False 發布時機刻意不動：整個迴圈維持 True 的話，
            #   任何一個漏掉的 return 出口都會讓相機再也不恢復。）
            self._announce_start = (runs == 1 or not last_dyn)
            outcome, res, path_id = self._follow_taught_once(
                goal_handle, plan, waypoint_name, deadline)

            if outcome == "cancel":
                return self._canceled(goal_handle, result)
            if outcome == "external_cancel":
                return self._abort(goal_handle, result, EXTERNAL_STOP_MSG)
            if outcome == "timeout":
                return self._abort(goal_handle, result, "導航逾時，已自動取消")
            if outcome == "reject":
                if runs == 1:
                    return None
                return self._abort(goal_handle, result, "重播請求被拒絕")

            if outcome == "success":
                self._delete_taught(path_id)
                result.success = True
                extra = f"（重新規劃 {runs - 1} 次後）" if runs > 1 else ""
                result.message = (f"導航成功{extra}（終點誤差 {res.result.final_error_m:.2f} m、"
                                  f"朝向差 {res.result.final_yaw_error_deg:.1f} 度）")
                goal_handle.succeed()
                self.get_logger().info(f"✓ {result.message}")
                return result

            # ── 失敗 ──
            last_why = getattr(res.result, "message", "未知") if res is not None else "無回應"
            last_dyn = last_why.startswith(NEW_OBSTACLE_TAG)
            # ★ 2026-10-09（審查 [10]）：「[新障礙]」不算重試，擋路期間一輪約 10~12 秒，
            #   180~300 秒內可以有十幾二十輪；每輪都留一條 _auto_* 臨時路徑，路徑資料庫
            #   （每次都整份重寫的 JSON）就隨輪數線性長大。成功的路徑本來就刪；一般失敗最多 3 條，
            #   照舊保留供查驗；[新障礙] 失敗只在「還要繼續等／繞」時刪（見下方），
            #   **最後放棄的那一次留著**。
            if last_dyn:
                self.get_logger().error(f"教導-重現第 {runs} 次失敗：{last_why}")
            else:
                self.get_logger().error(
                    f"教導-重現第 {runs} 次失敗：{last_why}"
                    f"　★ 臨時路徑 {path_id} 已保留供查驗")
            # ★ 2026-09-25：失敗之後、決定重試或放棄之前先看取消。舊版只在下一輪開頭檢查，
            #   最後一輪與「原地失敗兩次」這兩個出口會漏掉，把使用者的取消回報成失敗
            if goal_handle.is_cancel_requested:
                return self._canceled(goal_handle, result)

            now_xy = self._robot_xy()
            if last_dyn:
                # 被新障礙擋住：不算重試、不算原地失敗。換了地方（離上次被擋處 > 0.5 m）
                # 就是新的一處擋路，等待耐心重新起算。
                d_blk = self._xy_dist(blocked_xy, now_xy)
                if blocked_since is None or (d_blk is not None and d_blk > 0.5):
                    blocked_since = time.monotonic()
                    blocked_xy = now_xy
                elif time.monotonic() - blocked_since > self.blocked_patience_sec:
                    return self._abort(
                        goal_handle, result,
                        f"導航失敗：同一處被擋住超過 {self.blocked_patience_sec:.0f} 秒（{last_why}）")
                self._delete_taught(path_id)
                prev_xy = now_xy
                continue

            # ★ 2026-10-01：車身外緣被硬停而失敗 = 車子貼著東西。直接重新規劃的話，
            #   規劃器常常給一條同方向的路（它覺得過得去），車子再撞一次 6 cm 硬停。
            #   模擬 DYN_BASE：停在桌角下 4 cm，之後兩個目標各失敗一次都卡在同一處。
            #   先請 /unwedge 拉開（車頭／車尾貼著就直線拉開，側面貼著就打舵推開）。
            if "車身外緣持續不足" in last_why:
                # ★ 2026-10-09（審查 [6]）：推車之前的三道守門 ——
                #   (1) 還有下一次重試可用（已是最後一次就推了也用不上，下一圈開頭就放棄）
                #   (2) 離總期限還有 UNWEDGE_MIN_REMAINING_S 秒（/unwedge 最久約 37 秒）
                #   (3) 沒有被取消（它繞過防撞、只能靠 unwedge_abort 中止，取消了就別再推）
                if (attempt < self.max_replan_attempts
                        and time.monotonic() < deadline - UNWEDGE_MIN_REMAINING_S
                        and not goal_handle.is_cancel_requested):
                    self._unwedge_goal = goal_handle
                    self._call_unwedge()
                    # 脫困期間才按急停：回報取消，不要把它算成「原地失敗」
                    if goal_handle.is_cancel_requested:
                        return self._canceled(goal_handle, result)
                    now_xy = self._robot_xy()
                else:
                    self.get_logger().info(
                        "車身外緣持續不足，但不請 /unwedge 推車（已是最後一次重試、離總期限不到 "
                        f"{UNWEDGE_MIN_REMAINING_S:.0f} 秒、或已被取消）")

            # 「有沒有移動」決定值不值得再試。卡在同一個地方時，重新規劃
            # 拿到的會是幾乎一樣的路徑與一樣的結果 —— 那是浪費電量與時間。
            moved = self._xy_dist(prev_xy, now_xy)
            prev_xy = now_xy
            if moved is not None and moved < self.replan_min_progress_m:
                stuck_rounds += 1
                self.get_logger().warn(
                    f"這一輪只移動了 {moved:.2f} m（門檻 {self.replan_min_progress_m:.2f} m）"
                    f"，連續第 {stuck_rounds} 次原地失敗")
                if stuck_rounds >= 2:
                    return self._abort(
                        goal_handle, result,
                        f"導航失敗：車子卡在原地，重新規劃無效（{last_why}）")
            else:
                stuck_rounds = 0
            # 重試次數用完的出口在迴圈開頭（attempt > max_replan_attempts）

    def _depart_gate(self, goal_handle, path_id):
        """出發前確認第一段行進方向淨空（見 depart_gate.py）。回 False＝等的時候被取消。

        只在這段期間訂閱 /scan（放在可重入的 client_cb_group，動作執行緒在這裡等的時候它照樣收得到）。
        """
        if not bool(self.get_parameter("depart_gate_enable").value):
            return True
        holder = {"scan": None}
        sub = self.create_subscription(LaserScan, "/scan", lambda m: holder.__setitem__("scan", m),
                                       qos_profile_sensor_data, callback_group=self.client_cb_group)
        try:
            return depart_gate.wait_until_clear(
                lambda: holder["scan"], lambda: goal_handle.is_cancel_requested,
                lambda txt: self.speech_text_pub.publish(String(data=txt)),
                lambda txt: self.get_logger().info(txt),
                direction=depart_gate.first_direction(path_id),
                clear_m=float(self.get_parameter("depart_clear_m").value),
                sector_deg=float(self.get_parameter("depart_sector_deg").value),
                wait_max_sec=float(self.get_parameter("depart_wait_max_sec").value))
        finally:
            self.destroy_subscription(sub)

    def _wait_for_passage(self, goal_handle, waypoint_name, target_pose, deadline, blocked_since):
        """通道被地圖上沒有的障礙整個擋住（繞也繞不過）—— 原地等，定期重新規劃，一通就走。

        回傳 (plan, 原因)：plan 不是 None = 路通了；否則原因是 cancel / timeout / patience。

        ★ 等的時候車子是靜止的：沒有人發 cmd_vel，底盤的指令逾時 1.0 秒會自己停住。
        ★ 每次重新規劃，path_teach 都會先清殘影、再等障礙層把「現在」看得到的東西
          標回去（plan_clear_settle_sec）—— 擋路的東西一移走，下一輪就規劃得出來；
          還在的話規劃器照樣看得到它，不會給出一條穿過它的路。
        ★ 這就是「隨時可能恢復通道」的判斷：不是等固定秒數，而是每一輪都重新問一次地圖。
        """
        start = blocked_since if blocked_since is not None else time.monotonic()
        self.speech_text_pub.publish(String(data=self.blocked_speech_wait))
        self.get_logger().warn(
            f"通道被擋住、也繞不過去 —— 原地等它淨空（每 {self.blocked_poll_sec:.0f} 秒"
            f"重新規劃一次，同一處最多等 {self.blocked_patience_sec:.0f} 秒）")
        polls = 0
        fails_in_row = 0
        tried_unwedge = False        # ★ 每次進入等通道最多脫困一次（區域旗標：下一次被擋可以再試一次）
        noted_occupied = False
        while True:
            next_t = time.monotonic() + self.blocked_poll_sec
            while time.monotonic() < next_t:
                if goal_handle.is_cancel_requested:
                    return None, "cancel"
                time.sleep(0.2)
            now = time.monotonic()
            if now > deadline:
                return None, "timeout"
            if now - start > self.blocked_patience_sec:
                return None, "patience"
            polls += 1
            plan = self._plan_taught_once(waypoint_name, target_pose, quiet=True)
            if plan is not None:
                return self._passage_cleared(goal_handle, plan, start, polls)
            fails_in_row += 1
            # ★ 2026-10-09（審查 [7]）：車子停在障礙前 10~15 cm（collision_monitor 把車煞在那裡）時，
            #   規劃器多半回「Start occupied」—— 起點落在成本地圖的膨脹區裡，規劃器不肯出手。
            #   這時再等多久都一樣（模擬 E4 的 log 就是這樣），而障礙旁邊其實有繞行空間。
            #   所以連續兩輪（約 8 秒；先給擋路的人走開的時間窗）規劃失敗，請 /unwedge 往空的那頭
            #   拉開一次、拉開了立刻再規劃。/unwedge 自己有保護：四周有裕度就不動、只走直線、
            #   行進方向要有 0.47 m 淨空、6 cm 車身外緣硬停；而且可以被 unwedge_abort 中止。
            msg_l = (getattr(self, "_last_plan_fail_msg", "") or "").lower()
            if (not noted_occupied) and "occupied" in msg_l:
                noted_occupied = True
                self.get_logger().warn(
                    "規劃回「起點被占用」：車子停在障礙前面（防撞煞住的位置），起點落在成本地圖的膨脹區裡，"
                    "規劃器不肯出手 —— 不是通道真的被堵死。等人走開的時間窗過後，會先請 /unwedge 拉開再規劃")
            if (fails_in_row >= 2 and not tried_unwedge
                    and deadline - time.monotonic() >= UNWEDGE_MIN_REMAINING_S
                    and not goal_handle.is_cancel_requested):
                tried_unwedge = True
                self._unwedge_goal = goal_handle
                moved = self._call_unwedge()
                if goal_handle.is_cancel_requested:
                    return None, "cancel"
                if moved:
                    plan = self._plan_taught_once(waypoint_name, target_pose, quiet=True)
                    if plan is not None:
                        return self._passage_cleared(goal_handle, plan, start, polls)
            if polls % 5 == 0:
                self.get_logger().info(
                    f"通道仍被擋住（已等 {time.monotonic() - start:.0f}/"
                    f"{self.blocked_patience_sec:.0f} 秒）")

    def _passage_cleared(self, goal_handle, plan, start, polls):
        """等通道時規劃得出路徑了 -> (plan, "ok")；但規劃的當下已被取消就不要起跑。

        ★ 2026-10-09（審查 [5]）：規劃最長會阻塞 30 秒（不可取消），急停恰好落在這段時，
          以前會對旁邊的人說「路通了，繼續前進」然後起跑。
        """
        if goal_handle.is_cancel_requested:
            return None, "cancel"
        self.get_logger().info(
            f"✓ 通道淨空了（擋了 {time.monotonic() - start:.0f} 秒、重新規劃 {polls} 次）"
            f"—— 繼續前進")
        self.speech_text_pub.publish(String(data=self.blocked_speech_resume))
        return plan, "ok"

    # ── 重試迴圈用的小工具 ────────────────────────────────────
    def _plan_taught_once(self, waypoint_name, target_pose, quiet=False):
        """從車子當下的位置規劃一條到目標的路徑。失敗回傳 None。

        quiet=True：規劃失敗只記 debug（等通道淨空時每幾秒就規劃一次，失敗是預期中的）。
        """
        self._taught_seq += 1
        req = PlanTaughtPath.Request()
        # 名稱帶 _auto_ 前綴，一眼看得出是導航自動產生的、不是人錄的
        req.name = f"_auto_{waypoint_name}_{self._taught_seq}"
        req.start_from_robot = True          # ★ 每次都從「現在」規劃，不是重跑舊路徑
        req.waypoints = [target_pose]
        try:
            plan = wait_for_future(
                self.plan_taught_client.call_async(req), timeout_sec=self.taught_plan_timeout
            )
        except Exception as exc:  # noqa: BLE001
            self._last_plan_fail_msg = str(exc)
            self.get_logger().warn(f"規劃教導路徑失敗: {exc}")
            return None
        if plan is None or not plan.success:
            msg = plan.message if plan is not None else "無回應"
            self._last_plan_fail_msg = msg      # 給 _wait_for_passage 認「Start occupied」用
            # ★ 2026-10-08：兩行各自固定等級（理由見 map_service 的 _run_unwedge：
            #   rclpy 同一個呼叫位置換等級會丟 ValueError，會把整個導航重試迴圈炸掉）
            if quiet:
                self.get_logger().debug(f"規劃教導路徑不成功: {msg}")
            else:
                self.get_logger().warn(f"規劃教導路徑不成功: {msg}")
            return None

        self._last_plan_fail_msg = ""
        # 折返點數是這條路好不好走的最強預測指標：
        # 2026-08-10 十趟實測「0 折返 6 勝（誤差 11~12 cm）／多折返 5 敗」。
        # 不因為有折返就放棄（它仍然比 MPPI 好），但要讓 log 說得出來。
        cusp_note = "（0 折返）" if plan.num_cusps == 0 else f"（★ {plan.num_cusps} 個折返，較易失敗）"
        self.get_logger().info(
            f"教導路徑規劃完成：{plan.num_points} 點、{plan.length_m:.2f} m {cusp_note}")
        return plan

    def _follow_taught_once(self, goal_handle, plan, waypoint_name, deadline):
        """跑一趟重播。回傳 (outcome, 結果, path_id)。

        outcome：success / fail / cancel / timeout / reject
        """
        # ★ 2026-10-09（獨立審查 [5]）：起跑前先看取消。規劃（最長 30 秒、不可取消）、
        #   等通道的規劃、脫困之後的重新規劃，三個出口都會走到這裡；急停恰好落在那段時，
        #   以前車子會先動一下才在 _wait_taught_result 的第一圈看到取消。
        #   送出重播請求之前就返回：不送 goal、不說「開始導航」、不動 /navigation_active。
        if goal_handle.is_cancel_requested:
            return "cancel", None, plan.path_id
        # ★ 10/27：起跑前確認行進方向淨空；有人就請他讓開、等淨空（等的時候被取消就不出發）
        if getattr(self, "_announce_start", True) and not self._depart_gate(goal_handle, plan.path_id):
            return "cancel", None, plan.path_id
        follow = FollowTaughtPath.Goal()
        follow.path_id = plan.path_id
        follow.reverse = False
        follow.speed_scale = 0.0             # 0 = 用節點預設速度（follow_speed）
        try:
            fh = wait_for_future(
                self.follow_taught_client.send_goal_async(follow), timeout_sec=10.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"送出重播請求失敗: {exc}")
            return "reject", None, plan.path_id
        if fh is None or not fh.accepted:
            self.get_logger().warn("重播請求被拒絕")
            return "reject", None, plan.path_id

        if getattr(self, "_announce_start", True):
            self.speech_text_pub.publish(String(data=f"開始導航到 {waypoint_name}"))
        self.get_logger().info(f"開始導航到 {waypoint_name}（純追蹤執行）")
        self.nav_active_pub.publish(Bool(data=True))
        try:
            res, outcome = self._wait_taught_result(goal_handle, fh, deadline)
        finally:
            # 不論成功失敗都要放開，否則相機再也不會恢復
            self.nav_active_pub.publish(Bool(data=False))

        if outcome in ("cancel", "timeout"):
            return outcome, res, plan.path_id
        if res is not None and res.status == GoalStatus.STATUS_CANCELED:
            # 重播被別人取消、外層卻沒收到取消 —— 當成「失敗」會重新規劃再開一次
            return "external_cancel", res, plan.path_id

        ok = res is not None and res.status == GoalStatus.STATUS_SUCCEEDED \
            and getattr(res.result, "success", False)
        return ("success" if ok else "fail"), res, plan.path_id

    def _delete_taught(self, path_id: str) -> None:
        """刪掉臨時路徑。★ 只在成功時呼叫 —— 失敗的要留著查。"""
        if not (self.delete_temp_taught and self.delete_taught_client.service_is_ready()):
            return
        try:
            d = DeleteTaughtPath.Request()
            d.path_id = path_id
            wait_for_future(self.delete_taught_client.call_async(d), timeout_sec=5.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"刪除臨時路徑失敗（不影響導航結果）: {exc}")

    def _call_unwedge(self) -> bool:
        """請 path_teach_cc 把車子從「規劃器不肯出手」的位置推開。

        回傳 True 代表車子有被推動過，值得再規劃一次；False 就別白費力氣。
        ★ 這支**會讓車子移動**，而且是繞過 collision_monitor 的
          （楔住時它會停止輸出，指令根本到不了底盤）。所以只在規劃已經失敗、
          也就是「不動就什麼都做不了」的時候呼叫。
        """
        if not self.unwedge_client.service_is_ready():
            self.get_logger().warn("規劃失敗，但 /unwedge 服務不在，無法自救")
            return False
        # ★ 2026-10-09（獨立審查 [6]）：原本是一次阻塞 60 秒的 wait_for_future，期間完全不看取消，
        #   而 /unwedge 繞過 collision_monitor、最久約 37 秒，HMI 的急停（零速＋取消 action）碰不到它。
        #   改成每 0.2 秒一小段、每段後看一次 self._unwedge_goal（呼叫端設定）的取消旗標：
        #   取消 → 對 unwedge_abort 發一則 Empty（只發一次，path_teach 的脫困迴圈每 50 ms 檢查），
        #   然後繼續等服務自己收尾（它收到中止會很快回來，並把 cmd_vel 歸零），最後照常回傳服務的結果；
        #   由呼叫端看到取消旗標走取消流程。60 秒總期限不變。_unwedge_goal 是 None 時退化成不偵測取消。
        t0 = time.monotonic()
        gh = getattr(self, "_unwedge_goal", None)
        sent_abort = False
        fut = self.unwedge_client.call_async(Trigger.Request())
        res = None
        while True:
            try:
                res = wait_for_future(fut, timeout_sec=0.2)
                break
            except TimeoutError:
                pass
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"呼叫 /unwedge 失敗: {exc}")
                return False
            if gh is not None and not sent_abort and gh.is_cancel_requested:
                sent_abort = True
                self.unwedge_abort_pub.publish(Empty())
                self.get_logger().warn("導航被取消，通知 path_teach 中止進行中的 /unwedge 脫困（unwedge_abort）")
            if time.monotonic() - t0 >= 60.0:
                self.get_logger().warn("呼叫 /unwedge 失敗: 等了 60 秒仍沒有回應")
                return False
        if res is None:
            self.get_logger().warn("/unwedge 沒有回應")
            return False
        self.get_logger().warn(
            f"規劃不出路徑，已嘗試脫困：{'成功' if res.success else '失敗'} —— {res.message}")
        return bool(res.success)

    def _clear_costmap_quiet(self) -> None:
        """清全域成本地圖的 obstacle 層；失敗只警告不擋流程。"""
        if not self.clear_costmap_client.service_is_ready():
            return
        try:
            wait_for_future(
                self.clear_costmap_client.call_async(ClearEntireCostmap.Request()),
                timeout_sec=5.0)
            self.get_logger().info("重試前已清空全域成本地圖的 obstacle 層")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"清空成本地圖失敗（繼續重試）: {exc}")

    def _robot_xy(self):
        """車子當下的 (x, y)，還沒收到 /amcl_pose 時回傳 None。"""
        with self._pose_lock:
            return self._last_xy

    @staticmethod
    def _xy_dist(a, b):
        if a is None or b is None:
            return None
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _canceled(self, goal_handle, result):
        result.success = False
        result.message = "導航請求已被系統或使用者取消"
        goal_handle.canceled()
        self.get_logger().info(result.message)
        return result

    def _wait_taught_result(self, goal_handle, fh, deadline=None):
        """等重播結果，同時把外層 Navigate 的「取消」轉發進去。

        回傳 (結果, outcome)，outcome 是 done / cancel / timeout / failed。

        ★ deadline 由呼叫端傳入（重試迴圈共用同一個總期限），這樣「重試三次」
          不會變成「總共可以跑 3 × navigation_timeout_sec」。沒傳就自己算一個。

        ★ 2026-08-12 修正：原本這裡直接 `wait_for_future(..., 300 秒)`，
          整條執行緒被阻塞住，期間**沒有任何人去看 goal_handle 的取消旗標**。
          後果是 HMI 按「取消導航」完全沒有作用 —— 動作伺服器收到取消請求、
          cancel_callback 也回了 ACCEPT，但車子照樣一路開到底。

          nav2/MPPI 那條路早就有 `_monitor_navigation` 在輪詢取消，
          教導-重現這條沒有。同一台車兩個導航入口，只有一個煞得住，
          而 8/10 之後預設走的正是**煞不住的那一個**。

          `path_teach_cc` 的 `_follow_loop` 自己是有查 `is_cancel_requested` 的
          （見該檔 :1666），所以只要把取消**送到**它那裡，車子就會停穩、
          回報 canceled。缺的一直只是這段轉發。
        """
        result_future = fh.get_result_async()
        if deadline is None:
            deadline = time.monotonic() + self.navigation_timeout_sec
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("收到取消請求，轉發給重播動作")
                self._cancel_taught(fh)
                # 給重播端時間把車停穩並回報結果，再回覆上層
                try:
                    wait_for_future(result_future, timeout_sec=5.0)
                except Exception:  # noqa: BLE001
                    pass
                return None, "cancel"
            if time.monotonic() > deadline:
                self.get_logger().error("重播逾時，取消重播")
                self._cancel_taught(fh)
                return None, "timeout"
            time.sleep(0.2)

        if not result_future.done():
            return None, "failed"
        res = result_future.result()
        # 急停同時取消重播與 /navigate：重播的結果常比外層取消旗標先到（見 _outer_cancel_arrived）
        if self._outer_cancel_arrived(goal_handle, res.status):
            return res, "cancel"
        return res, "done"

    def _cancel_taught(self, follow_goal_handle) -> None:
        try:
            wait_for_future(follow_goal_handle.cancel_goal_async(), timeout_sec=5.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"取消重播失敗: {exc}")

    def _nav2_feedback(self, feedback_msg) -> None:
        distance = feedback_msg.feedback.distance_remaining
        with self._feedback_lock:
            if distance < self._last_distance - self.progress_epsilon_m:
                self._last_distance = distance
                self._last_progress_time = time.monotonic()

    def _monitor_navigation(self, goal_handle, nav2_goal_handle) -> str:
        """盯著 Nav2 的結果，回傳 succeeded / cancel / timeout / stalled / external_cancel / failed

        external_cancel：nav2 被別人取消（急停、卡住偵測），外層卻沒收到取消 —— 不能當成一般失敗重試
        """
        result_future = nav2_goal_handle.get_result_async()
        deadline = time.monotonic() + self.navigation_timeout_sec

        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_nav2(nav2_goal_handle)
                return "cancel"

            if time.monotonic() > deadline:
                self._cancel_nav2(nav2_goal_handle)
                return "timeout"

            with self._feedback_lock:
                stalled_for = time.monotonic() - self._last_progress_time
            if stalled_for > self.progress_stall_timeout_sec:
                self._cancel_nav2(nav2_goal_handle)
                return "stalled"

            time.sleep(0.5)

        if not result_future.done():
            return "failed"

        status = result_future.result().status
        if self._outer_cancel_arrived(goal_handle, status):
            return "cancel"
        if status == GoalStatus.STATUS_SUCCEEDED:
            return "succeeded"
        if status == GoalStatus.STATUS_CANCELED:
            return "external_cancel"
        return "failed"

    def _outer_cancel_arrived(self, goal_handle, inner_status) -> bool:
        """內層（nav2 或重播）結束之後，判斷這次是不是外層 /navigate 的取消

        ★ 2026-09-25（審查）：平板急停會幾乎同時對 /navigate_to_pose 與 /navigate 送「取消全部」
          （nav2 排第一，自動探索與 MPPI 才停得住）。nav2 幾十毫秒就回 CANCELED，而監看迴圈每 0.5 秒
          才醒一次、醒來先看結果 —— 外層的取消旗標晚幾毫秒才設，於是被判成「失敗」。
          LLM 的提示詞要求「失敗先自動重試」，**車子就可能在急停之後又開出去**。
          所以：外層旗標已設就是取消；內層是 CANCELED 時最多再等 1 秒看外層旗標。
        """
        if goal_handle.is_cancel_requested:
            return True
        if inner_status != GoalStatus.STATUS_CANCELED:
            return False
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            if goal_handle.is_cancel_requested:
                return True
            time.sleep(0.05)
        return False

    def _cancel_nav2(self, nav2_goal_handle) -> None:
        try:
            wait_for_future(nav2_goal_handle.cancel_goal_async(), timeout_sec=5.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"取消 Nav2 導航失敗: {exc}")

    def _abort(self, goal_handle, result, message: str):
        result.success = False
        result.message = message
        goal_handle.abort()
        self.get_logger().warning(f"導航失敗: {message}")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionCcNode()
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
