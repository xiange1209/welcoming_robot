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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import Trigger

from smartnav_msgs.action import Navigate
from smartnav_msgs.srv import GetWaypoint


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

        self.max_covariance_norm = float(self.get_parameter("max_covariance_norm").value)
        self.navigation_timeout_sec = float(self.get_parameter("navigation_timeout_sec").value)
        self.progress_stall_timeout_sec = float(self.get_parameter("progress_stall_timeout_sec").value)
        self.progress_epsilon_m = float(self.get_parameter("progress_epsilon_m").value)
        self.initial_pose_wait_sec = float(self.get_parameter("initial_pose_wait_sec").value)
        self.align_before_navigate = bool(self.get_parameter("align_before_navigate").value)
        self.clear_costmap_before_navigate = bool(
            self.get_parameter("clear_costmap_before_navigate").value
        )

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
        # 導航前清全域成本地圖的 obstacle 層（見參數宣告處的說明）
        self.clear_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
            callback_group=self.client_cb_group,
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
        self.nav_active_pub = self.create_publisher(
            Bool, "navigation_active",
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

    def _nav2_feedback(self, feedback_msg) -> None:
        distance = feedback_msg.feedback.distance_remaining
        with self._feedback_lock:
            if distance < self._last_distance - self.progress_epsilon_m:
                self._last_distance = distance
                self._last_progress_time = time.monotonic()

    def _monitor_navigation(self, goal_handle, nav2_goal_handle) -> str:
        """盯著 Nav2 的結果，回傳 succeeded / cancel / timeout / stalled / failed"""
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
        return "succeeded" if status == GoalStatus.STATUS_SUCCEEDED else "failed"

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
