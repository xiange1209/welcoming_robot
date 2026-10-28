#!/usr/bin/env python3
"""導航仲裁節點：系統裡「讓車子去某個地點」的唯一入口。

    LLM ─┐
    迎賓頁 ─┼─ request_navigation (RequestNavigation) ─→ nav_arbiter ─→ navigate (Navigate)
    管理者 ─┘                                            身分判斷／單一進行中／取消與逾時

為什麼要有這一層：
  - 權限規則只有一份（access_policy.py），新增任何呼叫端都不可能漏掉檢查——它們根本拿不到 /navigate。
  - 同一時間只有一趟導航，由這裡裁決，不再靠各呼叫端各自預檢。
  - 取消、逾時、外部停止的語意統一成 outcome，呼叫端不必解析底層的訊息字串。

失效時一律拒絕：地點服務查不到、無法確定地點名稱時，非 OPERATOR 的請求回 UNAVAILABLE，
不會退回「先放行」。緊急停止不依賴本節點——HMI 直接對 /navigate 與 nav2 送取消全部。
"""

import threading
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from smartnav_msgs.action import Navigate, RequestNavigation
from smartnav_msgs.msg import AccessContext, GuideDestination, UserIdentity
from smartnav_msgs.srv import ListGuideDestinations, ListWaypoints

from smartnav_brain import access_policy
from smartnav_brain.user_types import UserType

Req = RequestNavigation
PHASE_AUTHORIZED = "AUTHORIZED"


class NavArbiterNode(Node):
    """身分判斷＋單一進行中＋轉送到 /navigate"""

    def __init__(self):
        super().__init__("nav_arbiter_node")

        # 時限定義在 RequestNavigation.action（TIMEOUT_SEC／CLIENT_WAIT_SEC），這裡不再有第二份
        self.navigate_timeout_sec = Req.Goal.TIMEOUT_SEC
        self.service_timeout_sec = (
            self.declare_parameter("service_timeout_sec", 5.0).get_parameter_value().double_value
        )
        self.identity_topic = (
            self.declare_parameter("user_identity_topic", "user_identity").get_parameter_value().string_value
        )
        self.context_rate_hz = (
            self.declare_parameter("access_context_rate_hz", 1.0).get_parameter_value().double_value
        )

        self.cb_group = ReentrantCallbackGroup()
        self._identity: dict = {}
        self._identity_lock = threading.Lock()
        # 進行中的導航（None = 閒置）。取得／釋放都在鎖內，BUSY 才不會有兩個同時通過
        self._active: Optional[str] = None
        self._active_lock = threading.Lock()
        self._last_context = None

        self.list_waypoints_client = self.create_client(ListWaypoints, "list_waypoints", callback_group=self.cb_group)
        self.navigate_client = ActionClient(self, Navigate, "navigate", callback_group=self.cb_group)

        self.create_subscription(UserIdentity, self.identity_topic, self._identity_cb, 10, callback_group=self.cb_group)
        self.context_pub = self.create_publisher(AccessContext, "access_context", 10)
        self.create_timer(1.0 / max(self.context_rate_hz, 0.1), self._publish_context, callback_group=self.cb_group)

        self.create_service(
            ListGuideDestinations, "list_guide_destinations", self._list_destinations_cb, callback_group=self.cb_group
        )
        self._server = ActionServer(
            self,
            RequestNavigation,
            "request_navigation",
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _handle: CancelResponse.ACCEPT,
            callback_group=self.cb_group,
        )
        self.get_logger().info(f"導航仲裁節點啟動：request_navigation → navigate（等待上限 {self.navigate_timeout_sec:.0f} 秒）")

    # ── 身分 ──────────────────────────────────────────────
    def _identity_cb(self, msg: UserIdentity) -> None:
        with self._identity_lock:
            self._identity = {
                "recognized": bool(msg.recognized),
                "user_type": UserType(int(msg.user_type.type)).name,
                "user_name": msg.user_name,
                "description": msg.description,
                "stamp": time.monotonic(),
            }

    def _current(self):
        """（tier, identity, policy）"""
        policy = access_policy.load_policy()
        with self._identity_lock:
            identity = dict(self._identity)
        return access_policy.tier_of(identity, policy), identity, policy

    def _publish_context(self) -> None:
        tier, identity, _policy = self._current()
        msg = AccessContext()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.tier = tier
        if tier == access_policy.TIER_VIP:
            msg.user_name = (identity.get("user_name") or "").strip()
            msg.description = (identity.get("description") or "").strip()
        self.context_pub.publish(msg)
        key = (tier, msg.user_name)
        if key != self._last_context:
            self._last_context = key
            self.get_logger().info(f"👤 客人等級：{tier}")

    # ── 地點 ──────────────────────────────────────────────
    def _call(self, client, request, timeout: float):
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            future.cancel()
            raise TimeoutError("服務呼叫逾時")
        return future.result()

    def _waypoints(self) -> list:
        """[(id, name)]；服務沒起來或失敗就丟 RuntimeError"""
        if not self.list_waypoints_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("地點服務未就緒")
        res = self._call(self.list_waypoints_client, ListWaypoints.Request(), self.service_timeout_sec)
        if not res.success:
            raise RuntimeError(res.message or "讀不到地點")
        return [(w.waypoint_id, w.waypoint_name) for w in res.waypoints_info]

    # ── 迎賓頁的目的地清單 ─────────────────────────────────
    def _list_destinations_cb(self, _request, response):
        tier, _identity, policy = self._current()
        with self._active_lock:
            response.busy = self._active is not None
        if tier == access_policy.TIER_BLOCKED:
            # 對外一律說 guest：畫面不能透露這個等級
            response.success, response.tier, response.can_navigate = True, access_policy.TIER_GUEST, False
            response.message = access_policy.BLOCKED_REPLY
            return response
        try:
            waypoints = self._waypoints()
        except (RuntimeError, TimeoutError) as exc:
            response.success, response.message, response.tier = False, str(exc), tier
            return response
        for waypoint_id, name in waypoints:
            if not name or name in policy["guest_menu_exclude"]:
                continue
            if access_policy.navigation_denial(tier, name, policy):
                continue
            response.destinations.append(
                GuideDestination(waypoint_id=waypoint_id, name=name, vip_only=name in policy["vip_only_waypoints"])
            )
        response.success, response.tier, response.can_navigate = True, tier, True
        return response

    # ── 導航請求 ──────────────────────────────────────────
    @staticmethod
    def _result(outcome: int, message: str):
        result = Req.Result()
        result.outcome = outcome
        result.success = outcome == Req.Result.OUTCOME_SUCCEEDED
        result.message = message
        return result

    def _authorize(self, goal):
        """（結果, 地點名稱）。結果為 None 表示放行；否則是可以直接回給呼叫端的 Result。"""
        operator = goal.source == Req.Goal.SOURCE_OPERATOR
        if goal.use_target_pose:
            if not operator:
                return self._result(Req.Result.OUTCOME_DENIED, access_policy.POSE_DENIED_REPLY), ""
            return None, goal.target_name or "指定位置"

        try:
            waypoints = self._waypoints()
        except (RuntimeError, TimeoutError) as exc:
            if operator:  # 管理者不受名稱規則限制；真的有問題底層導航自己會回失敗
                return None, goal.target_name or goal.waypoint_id
            self.get_logger().warning(f"✗ 無法確認地點，拒絕（預設拒絕）: {exc}")
            return self._result(Req.Result.OUTCOME_UNAVAILABLE, f"導航系統還沒準備好（{exc}）"), ""

        names = dict(waypoints)
        name = names.get(goal.waypoint_id)
        if name is None:
            known = [n for _i, n in waypoints if n]
            return self._result(
                Req.Result.OUTCOME_UNKNOWN_WAYPOINT, f"地點ID「{goal.waypoint_id}」不存在。現有地點：{known}"
            ), ""
        if operator:
            return None, name

        tier, _identity, policy = self._current()
        deny = access_policy.navigation_denial(tier, name, policy)
        if deny:
            self.get_logger().info(f"🚫 帶位拒絕：{name}（來源 {goal.source}，等級 {tier}）")
            return self._result(Req.Result.OUTCOME_DENIED, deny), name
        return None, name

    def _execute(self, goal_handle):
        goal = goal_handle.request
        token = object()
        with self._active_lock:
            if self._active is not None:
                busy = True
            else:
                busy = False
                self._active = token
        if busy:
            goal_handle.abort()
            return self._result(Req.Result.OUTCOME_BUSY, "我正在帶位中，請稍等一下")
        try:
            return self._run(goal_handle, goal)
        finally:
            with self._active_lock:
                self._active = None

    def _run(self, goal_handle, goal):
        denied, name = self._authorize(goal)
        if denied is not None:
            goal_handle.abort()
            return denied

        if not self.navigate_client.wait_for_server(timeout_sec=3.0):
            goal_handle.abort()
            return self._result(Req.Result.OUTCOME_UNAVAILABLE, "導航系統未啟動，無法帶位")

        nav_goal = Navigate.Goal()
        nav_goal.waypoint_id = goal.waypoint_id
        nav_goal.use_target_pose = goal.use_target_pose
        nav_goal.target_pose = goal.target_pose
        nav_goal.target_name = goal.target_name or name
        nav_handle = self._await(self.navigate_client.send_goal_async(nav_goal), 10.0)
        if nav_handle is None or not nav_handle.accepted:
            goal_handle.abort()
            return self._result(Req.Result.OUTCOME_FAILED, "導航請求未被接受")

        self.get_logger().info(f"🧭 放行導航 → {name}（來源 {goal.source}）")
        feedback = Req.Feedback()
        feedback.phase = PHASE_AUTHORIZED
        goal_handle.publish_feedback(feedback)

        done = threading.Event()
        result_future = nav_handle.get_result_async()
        result_future.add_done_callback(lambda _f: done.set())
        deadline = time.monotonic() + self.navigate_timeout_sec
        while not done.wait(0.2):
            if goal_handle.is_cancel_requested:
                self._cancel_downstream(nav_handle)
                goal_handle.canceled()
                return self._result(Req.Result.OUTCOME_CANCELED, "車子已經停下來了")
            if time.monotonic() > deadline:
                self._cancel_downstream(nav_handle)
                goal_handle.abort()
                return self._result(
                    Req.Result.OUTCOME_TIMEOUT, f"導航超過 {self.navigate_timeout_sec:.0f} 秒未完成，已中止本次請求"
                )

        wrapped = result_future.result()
        message = getattr(wrapped.result, "message", "") or ""
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success:
            goal_handle.succeed()
            return self._result(Req.Result.OUTCOME_SUCCEEDED, message or f"已到達{name}")
        # 底層被別人取消（急停）、或它自己說被外部停止：都是「不可重試」的停止，不是一般失敗
        if wrapped.status == GoalStatus.STATUS_CANCELED or "外部停止" in message:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return self._result(Req.Result.OUTCOME_CANCELED, message or "導航被取消")
        goal_handle.abort()
        return self._result(Req.Result.OUTCOME_FAILED, message or "導航過程中發生錯誤")

    def _await(self, future, timeout: float):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            future.cancel()
            return None
        return future.result()

    def _cancel_downstream(self, nav_handle) -> None:
        try:
            self._await(nav_handle.cancel_goal_async(), 5.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"✗ 送取消給 /navigate 失敗: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = NavArbiterNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
