#!/usr/bin/env python3

"""生命週期節點操作的共用工具 (_cc)

舊版的問題不在於「不會切生命週期」，而在於切得不完整：
map_server 只 deactivate 不 cleanup、slam_toolbox 建完圖之後根本沒有下線。
這裡把「把某個節點推到指定狀態」寫成一個會自己走完狀態圖的函式，
呼叫端只要說要 unconfigured / inactive / active，不用自己記轉換順序。
"""

import threading
from typing import Any, Optional

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState

# 生命週期狀態標籤
UNCONFIGURED = "unconfigured"
INACTIVE = "inactive"
ACTIVE = "active"
FINALIZED = "finalized"
UNKNOWN = "unknown"


def wait_for_future(future, timeout_sec: float) -> Any:
    """阻塞等待 future 完成

    只能在「不是 executor 執行緒」或 MultiThreadedExecutor 的工作執行緒裡呼叫，
    否則會把自己等死。
    """
    event = threading.Event()
    future.add_done_callback(lambda _f: event.set())
    if not event.wait(timeout=timeout_sec):
        raise TimeoutError("服務請求超時")
    return future.result()


class LifecycleNodeClient:
    """單一生命週期節點的遙控器"""

    def __init__(self, node, node_name: str, callback_group=None, transition_timeout: float = 20.0):
        self._node = node
        self.node_name = node_name
        self._transition_timeout = transition_timeout
        self._get_state_client = node.create_client(
            GetState, f"/{node_name}/get_state", callback_group=callback_group
        )
        self._change_state_client = node.create_client(
            ChangeState, f"/{node_name}/change_state", callback_group=callback_group
        )
        # 同一個節點的狀態轉換必須序列化，否則兩個呼叫端可能同時送互相矛盾的轉換
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # 基本操作
    # ------------------------------------------------------------------
    def wait_until_available(self, timeout_sec: float = 2.0) -> bool:
        """等生命週期服務出現"""
        ok = self._get_state_client.wait_for_service(timeout_sec=timeout_sec)
        ok = self._change_state_client.wait_for_service(timeout_sec=timeout_sec) and ok
        return ok

    def get_state(self) -> str:
        """讀取目前狀態，讀不到回 'unknown'"""
        try:
            future = self._get_state_client.call_async(GetState.Request())
            res = wait_for_future(future, timeout_sec=5.0)
            return res.current_state.label
        except Exception:
            return UNKNOWN

    def _transition(self, transition_id: int) -> bool:
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self._change_state_client.call_async(req)
        res = wait_for_future(future, timeout_sec=self._transition_timeout)
        return bool(res.success)

    # ------------------------------------------------------------------
    # 狀態圖導航
    # ------------------------------------------------------------------
    def ensure_state(self, target: str) -> bool:
        """把節點推到 target 狀態，會自己補齊中間的轉換

        lifecycle 狀態圖：
            unconfigured --configure--> inactive --activate--> active
            active --deactivate--> inactive --cleanup--> unconfigured

        target 只接受 unconfigured / inactive / active。
        """
        if target not in (UNCONFIGURED, INACTIVE, ACTIVE):
            raise ValueError(f"不支援的目標狀態: {target}")

        with self._lock:
            for _ in range(6):  # 最遠的路徑 (active -> unconfigured) 只要 2 步，6 是安全上限
                state = self.get_state()

                if state == target:
                    return True

                if state == UNKNOWN:
                    self._node.get_logger().error(f"{self.node_name}: 讀不到生命週期狀態")
                    return False

                if state == FINALIZED:
                    self._node.get_logger().error(f"{self.node_name}: 已經 finalized，無法再轉換")
                    return False

                transition = self._next_transition(state, target)
                if transition is None:
                    self._node.get_logger().error(f"{self.node_name}: {state} -> {target} 找不到轉換路徑")
                    return False

                if not self._transition(transition):
                    self._node.get_logger().error(
                        f"{self.node_name}: 轉換 {self._transition_name(transition)} 失敗 (目前 {state})"
                    )
                    return False

            self._node.get_logger().error(f"{self.node_name}: 轉換次數超過上限，仍未到達 {target}")
            return False

    @staticmethod
    def _next_transition(current: str, target: str) -> Optional[int]:
        """從 current 往 target 走的下一步"""
        order = {UNCONFIGURED: 0, INACTIVE: 1, ACTIVE: 2}
        if current not in order or target not in order:
            return None

        if order[current] < order[target]:
            # 往上爬
            return (
                Transition.TRANSITION_CONFIGURE
                if current == UNCONFIGURED
                else Transition.TRANSITION_ACTIVATE
            )
        # 往下走
        return (
            Transition.TRANSITION_DEACTIVATE
            if current == ACTIVE
            else Transition.TRANSITION_CLEANUP
        )

    @staticmethod
    def _transition_name(transition_id: int) -> str:
        return {
            Transition.TRANSITION_CONFIGURE: "configure",
            Transition.TRANSITION_ACTIVATE: "activate",
            Transition.TRANSITION_DEACTIVATE: "deactivate",
            Transition.TRANSITION_CLEANUP: "cleanup",
        }.get(transition_id, str(transition_id))


__all__ = [
    "ACTIVE",
    "FINALIZED",
    "INACTIVE",
    "LifecycleNodeClient",
    "UNCONFIGURED",
    "UNKNOWN",
    "wait_for_future",
]
