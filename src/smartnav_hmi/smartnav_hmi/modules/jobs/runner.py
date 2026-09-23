"""ROS 服務／動作呼叫的共用機制：阻塞呼叫、長時間 action 的 job 登記簿、
緊急停止用的「取消全部」。

從 hmi_server_node.py 搬出來，被 navigation/maps.py、navigation/waypoints.py、
navigation/paths.py、users/registration.py、navigation/teleop.py（雷達參數）
共用。仍吃 `node`：service/action client 都是它的屬性，建立取消用的
service client 也要用 node.create_client。
"""

import itertools
import threading
import time
from typing import Any, Dict, List, Optional

from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalStatus

from ...core.constants import ACTION_TIMEOUTS


class JobRunner:
    # ★★ 緊急停止原本停不住「不是 HMI 發起的」導航 ★★
    #
    # 實機發生過：使用者用語音叫車子做全域定位，按平板的緊急停止，
    # log 回「已送零速，取消 0 個作業」而車子照走——因為 cancel_job() 只能
    # 取消 _job_handles 裡的 goal handle，那是 HMI 自己送出去才會登記的。
    # LLM 的工具是 llm_service_node -> /navigate 直接送，HMI 從頭到尾不知道。
    #
    # 解法：直接對 action 的 cancel 服務送「取消全部」。ROS 2 action 規格：
    # goal_id 全零 + stamp 全零 = 取消該伺服器上**所有**目標，與是誰送出的無關。
    ACTION_CANCEL_ALL = ("navigate", "follow_taught_path", "global_localization", "create_map")

    def __init__(self, node, state, service_clients: Dict[str, Any], action_clients: Dict[str, Any],
                 service_timeout: float, cancel_callback_group):
        self._node = node
        self.state = state
        self.service_clients = service_clients
        self.action_clients = action_clients
        self.service_timeout = service_timeout
        self._cancel_cb_group = cancel_callback_group
        self._cancel_srv_clients: Dict[str, Any] = {}

        self._job_counter = itertools.count(1)
        # 進行中的動作 goal handle 登記簿，供取消使用
        self._job_handles: Dict[str, Any] = {}
        self._job_handles_lock = threading.Lock()

    # ── 阻塞版 service/action 呼叫（給執行緒池用）──────────

    @staticmethod
    def await_future(future, timeout: Optional[float]) -> Any:
        """阻塞等待 rclpy future

        用 threading.Event 而不是輪詢——future 的完成由 executor 執行緒觸發，
        在這裡自旋只是白燒 CPU（Pi4 上 load average 本來就吃緊）。
        """
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            raise TimeoutError("動作等待逾時")
        return future.result()

    def call_service(self, name: str, request: Any, timeout: Optional[float] = None) -> Any:
        """同步呼叫服務並回傳 response

        這個函式會阻塞，必須從 asyncio 的執行緒池（asyncio.to_thread）呼叫，
        不能直接在事件迴圈裡跑。
        """
        client = self.service_clients[name]
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"服務 {name} 未就緒（對應節點沒啟動？）")

        limit = self.service_timeout if timeout is None else timeout
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=limit):
            future.cancel()
            raise TimeoutError(f"服務 {name} 呼叫逾時（{limit:.0f} 秒）")
        return future.result()

    # ── 長時間 action 的 job 登記簿 ────────────────────────

    def start_action(self, name: str, goal: Any, label: str) -> str:
        """送出動作目標，回傳 job_id；實際等待在背景執行緒進行"""
        job_id = f"job{next(self._job_counter)}"
        self.state.set_job(
            job_id,
            action=name,
            label=label,
            status="pending",
            message="等待動作伺服器接受目標…",
            started_at=time.time(),
        )
        threading.Thread(target=self._run_action, args=(job_id, name, goal), daemon=True).start()
        return job_id

    def _run_action(self, job_id: str, name: str, goal: Any) -> None:
        """在背景執行緒跑完整個動作生命週期並持續更新 job 狀態"""
        logger = self._node.get_logger()
        client = self.action_clients[name]
        try:
            if not client.wait_for_server(timeout_sec=3.0):
                self.state.set_job(job_id, status="failed", message=f"動作 {name} 未就緒（對應節點沒啟動？）")
                return

            send_future = client.send_goal_async(goal)
            goal_handle = self.await_future(send_future, timeout=10.0)
            if goal_handle is None or not goal_handle.accepted:
                self.state.set_job(job_id, status="rejected", message="動作目標被拒絕")
                return

            with self._job_handles_lock:
                self._job_handles[job_id] = goal_handle
            self.state.set_job(job_id, status="running", message="執行中…")

            result_future = goal_handle.get_result_async()
            # 動作本身有自己的逾時（見 ACTION_TIMEOUTS 的註解），
            # 這裡的上限只是為了在動作伺服器整個掛掉時不要留下永遠不結束的執行緒。
            action_timeout = ACTION_TIMEOUTS.get(name, 300.0)
            try:
                wrapped = self.await_future(result_future, timeout=action_timeout)
            except TimeoutError:
                # ★ 等不到結果時**主動叫停**，不能只是自己放棄。
                #
                # 這條路只有在「動作節點沒有依約回報」時才會走到，而那正是最危險
                # 的情形：HMI 認定作業結束、finally 把 goal handle 從登記簿移除，
                # 於是連緊急停止都找不到它 —— 而車子完全沒被通知過，還在照原本的
                # 路徑開。送出取消至少讓節點端的取消檢查有機會把車停穩。
                logger.error(
                    f"動作 {name} 等待結果逾時（{action_timeout:.0f} 秒），" "主動送出取消以免車子繼續移動"
                )
                try:
                    goal_handle.cancel_goal_async()
                except Exception as e:  # noqa: BLE001
                    logger.error(f"逾時後送出取消也失敗: {e}")
                self.state.set_job(job_id, status="failed", message="等待結果逾時，已送出取消（請確認車輛已停止）")
                return

            result = getattr(wrapped, "result", None)
            status = getattr(wrapped, "status", None)

            if status == GoalStatus.STATUS_CANCELED:
                self.state.set_job(job_id, status="cancelled", message="已取消")
                return

            success = bool(getattr(result, "success", False))
            message = getattr(result, "message", "") or ("完成" if success else "失敗")
            self.state.set_job(job_id, status="succeeded" if success else "failed", message=message)
        except Exception as e:
            self.state.set_job(job_id, status="failed", message=f"動作執行錯誤: {e}")
            logger.error(f"動作 {name} 執行錯誤: {e}")
        finally:
            with self._job_handles_lock:
                self._job_handles.pop(job_id, None)

    def cancel_job(self, job_id: str) -> bool:
        """要求取消進行中的動作"""
        with self._job_handles_lock:
            goal_handle = self._job_handles.get(job_id)
        if goal_handle is None:
            return False
        goal_handle.cancel_goal_async()
        self.state.set_job(job_id, status="cancelling", message="取消中…")
        return True

    def cancel_all_action_goals(self) -> List[str]:
        """對所有會讓車子移動的 action 送『取消全部』。回傳成功送出的動作名。"""
        node = self._node
        sent: List[str] = []
        for name in self.ACTION_CANCEL_ALL:
            client = self.action_clients.get(name)
            if client is None:
                continue
            try:
                srv = self._cancel_srv_clients.get(name)
                if srv is None:
                    # action 的取消服務固定是 <action_name>/_action/cancel_goal
                    srv = node.create_client(
                        CancelGoal, f"{client._action_name}/_action/cancel_goal", callback_group=self._cancel_cb_group
                    )
                    self._cancel_srv_clients[name] = srv
                if not srv.service_is_ready():
                    continue
                # 全零 goal_id + 全零 stamp = 取消全部
                srv.call_async(CancelGoal.Request())
                sent.append(name)
            except Exception as e:  # noqa: BLE001
                node.get_logger().warning(f"送出 {name} 的取消全部失敗: {e}")
        return sent
