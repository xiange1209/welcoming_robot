"""人臉註冊：背景採樣執行緒、進度追蹤、逾時看門狗。

從 hmi_server_node.py 搬出來。register_face 服務只負責「開始」——它同步
建好使用者資料列就立刻回應，樣本是之後由 user_auth_node 在人臉回呼裡
一張張累積的，實際進度由 registration_progress 話題推送。
"""

import threading
import time
from typing import Any

from smartnav_msgs.msg import RegistrationProgress, UserType
from smartnav_msgs.srv import RegisterFace


class RegistrationRunner:
    # 對齊 user_auth_node._register_face_callback 裡的 threading.Timer(20.0)。
    # 那支計時器一到就會把樣本不足的使用者刪掉，這裡不能等得比它久。
    REG_TIMEOUT_SEC = 20.0
    # 輪詢註冊進度的間隔。1 秒足以看到張數跳動，又不會把 list_users 打爆。
    REG_POLL_SEC = 1.0

    def __init__(self, node, state, jobs):
        self._node = node
        self.state = state
        self._jobs = jobs

    def _fail_registration(self, name: str, message: str, collected: int = 0) -> None:
        self.state.set_registration(
            {
                "status": "failed",
                "user_name": name,
                "message": message,
                "collected": collected,
                "finished_at": time.time(),
            }
        )

    def on_progress(self, msg: RegistrationProgress) -> None:
        """user_auth_node 每收一張樣本就發一次，取代原本每秒輪詢 list_users"""
        payload: Any = {
            "status": msg.status,
            "user_name": msg.user_name,
            "user_uuid": msg.user_uuid,
            "collected": int(msg.collected_samples),
            "target": int(msg.target_samples),
            "message": msg.message,
        }
        if msg.active:
            payload["finished_at"] = 0.0
        else:
            payload["finished_at"] = time.time()
        self.state.merge_registration(payload)

        # 採樣期間名單的 num_samples 一直在變，結束時再刷一次讓前端拿到定案值
        if not msg.active and msg.status == "succeeded":
            self._node.get_logger().info(f"註冊完成: {msg.message}")

    def run(self, name: str, user_type: int, description: str, num_samples: int) -> None:
        """在背景執行緒發起人臉註冊，並看守它有沒有正常結束

        這裡只負責發起與看門狗；實際進度由 registration_progress 話題推送
        （見 on_progress）。
        """
        try:
            request = RegisterFace.Request()
            request.user_name = name
            request.user_type = UserType(type=user_type)
            request.description = description
            request.num_samples = num_samples
            res = self._jobs.call_service("register_face", request)
            if not bool(res.success):
                self._fail_registration(name, res.message)
                return
        except Exception as e:
            self._node.get_logger().error(f"人臉註冊錯誤: {e}")
            self._fail_registration(name, f"註冊失敗: {e}")
            return

        # 進度由 registration_progress 話題推送，這裡只當看門狗：
        # user_auth_node 若中途掛掉就不會再有任何訊息，狀態會永遠卡在 running。
        # ★ 這支是看門狗，用牆鐘等於把它自己也交給一個會跳的東西——
        #   NTP 往後階躍時它永遠不會響，而「永遠卡在 running」正是它存在要防的那件事。
        deadline = time.monotonic() + self.REG_TIMEOUT_SEC + 5.0
        while time.monotonic() < deadline:
            time.sleep(0.5)
            if not self.state.registration_active():
                return

        self._fail_registration(name, "註冊逾時且沒有收到進度回報，請確認 user_auth_node 是否正常")

    def start_background(self, name: str, user_type: int, description: str, num_samples: int) -> None:
        threading.Thread(
            target=self.run,
            args=(name, user_type, description, num_samples),
            daemon=True,
        ).start()
