"""HMI 共享狀態。

ROS callback（多執行緒）寫入、asyncio（WebSocket）讀取，所以每一次存取
都要過同一把鎖。這個檔案不依賴任何 ROS 型別——傳進來的一律是純 dict／純量，
由節點那邊負責把訊息攤平。這讓它可以單獨測試，也讓「狀態長什麼樣」
有一個唯一的、讀得完的定義。
"""

import threading
import time
from typing import Any, Dict, List, Optional


class HmiState:
    """HMI 共享狀態

    ROS callback (多執行緒) 寫入、asyncio (WebSocket) 讀取。
    每次寫入遞增 version，WebSocket 端靠比對 version 決定是否推播。
    """

    def __init__(self, max_messages: int = 40):
        self._lock = threading.Lock()
        self._version = 0
        self._messages_version = 0
        self.max_messages = max_messages

        self.identity: Dict[str, Any] = {
            "user_name": "",
            "user_type": "GUEST",
            "recognized": False,
            "similarity": 0.0,
            "description": "",
            "bbox": [],
            "updated_at": 0.0,
        }
        self.messages: List[Dict[str, Any]] = []
        self.partial_text: str = ""
        self.llm_streaming: str = ""
        self.system: Dict[str, Any] = {
            "speaking": False,
            "voltage": None,
            "map_id": "",
            "camera_fps": 0.0,
            "started_at": time.time(),
            "llm_model": "",
            "charging": None,  # 底盤 robot_charging_flag
            "charge_current": None,  # 底盤 robot_charging_current（安培）
            "speed": None,  # /odom 的前進速度（m/s，帶正負號）
            "cpu_temp": None,  # Pi 核心溫度（°C）——導航時會逼近降頻門檻
            "mem_avail_mb": None,  # 可用記憶體
            "mem_used_pct": None,
            "cpu_load": None,  # 每分鐘平均負載
        }
        self.map_meta: Optional[Dict[str, Any]] = None
        self.robot_pose: Optional[Dict[str, float]] = None
        # 要畫在地圖上的規劃路徑（見 set_nav_path）
        self.nav_path: Optional[Dict[str, Any]] = None
        self.jobs: Dict[str, Dict[str, Any]] = {}
        # 人臉註冊進度。放在共用狀態而不是前端區域變數，這樣平板、筆電
        # 等所有連線中的裝置看到的採樣倒數與結果都是同一份。
        self.registration: Dict[str, Any] = {}

    # ── 寫入端（ROS callback）──────────────────────────────

    def _bump(self) -> None:
        self._version += 1

    def set_identity(self, **kwargs) -> None:
        with self._lock:
            self.identity.update(kwargs)
            self.identity["updated_at"] = time.time()
            self._bump()

    def add_message(self, role: str, text: str, stats: Optional[Dict[str, Any]] = None) -> None:
        text = (text or "").strip()
        if not text:
            return
        with self._lock:
            # 同一角色連續送出完全相同的內容就不重複記（TTS 與 LLM 常會重疊）
            if (
                self.messages
                and self.messages[-1]["role"] == role
                and self.messages[-1]["text"] == text
                and time.time() - self.messages[-1].get("ts", 0.0) < 3.0
            ):
                return
            entry: Dict[str, Any] = {"role": role, "text": text, "ts": time.time()}
            if stats:
                entry["stats"] = stats
            self.messages.append(entry)
            del self.messages[: max(0, len(self.messages) - self.max_messages)]
            self._messages_version += 1
            self._bump()

    def clear_messages(self) -> None:
        """清空對話紀錄與串流中的殘句

        串流中的文字一併清掉，否則清完畫面後上一輪還沒收尾的半句會馬上又冒出來。
        """
        with self._lock:
            if not self.messages and not self.llm_streaming and not self.partial_text:
                return
            self.messages.clear()
            self.llm_streaming = ""
            self.partial_text = ""
            self._messages_version += 1
            self._bump()

    def append_stream(self, token: str) -> None:
        """把串流 token 接到目前這段回覆後面

        llm_stream 話題送的是「單一 token」不是累積字串，直接覆蓋的話
        畫面上只會看到最新那一個字，不會有逐字浮現的效果。
        """
        if not token:
            return
        with self._lock:
            self.llm_streaming += token
            self._bump()

    def set_partial(self, text: str) -> None:
        with self._lock:
            if self.partial_text != text:
                self.partial_text = text
                self._bump()

    def set_stream(self, text: str) -> None:
        with self._lock:
            if self.llm_streaming != text:
                self.llm_streaming = text
                self._bump()

    def set_system(self, **kwargs) -> None:
        with self._lock:
            changed = any(self.system.get(k) != v for k, v in kwargs.items())
            self.system.update(kwargs)
            if changed:
                self._bump()

    def set_map_meta(self, meta: Optional[Dict[str, Any]]) -> None:
        with self._lock:
            self.map_meta = meta
            self._bump()

    def set_nav_path(self, source: str, pts: list) -> None:
        """更新要畫在地圖上的路徑（2026-08-14 新增）

        使用者要求：「我想要在 HMI 看到它規劃的路徑（按下導航或錄製重現）」。
        資料本來就存在，只是從來沒有被送到前端 ——
          `taught_path`  path_teach_cc 在重播開始與規劃完成時發（latched）
          `plan`         nav2 planner_server 的全域路徑

        ★ 這裡**不做座標轉換**。前端已經有 map_meta（resolution/origin）與現成的
          世界座標→像素轉換（畫機器人用的那個），所以送原始 map frame 的公尺值
          最單純，也不會因為換地圖而失效。

        ★ 空清單代表「清掉畫面上的路徑」（導航結束時）。
        """
        with self._lock:
            new = {"source": source, "points": pts} if pts else None
            old = self.nav_path
            # 點數與來源都一樣時就不算變 —— 這個東西會 5 Hz 進來，
            # 每次都 _bump() 會讓 WebSocket 一直推整包狀態。
            same = (old is None and new is None) or (
                old is not None
                and new is not None
                and old["source"] == new["source"]
                and len(old["points"]) == len(new["points"])
            )
            self.nav_path = new
            if not same:
                self._bump()

    def set_robot_pose(self, pose: Optional[Dict[str, float]]) -> None:
        """更新機器人位姿

        位姿以 5 Hz 更新，若每次都遞增版本號會讓 WebSocket 一直推整包狀態。
        因此只有移動超過 2 公分或轉超過 2 度才視為「變了」。
        """
        with self._lock:
            old = self.robot_pose
            if pose is None:
                changed = old is not None
            elif old is None:
                changed = True
            else:
                changed = (
                    abs(pose["x"] - old["x"]) > 0.02
                    or abs(pose["y"] - old["y"]) > 0.02
                    or abs(pose["yaw"] - old["yaw"]) > 0.035
                )
            self.robot_pose = pose
            if changed:
                self._bump()

    def set_job(self, job_id: str, **kwargs) -> None:
        with self._lock:
            job = self.jobs.setdefault(job_id, {"job_id": job_id})
            job.update(kwargs)
            # 只保留最近 20 筆，避免長時間運行後無限增長
            if len(self.jobs) > 20:
                for stale in sorted(self.jobs, key=lambda k: self.jobs[k].get("started_at", 0))[:-20]:
                    del self.jobs[stale]
            self._bump()

    def set_registration(self, payload: Optional[Dict[str, Any]]) -> None:
        """整份取代註冊狀態。傳 None 表示清除"""
        with self._lock:
            new = dict(payload) if payload else {}
            if new == self.registration:
                return
            self.registration = new
            self._bump()

    def merge_registration(self, payload: Dict[str, Any]) -> None:
        """疊加更新註冊狀態

        進度來自 registration_progress 話題，但倒數要用的 started_at/deadline
        是發起端點寫的——用疊加才不會被進度訊息洗掉。
        """
        with self._lock:
            new = dict(self.registration)
            new.update(payload)
            if new == self.registration:
                return
            self.registration = new
            self._bump()

    def registration_active(self) -> bool:
        with self._lock:
            return self.registration.get("status") == "running"

    # ── 讀取端（asyncio）──────────────────────────────────

    def snapshot(self, since_messages_version: Optional[int] = None) -> Dict[str, Any]:
        """取得狀態快照

        傳入 `since_messages_version` 時，只有對話真的變過才會帶上 `messages`
        欄位——前端沿用自己那份即可。省下的是 LLM 串流期間每秒 10 次、
        每次數十則訊息的序列化成本。
        """
        with self._lock:
            payload = {
                "version": self._version,
                # 伺服器時鐘基準。前端要判斷 identity 有多舊，但平板時鐘不一定
                # 跟機器人同步，所以送出當下的伺服器時間讓前端算差值而不是絕對值。
                "now": time.time(),
                "identity": dict(self.identity),
                "messages_version": self._messages_version,
                "partial_text": self.partial_text,
                "llm_streaming": self.llm_streaming,
                "system": dict(self.system),
                "map_meta": dict(self.map_meta) if self.map_meta else None,
                "robot_pose": dict(self.robot_pose) if self.robot_pose else None,
                "nav_path": dict(self.nav_path) if self.nav_path else None,
                "jobs": [dict(j) for j in self.jobs.values()],
                "registration": dict(self.registration) if self.registration else None,
            }
            if since_messages_version is None or since_messages_version != self._messages_version:
                payload["messages"] = list(self.messages)
            return payload

    @property
    def version(self) -> int:
        with self._lock:
            return self._version

    @property
    def messages_version(self) -> int:
        with self._lock:
            return self._messages_version

    def map_meta_copy(self) -> Optional[Dict[str, Any]]:
        """只取地圖中繼資料

        原本 /api/map/meta 是呼叫 snapshot() 再挑一個欄位出來，等於為了拿六個
        數字把整份對話紀錄也複製一遍。
        """
        with self._lock:
            return dict(self.map_meta) if self.map_meta else None

    def jobs_copy(self) -> List[Dict[str, Any]]:
        """只取作業清單（理由同 map_meta_copy）"""
        with self._lock:
            return [dict(j) for j in self.jobs.values()]
