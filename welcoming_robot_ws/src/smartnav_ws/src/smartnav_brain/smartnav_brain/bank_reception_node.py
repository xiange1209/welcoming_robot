#!/usr/bin/env python3
"""銀行迎賓劇本：把人臉辨識事件變成語音、通報與訪客記錄。

這是專題「VIP 迎賓與安全通報系統」的劇本中樞——辨識管線只負責認出是誰，
「認出之後要做什麼」全部在這裡。

## 事件流

    相機 -> face_embedding_node -> user_auth_node
                                      |
                                      v  /user_identity (UserIdentity)
                              bank_reception_node
                                      |
              +-----------------------+------------------------+
              v                       v                        v
        /speech_text            通報後端                 SQLite 訪客記錄
        (TTS 播報)          (Telegram Bot API)          ~/.smartnav/visit_log.db
              |
              v  /brain_event (JSON)
           HMI 顯示

## 2026-08-01 從 git 歷史復原並移植

原版（commit b7acf77）訂閱的是 `RecognitionResult`，那個訊息在 vision
重構後**已經不存在**，所以這個節點在 84f8e3e 被整個刪掉。這次復原時
移植到新介面 `UserIdentity`，欄位對應：

    person_type (string)  ->  user_type.type (uint8) 需查表轉回名稱
    person_name           ->  user_name
    person_uuid           ->  user_uuid
    confidence            ->  similarity
    （新增）                  recognized —— 是否通過相似度門檻

**新介面多了 `recognized` 這個布林值**，語意比舊版的「信心值 > 門檻」更明確：
辨識管線自己已經判斷過了，劇本不該再自己訂一套門檻去覆寫它。所以下面
`min_confidence` 只當成「額外的保守門檻」，而不是唯一依據。

## 冷卻設計（為什麼需要）

人臉辨識是連續發布的（每幀都發），沒有冷卻的話同一個人站在鏡頭前
會讓機器人每秒重複播報十幾次。兩種冷卻分開計：

    VIP / 黑名單  以 uuid 為 key，各自獨立冷卻 60 秒
    一般訪客      統一用 "visitor" 當 key，冷卻 300 秒

訪客不分人是刻意的：大廳人來人往，若每個路人都觸發一次問候，
機器人會變成不停說話的噪音源。
"""
import json
import sqlite3
import threading
import time
import urllib.request
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from smartnav_msgs.msg import UserIdentity

from smartnav_brain.user_manager import UserType


class BankReceptionNode(Node):
    """銀行迎賓與安全通報的劇本節點"""

    def __init__(self):
        super().__init__("bank_reception_node")

        # ── 參數 ──────────────────────────────────────────────
        self.cooldown_sec = self.declare_parameter(
            "cooldown_sec", 60.0).get_parameter_value().double_value
        self.visitor_cooldown_sec = self.declare_parameter(
            "visitor_cooldown_sec", 300.0).get_parameter_value().double_value
        # 額外的保守門檻。user_auth_node 的 recognized 已經判斷過一次，
        # 這裡只是再收緊一點，避免遠距離的低品質辨識觸發劇本。
        self.min_confidence = self.declare_parameter(
            "min_confidence", 0.5).get_parameter_value().double_value

        self.notify_backend = self.declare_parameter(
            "notify_backend", "none").get_parameter_value().string_value
        self.telegram_bot_token = self.declare_parameter(
            "telegram_bot_token", "").get_parameter_value().string_value
        self.telegram_chat_id = self.declare_parameter(
            "telegram_chat_id", "").get_parameter_value().string_value

        self.enable_visit_log = self.declare_parameter(
            "enable_visit_log", True).get_parameter_value().bool_value
        self.visit_log_path = self.declare_parameter(
            "visit_log_path",
            str(Path.home() / ".smartnav" / "visit_log.db")).get_parameter_value().string_value

        self.identity_topic = self.declare_parameter(
            "user_identity_topic", "user_identity").get_parameter_value().string_value

        self.vip_greeting = self.declare_parameter(
            "vip_greeting",
            "{name}貴賓您好，歡迎蒞臨，需要為您帶位到貴賓室嗎",
        ).get_parameter_value().string_value
        self.visitor_greeting = self.declare_parameter(
            "visitor_greeting",
            "您好，歡迎光臨智慧銀行，需要協助請跟我說",
        ).get_parameter_value().string_value
        self.blacklist_announce = self.declare_parameter(
            "blacklist_announce",
            "已通知行員前來協助",
        ).get_parameter_value().string_value

        # ── 狀態 ──────────────────────────────────────────────
        self._last_seen: dict = {}
        self._state = "IDLE"
        self._db_lock = threading.Lock()
        self._init_visit_db()

        # ── 介面 ──────────────────────────────────────────────
        self.speech_pub = self.create_publisher(String, "speech_text", 10)
        self.event_pub = self.create_publisher(String, "brain_event", 10)
        self.identity_sub = self.create_subscription(
            UserIdentity, self.identity_topic, self.identity_callback, 10)
        # LLM 的通報工具走這個話題進來，讓「通報」只有一個出口
        self.notify_sub = self.create_subscription(
            String, "staff_notify_request", self.notify_callback, 10)

        self.get_logger().info(
            f"銀行迎賓劇本啟動：訂閱 {self.identity_topic}，"
            f"通報後端 {self.notify_backend}"
            f"{'（dry-run，只印 log 不實際送出）' if self.notify_backend != 'telegram' else ''}"
        )

    # ==================================================================
    def identity_callback(self, msg: UserIdentity) -> None:
        name = msg.user_name or "Unknown"
        try:
            ptype = UserType(msg.user_type.type).name
        except ValueError:
            ptype = "GUEST"

        # 未通過辨識門檻的一律當一般訪客 —— 不可以把「認不出來的人」
        # 當成黑名單，那會讓誤報率高到無法展示。
        if not msg.recognized:
            ptype = "GUEST"

        # VIP 與黑名單是會觸發「說出名字」與「通報行員」的動作，
        # 誤判的代價比一般問候高得多，所以再加一道保守門檻。
        if ptype in ("VIP", "BLACKLIST") and msg.similarity < self.min_confidence:
            return

        cooldown_key = msg.user_uuid or name
        cooldown = self.cooldown_sec
        if ptype not in ("VIP", "BLACKLIST"):
            cooldown_key = "visitor"          # 訪客不分人，統一冷卻
            cooldown = self.visitor_cooldown_sec

        now = time.monotonic()
        if now - self._last_seen.get(cooldown_key, -1e9) < cooldown:
            return
        self._last_seen[cooldown_key] = now

        if ptype == "VIP":
            self._state = "GREETING"
            self._say(self.vip_greeting.format(name=name))
            self._emit_event("vip_greeting", name, msg.similarity)
        elif ptype == "BLACKLIST":
            self._state = "ALERTING"
            # 現場只說中性的話 —— 當著本人的面播報「偵測到黑名單」
            # 會讓情況立刻升溫，而且對誤判的人是嚴重冒犯。
            # 真正的細節只送給行員。
            self._say(self.blacklist_announce)
            self._send_notification(
                f"⚠️ 偵測到黑名單人員：{name}（相似度 {msg.similarity:.2f}）"
                f"請行員立即前往現場"
            )
            self._emit_event("blacklist_alert", name, msg.similarity)
        else:
            self._state = "GREETING"
            self._say(self.visitor_greeting)
            self._emit_event("visitor_greeting", name, msg.similarity)

        self._log_visit(msg, ptype)
        self._state = "IDLE"

    # ── LLM 工具的通報請求（通報中樞入口）─────────────────────
    def notify_callback(self, msg: String) -> None:
        self.get_logger().info(f"📨 收到通報請求: {msg.data}")
        self._send_notification(msg.data)

    # ── 動作實作 ──────────────────────────────────────────────
    def _say(self, text: str) -> None:
        self.speech_pub.publish(String(data=text))
        self.get_logger().info(f"🗣️ 迎賓語音: {text}")

    def _emit_event(self, event: str, name: str, confidence: float) -> None:
        payload = json.dumps(
            {"event": event, "name": name,
             "confidence": round(float(confidence), 3), "ts": int(time.time())},
            ensure_ascii=False,
        )
        self.event_pub.publish(String(data=payload))

    def _send_notification(self, text: str) -> None:
        if self.notify_backend != "telegram":
            # dry-run 是刻意的預設：沒設 token 也能完整演一遍劇本，
            # log 裡看得到「本來會送出什麼」。
            self.get_logger().warn(f"🔕 通報（dry-run，未設定後端）: {text}")
            return
        if not self.telegram_bot_token or not self.telegram_chat_id:
            self.get_logger().error(
                "✗ notify_backend=telegram 但 telegram_bot_token/telegram_chat_id 未設定")
            return
        # 推播走獨立執行緒：網路慢也不能卡住辨識 callback，
        # 卡住的話後面的人臉事件會整個塞住。
        threading.Thread(target=self._telegram_send, args=(text,), daemon=True).start()

    def _telegram_send(self, text: str) -> None:
        # 用標準庫的 urllib 而不是 requests：零新依賴，Pi 上不用再 pip install
        url = f"https://api.telegram.org/bot{self.telegram_bot_token}/sendMessage"
        data = json.dumps({"chat_id": self.telegram_chat_id, "text": text}).encode("utf-8")
        req = urllib.request.Request(url, data=data,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as resp:
                if resp.status == 200:
                    self.get_logger().info("✓ Telegram 通報已送出")
                else:
                    self.get_logger().error(f"✗ Telegram 回應異常: HTTP {resp.status}")
        except Exception as e:
            self.get_logger().error(f"✗ Telegram 通報失敗: {e}")

    # ── 訪客記錄（SQLite）────────────────────────────────────
    def _init_visit_db(self) -> None:
        if not self.enable_visit_log:
            return
        try:
            Path(self.visit_log_path).parent.mkdir(parents=True, exist_ok=True)
            with self._db_lock, sqlite3.connect(self.visit_log_path) as conn:
                conn.execute(
                    "CREATE TABLE IF NOT EXISTS visits ("
                    "ts INTEGER, person_uuid TEXT, person_name TEXT, "
                    "person_type TEXT, confidence REAL)"
                )
        except Exception as e:
            # 記錄失敗不該讓整個劇本停擺 —— 迎賓比記錄重要
            self.get_logger().warn(f"✗ 訪客記錄資料庫初始化失敗（記錄停用）: {e}")
            self.enable_visit_log = False

    def _log_visit(self, msg: UserIdentity, person_type: str) -> None:
        if not self.enable_visit_log:
            return
        try:
            with self._db_lock, sqlite3.connect(self.visit_log_path) as conn:
                conn.execute(
                    "INSERT INTO visits VALUES (?, ?, ?, ?, ?)",
                    (int(time.time()), msg.user_uuid, msg.user_name,
                     person_type, float(msg.similarity)),
                )
        except Exception as e:
            self.get_logger().warn(f"✗ 訪客記錄寫入失敗: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BankReceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
