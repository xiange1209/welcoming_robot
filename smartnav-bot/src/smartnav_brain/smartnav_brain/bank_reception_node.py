#!/usr/bin/env python3

"""銀行迎賓劇本節點（bank_reception_node）

訂閱人臉辨識結果，依身份類型觸發對應劇本：
  VIP       → 語音迎賓（含姓名）
  BLACKLIST → 語音警告 ＋ 通報行員（Telegram）
  VISITOR   → 一般歡迎詞

同時作為全系統的通報中樞：LLM Agent 的 notify_staff 工具
發布到 /staff_notify_request，由本節點統一送出 Telegram 推播。
每次觸發（通過冷卻）寫入 SQLite 訪客記錄，供日後統計。

無硬體測試（在家）：
  ros2 topic pub --once /face_recognition/result smartnav_msgs/msg/RecognitionResult \
    "{person_name: '陳佳憲', person_type: 'VIP', confidence: 0.95, person_uuid: 'test-uuid-1'}"
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

from smartnav_msgs.msg import RecognitionResult


class BankReceptionNode(Node):
    """迎賓劇本狀態機與通報中樞"""

    def __init__(self):
        super().__init__("bank_reception_node")

        # ── 參數 ──────────────────────────────────────────────
        self.cooldown_sec = self.declare_parameter("cooldown_sec", 60.0).get_parameter_value().double_value
        self.visitor_cooldown_sec = (
            self.declare_parameter("visitor_cooldown_sec", 300.0).get_parameter_value().double_value
        )
        self.min_confidence = self.declare_parameter("min_confidence", 0.5).get_parameter_value().double_value
        # 通報後端：none（僅記錄，dry-run）或 telegram
        self.notify_backend = self.declare_parameter("notify_backend", "none").get_parameter_value().string_value
        self.telegram_bot_token = (
            self.declare_parameter("telegram_bot_token", "").get_parameter_value().string_value
        )
        self.telegram_chat_id = self.declare_parameter("telegram_chat_id", "").get_parameter_value().string_value
        self.enable_visit_log = self.declare_parameter("enable_visit_log", True).get_parameter_value().bool_value
        self.visit_log_path = (
            self.declare_parameter("visit_log_path", str(Path.home() / ".smartnav" / "visit_log.db"))
            .get_parameter_value()
            .string_value
        )
        # 迎賓詞樣板（{name} 會被替換成辨識到的姓名）
        self.vip_greeting = (
            self.declare_parameter("vip_greeting", "{name}貴賓您好，歡迎蒞臨，需要為您帶位到貴賓室嗎")
            .get_parameter_value()
            .string_value
        )
        self.visitor_greeting = (
            self.declare_parameter("visitor_greeting", "您好，歡迎光臨智慧銀行，需要協助請跟我說")
            .get_parameter_value()
            .string_value
        )
        self.blacklist_announce = (
            self.declare_parameter("blacklist_announce", "已通知行員前來協助")
            .get_parameter_value()
            .string_value
        )

        # ── 內部狀態 ──────────────────────────────────────────
        # 冷卻表：key = person_uuid（VISITOR 統一用 "visitor"），value = 上次觸發時間
        self._last_seen: dict = {}
        self._state = "IDLE"  # IDLE / GREETING / ALERTING（預留給未來帶位流程擴充）
        self._db_lock = threading.Lock()
        self._init_visit_db()

        # ── 通訊介面 ──────────────────────────────────────────
        self.speech_pub = self.create_publisher(String, "speech_text", 10)
        self.event_pub = self.create_publisher(String, "brain_event", 10)
        self.face_sub = self.create_subscription(
            RecognitionResult, "/face_recognition/result", self.face_callback, 10
        )
        self.notify_sub = self.create_subscription(String, "/staff_notify_request", self.notify_callback, 10)

        backend_desc = self.notify_backend if self.notify_backend != "none" else "none（dry-run，只記錄不推播）"
        self.get_logger().info(f"✓ 銀行迎賓劇本節點已初始化（通報後端: {backend_desc}）")

    # ── 人臉事件 → 劇本 ──────────────────────────────────────

    def face_callback(self, msg: RecognitionResult) -> None:
        person_type = (msg.person_type or "").upper()
        name = msg.person_name or "Unknown"

        # 低信心或未辨識不觸發劇本（避免路人閃過就說話）
        if msg.confidence < self.min_confidence and person_type in ("VIP", "BLACKLIST"):
            return

        cooldown_key = msg.person_uuid or name
        cooldown = self.cooldown_sec
        if person_type not in ("VIP", "BLACKLIST"):
            cooldown_key = "visitor"  # 訪客不分人，統一冷卻
            cooldown = self.visitor_cooldown_sec

        now = time.monotonic()
        if now - self._last_seen.get(cooldown_key, -1e9) < cooldown:
            return
        self._last_seen[cooldown_key] = now

        if person_type == "VIP":
            self._state = "GREETING"
            speech = self.vip_greeting.format(name=name)
            self._say(speech)
            self._emit_event("vip_greeting", name, msg.confidence)
        elif person_type == "BLACKLIST":
            self._state = "ALERTING"
            self._say(self.blacklist_announce)
            reason = f"⚠️ 偵測到黑名單人員：{name}（信心度 {msg.confidence:.2f}）請行員立即前往現場"
            self._send_notification(reason)
            self._emit_event("blacklist_alert", name, msg.confidence)
        else:
            self._state = "GREETING"
            self._say(self.visitor_greeting)
            self._emit_event("visitor_greeting", name, msg.confidence)

        self._log_visit(msg, person_type or "VISITOR")
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
            {"event": event, "name": name, "confidence": round(float(confidence), 3), "ts": int(time.time())},
            ensure_ascii=False,
        )
        self.event_pub.publish(String(data=payload))

    def _send_notification(self, text: str) -> None:
        if self.notify_backend != "telegram":
            self.get_logger().warn(f"🔕 通報（dry-run，未設定後端）: {text}")
            return
        if not self.telegram_bot_token or not self.telegram_chat_id:
            self.get_logger().error("✗ notify_backend=telegram 但 telegram_bot_token/telegram_chat_id 未設定")
            return
        # 推播走獨立執行緒，網路慢也不會卡住辨識 callback
        threading.Thread(target=self._telegram_send, args=(text,), daemon=True).start()

    def _telegram_send(self, text: str) -> None:
        url = f"https://api.telegram.org/bot{self.telegram_bot_token}/sendMessage"
        data = json.dumps({"chat_id": self.telegram_chat_id, "text": text}).encode("utf-8")
        req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
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
                    "ts INTEGER, person_uuid TEXT, person_name TEXT, person_type TEXT, confidence REAL)"
                )
        except Exception as e:
            self.get_logger().warn(f"✗ 訪客記錄資料庫初始化失敗（記錄停用）: {e}")
            self.enable_visit_log = False

    def _log_visit(self, msg: RecognitionResult, person_type: str) -> None:
        if not self.enable_visit_log:
            return
        try:
            with self._db_lock, sqlite3.connect(self.visit_log_path) as conn:
                conn.execute(
                    "INSERT INTO visits VALUES (?, ?, ?, ?, ?)",
                    (int(time.time()), msg.person_uuid, msg.person_name, person_type, float(msg.confidence)),
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
