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
機器人會變成不停說話的噪音源。但鏡頭前「沒人」（user_auth 的無人訊息）
時會解除訪客冷卻，下一位走進來的人一定會被問候（2026-11-03）。
"""

import json
import math
import sqlite3
import threading
import time
import urllib.request
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from smartnav_msgs.msg import UserIdentity

from smartnav_brain.user_types import UserType


class BankReceptionNode(Node):
    """銀行迎賓與安全通報的劇本節點"""

    def __init__(self):
        super().__init__("bank_reception_node")

        # ── 參數 ──────────────────────────────────────────────
        self.cooldown_sec = self.declare_parameter("cooldown_sec", 60.0).get_parameter_value().double_value
        self.visitor_cooldown_sec = (
            self.declare_parameter("visitor_cooldown_sec", 300.0).get_parameter_value().double_value
        )
        # ★ 2026-11-24 使用者：「迎賓詞太容易觸發了 一直說某某貴賓你好 打斷我們講話」。
        #   11/24 17:04～18:09（65 分）實測：VIP 迎賓 23 次（間隔中位數 85 秒）、一般迎賓 19 次。原因：
        #   ① 冷卻算的是「上次問候」不是「上次看到」→ 貴賓一直站著，每 60 秒就重問一次；
        #   ② 貴賓角度不好、沒過門檻的那幾幀被當成陌生訪客 → 播一般迎賓詞；
        #   ③ 鏡頭 10 秒沒看到臉（user_auth 的「無人」）就解除訪客冷卻，講話時一轉頭就會發生。
        #   改成（_should_greet）：同一位貴賓／黑名單要「離開鏡頭」regreet_absent_sec 才再問候；
        #   剛認出貴賓 vip_shadow_sec 內認不出的臉當成同一位、不另外問候；鏡頭前完全沒有臉
        #   visitor_absent_reset_sec 才算新訪客；最近 quiet_after_talk_sec 內有人講話／LLM 剛回答 → 不出聲
        #   （行員通知、事件、來訪紀錄照常）。cooldown_sec 仍是同一人兩次問候的最短間隔。
        self.regreet_absent_sec = float(self.declare_parameter("regreet_absent_sec", 180.0).value)
        self.vip_shadow_sec = float(self.declare_parameter("vip_shadow_sec", 60.0).value)
        self.visitor_absent_reset_sec = float(self.declare_parameter("visitor_absent_reset_sec", 30.0).value)
        self.quiet_after_talk_sec = float(self.declare_parameter("quiet_after_talk_sec", 15.0).value)
        # 對話中「延後」不是取消：先記著，安靜 quiet_after_talk_sec 後人還在就補講；一直講超過這麼久就不補了
        #   （重播 11/24 17:04～18:09：直接取消的話 65 分鐘只剩 1 次貴賓迎賓 —— 旁人講話也會擋掉真的貴賓）
        self.greet_pending_max_sec = float(self.declare_parameter("greet_pending_max_sec", 30.0).value)
        # 認不出的臉要連續出現這麼久才用一般迎賓詞：貴賓走近時常先「認不出」、下一幀才認出，
        # 不等的話會先說「您好歡迎光臨」、緊接著又說「某某貴賓您好」
        self.visitor_confirm_sec = float(self.declare_parameter("visitor_confirm_sec", 4.0).value)
        # ★ 2026-11-25：黑名單的「行員通知」和「問候」分開算。問候照 regreet_absent_sec（180 秒）；通知改成
        #   「離開鏡頭 blacklist_absent_sec 再出現＝新的一次到訪，每次到訪通知一次」，兩次通知至少隔
        #   blacklist_notify_sec。原本共用 180 秒的判斷，黑名單 3 分鐘內再走進來 Telegram 也不會發。
        self.blacklist_absent_sec = float(self.declare_parameter("blacklist_absent_sec", 30.0).value)
        self.blacklist_notify_sec = float(self.declare_parameter("blacklist_notify_sec", 60.0).value)
        # 額外的保守門檻。user_auth_node 的 recognized 已經判斷過一次，
        # 這裡只是再收緊一點，避免遠距離的低品質辨識觸發劇本。
        self.min_confidence = self.declare_parameter("min_confidence", 0.5).get_parameter_value().double_value
        # ★ 2026-11-03：黑名單專用門檻 blacklist_min_confidence（0.80）已移除。
        #   黑名單與 VIP 一樣只看 user_auth_node 的 recognition_threshold（0.70）——
        #   全系統只有一個辨識門檻，要調就調那一個。
        #   0.80 的實測問題：黑名單本人站在鏡頭前相似度 0.747~0.795，一次都沒通報。

        # ★ 2026-11-25：從平板啟動時沒有參數可帶 → 展示當天要調秒數就寫 ~/.smartnav/bank_reception.json
        self._apply_overrides(self._load_overrides())

        self.notify_backend = self.declare_parameter("notify_backend", "none").get_parameter_value().string_value
        self.telegram_bot_token = self.declare_parameter("telegram_bot_token", "").get_parameter_value().string_value
        self.telegram_chat_id = self.declare_parameter("telegram_chat_id", "").get_parameter_value().string_value

        # ★★ 2026-08-14：沒給參數時自己去讀密鑰檔。★★
        #
        # 問題：HMI 系統面板走 `run_node_cc.sh`，而它（在今天之前）**不傳任何參數**，
        # 所以從平板按「迎賓劇本」啟動 = `notify_backend` 停在預設的 "none"
        # = **Telegram 通報靜默關閉，只印 log**。而發表當天沒有鍵盤，
        # 面板就是唯一的啟動路徑 —— 等於黑名單通報永遠不會真的送出。
        #
        # 為什麼不是把權杖寫進面板指令：那會讓 Bot token 出現在
        # 行程指令列（`ps` 看得到）、log 裡、以及任何人截的圖裡。
        # 讓節點自己讀權限 600 的檔案是唯一乾淨的做法。
        #
        # ★ 只在「參數沒設」時才讀 —— 明確用 `--ros-args -p` 指定的值優先，
        #   否則現場想臨時關掉通報就沒辦法了。
        self._load_secrets_if_unset()

        self.enable_visit_log = self.declare_parameter("enable_visit_log", True).get_parameter_value().bool_value
        self.visit_log_path = (
            self.declare_parameter("visit_log_path", str(Path.home() / ".smartnav" / "visit_log.db"))
            .get_parameter_value()
            .string_value
        )

        self.identity_topic = (
            self.declare_parameter("user_identity_topic", "user_identity").get_parameter_value().string_value
        )

        self.vip_greeting = (
            self.declare_parameter(
                "vip_greeting",
                "{name}貴賓您好，歡迎蒞臨，需要為您帶位到貴賓室嗎",
            )
            .get_parameter_value()
            .string_value
        )
        self.visitor_greeting = (
            self.declare_parameter(
                "visitor_greeting",
                "您好，歡迎光臨智慧銀行，需要協助請跟我說",
            )
            .get_parameter_value()
            .string_value
        )
        # ★ 2026-11-03（使用者決定）：登入的平板是**另一台、放在別處給服務人員看的**，
        #   它只唸「出現了誰」，不唸客人那一側的迎賓詞／對話（前端 lib/tts.ts、lib/staffAlert.ts）。
        #   {name} 會換成辨識出的名字。空字串＝這一類不提示。
        self.staff_alert_vip = (
            self.declare_parameter("staff_alert_vip", "出現貴賓{name}").get_parameter_value().string_value
        )
        self.staff_alert_blacklist = (
            self.declare_parameter("staff_alert_blacklist", "出現黑名單{name}").get_parameter_value().string_value
        )
        self.staff_alert_visitor = (
            self.declare_parameter("staff_alert_visitor", "出現訪客").get_parameter_value().string_value
        )

        # ── 狀態 ──────────────────────────────────────────────
        self._last_seen: dict = {}        # key → 最後一次「問候」（訪客共用 "visitor"）
        self._seen_at: dict = {}          # 貴賓／黑名單 key → 最後一次「看到」（每一則都更新）
        self._notified_at: dict = {}      # 黑名單 key → 最後一次通知行員的時間
        self._visit_notified: dict = {}   # 黑名單 key → 這次到訪通知過了沒
        self._visit_greeted: dict = {}    # 貴賓／黑名單 key → 這次到訪問候過了沒
        self._last_face_at = -1e9         # 任何一張臉最後出現的時間
        self._last_vip_at = -1e9          # 最後一次認出貴賓
        self._last_talk_at = -1e9         # 最後一次有人講話（語音觸發／辨識出句子）或 LLM 回答
        self._unknown_since = None        # 這一輪「認不出的臉」從什麼時候開始出現
        self._pending = None              # 對話中延後的迎賓詞 (文字, 人的 key, 開始等的時間)
        self._state = "IDLE"
        self._db_lock = threading.Lock()
        self._init_visit_db()

        # ── 介面 ──────────────────────────────────────────────
        self.speech_pub = self.create_publisher(String, "speech_text", 10)
        # ★ 2026-11-03：只給行員聽的語音提示。speech_text 會被**每一台平板與機器人喇叭**唸出來（客人聽得到），
        #   所以黑名單提示改走這個話題：HMI 轉給「已登入的平板」才唸，機器人喇叭與迎賓平板都不出聲。
        self.staff_alert_pub = self.create_publisher(String, "staff_alert_speech", 10)
        self.event_pub = self.create_publisher(String, "brain_event", 10)
        self.identity_sub = self.create_subscription(UserIdentity, self.identity_topic, self.identity_callback, 10)
        # LLM 的通報工具走這個話題進來，讓「通報」只有一個出口
        self.notify_sub = self.create_subscription(String, "staff_notify_request", self.notify_callback, 10)
        # ★ 2026-11-24：對話中不出聲迎賓 —— 看語音辨識結果、LLM 回答、語音開始收音
        self.create_subscription(String, "/user_text", self._on_talk, 10)
        self.create_subscription(String, "llm_response", self._on_talk, 10)
        self.create_subscription(Bool, "/voice_triggered", self._on_voice_triggered, 10)

        self.get_logger().info(
            f"銀行迎賓劇本啟動：訂閱 {self.identity_topic}，"
            f"通報後端 {self.notify_backend}"
            f"{'（dry-run，只印 log 不實際送出）' if self.notify_backend != 'telegram' else ''}"
        )

    # ── 現場可調的秒數 ────────────────────────────────────────
    OVERRIDES_PATH = Path.home() / ".smartnav" / "bank_reception.json"
    OVERRIDE_KEYS = (
        "cooldown_sec", "visitor_cooldown_sec", "regreet_absent_sec", "vip_shadow_sec", "visitor_absent_reset_sec",
        "quiet_after_talk_sec", "greet_pending_max_sec", "visitor_confirm_sec", "blacklist_absent_sec",
        "blacklist_notify_sec",
    )

    def _load_overrides(self) -> dict:
        if not self.OVERRIDES_PATH.is_file():
            return {}
        try:
            # utf-8-sig：Windows 記事本存的檔案開頭有 BOM
            data = json.loads(self.OVERRIDES_PATH.read_text(encoding="utf-8-sig"))
        except (OSError, ValueError) as exc:
            self.get_logger().warning(f"⚠ {self.OVERRIDES_PATH} 讀不懂，全部用預設值：{exc}")
            return {}
        if not isinstance(data, dict):
            self.get_logger().warning(f"⚠ {self.OVERRIDES_PATH} 的內容要是 {{...}}，全部用預設值")
            return {}
        return data

    def _apply_overrides(self, data: dict) -> None:
        """只收上面列的秒數；寫錯的略過並講出來（寫錯檔案不能讓迎賓起不來，也不能讓人以為生效了）"""
        for key, value in data.items():
            if key not in self.OVERRIDE_KEYS:
                self.get_logger().warning(f"⚠ {self.OVERRIDES_PATH.name}：不認得 {key}，略過（可用：{', '.join(self.OVERRIDE_KEYS)}）")
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                self.get_logger().warning(f"⚠ {self.OVERRIDES_PATH.name}：{key} 要是 0 以上的數字，{value!r} 略過")
                continue
            setattr(self, key, float(value))
            self.get_logger().info(f"⚙ {key} = {float(value):g}（{self.OVERRIDES_PATH.name}）")

    # ── 密鑰檔 ────────────────────────────────────────────────
    SECRETS_PATH = Path.home() / ".smartnav" / "secrets" / "bank_reception.yaml"

    def _load_secrets_if_unset(self) -> None:
        """參數沒設時，從 ~/.smartnav/secrets/bank_reception.yaml 補上。

        檔案格式（權限請設 600）：

            notify_backend: telegram
            telegram_bot_token: "123456:AA..."
            telegram_chat_id: "987654321"

        ★ 這個檔案**不在打包裡**（`.smartnav` 一律排除），也不該進版控。
        ★ 讀不到就安靜地維持 dry-run —— 沒有密鑰檔仍然要能完整演一遍劇本。
          但**有檔案卻讀失敗**要出聲，那是設定錯誤不是沒設定。
        """
        if self.notify_backend and self.notify_backend != "none":
            return  # 已經用 --ros-args 明確指定了，尊重它
        if not self.SECRETS_PATH.exists():
            return
        try:
            import yaml

            with open(self.SECRETS_PATH, "r", encoding="utf-8") as fh:
                data = yaml.safe_load(fh) or {}
            # ★★ 2026-08-19：ROS 參數檔格式也要吃 ★★
            #
            # 實機踩到：密鑰檔是用 **ROS 參數檔格式**寫的
            #     /**:
            #       ros__parameters:
            #         notify_backend: telegram
            # 而這裡原本只讀扁平鍵，於是 `data.get("notify_backend")` 讀到 None，
            # **安靜地退回 dry-run** —— log 印「✓ 已從密鑰檔載入」但 backend=none、
            # token=無，看起來像「檔案裡沒東西」而不是「格式沒對上」。
            #
            # 兩種格式都合理（ROS 格式還能直接餵 `--params-file`），所以接受兩種，
            # 不要求使用者改檔案 —— 改檔案的那一刻權杖就有可能被複製到別的地方。
            for _k in ("/**", "/*", "**"):
                if isinstance(data.get(_k), dict) and "ros__parameters" in data[_k]:
                    data = data[_k]["ros__parameters"] or {}
                    break
            else:
                if "ros__parameters" in data and isinstance(data["ros__parameters"], dict):
                    data = data["ros__parameters"] or {}
            backend = str(data.get("notify_backend", "") or "")
            token = str(data.get("telegram_bot_token", "") or "")
            chat = str(data.get("telegram_chat_id", "") or "")
            if backend:
                self.notify_backend = backend
            if token and not self.telegram_bot_token:
                self.telegram_bot_token = token
            if chat and not self.telegram_chat_id:
                self.telegram_chat_id = chat
            # ★ 只印「有沒有讀到」，絕不印權杖本身 —— log 會被截圖、會被貼進交接文件
            self.get_logger().info(
                f"✓ 已從密鑰檔載入通報設定：backend={self.notify_backend}"
                f"、token={'有' if self.telegram_bot_token else '無'}"
                f"、chat_id={'有' if self.telegram_chat_id else '無'}"
            )
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"✗ 密鑰檔存在但讀取失敗（維持 dry-run）: {self.SECRETS_PATH} — {e}")

    # ==================================================================
    def identity_callback(self, msg: UserIdentity) -> None:
        # ★★ 2026-08-20：先擋掉「現在沒人」那一則 ★★
        #
        # user_auth_node 從 8/19 起會在連續 identity_timeout 秒沒看到人臉時
        # 補發一則「無人」（`_identity_timeout_tick`），讓平板把相似度與 VIP
        # 名字清掉。那一則長這樣：recognized=False、similarity=0.0、
        # user_name="Unknown"、**bbox=[0,0,0,0]**。
        #
        # 但下面第一條規則是「未通過辨識門檻的一律當一般訪客」，
        # 於是這則「沒人」會被當成**一位真訪客**：冷卻一過就
        # 對著空無一人的鏡頭播迎賓詞，還寫一筆假的來訪紀錄。
        # （明天段四驗「遮住鏡頭 -> 身份消失」時會當場發作，
        #   看起來會像人臉系統壞了，其實是這裡沒分辨。）
        #
        # 判別依據用 bbox 面積，因為它是唯一能區分兩種情況的欄位：
        #   真的有人臉  -> msg.bbox = face_msg.bbox，一定有面積
        #   沒人        -> user_auth 明確寫死 [0,0,0,0]
        # ★ 刻意**不**改 UserIdentity.msg 加欄位 —— 改介面要全 workspace
        #   重建，type hash 對不上時症狀是「節點照跑但收不到訊息且沒有錯誤」，
        #   上機前一天不值得冒這個險。
        if len(msg.bbox) >= 4 and (msg.bbox[2] - msg.bbox[0]) <= 0.0 and (msg.bbox[3] - msg.bbox[1]) <= 0.0:
            self._state = "IDLE"
            # ★★ 2026-11-03：鏡頭前沒人了 -> 解除一般訪客的冷卻 ★★
            #
            # 一般訪客共用一個 "visitor" 冷卻 300 秒，原意是「同一群人站著不要
            # 一直重複問候」。但它不分人，於是第一位陌生人被問候後，**接下來
            # 5 分鐘走進來的每一位陌生人都沒有迎賓語音**。實機 log（11/03）：
            # 41 次未認出的人臉只播了 2 次迎賓詞，中間鏡頭已經空過 4 次。
            #
            # 「無人」訊息是 user_auth 連續 identity_timeout 秒沒看到臉才發的，
            # 代表上一位已經離開 —— 下一位出現就是新訪客，應該要問候。
            # 有人一直站在鏡頭前時不會發「無人」，300 秒冷卻照樣擋住重複問候。
            # ★ 2026-11-24：「無人」只要 10 秒，講話時客人一轉頭就會發 → 改成看「鏡頭前完全沒臉多久」
            #   （下一張臉出現時，距離上一張臉 ≥ visitor_absent_reset_sec 才解除訪客冷卻，見 identity_callback）。
            return

        now = time.monotonic()
        face_gap = now - self._last_face_at
        self._last_face_at = now
        if face_gap >= self.visitor_absent_reset_sec:
            self._last_seen.pop("visitor", None)       # 鏡頭前空了一陣子 → 這是新來的客人
            self._unknown_since = None

        name = msg.user_name or "Unknown"
        try:
            ptype = UserType(msg.user_type.type).name
        except ValueError:
            ptype = "GUEST"

        # 未通過辨識門檻的一律當一般訪客 —— 不可以把「認不出來的人」
        # 當成黑名單，那會讓誤報率高到無法展示。
        if not msg.recognized:
            ptype = "GUEST"

        # 黑名單：recognized=True 就通報，門檻統一由 user_auth_node 的
        # recognition_threshold（0.70）決定（2026-11-03 移除 0.80 專用門檻，見 __init__）。
        # 實測依據（user_auth_node.py）：同一人 1 m 平均 0.7603、兩個不同的人 0.515；
        # 11/03 未認出者的最高分落在 0.586~0.639，都在 0.70 之下。
        if ptype == "VIP" and msg.similarity < self.min_confidence:
            return

        person = (msg.user_uuid or name) if ptype in ("VIP", "BLACKLIST") else "visitor"
        self._flush_pending(person, now)
        seen_gap = now - self._seen_at.get(person, -1e9)     # 要在 _should_greet 更新 _seen_at 之前取
        greet = self._should_greet(ptype, person, now)
        if ptype == "BLACKLIST" and self._should_notify_blacklist(person, seen_gap, now):
            # ★ 2026-11-03：以前現場播中性的「已通知行員前來協助」（speech_text，客人聽得到）。
            #   使用者決定改成「出現黑名單」、而且只在已登入（行員）的平板上唸 —— 客人那一側完全不出聲：
            #   當著本人的面播報會讓情況升溫，對誤判的人也是冒犯，所以絕不能走 speech_text／機器人喇叭。
            self._staff_alert(self.staff_alert_blacklist, name)
            # 送不出去就把「這次到訪通知過了」收回，60 秒冷卻後人還在會再通知一次
            self._send_notification(
                f"⚠️ 偵測到黑名單人員：{name}（相似度 {msg.similarity:.2f}）請行員立即前往現場",
                on_fail=lambda key=person: self._visit_notified.__setitem__(key, False))
            self._emit_event("blacklist_alert", name, msg.similarity)
        if not greet:
            return
        quiet = now - self._last_talk_at < self.quiet_after_talk_sec

        if ptype == "VIP":
            self._state = "GREETING"
            self._say_or_defer(self.vip_greeting.format(name=name), person, now, quiet)
            self._staff_alert(self.staff_alert_vip, name)
            self._emit_event("vip_greeting", name, msg.similarity)
        elif ptype == "BLACKLIST":
            self._state = "ALERTING"
            # ★ 2026-11-03（使用者決定）：客人那一側要跟「未知訪客」聽起來一模一樣 ——
            #   播一般訪客的迎賓詞（speech_text：迎賓平板＋機器人喇叭），而且遵守同一個「visitor」冷卻：
            #   前一位訪客剛被問候過的 5 分鐘內就不播，跟一般陌生人的行為完全相同，旁人聽不出差別。
            if now - self._last_seen.get("visitor", -1e9) >= self.visitor_cooldown_sec:
                self._last_seen["visitor"] = now
                self._say_or_defer(self.visitor_greeting, person, now, quiet)
        else:
            self._state = "GREETING"
            self._say_or_defer(self.visitor_greeting, person, now, quiet)
            self._staff_alert(self.staff_alert_visitor, name)
            self._emit_event("visitor_greeting", name, msg.similarity)

        self._log_visit(msg, ptype)
        self._state = "IDLE"

    # ── LLM 工具的通報請求（通報中樞入口）─────────────────────
    def notify_callback(self, msg: String) -> None:
        self.get_logger().info(f"📨 收到通報請求: {msg.data}")
        self._send_notification(msg.data)

    # ── 動作實作 ──────────────────────────────────────────────
    def _should_greet(self, ptype: str, key: str, now: float) -> bool:
        """這一張臉要不要觸發迎賓（含行員通知、事件、來訪紀錄）。同時更新「看到」與「問候」的時間。"""
        if ptype in ("VIP", "BLACKLIST"):
            gap = now - self._seen_at.get(key, -1e9)
            self._seen_at[key] = now
            if ptype == "VIP":
                self._last_vip_at = now
                self._unknown_since = None   # 剛才認不出的多半就是這位
            if gap >= self.regreet_absent_sec:
                self._visit_greeted[key] = False
            if self._visit_greeted.get(key, False):
                return False                 # 一直都在（中間沒離開夠久）→ 不再問候
            # ★ 2026-11-25：冷卻中回來的不會整次到訪都漏掉 —— 冷卻一過、人還在就問候
            #   （regreet_absent_sec 調得比 cooldown_sec 小時才會遇到，展示日的建議值就是）
            if now - self._last_seen.get(key, -1e9) < self.cooldown_sec:
                return False
            self._visit_greeted[key] = True
            self._last_seen[key] = now
            return True
        if now - self._last_vip_at < self.vip_shadow_sec:
            return False                     # 剛認出貴賓：多半是同一位角度不好，不當新訪客問候
        if self._unknown_since is None:
            self._unknown_since = now
        if now - self._unknown_since < self.visitor_confirm_sec:
            return False                     # 先等幾秒，給辨識一個機會認出是貴賓
        if now - self._last_seen.get("visitor", -1e9) < self.visitor_cooldown_sec:
            return False
        self._last_seen["visitor"] = now
        return True

    def _should_notify_blacklist(self, key: str, seen_gap: float, now: float) -> bool:
        """黑名單要不要（再）通知行員：每次到訪一次，與有沒有被問候過無關。

        冷卻中走進來的不會漏掉：這次到訪還沒通知過，冷卻一過、人還在就通知。
        """
        if seen_gap >= self.blacklist_absent_sec:
            self._visit_notified[key] = False
        if self._visit_notified.get(key, False) or now - self._notified_at.get(key, -1e9) < self.blacklist_notify_sec:
            return False
        self._visit_notified[key] = True
        self._notified_at[key] = now
        return True

    def _say_or_defer(self, text: str, person: str, now: float, quiet: bool) -> None:
        if not quiet:
            self._say(text)
            return
        self._pending = (text, person, now)
        self.get_logger().info(
            f"🤫 {now - self._last_talk_at:.0f} 秒前有人講話／LLM 剛回答：迎賓詞先不講，安靜 "
            f"{self.quiet_after_talk_sec:.0f} 秒後人還在就補（最多等 {self.greet_pending_max_sec:.0f} 秒）")

    def _flush_pending(self, person: str, now: float) -> None:
        """延後的迎賓詞：同一個人還在鏡頭前、而且已經安靜夠久 → 補講；等太久（一直在講話）→ 不講了。"""
        if self._pending is None:
            return
        text, who, since = self._pending
        if now - since > self.greet_pending_max_sec:
            self._pending = None
            self.get_logger().info("🤫 一直在對話，延後的迎賓詞不補了")
        elif who == person and now - self._last_talk_at >= self.quiet_after_talk_sec:
            self._pending = None
            self._say(text)

    def _on_talk(self, _msg) -> None:
        self._last_talk_at = time.monotonic()

    def _on_voice_triggered(self, msg: Bool) -> None:
        if msg.data:
            self._last_talk_at = time.monotonic()

    def _say(self, text: str) -> None:
        self.speech_pub.publish(String(data=text))
        self.get_logger().info(f"🗣️ 迎賓語音: {text}")

    def _staff_alert(self, template: str, name: str) -> None:
        """只給服務人員平板（已登入）唸的提示；客人那一側聽不到（見 staff_alert_pub）"""
        if not template:
            return
        text = template.format(name=name if name and name != "Unknown" else "")
        self.staff_alert_pub.publish(String(data=text))
        self.get_logger().info(f"🔒 行員提示（只在已登入的平板唸）: {text}")

    def _emit_event(self, event: str, name: str, confidence: float) -> None:
        payload = json.dumps(
            {"event": event, "name": name, "confidence": round(float(confidence), 3), "ts": int(time.time())},
            ensure_ascii=False,
        )
        self.event_pub.publish(String(data=payload))

    def _send_notification(self, text: str, on_fail=None) -> None:
        if self.notify_backend != "telegram":
            # dry-run 是刻意的預設：沒設 token 也能完整演一遍劇本，
            # log 裡看得到「本來會送出什麼」。
            self.get_logger().warn(f"🔕 通報（dry-run，未設定後端）: {text}")
            return
        if not self.telegram_bot_token or not self.telegram_chat_id:
            self.get_logger().error("✗ notify_backend=telegram 但 telegram_bot_token/telegram_chat_id 未設定")
            return
        # 推播走獨立執行緒：網路慢也不能卡住辨識 callback，
        # 卡住的話後面的人臉事件會整個塞住。
        threading.Thread(target=self._telegram_send, args=(text, on_fail), daemon=True).start()

    # 熱點瞬斷時重送：馬上、3 秒後、再 6 秒後
    TELEGRAM_RETRY_DELAYS = (0.0, 3.0, 6.0)

    def _telegram_send(self, text: str, on_fail=None) -> None:
        # 用標準庫的 urllib 而不是 requests：零新依賴，Pi 上不用再 pip install
        url = f"https://api.telegram.org/bot{self.telegram_bot_token}/sendMessage"
        data = json.dumps({"chat_id": self.telegram_chat_id, "text": text}).encode("utf-8")
        req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
        for i, delay in enumerate(self.TELEGRAM_RETRY_DELAYS, 1):
            if delay:
                time.sleep(delay)
            try:
                with urllib.request.urlopen(req, timeout=5) as resp:
                    if resp.status == 200:
                        self.get_logger().info("✓ Telegram 通報已送出")
                        return
                    self.get_logger().error(f"✗ Telegram 回應異常: HTTP {resp.status}（第 {i} 次）")
            except Exception as e:
                self.get_logger().error(f"✗ Telegram 通報失敗（第 {i} 次）: {e}")
        if on_fail is not None:
            on_fail()

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
                    (int(time.time()), msg.user_uuid, msg.user_name, person_type, float(msg.similarity)),
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
