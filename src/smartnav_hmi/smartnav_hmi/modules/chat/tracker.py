"""LLM 對話追蹤：串流、輪次計時、speech_text 與 llm_response 的去重。

從 hmi_server_node.py 搬出來。只吃 state + logger，不碰任何 ROS 訂閱/發布
本身（那些留在 hmi_server_node.py，callback 轉發到這裡的方法）。
"""

import threading
import time
from typing import List, Tuple

from std_msgs.msg import Bool, Empty, String

from ...core.constants import NON_WORD_RE


class ChatTracker:
    # 最後一個 token 之後多久內，仍視為 LLM 還在產生這一輪的回覆。
    # agent 多步驟決策時兩次 invoke 之間會停頓，值太小會把中段的句子放出去。
    STREAM_ACTIVE_SEC = 5.0
    # 扣住的句子等不到 llm_response 就直接顯示，避免 LLM 掛掉時訊息消失
    SPEECH_HOLD_SEC = 12.0

    def __init__(self, node, state):
        self._node = node
        self.state = state

        self._turn_lock = threading.Lock()
        self._turn_started = 0.0
        self._first_token_at = 0.0
        self._last_llm_text = ""
        self._last_llm_at = 0.0
        self._last_token_at = 0.0
        # LLM 這一輪期間收到的 speech_text 先扣住，等完整回覆到了再比對。
        # 不能當下就判斷：llm_stream 與 speech_text 是兩個話題，DDS 不保證
        # 跨話題順序，整句常比構成它的 token 更早抵達。
        self._pending_speech: List[Tuple[float, str]] = []

    def on_user_text(self, msg: String) -> None:
        self.state.set_partial("")
        self.state.add_message("user", msg.data)
        # 一句話送出去就是一輪的起點，用來算這輪 LLM 花了多久
        with self._turn_lock:
            self._turn_started = time.time()
            self._first_token_at = 0.0
        self.state.set_stream("")

    def on_partial_text(self, msg: String) -> None:
        self.state.set_partial(msg.data)

    def on_llm_response(self, msg: String) -> None:
        now = time.time()
        with self._turn_lock:
            started, first = self._turn_started, self._first_token_at
            self._turn_started = 0.0
            self._first_token_at = 0.0
            # llm_response 之後才送達的 speech_text 仍屬於這輪，留著比對
            self._last_llm_text = msg.data
            self._last_llm_at = now

        stats = {"chars": len(msg.data.strip())}
        if started:
            stats["total"] = round(now - started, 2)
            if first:
                # 首字延遲＝送出到第一個 token，中間可能包含工具呼叫
                stats["think"] = round(first - started, 2)
                stats["generate"] = round(now - first, 2)
                if now - first > 0.05:
                    stats["cps"] = round(stats["chars"] / (now - first), 1)

        # 扣住的句子先處理掉，屬於這段回覆的丟棄，其餘（例如導航播報插進來）保留
        self.flush_pending_speech(final_text=msg.data)
        self.state.set_stream("")
        self.state.add_message("robot", msg.data, stats=stats)

    def on_llm_stream(self, msg: String) -> None:
        now = time.time()
        with self._turn_lock:
            if not self._first_token_at:
                self._first_token_at = now
            self._last_token_at = now
        self.state.append_stream(msg.data)

    def on_llm_stream_reset(self, msg: Empty) -> None:
        """作廢目前串流中的灰字（這一輪只是去呼叫工具，不是要講給客戶聽的答案）

        只清畫面緩衝，**不**動 _turn_started / _first_token_at ——
        那兩個是整輪（含工具執行時間）的計時起點，清掉的話最終回覆的
        think/generate/cps 統計就沒了。也不碰 _pending_speech，
        那條線本來就由 on_llm_response 或看門狗負責。
        """
        self.state.set_stream("")

    def on_speech_text(self, msg: String) -> None:
        """語音輸出也記進對話流——但要濾掉 LLM 回覆的回音

        llm_service_node 把同一段回覆送兩個地方：逐句去標點後發到 speech_text
        給 TTS，最後再把完整版發到 llm_response。兩個都收的話，一次回答會變成
        「數個殘句 + 一句完整」重複顯示。

        判斷不能在收到當下做：llm_stream 與 speech_text 是兩個話題，DDS 不保證
        跨話題順序，實測整句常比構成它的 token 更早抵達，此時串流緩衝區還沒有
        那段字，比對就會漏。所以 LLM 這一輪期間先扣住，等 llm_response 來了再用
        完整回覆比對。非 LLM 來源（導航播報、語音測試頁）照舊立即顯示。
        """
        text = (msg.data or "").strip()
        if not text:
            return

        now = time.time()
        with self._turn_lock:
            active = now - self._last_token_at < self.STREAM_ACTIVE_SEC
            if active:
                self._pending_speech.append((now, text))
                return
            last_text, last_at = self._last_llm_text, self._last_llm_at

        # 不在 LLM 產生期間。仍可能是上一輪的尾句，給一小段寬限期再比對一次。
        if last_text and now - last_at < 15.0:
            if NON_WORD_RE.sub("", text) in NON_WORD_RE.sub("", last_text):
                return
        self.state.add_message("robot", text)

    def flush_pending_speech(self, final_text: str = "") -> None:
        """處理扣住的 speech_text

        傳入 final_text 時（llm_response 到了）：屬於回覆一部分的丟掉，其餘顯示。
        沒傳時（看門狗）：只放行等太久的，避免 LLM 掛掉導致訊息永遠不出現。
        """
        now = time.time()
        with self._turn_lock:
            if final_text:
                ready = [t for _ts, t in self._pending_speech]
                self._pending_speech = []
            else:
                ready = [t for ts, t in self._pending_speech if now - ts > self.SPEECH_HOLD_SEC]
                self._pending_speech = [(ts, t) for ts, t in self._pending_speech if now - ts <= self.SPEECH_HOLD_SEC]

        norm_final = NON_WORD_RE.sub("", final_text) if final_text else ""
        for text in ready:
            if norm_final and NON_WORD_RE.sub("", text) in norm_final:
                continue
            self.state.add_message("robot", text)

    def on_playback(self, msg: Bool) -> None:
        self.state.set_system(speaking=bool(msg.data))

    def on_llm_model(self, msg: String) -> None:
        self.state.set_system(llm_model=msg.data)
