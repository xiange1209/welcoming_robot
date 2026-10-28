"""LLM 對話追蹤：串流、輪次計時、speech_text 與 llm_response 的去重。

從 hmi_server_node.py 搬出來。只吃 state + logger，不碰任何 ROS 訂閱/發布
本身（那些留在 hmi_server_node.py，callback 轉發到這裡的方法）。
"""

import json
import threading
import time
from typing import Dict, List, Tuple

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
        # LLM 節點回報的「首字之前」耗時拆解 {"retrieve": 秒, "tool": 秒}，每輪開始時清掉
        self._timing: Dict[str, float] = {}
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
            self._timing = {}
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
            timing = dict(self._timing)

        stats = {"chars": len(msg.data.strip())}
        if started:
            stats["total"] = round(now - started, 2)
            if first:
                # 首字延遲＝送出到第一個 token，中間可能包含檢索與工具呼叫
                stats["think"] = round(first - started, 2)
                stats["generate"] = round(now - first, 2)
                if now - first > 0.05:
                    stats["cps"] = round(stats["chars"] / (now - first), 1)
                stats.update(self.split_think(stats["think"], timing))

        # 扣住的句子先處理掉，屬於這段回覆的丟棄，其餘（例如導航播報插進來）保留
        self.flush_pending_speech(final_text=msg.data)
        self.state.set_stream("")
        self.state.add_message("robot", msg.data, stats=stats)

    @staticmethod
    def split_think(think: float, timing: Dict[str, float]) -> Dict[str, float]:
        """把「等待首字」（think：客人說完 -> 第一個字）拆成 檢索／工具／TTFT／其他。

        TTFT 是 LLM 節點量的、模型自己的首 token 時間（請求送出 -> 第一個 token），不含檢索與工具。
        「其他」是扣掉這三項之後剩下的：ROS 傳遞、提示組裝、節點排程。扣出負數（兩邊時鐘起點略有差）就當 0。
        沒有 LLM 節點的回報就不拆（只留總數）；舊版節點只回報檢索與工具、沒有 ttft 時，
        剩下的整段仍記成 model（它包含了 TTFT 與「其他」，無法再分）。
        """
        if not timing:
            return {}
        retrieve = max(float(timing.get("retrieve", 0.0)), 0.0)
        tool = max(float(timing.get("tool", 0.0)), 0.0)
        out = {"retrieve": round(retrieve, 2)}
        if tool >= 0.05:
            out["tool"] = round(tool, 2)
        if "ttft" in timing:
            ttft = max(float(timing["ttft"]), 0.0)
            out["ttft"] = round(ttft, 2)
            out["other"] = round(max(think - retrieve - tool - ttft, 0.0), 2)
        else:
            out["model"] = round(max(think - retrieve - tool, 0.0), 2)
        return out

    def on_llm_timing(self, msg: String) -> None:
        """LLM 節點回報首字之前的耗時拆解（累計值，取最後一筆）

        正常情況它比 llm_response 早到很多。但兩個話題之間 DDS 不保證順序，
        若這一輪的 llm_response 已經處理完，就回頭補在那則回覆的統計上。
        """
        try:
            timing = {k: float(v) for k, v in json.loads(msg.data).items()}
        except (ValueError, TypeError, AttributeError):
            return
        with self._turn_lock:
            if self._turn_started != 0.0:
                self._timing = timing  # 這一輪還在進行，等 llm_response 來取
                return
            # 這一輪的 llm_response 已經處理完才收到：補在最後一則機器人回覆上
            late = bool(self._last_llm_at) and time.time() - self._last_llm_at < 3.0
        if late:
            def make_extra(s):
                if "think" not in s:
                    return {}
                extra = self.split_think(s["think"], timing)
                if "ttft" in extra:
                    extra["model"] = None  # 先前沒有 TTFT 時記下的「模型」殘值作廢，改用 TTFT／其他
                return extra

            self.state.patch_last_robot_stats(make_extra)

    def on_llm_stream(self, msg: String) -> None:
        now = time.time()
        with self._turn_lock:
            # 這一輪的 llm_response 已經處理完（_turn_started 歸零），卻又收到 token：
            # 是跨話題亂序晚到的尾巴。此時 on_llm_response 已經清空串流緩衝，
            # 再接上去只會在畫面上留下一則重複、永遠不會被清掉的灰字。
            # 新的一輪一定先經過 on_user_text（會重設 _turn_started），不會被誤擋。
            if self._turn_started == 0.0 and self._last_llm_at and now - self._last_llm_at < 2.0:
                return
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
        """語音輸出：LLM 回覆的回音要濾掉；其餘不是 LLM 推論的語音記成「隱藏」訊息

        llm_service_node 把同一段回覆送兩個地方：逐句去標點後發到 speech_text
        給 TTS，最後再把完整版發到 llm_response。兩個都收的話，一次回答會變成
        「數個殘句 + 一句完整」重複顯示。

        判斷不能在收到當下做：llm_stream 與 speech_text 是兩個話題，DDS 不保證
        跨話題順序，實測整句常比構成它的 token 更早抵達，此時串流緩衝區還沒有
        那段字，比對就會漏。所以 LLM 這一輪期間先扣住，等 llm_response 來了再用
        完整回覆比對。

        非 LLM 來源（黑名單公告、導航播報「我要倒車…」、迎賓詞、語音測試頁）：
        對話框只放客人說的話與 LLM 的回覆，所以這些不顯示（hidden）；但仍要留在清單，
        因為平板朗讀是讀清單的最新機器人訊息，拿掉就不唸了。
        """
        text = (msg.data or "").strip()
        if not text:
            return

        now = time.time()
        with self._turn_lock:
            last_text, last_at = self._last_llm_text, self._last_llm_at
            # 這一輪的 llm_response 已經到了（on_llm_response 會把 _turn_started 歸零）。
            # 此時才抵達的句子是「比完整回覆晚到」的尾句，不會再有人幫它比對了：
            # 扣住的話要等看門狗放行，而且舊版放行時完全不比對 ->
            # 完整回覆下面多出一則標點後面那一段。直接往下和完整回覆比對。
            response_arrived = self._turn_started == 0.0 and bool(last_text)
            active = now - self._last_token_at < self.STREAM_ACTIVE_SEC
            if active and not response_arrived:
                self._pending_speech.append((now, text))
                return

        # 不在 LLM 產生期間，或這一輪的完整回覆已到。仍可能是這一輪的尾句，給一小段寬限期比對。
        if last_text and now - last_at < 15.0:
            if NON_WORD_RE.sub("", text) in NON_WORD_RE.sub("", last_text):
                return
        self.state.add_message("robot", text, hidden=True)

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
                # 看門狗放行也要和最近一次的完整回覆比對，否則晚到的尾句會在 12 秒後重複出現
                final_text = self._last_llm_text

        norm_final = NON_WORD_RE.sub("", final_text) if final_text else ""
        for text in ready:
            if norm_final and NON_WORD_RE.sub("", text) in norm_final:
                continue
            self.state.add_message("robot", text, hidden=True)

    def on_playback(self, msg: Bool) -> None:
        self.state.set_system(speaking=bool(msg.data))

    def on_llm_model(self, msg: String) -> None:
        self.state.set_system(llm_model=msg.data)
