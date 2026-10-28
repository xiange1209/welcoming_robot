#!/usr/bin/env python3

"""LLM 對話服務節點（迎賓對話）

職責：聽懂客人說的話（user_text）、用 Ollama 的模型回話（llm_response／speech_text），
需要動作時呼叫 tools.py 的工具提出請求。本節點不做業務決策：
  - 能不能帶位、帶去哪：smartnav_brain 的 nav_arbiter 判斷（本節點只送 RequestNavigation）
  - 通報行員的實際送出：smartnav_brain 的 bank_reception_node
  - 地圖與地點的建立、切換：HMI 的管理功能，不在這裡
"""

import json
import re
import requests
import threading
import time
from collections import deque
from pathlib import Path
from typing import List, Any
from langchain_ollama import ChatOllama
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.callbacks import BaseCallbackHandler
from langchain_core.messages import BaseMessage, AIMessage, HumanMessage, ToolMessage
from langchain_core.runnables import RunnablePassthrough

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Empty, String
from action_msgs.msg import GoalStatus

from smartnav_msgs.srv import ListWaypoints
from smartnav_msgs.msg import AccessContext
from smartnav_msgs.action import RequestNavigation
from smartnav_llm.llm_utils import get_config_path
from smartnav_llm import customer_prompt, nav_intent, tool_gate
from smartnav_llm.knowledge_base import KnowledgeBase, OllamaEmbedder
from smartnav_llm.tools import make_tools


class RosStreamHandler(BaseCallbackHandler):
    """自訂串流處理器：將 LLM 回應的 Token 即時發布到 ROS 2 Topic

    ★ 2026-08-14：加入「暫扣」機制。

    這個 handler 掛在 ChatOllama 上，所以 agent 迴圈的**每一輪**推論都會觸發它，
    包含「決定要呼叫工具」那一輪。qwen2.5:3b 在呼叫工具前常會先講一段話
    （「我會嘗試為您安排與櫃檯行員的連線，請問是否有特定櫃檯…」），
    那段話原本會直接進 /speech_text 被平板唸出來，等工具跑完之後真正的答案
    再唸一次 —— 現場聽起來就是機器人把同一件事講兩遍，而且第一遍還會問一個
    它自己不會等答案的問題。

    2026-08-14 實測（見 ~/LLM工具選擇_20260814.md）：「我要找真人服務」一題的
    /speech_text 有 10 段，前 5 段全是這種旁白。使用者觀察到的
    「模型用講的而不是真的呼叫工具」也是同一件事 —— 工具其實有呼叫
    （節點 log 有 🛠️），只是旁白先被唸出去了。

    作法：每一輪開始前呼叫 begin_turn()，句子先扣在 held 裡；invoke() 回來後
    由 agent 迴圈判斷這一輪有沒有 tool_calls：
      有   → drop_held()  旁白丟掉，不唸
      沒有 → flush_held() 這是最終答案，才唸出來
    /llm_stream 的逐字串流不受影響，畫面字幕仍然是即時的。
    """

    def __init__(self, stream_publisher, speech_text_publisher, convert=None):
        super().__init__()
        self.stream_publisher = stream_publisher
        self.speech_text_publisher = speech_text_publisher
        # 簡轉繁。逐 token 轉沒有意義（一個字轉一個字），所以只在
        # 「整句」送出去的時候轉——畫面上的串流字幕會短暫是簡體，
        # 但送進 TTS 與最終回覆的都是繁體。
        self.convert = convert or (lambda s: s)
        # ★★ 2026-08-26：數字之間的 . : , 不是標點 ★★
        #
        # 舊版把半形句點、冒號、逗號一律當成斷句符，而銀行機器人講的話裡
        # 這三個符號幾乎都夾在數字中間。實測（模擬逐 token 串流，
        # 見 ~/maprun/tools_0826/tts_split_check.py）：
        #
        #   「營業時間是上午9:00到下午3:30」-> 唸成「上午9」「00到下午3」「30」
        #   「年利率3.5%，匯率31.25」      -> 唸成「年利率3」「5」「匯率31」「25」
        #   「餘額是1,250,000元」           -> 唸成「1」「250」「000元」
        #
        # 而 config/bank_faq.txt:4 的營業時間就寫成 09:00-15:30 ——
        # 客人問「幾點開門」正好會踩到。
        #
        # ★ 這個 bug 特別難察覺：HMI 畫面顯示的是 /llm_response（完整正確的
        #   final_reply），平板唸的是 /speech_text（被切碎的）。**畫面對、聲音錯**。
        #
        # 全形標點（，。！？；：）不加 lookaround —— 它們不會出現在數字中間。
        self.split_pattern = re.compile(r"((?<!\d)[,.:;](?!\d)|[!?，。！？；：\n])")
        # 千分位逗號先拿掉：1,250,000 -> 1250000，TTS 才唸得出「一百二十五萬」
        self.thousands_pattern = re.compile(r"(?<=\d),(?=\d\d\d)")
        # ★ . : % 要保留（小數、時刻、百分比），其餘符號照舊刪掉。
        #   `-` 不在保留集合裡，仍會被第一個分支刪掉；`_` 屬於 \w，要另外列出。
        self.clean_pattern = re.compile(r"[^\w一-龥\s.:%]|_")
        self.buffer = ""
        self.current_sentence = ""
        self.held: List[str] = []
        self.hold = True
        # 這一整輪（跨 agent 迴圈的所有推論）是否已經送出過第一個 token。
        # 給耗時拆解用：第一個字之前的工具執行時間才算進「首字延遲」。
        self.seen_token = False
        # TTFT（time to first token）：把請求送給模型 -> 第一個 token 回來，只含模型這一段，
        # 不含檢索與工具。取這一整輪第一個吐出 token 的那次推論（begin_turn 到第一個 token）。
        self.round_start = None
        self.ttft = None

    def start_turn(self) -> None:
        """一輪對話開始（客人說完一句話）。和 begin_turn 不同：它每次推論都會呼叫，這個一輪只呼叫一次"""
        self.seen_token = False
        self.ttft = None

    def begin_turn(self, hold: bool = True) -> None:
        """開始新一輪推論：清掉上一輪的殘留與暫扣區

        Args:
            hold: True 表示這一輪的句子先扣著不唸（還不知道會不會去呼叫工具）。

        ★★ 2026-08-17 修正：原本只在第 0 輪扣，註解寫「第二輪之後幾乎一定是最終答案」
        —— **實機 log 證偽了這句話**。`LLM修復_20260817.md` §1.3 的 8/17 log：

            iteration 0  旁白 -> 呼叫 global_localization_tool
            iteration 1  旁白（「全域定位已經完成，現在我可以開始導航了…」）-> 又呼叫工具

        那句「全域定位已經完成…」就是 iteration 1 的旁白，客戶**真的聽到了**，
        而它根本不是最終答案（7 分鐘後真正的回覆是完全不同的另一句）。
        所以只扣第 0 輪擋不住 —— 現在每一輪都扣。

        代價：最終答案那一輪不再逐句串流，要整輪生成完才開口（qwen2.5:3b 約多等
        2~5 秒）。這是刻意的取捨：**寧可晚幾秒開口，也不要先唸一句假的**。
        要換回舊行為（低延遲、但可能唸旁白）：`-p hold_all_iterations:=false`。
        """
        self.buffer = ""
        self.current_sentence = ""
        self.held = []
        self.hold = hold
        self.round_start = time.monotonic()  # 緊接著就是 invoke()，TTFT 從這裡起算

    def _emit(self, sentence: str) -> None:
        if self.hold:
            self.held.append(sentence)
        else:
            self.speech_text_publisher.publish(String(data=self.convert(sentence)))

    def flush_held(self) -> None:
        """這一輪是最終答案，把暫扣的句子依序送去語音"""
        for sentence in self.held:
            self.speech_text_publisher.publish(String(data=self.convert(sentence)))
        self.held = []

    def drop_held(self) -> None:
        """這一輪只是去呼叫工具，旁白不要唸出來"""
        self.held = []

    def _clean(self, text: str) -> str:
        """把一句話整理成適合 TTS 唸的形式。

        ★ 2026-08-26：順序很重要 —— 千分位逗號要**先**拿掉。
          若先跑 clean_pattern，1,250,000 的逗號會被刪成 "1 250 000"（三個數），
          TTS 就唸成「一、二百五十、零」。
        """
        return " ".join(self.clean_pattern.sub("", self.thousands_pattern.sub("", text)).split())

    def on_llm_new_token(self, token: str, **kwargs: Any) -> None:
        if not token:
            return

        self.stream_publisher.publish(String(data=token))
        if not self.seen_token and self.round_start is not None:
            self.ttft = time.monotonic() - self.round_start
        self.seen_token = True

        self.buffer += token
        parts = self.split_pattern.split(self.buffer)
        self.buffer = parts.pop() if parts else ""

        # ★★ 2026-08-26：buffer 剛好停在分隔符上時，退回去等下一個 token ★★
        #
        # split_pattern 用 (?!\d) 判斷「這個冒號是時刻還是標點」，而 lookahead
        # 需要**下一個字元**才能判斷。但 token 是一個一個到的：
        # buffer 是「...上午9:」時 0 還沒到，lookahead 看到的是字串結尾，
        # 判定為標點 -> 照樣切錯。退回去等一個 token 就看得到 0。
        #
        # 若那個分隔符真的是句尾（後面沒有字了），on_llm_end 會把殘留 flush
        # 出去，不會漏掉。代價只是最後一句晚一個 token 送出。
        if parts and self.split_pattern.fullmatch(parts[-1]):
            self.buffer = parts.pop() + self.buffer

        for item in parts:
            if not item:
                continue

            if self.split_pattern.match(item):
                speech_sentence = self._clean(self.current_sentence)
                if speech_sentence:
                    self._emit(speech_sentence)
                self.current_sentence = ""
            else:
                self.current_sentence += item

    def on_llm_end(self, response: Any, **kwargs: Any) -> None:
        remaining_text = self._clean(self.current_sentence + self.buffer)
        if remaining_text:
            self._emit(remaining_text)
        self.buffer = ""
        self.current_sentence = ""


class ConversationMemory:
    """對話記憶管理類別"""

    def __init__(self, max_history: int = 2):
        # 記憶對話組數
        self.history: deque = deque(maxlen=max_history * 2)

    def add_message(self, message: BaseMessage) -> None:
        self.history.append(message)

    def get_history_messages(self) -> List[BaseMessage]:
        return list(self.history)

    def clear(self) -> None:
        """清空所有對話記憶"""
        self.history.clear()


class LLMServiceNode(Node):
    """LLM 服務節點類別"""

    def __init__(self):
        super().__init__("llm_service_node")

        # 宣告與讀取參數
        # 預設是目前跑 Ollama 的那台機器（192.168.11.156）。換網路或換機器時用
        # `-p ollama_base_url:=http://<IP>:11434` 覆寫；網段不同就連不到（節點仍會啟動，對話時才失敗）。
        # ★ 沒有 launch 檔會覆寫這個值 —— HMI 經 run_node_cc.sh 直接 `ros2 run` 啟動，
        #   參數來源鏈上只有這一環，所以這個預設值就是實際生效的值。
        self.ollama_base_url = (
            self.declare_parameter("ollama_base_url", "http://192.168.11.156:11434").get_parameter_value().string_value
        )
        # ★ 2026-08-31：等導航堆疊的**總期限**（見 _wait_for_services）。
        #   等不到就降級啟動——對話一定可用，只有導航工具會在被呼叫時失敗。
        #   設 0 或負數＝無限等（舊行為，不建議）。
        self.services_wait_timeout_sec = float(
            self.declare_parameter("services_wait_timeout_sec", 45.0).get_parameter_value().double_value
        )
        # 2026-10-28 起預設改用 llama3.1:8b（支援工具呼叫）。2026-10-28 用它跑過一輪端到端對話（Ollama 在
        # 192.168.11.156）：帶位、權限、FAQ、通報、時間查詢的流程都通；已知的行為問題是「你好」會多叫
        # list_waypoints_tool 並把地點 ID 唸出來、「幾點關門」會誤叫時間工具、通報原因會亂碼。
        # 先前用過的 qwen2.5:3b 與 orieg/gemma3-tools 的比較（延遲、工具觸發）已不適用，不再保留。
        self.model_name = self.declare_parameter("model_name", "llama3.1:8b").get_parameter_value().string_value
        self.temperature = self.declare_parameter("temperature", 0.0).get_parameter_value().double_value
        # ★★ 2026-08-19：限制回覆長度 ★★
        #
        # 使用者回報「LLM 回覆太多字」。查下來**不是 RAG 太多** ——
        # `config/bank_faq.txt` 只有 1147 bytes / 26 行，塞不爆任何上下文。
        # 真因是兩個都沒設限：
        #   1. ChatOllama 沒有給 `num_predict`，生成長度**完全沒有上限**
        #   2. 系統提示詞 79 行裡**沒有任何一條講回覆長度**
        #
        # 為什麼長回覆在這個專案特別糟（不只是囉唆）：
        #   - 出聲端是平板 TTS。120 字 = 唸 26 秒，客人站在那裡等
        #   - 平板固定在車上，唸愈久麥克風收自己的聲音就愈久
        #   - `hold_all_iterations` 之後最終答案要整輪生成完才開口，
        #     生成愈長，開口愈慢
        #
        # 120 token 對中文約 80~100 字，夠講完「營業時間 + 一句補充」。
        # ★ 這是**硬上限**，會直接截斷。真正該讓它短的是提示詞裡的規則，
        #   這個只是保險 —— 所以留得比期望長度寬一點。
        #
        # ★★ 2026-08-20 修正：120 -> 256 ★★
        #
        # 120 是照「最終答案該多長」訂的，但這個上限**套在每一輪**上，
        # 包含「決定要呼叫哪個工具」的那幾輪（`agent_chain` 只建一次，
        # 見 :665 的 bind_tools，迴圈每輪都用同一個）。
        #
        # 那幾輪的輸出是「旁白 + 工具呼叫 JSON」。qwen2.5:3b 很愛先講
        # 一段旁白（這正是 hold_all_iterations 要擋的東西），旁白 50~80
        # token 之後才輪到 JSON —— 120 一到就從 JSON 中間切斷。
        #
        # 被切斷的後果**不是報錯，是靜默降級**：
        #   JSON 不完整 -> Ollama 解不出 tool_calls -> response.tool_calls 是空的
        #   -> 迴圈 :771 判定「沒有工具呼叫 = 這是最終答案」
        #   -> **把那段旁白當成答案唸給客人聽，工具永遠不會執行**
        # 症狀會是「它嘴上說要幫我查，然後就沒有然後了」。
        #
        # 256 對中文約 170~200 字，旁白加 JSON 放得下。真正讓回覆變短的是
        # 系統提示詞裡那條「40 字 / 最多兩句」的規則，這個只是保險。
        # ★ 下面迴圈裡加了截斷偵測，真的撞到上限會 WARN，不會再靜默。
        self.num_predict = self.declare_parameter("num_predict", 256).get_parameter_value().integer_value
        # ★★ 2026-10-15：明確設定上下文長度 num_ctx（原本沒設，吃 Ollama 的預設）★★
        #
        # 筆電的 Ollama 日誌（2026-10-15 17:11）寫著 `vram-based default context total_vram="4.0 GiB"
        # default_num_ctx=4096`：4 GB 顯存的預設上下文只有 4096 tokens。而銀行模式光是系統提示詞
        # （約 2,524 tokens）加 13 個工具描述（約 1,275 tokens）就約 3,800 tokens（T5 報告 §6.3，
        # Qwen tokenizer 估算、誤差約 ±5%），再加歷史、工具結果與 256 的生成上限，一定超過 4096。
        # 超過時 Ollama 會截掉輸入；被截掉的若是系統提示詞的規則（例如「查不到就說查不到」），
        # 規則就靜默失效。截斷只會印在 Ollama 的伺服器日誌（truncating input prompt），這個節點看不到。
        # 上面的 token 數是用 Qwen tokenizer 估的；llama3.1:8b 的 tokenizer 不同，實際數字未重算。
        # 8B 模型的權重（Q4 約 4.9 GB）在 4 GB 顯存放不下，Ollama 會把一部分層放在 CPU（變慢），
        # 這時 num_ctx 越大越吃記憶體——若延遲不能接受，先降 num_ctx 或換顯存更大的機器。
        self.num_ctx = self.declare_parameter("num_ctx", 8192).get_parameter_value().integer_value
        # ★★ 2026-10-21：模型常駐顯存（keep_alive）與啟動暖機 ★★
        #
        # 筆電 Ollama 日誌（server-1.log，10/20）：當天第一次載入 46.83 秒（14:13，從硬碟讀模型），
        # 之後每次閒置超過 5 分鐘就被卸載、下一題重載 2.3~3.8 秒——共 9 次，正好對上車上 log 裡
        # 「閒置 >5 分後的第一題 5.1~8.2 秒」那 9 輪（間隔 <5 分的一律 <3.2 秒）。Ollama 預設 keep_alive 5 分鐘。
        # "-1" = 常駐到 Ollama 重啟（測試／展示日筆電專門跑這個）；也可給 "60m" 這類時間；"" = 用 Ollama 預設。
        # 要釋放顯存：執行 Ollama 的那台機器上跑 `ollama stop llama3.1:8b`。
        self.keep_alive = self.declare_parameter("keep_alive", "-1").get_parameter_value().string_value
        # 節點啟動後在背景先問一次（結果丟掉），把模型與系統提示載進顯存，第一位客人不用等 47 秒。
        self.warmup_on_start = self.declare_parameter("warmup_on_start", True).get_parameter_value().bool_value
        # ★★ 2026-10-28：提示快取保溫（keep_warm_sec，<=0 關閉）★★
        #
        # 實測（筆電 Ollama 0.30.4、llama3.1:8b、同一份 1,513 token 提示）：兩次請求相隔 ≤6 秒，
        # 第二次 prefill 只要 33 毫秒（快取命中）；相隔 ≥8 秒就失效，prefill 1.9~2.7 秒，
        # 閒置 3 分鐘 4.3 秒、6 分鐘 5.1 秒。客人講話的節奏是 15~60 秒一句，所以幾乎每一句都踩在失效區，
        # HMI 上的首字延遲 5~9 秒就是這個。快取壽命這麼短的原因在筆電那邊，從這裡看不到。
        #
        # 對策：句子之間每 keep_warm_sec 秒送一個只含系統提示、num_predict=1 的極小請求（熱的時候約 35 毫秒）。
        # 實測每 4 秒一次保溫 60 秒後，真實提問 prefill 94 毫秒、整個請求 1.1 秒。
        # ★ 間隔必須小於快取壽命（約 6~8 秒）：每 10 秒保溫實測無效，每個保溫請求自己都沒命中（各 2.5 秒）。
        # 代價：Ollama 那台每 4 秒多算一次，GPU 不會進省電。
        self.keep_warm_sec = float(self.declare_parameter("keep_warm_sec", 4.0).get_parameter_value().double_value)
        self._llm_busy = False  # 主迴圈正在等模型推論；保溫這時不送，免得和真實請求搶 GPU
        self._ollama_last = time.monotonic()  # 最近一次對聊天模型的請求（真實或保溫）結束的時刻

        # ── 簡體轉繁體 ───────────────────────────────────
        # 系統提示詞早就寫了「回覆一律使用繁體中文」，但小模型會忽略它：
        # 2026-08-10 實測 qwen2.5:3b 回「下午 3 点 57 分」「柜台位置」。
        # 提示詞是請求，轉換才是保證，所以在輸出端強制轉一次。
        # s2twp = 簡體 → 繁體（台灣正體，含用詞轉換）：柜台 → 櫃檯、鼠标 → 滑鼠。
        # ★ 同樣的問題 ASR 也有（sherpa-onnx 的中文模型輸出簡體），
        #   那一端要各自再轉一次，不要指望這裡幫它轉。
        self._cc = None
        if self.declare_parameter("to_traditional", True).get_parameter_value().bool_value:
            try:
                import opencc

                self._cc = opencc.OpenCC("s2twp")
                self.get_logger().info("✓ 簡轉繁已啟用（s2twp）")
            except Exception as exc:  # 沒裝就照原樣輸出，不要讓節點起不來
                self.get_logger().warning(f"opencc 不可用，略過簡轉繁：{exc}")

        # ── 知識庫檢索（RAG）：每一輪由程式先檢索、把相關片段附在系統提示最後（不是工具，見 knowledge_base.py）──
        # 向量模型跑在 ollama_base_url 那台（要先 `ollama pull bge-m3`）；Ollama 連不上或沒裝 chromadb 會自動退到
        # 關鍵字檢索，enable_rag:=false 則每輪附上整份 FAQ（舊行為）。
        self.enable_rag = self.declare_parameter("enable_rag", True).get_parameter_value().bool_value
        self.embed_model = self.declare_parameter("embed_model", "bge-m3").get_parameter_value().string_value
        self.rag_top_k = self.declare_parameter("rag_top_k", 3).get_parameter_value().integer_value
        # 相似度（1 − cosine 距離）低於這個值的片段不附。2026-10-28 用 bge-m3 對 16 個正向題＋5 個無關題量過
        # （maprun/tools_1021/rag_eval.py）：正向最低 0.539、無關最高 0.489，空間只有 0.05、樣本很小；
        # 知識庫內容或 embedding 模型一換就要重跑 rag_eval.py 重新量。
        self.rag_min_similarity = (
            self.declare_parameter("rag_min_similarity", 0.50).get_parameter_value().double_value
        )
        knowledge_db = self.declare_parameter("knowledge_db_dir", "").get_parameter_value().string_value
        self.enable_realtime_info = (
            self.declare_parameter("enable_realtime_info", True).get_parameter_value().bool_value
        )

        # 初始化對話記憶
        self.memory = ConversationMemory()
        self.memory_lock = threading.Lock()
        self.agent_busy_lock = threading.Lock()
        self.is_agent_running = False
        # ★ 2026-10-22：正在等結果、會讓車移動的 action 目標（帶位／導航）。客人在途中說「停下來」時拿來取消
        self._active_goals: dict = {}  # id(goal_handle) -> goal_handle（ClientGoalHandle 不能 hash，不能放 set）
        self._active_goals_lock = threading.Lock()

        # 建立 Callback Group
        self.cb_group = ReentrantCallbackGroup()

        # 建立服務/動作/客戶端（唯讀的地點查詢；導航一律經 nav_arbiter）
        self.list_waypoints_client = self.create_client(ListWaypoints, "list_waypoints", callback_group=self.cb_group)

        # 對話行為
        # direct_nav：明確的帶路要求由程式直接送給 nav_arbiter（見 nav_intent.py）；
        # nav_guard：模型嘴上答應帶路、這一輪卻沒呼叫導航時攔下來。
        self.direct_nav = self.declare_parameter("direct_nav", True).get_parameter_value().bool_value
        self.nav_guard = self.declare_parameter("nav_guard", True).get_parameter_value().bool_value
        # 機器人最近講過的話（含導航節點的播報），擋被麥克風收回來的回音；0 = 關閉
        self.robot_echo_sec = float(self.declare_parameter("robot_echo_sec", 20.0).get_parameter_value().double_value)
        self._robot_said = deque(maxlen=20)  # (monotonic 時刻, 文字)
        self._robot_said_lock = threading.Lock()
        faq_path = get_config_path("bank_faq.txt")
        self.knowledge = KnowledgeBase(
            faq_path or Path("bank_faq.txt"),
            OllamaEmbedder(self.ollama_base_url, self.embed_model) if self.enable_rag else None,
            self.embed_model,
            db_dir=Path(knowledge_db) if knowledge_db else Path.home() / ".smartnav" / "knowledge_db",
            top_k=self.rag_top_k,
            min_similarity=self.rag_min_similarity,
            logger=self.get_logger(),
        )
        # 建索引會呼叫遠端 Ollama，放背景；建好之前的對話先附整份 FAQ
        threading.Thread(target=self.knowledge.load, daemon=True).start()
        # ★ 2026-08-17：每一輪都扣住旁白（見 RosStreamHandler.begin_turn 的 docstring）。
        #   false = 只扣第 0 輪 = 2026-08-17 之前的行為（較低延遲、但可能唸出旁白）。
        self.hold_all_iterations = self.declare_parameter("hold_all_iterations", True).get_parameter_value().bool_value
        # 通報只發話題，實際送出由 bank_reception_node 負責（通報中樞）
        self.staff_notify_pub = self.create_publisher(String, "staff_notify_request", 10)
        # ★ 導航只能經 smartnav_brain 的 nav_arbiter（身分判斷、單一進行中都在那裡）；本節點不碰 /navigate
        self.request_navigation_client = ActionClient(
            self, RequestNavigation, "request_navigation", callback_group=self.cb_group
        )
        self.current_user_text = ""  # 目前這一輪客人說的話；同一時間只會有一輪在跑（is_agent_running）
        self._nav_local = threading.local()  # 這條執行緒上目前這趟導航的「放行時」回呼與最後結果

        # 建立發佈者和訂閱者
        self.llm_response_pub = self.create_publisher(String, "llm_response", 10)
        self.llm_stream_pub = self.create_publisher(String, "llm_stream", 10)
        # 首字延遲的拆解（JSON，單位秒）：{"retrieve": 知識庫檢索, "tool": 第一個字之前花在工具上的時間}。
        # HMI 用它把「首字延遲」拆成 檢索／工具／模型。每次有新數字就發一次（累計值），HMI 取最後一筆。
        self.llm_timing_pub = self.create_publisher(String, "llm_timing", 10)
        # ★ 2026-08-17：串流「作廢」訊號。
        #
        # HMI 端的 llm_streaming 緩衝只會「累加」（append_stream），而清空它的
        # 只有兩件事：新的 user_text，或 llm_response 抵達。
        # agent 迴圈每一輪的旁白都會被逐字送到 /llm_stream，但呼叫工具那幾輪
        # **不會**發 llm_response —— 於是那段旁白就以灰色斜體卡在畫面上，
        # 一路撐到整個迴圈跑完為止（8/17 實測卡了 7 分鐘，見修復筆記）。
        #
        # 不能改用「補發 llm_response」來清：llm_response 會被 HMI 當成一則
        # 已完成的機器人訊息加進對話，而平板前端會把「已完成且非 ghost」的
        # 訊息唸出來（index.html renderChat / ttsOnChat）——那正是 8/14
        # 「旁白被唸兩遍」那個坑。所以另開一個不帶內容的作廢訊號。
        self.llm_stream_reset_pub = self.create_publisher(Empty, "llm_stream_reset", 10)
        self.speech_text_pub = self.create_publisher(String, "speech_text", 10)
        self.user_text_sub = self.create_subscription(
            String, "user_text", self.user_text_callback, 10, callback_group=self.cb_group
        )
        self.clear_history_sub = self.create_subscription(
            Empty, "clear_conversation", self.clear_conversation_callback, 10, callback_group=self.cb_group
        )
        # 客人等級（VIP 個人化用）。判斷在 brain：這裡只是拿來調整語氣，帶位准不准以 nav_arbiter 的回覆為準
        self.access_context_stale_sec = (
            self.declare_parameter("access_context_stale_sec", 5.0).get_parameter_value().double_value
        )
        self._access_context = None  # (AccessContext, 收到時的 monotonic)
        self.access_context_sub = self.create_subscription(
            AccessContext, "access_context", self._access_context_cb, 10, callback_group=self.cb_group
        )
        # ★ 2026-10-21：記下機器人講過的話（10/20 14:59 LLM 收到「開始導航到 廁所」——
        #   那是導航節點自己的播報被麥克風收回來）
        self.robot_speech_sub = self.create_subscription(
            String, "speech_text", self._robot_speech_cb, 10, callback_group=self.cb_group
        )

        # 模型名稱以 TRANSIENT_LOCAL 發布，讓後啟動的 HMI 也能取得並顯示
        model_info_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.llm_model_pub = self.create_publisher(String, "llm_model", model_info_qos)
        self.llm_model_pub.publish(String(data=self.model_name))

        # 等待所有服務就緒
        self._init_timer = self.create_timer(0.1, self._init_callback, callback_group=self.cb_group)
        # 初始化工具鏈
        self._init_modern_llm_tools()

        # ★ 啟動時就確認提示詞檔存在（ros2 param get 只證明參數伺服器存了值，不證明節點讀得到那個檔）
        self.get_logger().info(
            f"✓ 系統提示詞: {self.SYSTEM_PROMPT_FILE} "
            f"({'已找到' if get_config_path(self.SYSTEM_PROMPT_FILE) else '★找不到，會用內建預設'})"
        )
        self.get_logger().info(f"✓ LLM 對話服務節點已初始化 (模型: {self.model_name})")
        self.get_logger().info(
            f"✓ 對話設定：知識庫檢索={'開（' + self.embed_model + '）' if self.enable_rag else '關（整份 FAQ）'}、"
            f"程式端帶位={self.direct_nav}、帶位防呆={self.nav_guard}、回音擋 {self.robot_echo_sec:.0f} 秒"
        )
        if self.warmup_on_start:
            threading.Thread(target=self._warmup_llm, daemon=True).start()
        if self.keep_warm_sec > 0:
            threading.Thread(target=self._keep_warm_loop, daemon=True).start()
            self.get_logger().info(f"✓ 提示快取保溫已啟用：每 {self.keep_warm_sec:g} 秒一次（keep_warm_sec:=0 可關閉）")

    def _init_callback(self) -> None:
        """初始化回調函式"""
        self._init_timer.cancel()
        self._wait_for_services()

    def _wait_for_services(self) -> None:
        """等待所有基礎服務或動作就緒"""
        services = [("list_waypoints", self.list_waypoints_client)]
        actions = [("request_navigation", self.request_navigation_client)]
        # ★★ 2026-08-31 修正：加上**總期限**，等不到就降級啟動，不要卡死 ★★
        #
        #   舊寫法是 `while not client.wait_for_service(timeout_sec=2.0)` ——
        #   那個 timeout_sec 只是**單次輪詢的間隔**，迴圈本身沒有出口。
        #   上面 8/17 的註解已經記錄過這個坑的後果：某個 server 沒起來時
        #   「整個 LLM 節點就卡在初始化、連『你好』都不會回」。
        #   當時的解法是把 global_localization 移出清單，但**剩下六項全是
        #   導航堆疊的相依**（map_service / waypoint_service / navigation_action），
        #   而對話功能一個都不需要。
        #
        #   實際會踩到的情境：
        #     - demo.launch.py 帶 nav:=false
        #     - nav2 的 lifecycle activate 在 CPU 尖峰下逾時（本專案記錄過）
        #     - 導航堆疊被單獨重啟
        #   症狀都是「平板打字沒反應」，看起來像 LLM 壞了。
        #
        #   降級是安全的：ROS 的 client 會持續探索，server 晚點起來照樣能用；
        #   真的沒起來時工具呼叫會自己逾時並回報（工具例外已於 cfcf124 修過
        #   不會炸穿對話）。對話一定可用，比「全部或全不」好。
        _wait_budget = float(self.services_wait_timeout_sec)
        _deadline = (time.monotonic() + _wait_budget) if _wait_budget > 0 else float("inf")
        _missing = []
        for service_name, client in services:
            while not client.wait_for_service(timeout_sec=2.0):
                if time.monotonic() > _deadline:
                    _missing.append(service_name)
                    break
                self.get_logger().warn(f"⌛ 等待服務 {service_name}...")
        for action_name, client in actions:
            while not client.wait_for_server(timeout_sec=2.0):
                if time.monotonic() > _deadline:
                    _missing.append(action_name)
                    break
                self.get_logger().warn(f"⌛ 等待動作 {action_name}...")
        if _missing:
            self.get_logger().warn(
                f"⚠ 等不到（{self.services_wait_timeout_sec:.0f} 秒）："
                + "、".join(_missing)
                + " —— **仍然啟動**，對話功能完全可用，"
                + "只有需要這些的導航工具會在被呼叫時失敗。"
                + " 導航堆疊晚點起來的話會自動接上，不必重啟本節點。"
            )
        else:
            self.get_logger().info("✓ 所有基礎服務與動作已就緒")

    def _wait_for_future(self, future, timeout_sec: float) -> Any:
        """等待服務回應的輔助函式"""
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())

        # 阻塞當前獨立執行緒，直到被喚醒或超時
        if not event.wait(timeout=timeout_sec):
            raise TimeoutError("服務請求超時")

        return future.result()

    def _wait_for_action_result(self, goal_handle, timeout_sec: float, what: str) -> Any:
        """等 action 的最終結果；**逾時就主動取消目標**。

        Args:
            goal_handle: send_goal_async 回來的 handle
            timeout_sec: 等多久
            what: 給訊息用的中文名稱（「導航」「建立地圖」「帶位」）

        Returns:
            action 的結果物件

        Raises:
            TimeoutError: 逾時（取消指令已送出）

        ★★ 2026-08-26：原本三個會讓機器人**實際移動**的工具（建圖 500 秒、
        導航 200 秒、帶位 200 秒）逾時後只是本地不再等待那個 future，
        `goal_handle.cancel_goal_async()` **從來沒有被呼叫過**
        （全套件唯一的 `.cancel(` 是初始化計時器，與 action 無關）。

        後果不是「多等一下」，是**說法與動作矛盾**：
          - 工具回「執行結果: 失敗」-> 模型依系統提示詞 §3 告訴客人「沒辦法帶路」
          - `is_agent_running` 隨即釋放，客人可以再下指令
          - **但底層的 Navigate 目標還在跑，車子還在走**
          - 客人若再要求一次帶位，第二個目標會疊在第一個上面

        取消失敗不往外拋：那時本來就已經要回報逾時了，再多一個例外只會讓
        真正的原因（逾時）被蓋掉。取消失敗只記 log。
        """
        with self._active_goals_lock:
            self._active_goals[id(goal_handle)] = goal_handle
        try:
            return self._wait_for_future(goal_handle.get_result_async(), timeout_sec)
        except TimeoutError:
            try:
                goal_handle.cancel_goal_async()
                self.get_logger().warning(f"⚠ {what}超過 {timeout_sec:.0f} 秒未完成，已送出取消指令")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"✗ {what}逾時後送取消指令失敗: {exc}")
            raise TimeoutError(f"{what}超過 {timeout_sec:.0f} 秒未完成，已中止本次請求")
        finally:
            with self._active_goals_lock:
                self._active_goals.pop(id(goal_handle), None)

    def _nav_result_text(self, wrapped, success_text: str = "") -> str:
        """nav_arbiter 的 RequestNavigation 結果 → 給模型看的「執行結果: …」

        「取消」「逾時」「權限不足」「忙線」一律標「（不可重試）」：提示詞 §3 叫模型失敗先自動重試，
        平板按了緊急停止、或客人不能去那裡，模型卻再叫一次導航，車子又開出去（2026-09-25 審查）。
        """
        result = wrapped.result
        self._nav_local.last = result
        msg = result.message
        outcome = result.outcome
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success:
            return f"執行結果: 成功, 詳細信息: {success_text or msg}"
        if outcome == RequestNavigation.Result.OUTCOME_DENIED:
            return f"執行結果: 失敗（不可重試）, 詳細信息: 權限不足。請對客人說：{msg}"
        if outcome == RequestNavigation.Result.OUTCOME_CANCELED or wrapped.status == GoalStatus.STATUS_CANCELED:
            return f"執行結果: 取消（不可重試）, 詳細信息: {msg or '導航請求被系統或使用者取消'}"
        if outcome == RequestNavigation.Result.OUTCOME_TIMEOUT:
            return f"執行結果: 逾時（不可重試）, 詳細信息: {msg}"
        if outcome == RequestNavigation.Result.OUTCOME_BUSY:
            return f"執行結果: 失敗（不可重試）, 詳細信息: {msg}"
        if outcome == RequestNavigation.Result.OUTCOME_UNKNOWN_WAYPOINT:
            return f"執行結果: 失敗, 詳細信息: {msg}。請先呼叫 list_waypoints_tool 取得正確的地點ID再導航"
        return f"執行結果: 失敗, 詳細信息: {msg or '導航過程中發生錯誤'}"

    @staticmethod
    def _nav_timeout_text(msg: str) -> str:
        """等 nav_arbiter 超過 CLIENT_WAIT_SEC 仍沒結果（取消指令已送出）時給模型看的結果文字

        正常情況 nav_arbiter 會先自己逾時並回 OUTCOME_TIMEOUT；走到這裡代表 nav_arbiter 本身沒回應。
        """
        return f"執行結果: 逾時（不可重試）, 詳細信息: {msg}"

    def _navigate_to(self, waypoint_id: str, success_text: str = "") -> str:
        """經 nav_arbiter 帶位／導航，等到結果。回傳給模型看的「執行結果: …」。

        放行時（nav_arbiter 通過身分檢查、底層也接受目標）會呼叫這條執行緒上登記的
        self._nav_local.on_authorized；被拒絕就不會呼叫——程式端帶位靠這個決定要不要先說「請跟我來」。
        """
        on_authorized = getattr(self._nav_local, "on_authorized", None)
        self._nav_local.on_authorized = None
        self._nav_local.last = None
        if not self.request_navigation_client.server_is_ready():
            return "執行結果: 失敗, 詳細信息: 導航仲裁節點（nav_arbiter）未啟動，無法帶位"

        goal = RequestNavigation.Goal()
        goal.source = RequestNavigation.Goal.SOURCE_LLM
        goal.waypoint_id = waypoint_id

        def _feedback(msg) -> None:
            if msg.feedback.phase == "AUTHORIZED" and on_authorized:
                on_authorized()

        future = self.request_navigation_client.send_goal_async(goal, feedback_callback=_feedback)
        goal_handle = self._wait_for_future(future, timeout_sec=5.0)
        if goal_handle is None or not goal_handle.accepted:
            return "執行結果: 失敗, 詳細信息: 導航請求未被接受"
        # ★ 逾時要主動取消（見 _wait_for_action_result）。上限 CLIENT_WAIT_SEC 比 nav_arbiter 自己的逾時
        #   （TIMEOUT_SEC）再大一點，讓 nav_arbiter 的 OUTCOME_TIMEOUT 先到。
        try:
            wrapped = self._wait_for_action_result(goal_handle, RequestNavigation.Goal.CLIENT_WAIT_SEC, "導航")
        except TimeoutError as exc:
            return self._nav_timeout_text(str(exc))
        return self._nav_result_text(wrapped, success_text)

    def _init_modern_llm_tools(self) -> None:
        """掛上工具並建立 agent chain"""
        self.tools_map = make_tools(self, self.enable_realtime_info)
        self.get_logger().info(f"✓ 已載入 {len(self.tools_map)} 個工具: {', '.join(self.tools_map)}")

        # 初始化模型並綁定工具
        self.stream_handler = RosStreamHandler(self.llm_stream_pub, self.speech_text_pub, self._to_traditional)
        stream_handler = self.stream_handler
        self._raw_llm = self._make_chat_llm([stream_handler])
        self._chains: dict = {}  # 工具名稱組合 -> chain（每輪給的工具不同，見 tool_gate.py）
        self.get_logger().info(
            f"✓ Ollama 上下文 num_ctx={self.num_ctx}（Ollama 在 4 GB 顯存的預設是 4096；見 num_ctx 參數說明）、"
            f"keep_alive={self.keep_alive or '（Ollama 預設 5 分鐘）'}"
        )

        # 定義對話提示模板，包含系統提示、歷史消息和工具調用結果
        self.prompt_template = ChatPromptTemplate.from_messages(
            [
                ("system", "{system_prompt}"),
                MessagesPlaceholder(variable_name="history"),
                ("human", "{input}"),
                MessagesPlaceholder(variable_name="agent_scratchpad"),
            ]
        )

    def _chain_for(self, user_input: str):
        """這一輪的 agent chain：只綁定 tool_gate 認為這句話可能用到的工具（沒有工具就是純對話）"""
        names = tuple(tool_gate.allowed_tools(user_input, self.tools_map))
        chain = self._chains.get(names)
        if chain is None:
            llm = self._raw_llm.bind_tools([self.tools_map[n] for n in names]) if names else self._raw_llm
            chain = RunnablePassthrough() | self.prompt_template | llm
            self._chains[names] = chain
        self.get_logger().info(f"🧰 本輪給模型的工具: {', '.join(names) or '（無，純對話）'}")
        return chain

    # ── ★ 2026-10-21：Ollama 連線、暖機、FAQ、回音與程式端帶位 ─────────────────
    def _ollama_extra_kwargs(self) -> dict:
        """keep_alive 只在這版 langchain_ollama 支援時才帶（舊版多帶未知欄位會讓節點起不來）"""
        ka = (self.keep_alive or "").strip()
        if not ka:
            return {}
        fields = getattr(ChatOllama, "model_fields", None) or getattr(ChatOllama, "__fields__", None) or {}
        if "keep_alive" not in fields:
            self.get_logger().warning(
                "這版 langchain_ollama 的 ChatOllama 沒有 keep_alive 欄位，略過——模型閒置 5 分鐘仍會被卸載"
            )
            return {}
        try:
            return {"keep_alive": int(ka)}
        except ValueError:
            return {"keep_alive": ka}

    def _make_chat_llm(self, callbacks: list):
        """唯一的 ChatOllama 建構處（主迴圈與暖機共用：參數一致，暖機才會把同一份提示載進去）"""
        return ChatOllama(
            base_url=self.ollama_base_url,
            model=self.model_name,
            temperature=self.temperature,
            num_predict=self.num_predict,  # ★ 生成長度硬上限，見宣告處
            num_ctx=self.num_ctx,  # ★ 上下文長度，見宣告處（Ollama 在 4 GB 顯存的預設只有 4096）
            callbacks=callbacks,
            **self._ollama_extra_kwargs(),
        )

    def _warmup_llm(self) -> None:
        """背景暖機：用跟主迴圈一模一樣的提示與工具問一次，結果丟掉（不發布、不進記憶、不掛串流）"""
        try:
            time.sleep(2.0)  # 讓節點其他部分先起來
            t0 = time.monotonic()
            llm = self._make_chat_llm([]).bind_tools(list(self.tools_map.values()))
            chain = RunnablePassthrough() | self.prompt_template | llm
            chain.invoke(
                {"system_prompt": self._load_system_prompt(), "history": [], "input": "你好", "agent_scratchpad": []}
            )
            self.get_logger().info(f"✓ Ollama 暖機完成（{time.monotonic() - t0:.1f} 秒）——第一位客人不用再等模型載入")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Ollama 暖機失敗（不影響對話，只是第一題會比較慢）: {exc}")

    def _keep_warm_loop(self) -> None:
        """背景保溫：定時送極小請求，讓 Ollama 的系統提示快取不過期（原因與數據見 keep_warm_sec 宣告處）

        只含系統提示（不含知識庫片段與客人說明）：真實請求的前綴就是這一段，後面不同的部分才需要重算。
        參數（模型、num_ctx、keep_alive）要和主迴圈一致——num_ctx 不同會讓 Ollama 重載模型。
        """
        time.sleep(4.0)  # 讓暖機先走（暖機本身就會把快取燙熱）
        failing = False
        while rclpy.ok():
            wait = self.keep_warm_sec - (time.monotonic() - self._ollama_last)
            if wait > 0 or self._llm_busy:
                time.sleep(max(min(wait, self.keep_warm_sec), 0.2))
                continue
            t0 = time.monotonic()
            try:
                body = {
                    "model": self.model_name,
                    "stream": False,
                    "options": {"temperature": self.temperature, "num_predict": 1, "num_ctx": self.num_ctx},
                    "messages": [
                        {"role": "system", "content": self._load_system_prompt()},
                        {"role": "user", "content": "。"},
                    ],
                }
                body.update(self._ollama_extra_kwargs())  # keep_alive：和主迴圈同一套規則
                resp = requests.post(f"{self.ollama_base_url.rstrip('/')}/api/chat", json=body, timeout=30)
                resp.raise_for_status()
                if failing:
                    self.get_logger().info("✓ 提示快取保溫恢復")
                    failing = False
                took = time.monotonic() - t0
                if took > 1.0:
                    # 保溫請求慢 = 快取已經過期被它重新燙熱（剛啟動、剛被其他程式佔用、或剛換過提示）
                    self.get_logger().info(f"🔥 保溫請求花了 {took:.1f} 秒（快取已重新預熱）")
            except Exception as exc:  # noqa: BLE001
                if not failing:
                    self.get_logger().warning(f"提示快取保溫失敗（不影響對話，只是回應會變慢；30 秒後重試）: {exc}")
                failing = True
                self._ollama_last = time.monotonic() + 26.0  # 退避：下次約 30 秒後再試
                continue
            self._ollama_last = time.monotonic()

    def _knowledge_section(self, user_input: str, chat_history: List[BaseMessage]) -> str:
        """附在系統提示最後的「本行知識庫」一節。

        純問候不附（沒有業務問題，附了只會讓模型亂講營業時間）；檢索可用就附檢索到的片段；
        還沒建好、關鍵字模式查不到、或 enable_rag:=false，就附整份 FAQ。
        """
        if tool_gate.is_social(user_input):
            return ""
        kb = self.knowledge
        if self.enable_rag and kb.ready:
            # 客人的追問常常很短（「那週六呢」），檢索要帶上前一句才有主題
            query = user_input
            if len(user_input) < 6:
                prev = next((m.content for m in reversed(chat_history) if isinstance(m, HumanMessage)), "")
                query = f"{prev} {user_input}".strip()
            section = kb.context_for(query)
            if section:
                return section
        if kb.full_text:
            return "\n\n---\n\n# 本行知識庫（全文；回答本行業務問題只能依據這一節）\n\n" + kb.full_text
        return ""

    def _robot_speech_cb(self, msg: String) -> None:
        self._remember_robot_said(msg.data)

    def _remember_robot_said(self, text: str) -> None:
        if text and text.strip():
            with self._robot_said_lock:
                self._robot_said.append((time.monotonic(), text))

    def _is_robot_echo(self, text: str) -> bool:
        if self.robot_echo_sec <= 0:
            return False
        now = time.monotonic()
        with self._robot_said_lock:
            recent = [t for ts, t in self._robot_said if now - ts <= self.robot_echo_sec]
        return nav_intent.is_robot_echo(text, recent)

    def _publish_timing(self, retrieve_sec: float, tool_sec: float) -> None:
        """回報這一輪「首字之前」的耗時拆解給 HMI（累計值，HMI 取最後一筆）

        ttft 是模型自己的首 token 時間（見 RosStreamHandler.ttft），還沒吐字時沒有這個欄位。
        """
        timing = {"retrieve": round(retrieve_sec, 2), "tool": round(tool_sec, 2)}
        if self.stream_handler.ttft is not None:
            timing["ttft"] = round(self.stream_handler.ttft, 2)
        self.llm_timing_pub.publish(String(data=json.dumps(timing)))

    def _publish_reply(self, text: str) -> None:
        """發 /llm_response（平板會顯示並唸出來），同時記進「機器人講過的話」"""
        self._remember_robot_said(text)
        self.llm_response_pub.publish(String(data=text))

    def _waypoint_name_map(self) -> dict:
        """目前地圖的 {地點名稱: 地點ID}；查不到回空 dict（交給模型處理）"""
        # ★ 2026-10-26：順便記下「查不到」（導航沒起來／服務失敗）——跟「地圖上真的沒有地點」要分開，
        #   否則導航沒開時「洗手間在哪」會被答成「示範場地沒有設定洗手間」（地圖其實有廁所）。
        self._waypoints_unavailable = True
        try:
            if not self.list_waypoints_client.service_is_ready():
                return {}
            res = self._wait_for_future(self.list_waypoints_client.call_async(ListWaypoints.Request()), 3.0)
            if res is None or not res.success:
                return {}
            self._waypoints_unavailable = False
            return {wp.waypoint_name: wp.waypoint_id for wp in res.waypoints_info if wp.waypoint_name}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"帶位判斷：查地點失敗（交給模型）: {exc}")
            return {}

    def _try_direct_navigation(self, user_input: str, force_intent: bool = False) -> bool:
        """明確的帶路要求、又對得到唯一地點 → 程式直接導航並回話。回 True 表示已處理完這一輪。

        force_intent=True（模型已經答應帶路時用）：不再要求句子裡有「帶我去」這類字，只看地點。
        """
        names = self._waypoint_name_map()
        if not names and self._waypoints_unavailable and not force_intent and nav_intent.has_nav_intent(user_input):
            reply = "導航系統還沒準備好，暫時沒辦法帶位；請稍等一下再說一次，或洽詢櫃檯行員。"
            self.get_logger().warning(f"🧭 問路／帶路，但查不到地點（導航沒起來？）：'{user_input}'")
            self.get_logger().info(f"🤖 Agent 最終決策回應: {reply}")
            with self.memory_lock:
                self.memory.add_message(HumanMessage(content=user_input))
                self.memory.add_message(AIMessage(content=reply))
            self._publish_reply(reply)
            return True
        name = None
        if names and "navigate_tool" in self.tools_map:
            if force_intent:
                name = nav_intent.match_waypoint(user_input, names)
            else:
                name = nav_intent.detect_nav_request(user_input, names)
        if not name:
            # ★ 2026-10-21：問的是 FAQ 列為「示範場地沒有設定」的地點（洗手間、ATM…）、地圖上也沒有 →
            #   答案是固定的，由程式回答（筆電評估：模型會反問樓層或叫人「呼叫查詢地點工具」）。
            #   不需要導航堆疊，導航沒開時也照樣答對。
            place = None if force_intent else nav_intent.unconfigured_place(user_input, names)
            if not place:
                return False
            reply = f"示範場地沒有設定{place}，我沒辦法帶您過去；需要的話請洽詢櫃檯行員。"
            self.get_logger().info(f"🧭 未設定地點（程式回答，不經模型）：'{user_input}' -> {place}")
            self.get_logger().info(f"🤖 Agent 最終決策回應: {reply}")
            with self.memory_lock:
                self.memory.add_message(HumanMessage(content=user_input))
                self.memory.add_message(AIMessage(content=reply))
            self._publish_reply(reply)
            return True
        wid = names[name]
        self.get_logger().info(f"🧭 帶位直達（程式判斷，不經模型）：'{user_input}' -> {name}（{wid}）")
        # ★ 2026-10-27：客人講完話通常站在車子正前方，出發時會擋住（上機實測「前方障礙」就在起步）；
        #   規劃路徑約三成是先倒車，所以也不能叫人站後面 → 請客人先站到旁邊。
        ack = (
            "好的，請跟我來。請先站到我旁邊，等我開動再跟上。"  # 導航節點起跑時自己會播「開始導航到 X」，這裡不重複地點
        )
        # 這句只在 nav_arbiter 放行後才說；被拒絕（不能去、忙線）就直接回它給的話術
        acked = []

        def _say_ack() -> None:
            acked.append(True)
            self._publish_reply(ack)

        self._nav_local.on_authorized = _say_ack
        try:
            result = str(self.tools_map["navigate_tool"].invoke({"waypoint_id": wid}))
        except Exception as exc:  # noqa: BLE001
            result = f"執行結果: 失敗, 詳細信息: {exc}"
        self.get_logger().info(f"📥 帶位結果: {result}")
        last = getattr(self._nav_local, "last", None)
        if result.startswith("執行結果: 成功"):
            reply = f"{name}到了。"
        elif last is not None and last.outcome in (
            RequestNavigation.Result.OUTCOME_DENIED,
            RequestNavigation.Result.OUTCOME_BUSY,
        ):
            reply = last.message
        else:
            reply = self._nav_fail_reply(name, result)
        self.get_logger().info(f"🤖 Agent 最終決策回應: {reply}")
        with self.memory_lock:
            self.memory.add_message(HumanMessage(content=user_input))
            self.memory.add_message(AIMessage(content=f"{ack if acked else ''}{reply}"))
        self._publish_reply(reply)
        return True

    @staticmethod
    def _nav_fail_reply(name: str, result: str) -> str:
        """導航失敗時對客人講的話（完整原因已寫進 log）"""
        if any(k in result for k in ("取消", "停止", "急停", "逾時")):
            return "車子已經停下來了。需要的話，請再說一次要去哪裡。"
        return f"抱歉，這次沒辦法帶您到{name}，請洽詢櫃檯人員協助。"

    def _to_traditional(self, text: str) -> str:
        """把模型吐出來的簡體字轉成台灣正體。轉換器不可用時原樣回傳。"""
        if not self._cc or not text:
            return text
        try:
            return self._cc.convert(text)
        except Exception:
            return text

    def _access_context_cb(self, msg: AccessContext) -> None:
        self._access_context = (msg, time.monotonic())

    def _customer_context(self) -> str:
        """附在系統提示最後的「目前的客人」。brain 沒回報或太久沒更新就當一般訪客（不個人化）。"""
        entry = self._access_context
        if entry is None or time.monotonic() - entry[1] > self.access_context_stale_sec:
            return ""
        ctx = entry[0]
        return customer_prompt.prompt_context(ctx.tier, ctx.user_name, ctx.description)

    def user_text_callback(self, msg: String) -> None:
        """處理使用者輸入"""
        user_input = msg.data

        # ★ 2026-10-21：機器人自己的話被麥克風收回來，不處理（見 nav_intent.is_robot_echo）
        if self._is_robot_echo(user_input):
            self.get_logger().info(f"🔁 忽略疑似回音（機器人 {self.robot_echo_sec:.0f} 秒內講過）: {user_input}")
            return

        with self.agent_busy_lock:
            if self.is_agent_running:
                # ★ 2026-10-22：帶位途中客人說「停下來／取消」→ 真的取消目標。以前跟其他話一樣回
                #   「正在處理上一個指令」然後丟掉，車照樣走（只剩平板急停）。取消走的是跟平板急停
                #   同一條 action cancel 路徑；等結果的執行緒會收到「取消」，回「車子已經停下來了」。
                with self._active_goals_lock:
                    moving = list(self._active_goals.values())
                if moving and nav_intent.is_stop_request(user_input):
                    for gh in moving:
                        try:
                            gh.cancel_goal_async()
                        except Exception as exc:  # noqa: BLE001
                            self.get_logger().error(f"✗ 送取消指令失敗: {exc}")
                    self.get_logger().warn(f"🛑 客人在帶位途中說「{user_input}」→ 取消 {len(moving)} 個進行中的目標")
                    return
                warning_msg = "系統目前正在處理上一個指令，請稍後再試..."
                self.get_logger().warn(warning_msg)
                self._publish_reply(warning_msg)
                return
            self.is_agent_running = True

        self.get_logger().info(f"📥 收到使用者輸入: {user_input}")
        # 啟動獨立執行緒進行 LLM 多步驟循環推理
        threading.Thread(target=self._run_agent_loop, args=(user_input,)).start()

    def clear_conversation_callback(self, msg: Empty) -> None:
        """清除對話記憶

        供 HMI 的「清除對話」按鈕使用，讓畫面與模型記憶同步歸零，
        避免畫面已清空、模型卻還記得上一位客戶的對話內容。

        Args:
            msg: 觸發訊息，內容為空
        """
        with self.memory_lock:
            self.memory.clear()

        self.get_logger().info("🧹 已清除對話記憶")

    SYSTEM_PROMPT_FILE = "system_prompt.txt"

    def _load_system_prompt(self) -> str:
        """讀取系統提示詞（讀不到就退回內建預設）；知識庫那一節每輪另外附，見 _knowledge_section"""
        path = get_config_path(self.SYSTEM_PROMPT_FILE)
        if path:
            with open(path, "r", encoding="utf-8") as f:
                return f.read()

        self.get_logger().warning(f"✗ 無法找到 {self.SYSTEM_PROMPT_FILE}，使用內建預設提示語")
        return "你是一個專業的銀行迎賓服務機器人，請用繁體中文簡短回答客戶，需要帶路時呼叫導航工具。\n"

    def _run_agent_loop(self, user_input: str) -> None:
        """ReAct 自主思考迴圈"""
        self.current_user_text = user_input  # 通報行員時用客人的原句（見 tools.notify_staff_tool）

        try:
            # 準備工作也要在 try 裡：任何例外都必須走到 finally 清除 is_agent_running，否則節點之後每句話都回「正在處理上一個指令」
            customer_context = self._customer_context()
            if customer_context:
                self.get_logger().info("👤 本輪客人有專屬說明（系統提示已附上）")

            with self.memory_lock:
                chat_history = self.memory.get_history_messages()

            # 檢索會呼叫遠端 Ollama，所以放在這條 agent 執行緒裡（不是 ROS callback）
            self.stream_handler.start_turn()
            t_retrieve = time.monotonic()
            knowledge_section = self._knowledge_section(user_input, chat_history)
            retrieve_sec = time.monotonic() - t_retrieve
            tool_sec = 0.0  # 第一個字之前花在工具上的時間（第一個字之後才跑的工具不算「等待首字」）
            ttft_sent = False
            self._publish_timing(retrieve_sec, tool_sec)
            system_content = self._load_system_prompt() + knowledge_section + customer_context

            agent_chain = self._chain_for(user_input)
            agent_scratchpad: List[BaseMessage] = []
            max_iterations = 10
            nav_called = False  # 這一輪有沒有呼叫過會讓車移動的工具（給帶位防呆用）
            self.get_logger().info("🧠 LLM 開始進入多步驟決策鏈...")

            # ★ 2026-10-21：明確的帶路要求交給程式（10/20「帶我去門口」×5，模型零次呼叫導航）
            if self.direct_nav and self._try_direct_navigation(user_input):
                return

            for iteration in range(max_iterations):
                # ★ 2026-08-17：每一輪都扣。8/17 log 證實 iteration 1 也會
                #   「先講一段旁白、再去呼叫工具」（見 begin_turn 的 docstring）。
                self.stream_handler.begin_turn(hold=(self.hold_all_iterations or iteration == 0))

                self._llm_busy = True
                try:
                    response = agent_chain.invoke(
                        {
                            "system_prompt": system_content,
                            "history": chat_history,
                            "input": user_input,
                            "agent_scratchpad": agent_scratchpad,
                        }
                    )
                finally:
                    self._llm_busy = False
                    self._ollama_last = time.monotonic()
                if self.stream_handler.ttft is not None and not ttft_sent:
                    ttft_sent = True
                    self._publish_timing(retrieve_sec, tool_sec)  # 第一次有 TTFT：補發一次，HMI 取最後一筆

                # ★ 2026-08-20：撞到 num_predict 上限要吼出來。
                #   撞上限而且沒有 tool_calls，極可能是工具呼叫 JSON 被切斷
                #   （見 num_predict 宣告處）。不 log 的話這個失敗完全隱形：
                #   客人只會覺得「它答應要查，然後就沒下文了」。
                try:
                    _done = (response.response_metadata or {}).get("done_reason")
                except Exception:
                    _done = None
                if _done == "length":
                    self.get_logger().warning(
                        f"⚠️ 第 {iteration} 輪生成撞到 num_predict={self.num_predict} 上限"
                        + (
                            "（且沒解出 tool_calls -> 工具呼叫很可能被截斷，" "請調高 num_predict）"
                            if not response.tool_calls
                            else ""
                        )
                    )

                # 檢查 LLM 是否需要叫工具
                if not response.tool_calls:
                    # 沒有工具呼叫 = 這是要講給客戶聽的最終答案
                    final_reply = self._to_traditional(str(response.content))
                    # ★★ 2026-10-21：兩道防呆（10/20 實車：全天 75 輪有 4 次空白回應；
                    #   「帶我去門口」被回「請跟我來，我會帶您去門口」卻沒呼叫導航，車不會動）★★
                    #   要在 flush_held() 之前判斷：換掉的句子不能先被送去唸。
                    if not final_reply.strip():
                        self.stream_handler.drop_held()
                        self.get_logger().warning("⚠️ 模型回了空白，改用預設句")
                        final_reply = "不好意思，我沒有聽清楚，可以再說一次嗎？"
                    elif nav_intent.leaks_tool_call(final_reply):
                        # ★ 2026-10-26：模型把工具呼叫講成文字（3B 常見），原文不能唸給客人聽
                        self.stream_handler.drop_held()
                        self.get_logger().warning(f"⚠️ 模型把工具呼叫講成文字（沒有真的呼叫）：{final_reply}")
                        if self._try_direct_navigation(user_input, force_intent=True):
                            return
                        final_reply = "抱歉，這個我現在沒辦法處理，請洽詢櫃檯行員協助。"
                    elif self.nav_guard and not nav_called and nav_intent.promises_navigation(final_reply):
                        self.stream_handler.drop_held()
                        self.get_logger().warning(f"⚠️ 模型答應帶路、這一輪卻沒呼叫導航：{final_reply}")
                        if self._try_direct_navigation(user_input, force_intent=True):
                            return
                        final_reply = "抱歉，我現在沒辦法帶您過去，請洽詢櫃檯人員協助。"
                    else:
                        self.stream_handler.flush_held()
                    self.get_logger().info(f"🤖 Agent 最終決策回應: {final_reply}")

                    with self.memory_lock:
                        self.memory.add_message(HumanMessage(content=user_input))
                        self.memory.add_message(AIMessage(content=final_reply))

                    self._publish_reply(final_reply)
                    return

                # 這一輪是去呼叫工具，模型講的旁白不要唸出去
                if self.stream_handler.held:
                    self.get_logger().info(
                        f"🔇 已扣住呼叫工具前的旁白（{len(self.stream_handler.held)} 段）: "
                        f"{self.stream_handler.held[0][:40]}..."
                    )
                self.stream_handler.drop_held()

                # ★ 旁白丟掉的同時，也要把 HMI 上那段灰字作廢。
                #   drop_held() 只擋住 /speech_text（不唸），但 /llm_stream 的
                #   逐字內容早就送出去了，不清的話會一直灰在畫面上。
                self.llm_stream_reset_pub.publish(Empty())

                # 記錄 LLM 的工具調用請求
                agent_scratchpad.append(response)

                # 處理工具調用
                for tool_call in response.tool_calls:
                    tool_name = tool_call["name"]
                    tool_args = tool_call["args"]
                    tool_id = tool_call["id"]

                    self.get_logger().info(f"🛠️ LLM 決定呼叫服務: {tool_name}, 參數: {tool_args}")

                    if tool_name in self.tools_map:
                        if tool_name in nav_intent.NAV_TOOL_NAMES:
                            nav_called = True
                        # ★★ 2026-08-26：工具呼叫要包起來 ★★
                        #
                        # LangChain 的 BaseTool.invoke() 會先用 args_schema
                        # （由函式簽章自動產生的 pydantic model）驗證 LLM 給的參數，
                        # 而驗證發生在**進入函式本體之前** —— 所以各工具自己那層
                        # try/except 接不到。handle_tool_error / handle_validation_error
                        # 兩個旗標預設都是 False，例外會一路炸到 _run_agent_loop
                        # 最外層的 except，被 publish 成
                        #   「✗ Agent 執行時發生異常: 1 validation error for ...」
                        # 客人聽到的就是這串英文技術訊息。
                        #
                        # 而且那一炸會讓**這一輪剩下的工具呼叫全部中止** ——
                        # 原本「查地點 -> 導航」兩步，第一步驗證失敗就整個垮掉。
                        #
                        # 3B 模型生出缺參數或型別不對的 tool call 並不罕見
                        # （本檔多處註解都記錄過 qwen2.5:3b 的格式不穩）。
                        # 接住之後回一句模型看得懂的話，讓它自己重試或改口，
                        # 比整輪垮掉好。
                        t_tool = time.monotonic()
                        counts_toward_first_token = not self.stream_handler.seen_token
                        try:
                            tool_result = self.tools_map[tool_name].invoke(tool_args)
                        except Exception as exc:  # noqa: BLE001
                            self.get_logger().error(f"✗ 工具 {tool_name} 呼叫失敗（參數={tool_args}）: {exc}")
                            tool_result = (
                                "執行結果: 失敗, 詳細信息: 工具參數不完整或格式不正確，"
                                "請補齊必要資訊後重試，或改用其他方式協助客戶"
                            )
                        if counts_toward_first_token:
                            tool_sec += time.monotonic() - t_tool
                            self._publish_timing(retrieve_sec, tool_sec)
                    else:
                        tool_result = f"錯誤：找不到工具 {tool_name}"

                    self.get_logger().info(f"📥 服務回傳數據: {tool_result}")

                    # 紀錄 LLM 的工具調用結果
                    agent_scratchpad.append(
                        ToolMessage(
                            content=str(tool_result),
                            tool_call_id=tool_id,
                            name=tool_name,
                        )
                    )

            # 超過最大步數
            timeout_msg = "任務執行步驟過多，已強制中止防止機器人發生異常"
            self._publish_reply(timeout_msg)

        except Exception as e:
            error_msg = f"✗ Agent 執行時發生異常: {str(e)}"
            self.get_logger().error(error_msg)
            self._publish_reply(error_msg)
        finally:
            with self.agent_busy_lock:
                self.is_agent_running = False


def main(args=None):
    rclpy.init(args=args)
    node = LLMServiceNode()
    executor = MultiThreadedExecutor()
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
