#!/usr/bin/env python3

"""LLM 對話服務節點

使用 Ollama 調用本地模型，根據使用者輸入智能決定調用哪個服務
支援對話記憶、流式傳輸和服務自動化
另整合銀行知識庫檢索 (RAG) 與即時資訊查詢，可回答行內業務與股價匯率等問題
"""

import re
import threading
import time
from collections import deque
from pathlib import Path
from typing import List, Any
from langchain_ollama import ChatOllama
from langchain_core.tools import tool
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

from smartnav_msgs.srv import ListMaps, SwitchMap, CreateWaypoint, ListWaypoints
from smartnav_msgs.action import CreateMap, GlobalLocalization, Navigate
from smartnav_llm.llm_utils import get_config_path, get_knowledge_dir
from smartnav_llm.rag_store import BankKnowledgeStore
from smartnav_llm import web_tools


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
        return " ".join(
            self.clean_pattern.sub("", self.thousands_pattern.sub("", text)).split())

    def on_llm_new_token(self, token: str, **kwargs: Any) -> None:
        if not token:
            return

        self.stream_publisher.publish(String(data=token))

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
        # 192.168.137.1 是 Windows 行動熱點 (ICS) 的閘道，也就是跑 Ollama 的筆電本身；
        # Pi 在同一個熱點下拿到的是 192.168.137.106。舊值 192.168.11.101 是別的網段，連不到。
        # ★ 沒有 launch 檔會覆寫這個值 —— HMI 經 run_node_cc.sh 直接 `ros2 run` 啟動，
        #   參數來源鏈上只有這一環，所以這個預設值就是實際生效的值。
        self.ollama_base_url = (
            self.declare_parameter("ollama_base_url", "http://192.168.137.1:11434").get_parameter_value().string_value
        )
        # ★ 2026-08-31：等導航堆疊的**總期限**（見 _wait_for_services）。
        #   等不到就降級啟動——對話一定可用，只有導航工具會在被呼叫時失敗。
        #   設 0 或負數＝無限等（舊行為，不建議）。
        self.services_wait_timeout_sec = float(
            self.declare_parameter("services_wait_timeout_sec", 45.0)
            .get_parameter_value().double_value)
        # 2026-08-10 實測換模型：orieg/gemma3-tools 是 11.8B，「你好」要 117 秒，
        # 而且會為了打招呼誤叫 query_datetime_tool（回你日期而不是回你好）。
        # qwen2.5:3b 同樣四題平均 4.1 秒（1.5~7.6，7.6 是模型冷載入那次），
        # 工具觸發也正常：「你好」純閒聊、「現在幾點」才叫 datetime。
        # 第 4 週驗收要端到端 < 6 秒，只有 3b 這條線達得到。
        self.model_name = self.declare_parameter("model_name", "qwen2.5:3b").get_parameter_value().string_value
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
        self.num_predict = self.declare_parameter(
            "num_predict", 256).get_parameter_value().integer_value

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
            except Exception as exc:                      # 沒裝就照原樣輸出，不要讓節點起不來
                self.get_logger().warning(f"opencc 不可用，略過簡轉繁：{exc}")

        # 銀行知識庫與即時資訊查詢參數
        self.enable_rag = self.declare_parameter("enable_rag", True).get_parameter_value().bool_value
        self.enable_web_tools = self.declare_parameter("enable_web_tools", True).get_parameter_value().bool_value
        self.knowledge_dir = self.declare_parameter("knowledge_dir", "").get_parameter_value().string_value
        # 留空表示只用關鍵字檢索；填入 embedding 模型 (如 bge-m3) 可額外啟用語意檢索
        self.embed_model = self.declare_parameter("embed_model", "").get_parameter_value().string_value
        self.rag_top_k = self.declare_parameter("rag_top_k", 3).get_parameter_value().integer_value

        # 初始化對話記憶
        self.memory = ConversationMemory()
        self.memory_lock = threading.Lock()
        self.agent_busy_lock = threading.Lock()
        self.is_agent_running = False

        # 建立 Callback Group
        self.cb_group = ReentrantCallbackGroup()

        # 建立服務/動作/客戶端
        self.list_maps_client = self.create_client(ListMaps, "list_maps", callback_group=self.cb_group)
        self.switch_map_client = self.create_client(SwitchMap, "switch_map", callback_group=self.cb_group)
        self.create_waypoint_client = self.create_client(
            CreateWaypoint, "create_waypoint", callback_group=self.cb_group
        )
        self.list_waypoints_client = self.create_client(ListWaypoints, "list_waypoints", callback_group=self.cb_group)

        # ── 銀行場景（2026-08-01 復原）────────────────────────
        self.enable_bank_tools = self.declare_parameter(
            "enable_bank_tools", True).get_parameter_value().bool_value
        # ── 系統提示詞要用哪一份 ────────────────────────────────
        # ★ 2026-08-14：這裡原本寫死讀 system_prompt.txt。
        #
        # config/ 底下一直有兩份提示詞：
        #   system_prompt.txt       導航／地圖版（沒有一句提到通報行員）
        #   system_prompt_bank.txt  銀行迎賓版（明寫真人服務→notify_staff_tool）
        # 但 _run_agent_loop 寫死讀前者，**銀行版從來沒有被載入過**。
        #
        # 而導航版第 6 節寫著「凡是問題出現『今天』『現在』『幾點』『星期幾』，
        # 都必須先呼叫查詢時間的工具」——「你們幾點關門」裡有「幾點」，
        # 模型是**照著提示詞做**才去叫 query_datetime_tool 的。
        # 2026-08-14 實測：冷開場（無對話歷史）5 次全部叫 query_datetime_tool，
        # 然後拿當下時刻去幻覺出「我們銀行今天晚上 17:30 已經關門了」。
        #
        # 留空 = 自動：有銀行工具就用銀行版，沒有就用導航版。
        self.system_prompt_file = self.declare_parameter(
            "system_prompt_file", "").get_parameter_value().string_value
        # 帶位目標的地點名稱。做成參數而不是寫死：地圖上的地點名是使用者
        # 自己取的，寫死「貴賓室」會在他取名「VIP室」時直接失敗。
        self.vip_room_waypoint_name = self.declare_parameter(
            "vip_room_waypoint_name", "貴賓室").get_parameter_value().string_value
        # ★ 2026-08-17：每一輪都扣住旁白（見 RosStreamHandler.begin_turn 的 docstring）。
        #   false = 只扣第 0 輪 = 2026-08-17 之前的行為（較低延遲、但可能唸出旁白）。
        self.hold_all_iterations = self.declare_parameter(
            "hold_all_iterations", True).get_parameter_value().bool_value
        # 通報只發話題，實際送出由 bank_reception_node 負責（通報中樞）
        self.staff_notify_pub = self.create_publisher(String, "staff_notify_request", 10)
        self.global_localization_client = ActionClient(
            self, GlobalLocalization, "global_localization", callback_group=self.cb_group
        )
        self.create_map_client = ActionClient(self, CreateMap, "create_map", callback_group=self.cb_group)
        self.navigate_client = ActionClient(self, Navigate, "navigate", callback_group=self.cb_group)

        # 初始化銀行知識庫
        self.knowledge_store = self._init_knowledge_store()

        # 建立發佈者和訂閱者
        self.llm_response_pub = self.create_publisher(String, "llm_response", 10)
        self.llm_stream_pub = self.create_publisher(String, "llm_stream", 10)
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

        # ★ 啟動時就把「實際會載入哪一份提示詞」印出來並確認檔案存在。
        #   ros2 param get 只證明參數伺服器存了值，不證明節點讀得到那個檔，
        #   所以這裡直接去解析路徑（這個坑專案裡踩過三次）。
        prompt_name = self._resolve_system_prompt_name()
        prompt_path = get_config_path(prompt_name)
        self.get_logger().info(
            f"✓ 系統提示詞: {prompt_name} "
            f"({'已找到' if prompt_path else '★找不到，會用內建預設'})"
        )
        self.get_logger().info(f"✓ LLM 對話服務節點已初始化 (模型: {self.model_name})")

    def _init_knowledge_store(self):
        """初始化銀行知識庫

        Returns:
            Optional[BankKnowledgeStore]: 知識庫實例，停用或找不到資料夾時為 None
        """
        if not self.enable_rag:
            self.get_logger().info("銀行知識庫已停用 (enable_rag=False)")
            return None

        knowledge_dir = Path(self.knowledge_dir) if self.knowledge_dir else get_knowledge_dir()
        if not knowledge_dir or not knowledge_dir.is_dir():
            self.get_logger().warning("✗ 找不到知識庫資料夾，銀行問答功能將無法使用")
            return None

        store = BankKnowledgeStore(
            knowledge_dir=knowledge_dir,
            ollama_base_url=self.ollama_base_url if self.embed_model else "",
            embed_model=self.embed_model,
            top_k=self.rag_top_k,
        )
        chunk_count = store.load()
        if not chunk_count:
            self.get_logger().warning(f"✗ 知識庫沒有任何內容: {knowledge_dir}")
            return None

        return store

    def _init_callback(self) -> None:
        """初始化回調函式"""
        self._init_timer.cancel()
        self._wait_for_services()

    def _wait_for_services(self) -> None:
        """等待所有基礎服務或動作就緒"""
        services = [
            ("list_maps", self.list_maps_client),
            ("create_waypoint", self.create_waypoint_client),
            ("list_waypoints", self.list_waypoints_client),
            ("switch_map", self.switch_map_client),
        ]
        # ★★ 2026-08-17：`global_localization` 移出這張「非等到不可」的清單 ★★
        #
        # 工具已下架（見 tools_map 上方的說明），LLM 節點再也不會呼叫它，
        # 但它原本留在這裡 -> 那個 action server 沒起來，**整個 LLM 節點就卡在
        # 初始化、連「你好」都不會回**。這是一個純粹多餘的啟動硬相依：
        # 對話功能完全不需要它，而它屬於導航堆疊（常常晚啟動或根本沒啟動）。
        #
        # client 本身**保留**（`self.global_localization_client`），HMI 的手動
        # 「全域定位」按鈕走的是 HMI 自己的 ActionClient，兩邊都不受影響。
        actions = [
            ("create_map", self.create_map_client),
            ("navigate", self.navigate_client),
        ]
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
                + " 導航堆疊晚點起來的話會自動接上，不必重啟本節點。")
        else:
            self.get_logger().info("✓ 所有基礎服務與動作已就緒")

    def _wait_for_future(self, future, timeout_sec: float) -> Any:
        """等待服務回應的輔助函式"""
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())

        # 阻塞當前獨立執行緒，直到被喚醒或超時
        if not event.wait(timeout=timeout_sec):
            raise TimeoutError(f"服務請求超時")

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
        try:
            return self._wait_for_future(goal_handle.get_result_async(), timeout_sec)
        except TimeoutError:
            try:
                goal_handle.cancel_goal_async()
                self.get_logger().warning(
                    f"⚠ {what}超過 {timeout_sec:.0f} 秒未完成，已送出取消指令")
            except Exception as exc:                       # noqa: BLE001
                self.get_logger().error(f"✗ {what}逾時後送取消指令失敗: {exc}")
            raise TimeoutError(f"{what}超過 {timeout_sec:.0f} 秒未完成，已中止本次請求")
    def _init_modern_llm_tools(self) -> None:
        """定義工具對照表"""

        @tool
        def create_map_tool(map_name: str) -> str:
            """建立一張新地圖，並自動載入該地圖與座標，除非使用者要求建立地圖，否則不應該呼叫此工具"""
            goal = CreateMap.Goal()
            goal.map_name = map_name
            future = self.create_map_client.send_goal_async(goal)
            try:
                goal_handle = self._wait_for_future(future, timeout_sec=5.0)

                if not goal_handle.accepted:
                    return f"執行結果: 失敗, 詳細信息: 建立地圖請求被系統拒絕"

                # ★ 2026-08-26：逾時要主動取消，見 _wait_for_action_result
                action_result = self._wait_for_action_result(goal_handle, 500.0, "建立地圖")

                id = action_result.result.map_info.map_id
                name = action_result.result.map_info.map_name

                if action_result.status == GoalStatus.STATUS_SUCCEEDED:
                    return f"執行結果: 成功, 地圖信息(包含地圖ID與地圖名稱): ({id}, {name}), 詳細信息: {action_result.result.message}"
                elif action_result.status == GoalStatus.STATUS_CANCELED:
                    return "執行結果: 取消, 詳細信息: 建立地圖請求被系統或使用者取消"
                elif action_result.status == GoalStatus.STATUS_ABORTED:
                    return f"執行結果: 失敗, 詳細信息: 建立地圖過程中發生錯誤"
                else:
                    return f"執行結果: 失敗, 詳細信息: {action_result.result.message}"
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: {str(e)}"

        @tool
        def list_maps_tool() -> str:
            """查詢現有的地圖列表信息"""
            req = ListMaps.Request()
            future = self.list_maps_client.call_async(req)
            try:
                res = self._wait_for_future(future, timeout_sec=5.0)
                if res.success:
                    ids = [wp.map_id for wp in res.maps_info]
                    names = [wp.map_name for wp in res.maps_info]
                    return f"執行結果: 成功, 地圖列表信息(包含地圖ID與地圖名稱): {list(zip(ids, names))}, 詳細信息: {res.message}"
                else:
                    return f"執行結果: 失敗, 詳細信息: {res.message}"
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: {str(e)}"

        @tool
        def switch_map_tool(map_id: str) -> str:
            """切換地圖，呼叫此工具時不應該使用地圖名稱"""
            req = SwitchMap.Request()
            req.map_id = map_id
            future = self.switch_map_client.call_async(req)
            try:
                res = self._wait_for_future(future, timeout_sec=5.0)
                if res.success:
                    return f"執行結果: 成功, 詳細信息: {res.message}"
                else:
                    return f"執行結果: 失敗, 詳細信息: {res.message}"
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: {str(e)}"

        @tool
        def create_waypoint_tool(waypoint_name: str) -> str:
            """在當前載入的地圖中建立一個地點，除非使用者要求建立地點，否則不應該呼叫此工具"""
            req = CreateWaypoint.Request()
            req.waypoint_name = waypoint_name
            future = self.create_waypoint_client.call_async(req)
            try:
                res = self._wait_for_future(future, timeout_sec=5.0)
                if res.success:
                    id = res.waypoint_info.waypoint_id
                    name = res.waypoint_info.waypoint_name
                    map_id = res.waypoint_info.map_id
                    x = res.waypoint_info.pose.position.x
                    y = res.waypoint_info.pose.position.y
                    return f"執行結果: 成功, 地點信息(包含地點ID、地點名稱、地圖ID、地點座標): ({id}, {name}, {map_id}, ({x}, {y})), 詳細信息: {res.message}"
                else:
                    return f"執行結果: 失敗, 詳細信息: {res.message}"
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: {str(e)}"

        @tool
        def list_waypoints_tool() -> str:
            """查詢當前載入的地圖中的地點列表信息，如果不知道導航地點，應該優先呼叫此工具"""
            req = ListWaypoints.Request()
            future = self.list_waypoints_client.call_async(req)
            try:
                res = self._wait_for_future(future, timeout_sec=5.0)
                if res.success:
                    ids = [wp.waypoint_id for wp in res.waypoints_info]
                    names = [wp.waypoint_name for wp in res.waypoints_info]
                    map_ids = [wp.map_id for wp in res.waypoints_info]
                    x = [wp.pose.position.x for wp in res.waypoints_info]
                    y = [wp.pose.position.y for wp in res.waypoints_info]
                    return f"執行結果: 成功, 地點列表信息(包含地點ID、地點名稱、地圖ID、地點座標): {list(zip(ids, names, map_ids, list(zip(x, y))))}, 詳細信息: {res.message}"
                else:
                    return f"執行結果: 失敗, 詳細信息: {res.message}"
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: {str(e)}"

        # ★★ 2026-08-17：global_localization_tool 已下架，不再暴露給 LLM ★★
        #
        # 這裡原本有一個 @tool global_localization_tool。拿掉的理由是安全：
        # 全域定位的實作會讓車子繞一個約 0.9 m 半徑的圓弧，而我們的走廊只有
        # 0.99 m 寬 —— 也就是「一被呼叫就會擦牆」。
        #
        # 8/17 實機事故（log: ~/.ros/log/python3_5596_1786954243007.log）：
        # 客人只是對著平板說「大門口」，模型就自己叫了這個工具，車子開始繞圈；
        # 第一次跑了 129 秒，模型接著又叫了第二次，那次卡到 300 秒逾時才回來。
        # ★ 客人的任何一句話都不該能讓車子動起來做這件事。
        #
        # 下架的是「LLM 的入口」而已：
        #   - /global_localization action 本身完全沒動，
        #   - self.global_localization_client 也保留（啟動時仍會等它就緒），
        #   - HMI 的「全域定位」按鈕走的是 hmi_server_node 自己的 action client
        #     （POST /api/localize -> _start_action），與這裡無關，照常可用。
        # 走廊裡真正該用的是 HMI 的 /api/localize/here（不需移動的掃描對齊）。

        @tool
        def navigate_tool(waypoint_id: str) -> str:
            """控制機器人導航到特定地點，請提供地點ID而非座標，如果不知道導航地點，應該優先呼叫 list_waypoints_tool 工具來查詢地點列表信息"""
            # ★ 2026-08-26（W7）：先確認這個 ID 真的存在。
            #   小模型會自己編 waypoint_id（或把地點**名稱**當成 ID 傳進來）。
            #   不擋的話會直接送出一個查無此點的 Navigate goal，錯誤要等到
            #   action server 回來才知道，訊息還不會告訴模型「該去查列表」。
            #   list_waypoints 是本機服務，成本很低。查不到列表就放行（不因為
            #   驗證機制自己壞掉而擋住正常導航）。
            try:
                _lw = self._wait_for_future(
                    self.list_waypoints_client.call_async(ListWaypoints.Request()), 5.0)
                if _lw is not None and _lw.success:
                    _ids = {wp.waypoint_id for wp in _lw.waypoints_info}
                    if waypoint_id not in _ids:
                        _names = [wp.waypoint_name for wp in _lw.waypoints_info]
                        return (f"執行結果: 失敗, 詳細信息: 地點ID「{waypoint_id}」不存在。"
                                f"現有地點：{_names}。請先呼叫 list_waypoints_tool "
                                "取得正確的地點ID再導航")
            except Exception as _exc:                      # noqa: BLE001
                self.get_logger().warning(f"導航前地點驗證略過（查詢失敗）: {_exc}")

            goal = Navigate.Goal()
            goal.waypoint_id = waypoint_id
            future = self.navigate_client.send_goal_async(goal)
            try:
                goal_handle = self._wait_for_future(future, timeout_sec=5.0)

                if goal_handle is None or not goal_handle.accepted:
                    return f"執行結果: 失敗, 詳細信息: 導航請求未被接受"

                # ★ 2026-08-26：逾時要主動取消，見 _wait_for_action_result
                action_result = self._wait_for_action_result(goal_handle, 200.0, "導航")

                if action_result.status == GoalStatus.STATUS_SUCCEEDED:
                    return f"執行結果: 成功, 詳細信息: {action_result.result.message}"
                elif action_result.status == GoalStatus.STATUS_CANCELED:
                    return "執行結果: 取消, 詳細信息: 導航請求被系統或使用者取消"
                elif action_result.status == GoalStatus.STATUS_ABORTED:
                    return f"執行結果: 失敗, 詳細信息: 導航過程中發生錯誤"
                else:
                    return f"執行結果: 失敗, 詳細信息: {action_result.result.message}"
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: {str(e)}"

        @tool
        def search_bank_knowledge_tool(question: str) -> str:
            """查詢本行的業務規定與分行資訊。凡是客戶詢問營業時間、櫃檯位置、開戶、換匯、
            存款、貸款、信用卡、手續費、掛失、證件文件等與本行有關的問題，都必須先呼叫此工具，
            嚴禁自行回答。請將客戶問題原句作為參數傳入"""
            if not self.knowledge_store:
                return "執行結果: 失敗, 詳細信息: 知識庫尚未設定，請據實告知客戶你查不到，並引導客戶洽詢櫃檯行員"
            try:
                return self.knowledge_store.build_context(question)
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: 知識庫查詢發生錯誤 {str(e)}"

        @tool
        def query_stock_price_tool(stock: str) -> str:
            """查詢台股即時股價，參數請填股票名稱或代號，例如「台積電」或「2330」"""
            try:
                return web_tools.query_stock_price(stock)
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: 股價查詢發生錯誤 {str(e)}"

        @tool
        def query_exchange_rate_tool(currency: str, amount: float = 1.0) -> str:
            """查詢外幣對新台幣的即時參考匯率。currency 填幣別，例如「美金」「日幣」；
            amount 填客戶要換算的外幣金額，客戶沒說金額就不用填"""
            try:
                return web_tools.query_exchange_rate(currency, amount)
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: 匯率查詢發生錯誤 {str(e)}"

        @tool
        def query_weather_tool(city: str = "台北") -> str:
            """查詢目前天氣，參數填城市名稱，客戶沒指定城市就不用填"""
            try:
                return web_tools.query_weather(city)
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: 天氣查詢發生錯誤 {str(e)}"

        @tool
        def query_datetime_tool() -> str:
            """查詢**此刻的鐘面時間**，例如「現在幾點」「今天幾號」「今天星期幾」。
            ★ 只在客戶問當下時刻時使用。客戶問「你們幾點關門」「營業到幾點」問的是
            本行的營業時段，那要用查銀行業務資料的工具，不要用這個"""
            try:
                return web_tools.query_datetime()
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: 時間查詢發生錯誤 {str(e)}"

        # 對照表與工具集
        self.tools_map = {
            "create_map_tool": create_map_tool,
            "list_maps_tool": list_maps_tool,
            "switch_map_tool": switch_map_tool,
            "create_waypoint_tool": create_waypoint_tool,
            "list_waypoints_tool": list_waypoints_tool,
            "navigate_tool": navigate_tool,
            # ★ global_localization_tool 蓄意不列在這裡（2026-08-17，見上方說明）。
            #   bind_tools() 只綁 tools_map 裡的東西，所以不在這張表上 = 模型看不到、
            #   也叫不到。要恢復請先解決 0.9 m 圓弧 vs 0.99 m 走廊的問題。
        }

        # ★ 2026-08-14：知識庫工具與銀行 FAQ 工具**不可以同時掛上**。
        #
        # search_bank_knowledge_tool 的描述寫「營業時間、櫃檯位置、開戶、換匯…
        # 都必須先呼叫此工具」，query_bank_faq_tool 的描述寫「營業時間、開戶、
        # 匯兌等」——兩個工具對同一批問題都宣稱自己是正解，機率被劈成兩半，
        # 3B 模型挑哪個變成擲骰子。（兩者的資料來源其實還是同一份 FAQ。）
        #
        # 銀行場景下統一走 query_bank_faq_tool，它內部會優先用 RAG 檢索
        # （見 bank_tools.py），所以知識庫的檢索能力沒有損失，只是入口收斂成一個。
        if self.knowledge_store and not self.enable_bank_tools:
            self.tools_map["search_bank_knowledge_tool"] = search_bank_knowledge_tool

        if self.enable_web_tools:
            # 匯率與時間留著：匯率是銀行業務（bank_faq.txt 有提到），
            # 時間則被系統提示詞 §6 的規則直接依賴。
            self.tools_map.update(
                {
                    "query_exchange_rate_tool": query_exchange_rate_tool,
                    "query_datetime_tool": query_datetime_tool,
                }
            )
            # ★★ 2026-08-26（W5）：股價／天氣拆成獨立開關 ★★
            #
            # 工具數量直接影響小模型的工具選擇準確率 —— 這是本檔 :660 附近
            # 已經記錄過的實測結論（qwen2.5:3b 在工具太多時會挑錯）。
            # 股價與天氣**都不在銀行迎賓的故事線裡**，卻各佔一個工具名額。
            #
            # ★ 預設**維持 True**，行為與 8/26 之前完全相同 —— 這是刻意的：
            #   在上機當天悄悄拿掉功能，比多兩個工具危險。
            #   彩排時若觀察到選錯工具，用
            #       -p enable_stock_weather_tools:=false
            #   一行就能收斂到 11 個工具，不必改程式。
            if self.declare_parameter(
                    "enable_stock_weather_tools",
                    True).get_parameter_value().bool_value:
                self.tools_map.update(
                    {
                        "query_stock_price_tool": query_stock_price_tool,
                        "query_weather_tool": query_weather_tool,
                    }
                )

        # ── 銀行場景工具（2026-08-01 從 git 歷史復原）────────────
        #
        # 帶位／通報／FAQ 三個工具。做成可關閉的（enable_bank_tools）是因為
        # 工具數量直接影響小模型的工具選擇準確率——qwen2.5:3b 在工具太多時
        # 會挑錯。純導航測試時關掉，銀行展示時開啟。
        #
        # 通報工具只發 ROS topic 不直接打 Telegram：實際送出集中在
        # bank_reception_node（通報中樞），這樣「通報」永遠只有一個出口，
        # 也讓 brain 沒啟動時工具仍可執行（會回報提示而不是拋例外）。
        if self.enable_bank_tools:
            try:
                from smartnav_llm.bank_tools import make_bank_tools
                self.tools_map.update(make_bank_tools(self))
            except ImportError as exc:
                self.get_logger().warning(f"✗ 銀行工具載入失敗（略過）: {exc}")

        self.get_logger().info(f"✓ 已載入 {len(self.tools_map)} 個工具: {', '.join(self.tools_map)}")

        # 初始化模型並綁定工具
        self.stream_handler = RosStreamHandler(self.llm_stream_pub, self.speech_text_pub, self._to_traditional)
        stream_handler = self.stream_handler
        raw_llm = ChatOllama(
            base_url=self.ollama_base_url,
            model=self.model_name,
            temperature=self.temperature,
            num_predict=self.num_predict,   # ★ 生成長度硬上限，見宣告處
            callbacks=[stream_handler],
        )
        self.llm_with_tools = raw_llm.bind_tools(list(self.tools_map.values()))

        # 定義對話提示模板，包含系統提示、歷史消息和工具調用結果
        self.prompt_template = ChatPromptTemplate.from_messages(
            [
                ("system", "{system_prompt}"),
                MessagesPlaceholder(variable_name="history"),
                ("human", "{input}"),
                MessagesPlaceholder(variable_name="agent_scratchpad"),
            ]
        )

        # 定義整合工具的 Agent Chain，將提示模板與工具調用結合起來
        self.agent_chain = RunnablePassthrough() | self.prompt_template | self.llm_with_tools

    def _to_traditional(self, text: str) -> str:
        """把模型吐出來的簡體字轉成台灣正體。轉換器不可用時原樣回傳。"""
        if not self._cc or not text:
            return text
        try:
            return self._cc.convert(text)
        except Exception:
            return text

    def user_text_callback(self, msg: String) -> None:
        """處理使用者輸入"""
        user_input = msg.data

        with self.agent_busy_lock:
            if self.is_agent_running:
                warning_msg = "系統目前正在處理上一個指令，請稍後再試..."
                self.get_logger().warn(warning_msg)
                self.llm_response_pub.publish(String(data=warning_msg))
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

    def _resolve_system_prompt_name(self) -> str:
        """決定要載入哪一份系統提示詞

        Returns:
            str: 提示詞檔名。參數留空時依 enable_bank_tools 自動選擇。
        """
        if self.system_prompt_file:
            return self.system_prompt_file
        return "system_prompt_bank.txt" if self.enable_bank_tools else "system_prompt.txt"

    def _load_system_prompt(self) -> str:
        """讀取系統提示詞（讀不到就退回內建預設）"""
        name = self._resolve_system_prompt_name()
        path = get_config_path(name)
        if path:
            with open(path, "r", encoding="utf-8") as f:
                return f.read()

        self.get_logger().warning(f"✗ 無法找到 {name}，使用內建預設提示語")
        return (
            "你是一個專業的智慧導航機器人助手，職責是幫助使用者管理地圖並完成多步驟導航\n"
            "你可以連續、分步驟地呼叫工具來完成任務，如果使用者給予複合指令，請一步一步調用工具\n"
        )

    def _run_agent_loop(self, user_input: str) -> None:
        """ReAct 自主思考迴圈"""
        system_content = self._load_system_prompt()

        with self.memory_lock:
            chat_history = self.memory.get_history_messages()

        agent_scratchpad: List[BaseMessage] = []
        max_iterations = 10
        self.get_logger().info("🧠 LLM 開始進入多步驟決策鏈...")

        try:
            for iteration in range(max_iterations):
                # ★ 2026-08-17：每一輪都扣。8/17 log 證實 iteration 1 也會
                #   「先講一段旁白、再去呼叫工具」（見 begin_turn 的 docstring）。
                self.stream_handler.begin_turn(
                    hold=(self.hold_all_iterations or iteration == 0))

                response = self.agent_chain.invoke(
                    {
                        "system_prompt": system_content,
                        "history": chat_history,
                        "input": user_input,
                        "agent_scratchpad": agent_scratchpad,
                    }
                )

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
                        + ("（且沒解出 tool_calls -> 工具呼叫很可能被截斷，"
                           "請調高 num_predict）" if not response.tool_calls else ""))

                # 檢查 LLM 是否需要叫工具
                if not response.tool_calls:
                    # 沒有工具呼叫 = 這是要講給客戶聽的最終答案
                    self.stream_handler.flush_held()
                    final_reply = self._to_traditional(str(response.content))
                    self.get_logger().info(f"🤖 Agent 最終決策回應: {final_reply}")

                    with self.memory_lock:
                        self.memory.add_message(HumanMessage(content=user_input))
                        self.memory.add_message(AIMessage(content=final_reply))

                    self.llm_response_pub.publish(String(data=final_reply))
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
                        try:
                            tool_result = self.tools_map[tool_name].invoke(tool_args)
                        except Exception as exc:           # noqa: BLE001
                            self.get_logger().error(
                                f"✗ 工具 {tool_name} 呼叫失敗（參數={tool_args}）: {exc}")
                            tool_result = (
                                "執行結果: 失敗, 詳細信息: 工具參數不完整或格式不正確，"
                                "請補齊必要資訊後重試，或改用其他方式協助客戶")
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
            self.llm_response_pub.publish(String(data=timeout_msg))

        except Exception as e:
            error_msg = f"✗ Agent 執行時發生異常: {str(e)}"
            self.get_logger().error(error_msg)
            self.llm_response_pub.publish(String(data=error_msg))
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
