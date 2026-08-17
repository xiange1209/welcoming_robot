#!/usr/bin/env python3
"""語音辨識節點

使用 sherpa-onnx 實現 ASR
訂閱音訊話題，將音訊轉換為文字
"""

import queue
import threading
from typing import Optional
from opencc import OpenCC
import numpy as np
import sherpa_onnx

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String, Bool

from smartnav_msgs.msg import AudioData
from smartnav_audio.voice_utils import AudioCodec, get_model_path, validate_audio_data


class SpeechRecognizerNode(Node):
    """語音辨識節點

    使用 sherpa-onnx 進行 ASR 處理，輸出辨識結果

    Subscriptions:
        /audio_in (smartnav_msgs/AudioData): 音訊數據流

    Publishers:
        /user_text (std_msgs/String): 辨識到的完整文字
        /partial_text (std_msgs/String): 辨識過程中的部分文字
    """

    def __init__(self):
        """初始化語音辨識節點"""
        super().__init__("speech_recognizer_node")

        # 參數聲明
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("num_threads", 4)
        self.declare_parameter("user_text_topic", "/user_text", ParameterDescriptor(description="辨識到的完整文字話題"))
        self.declare_parameter(
            "partial_text_topic", "/partial_text", ParameterDescriptor(description="辨識過程中的部分文字話題")
        )
        self.declare_parameter("audio_topic", "/audio_in", ParameterDescriptor(description="音訊數據流話題"))

        # ── 後處理（2026-08-14 新增）────────────────────────────
        #
        # 使用者實測後的原話：「有時候會抓不到字或是辨識錯誤等等，
        # 它沒有自動修正或是自動刪除」。在這之前，辨識結果是**原封不動**
        # 直接發到 /user_text 的 —— 一個字的雜訊、重複字、同音錯字全部照發。
        #
        # ★ 分工刻意這樣切：
        #   這裡只做「一定是錯的」那種清洗（太短、整句同一個字）；
        #   同音錯字交給 LLM 的系統提示詞用語意判斷（見 system_prompt_bank.txt 第 10 節）。
        #   理由：同音字修正表是**脆的** —— 表越大，把對的改成錯的機會越高，
        #   而且每個新場景都要重寫。LLM 本來就在做語意理解，讓它一併吃掉更穩。
        #   下面的 corrections 只留「銀行情境下幾乎不可能是別的意思」那幾條當保險。
        self.declare_parameter(
            "min_text_length", 2,
            ParameterDescriptor(description="短於這個字數的最終結果直接丟棄（雜訊）"))
        self.declare_parameter(
            "drop_uniform_text", True,
            ParameterDescriptor(description="整句都是同一個字時丟棄，例如『嗯嗯嗯嗯』"))
        self.declare_parameter(
            "collapse_repeats", 3,
            ParameterDescriptor(
                description="同一個字連續達到這個次數就截短，保留前 N-1 個。"
                            "預設 3 = 連三個以上截成兩個（中文疊字合法，兩個要留）；0 = 關閉"))
        self.declare_parameter(
            "enable_corrections", True,
            ParameterDescriptor(description="是否套用內建的銀行詞彙同音修正表"))

        # ── 資源保護（2026-08-14 新增）★ 使用者：「我怕跟他講太多話它會把整機卡死」──
        #
        # 這個擔心是對的，而且**有一個具體的無界成長點**：
        #
        #   `is_final` 是由 voice_trigger 的 VAD 決定的，**不是** sherpa 自己判斷的。
        #   （2026-08-17 起 `is_endpoint()` 已接上，多了一條獨立的斷句路徑，
        #     但它需要**尾隨靜音**才會觸發 —— 持續噪音下一樣不會收句，
        #     所以下面這個時間上限仍然是最後一道防線，不能因為接了端點偵測就拿掉。）
        #   只要 VAD 一直判定「還在講話」（持續噪音、冷氣、人聲背景），
        #   同一個 stream 會一直被餵音訊，**永遠不會 reset**：
        #     - stream 內部的特徵緩衝隨時間線性成長
        #     - modified_beam_search 的假設集合也跟著長
        #   Pi 4 只有 8 GB 而相機＋導航已經吃掉大半，這是真的會撐爆的路徑。
        #
        # 對策：自己計時，超過 max_utterance_sec 就**強制收尾**（把目前結果發出去、
        # 重建 stream）。這同時也修好「一句話講太長就再也不出結果」的體感問題。
        # 20 秒是 sherpa 原本 rule3 的值，沿用它保持一致。
        self.declare_parameter(
            "max_utterance_sec", 20.0,
            ParameterDescriptor(description="單一語句最長時間，超過就強制收尾並重建 stream（防記憶體無界成長）"))
        # 佇列積壓警告門檻。RTF 實測 0.76，餘裕只有 24%，導航/相機一忙就會
        # 超過 1.0 -> 佇列愈積愈多 -> 延遲愈來愈大，而**這件事原本完全看不見**。
        self.declare_parameter(
            "queue_warn_depth", 20,
            ParameterDescriptor(description="音訊佇列積壓超過這個數就警告（上限 50）"))

        # ── ★★ 2026-08-17（筆電端）：接上 sherpa 自己的端點偵測 ★★ ─────────
        #
        # 在此之前，`enable_endpoint_detection=True` 與那三條 rule 全是**死設定**
        # ——程式從來沒有呼叫過 `recognizer.is_endpoint(stream)`。斷句 100% 由
        # voice_trigger 的 VAD 決定，於是兩個方向都會壞：
        #
        #   VAD 太早收   -> 句子被切一半，漏字
        #   VAD 一直不收 -> 好幾句累積在同一個 stream，解碼結果黏在一起且重複
        #                   （8/17 實機症狀：「你是誰**你誰你好你好**喂」）
        #
        # 現在多一個**獨立**的斷句來源：模型自己聽到足夠的尾隨靜音就收句。
        # 兩者是 **OR**：VAD 說結束仍然算結束，所以這是純粹的增益，不會讓
        # 原本會斷的地方變得不斷。
        #
        # ★ 為什麼 sherpa 這條真的會先觸發：VAD 的 silence_timeout 是 500 ms，
        #   照理說永遠比 rule2 的 1.2 s 早。但實機黏句正好證明**噪音讓 VAD 卡在
        #   「還在講話」**——那正是 sherpa 這條當備援的時機。它看的是自己解碼出
        #   的 blank 幀，不受 VAD 的能量門檻影響。
        # ★ 2026-08-17：模型檔精度。原本寫死 encoder/decoder 用 fp32、只有 joiner 用 int8
        #   —— 把最不影響效能的那塊量化了，最重的 encoder（315 MB fp32）反而沒有。
        #   詳見 _init_sherpa_onnx_asr 的長註解。false = 退回舊行為。
        self.declare_parameter(
            "prefer_int8_model", True,
            ParameterDescriptor(description="模型檔有 .int8 版就優先載入（RPi4 上明顯較快，準確率同級）"))

        self.declare_parameter(
            "use_sherpa_endpoint", True,
            ParameterDescriptor(description="除了 VAD 之外，也讓 sherpa 自己判斷句尾（OR 關係）"))
        # ★ 這三個值在接上 is_endpoint() 之後**才開始真的有作用**。
        #   （舊註解寫「調這幾個數字會白調」——那在 2026-08-17 之前是對的，現在不是了。）
        self.declare_parameter(
            "rule1_min_trailing_silence", 2.4,
            ParameterDescriptor(description="還沒解出任何字時，靜音多久算句尾（秒）"))
        self.declare_parameter(
            "rule2_min_trailing_silence", 1.2,
            ParameterDescriptor(description="★ 已經解出字之後，靜音多久算句尾（秒）—— 這條最常觸發"))
        self.declare_parameter(
            "rule3_min_utterance_length", 20.0,
            ParameterDescriptor(description="單句長度上限（秒），與 max_utterance_sec 對齊"))

        # ── 熱詞（contextual biasing）：專治同音錯字，見 _init_sherpa_onnx_asr ──
        self.declare_parameter(
            "hotwords_file", "",
            ParameterDescriptor(description="熱詞檔路徑；留空會自動找模型目錄或 config/hotwords.txt"))
        self.declare_parameter(
            "hotwords_score", 1.8,
            ParameterDescriptor(description="熱詞加分權重。太高會讓模型把不相干的話也扭曲成熱詞"))
        self.declare_parameter(
            "max_active_paths", 4,
            ParameterDescriptor(
                description="beam 寬度。調大較準但較慢 —— 實測 RTF 已經 0.76，"
                            "只剩 24% 餘裕，調到 8 很可能讓佇列積壓，先確認 CPU 有空再動"))

        self.max_utterance_sec = self.get_parameter("max_utterance_sec").get_parameter_value().double_value
        self.queue_warn_depth = self.get_parameter("queue_warn_depth").get_parameter_value().integer_value
        self._stream_samples = 0          # 目前這個 stream 已經吃進多少取樣點
        self._queue_warned = False

        # 端點偵測（2026-08-17）。★ `_sherpa_endpoint_ok` 是**執行期**的能力旗標：
        # 舊版 sherpa-onnx 沒有 `is_endpoint()`，第一次呼叫失敗就永久退回純 VAD 斷句，
        # 只警告一次。絕對不能讓「多一個斷句來源」這種增益功能害整個辨識掛掉。
        self.prefer_int8_model = self.get_parameter(
            "prefer_int8_model").get_parameter_value().bool_value
        self.use_sherpa_endpoint = self.get_parameter(
            "use_sherpa_endpoint").get_parameter_value().bool_value
        self.rule1_min_trailing_silence = self.get_parameter(
            "rule1_min_trailing_silence").get_parameter_value().double_value
        self.rule2_min_trailing_silence = self.get_parameter(
            "rule2_min_trailing_silence").get_parameter_value().double_value
        self.rule3_min_utterance_length = self.get_parameter(
            "rule3_min_utterance_length").get_parameter_value().double_value
        self._sherpa_endpoint_ok = self.use_sherpa_endpoint
        self._endpoint_hits = 0           # sherpa 斷了幾句（VAD 沒斷的那些）

        self.min_text_length = self.get_parameter("min_text_length").get_parameter_value().integer_value
        self.drop_uniform_text = self.get_parameter("drop_uniform_text").get_parameter_value().bool_value
        self.collapse_repeats = self.get_parameter("collapse_repeats").get_parameter_value().integer_value
        self.enable_corrections = self.get_parameter("enable_corrections").get_parameter_value().bool_value

        # 參數獲取
        self.sample_rate = self.get_parameter("sample_rate").get_parameter_value().integer_value
        self.num_threads = self.get_parameter("num_threads").get_parameter_value().integer_value
        self.user_text_topic = self.get_parameter("user_text_topic").get_parameter_value().string_value
        self.partial_text_topic = self.get_parameter("partial_text_topic").get_parameter_value().string_value
        audio_topic = self.get_parameter("audio_topic").get_parameter_value().string_value

        # 初始化 OpenCC 轉換器 (簡體中文 -> 繁體中文)
        self.opencc = None
        try:
            # 2026-08-14：這裡原本寫 "s2twp.json"，但這版 opencc 會自己補上 .json 副檔名，
            # 變成找 s2twp.json.json → 初始化失敗被 except 吃掉，辨識結果就一直是簡體。
            # smartnav_llm/llm_service_node.py 用的是不帶副檔名的 "s2twp"，這裡對齊它。
            self.opencc = OpenCC("s2twp")  # 簡體 -> 繁體
            self.get_logger().info("OpenCC 轉換器已初始化")
        except Exception as e:
            self.get_logger().warning(f"OpenCC 初始化失敗: {e}")

        # 初始化 sherpa-onnx ASR 模型
        self.recognizer = None
        self.stream = None
        self._init_sherpa_onnx_asr()

        # 追蹤上一次發佈的部分結果，避免發佈重複的結果
        self.last_partial_result = None

        # 多線程處理隊列
        self.audio_queue = queue.Queue(maxsize=50)
        self.is_running = True
        self.worker_thread = threading.Thread(target=self._process_audio_queue, daemon=True)
        self.worker_thread.start()

        self.is_playing = False
        self._playing_lock = threading.Lock()

        self.callback_group = ReentrantCallbackGroup()

        # QoS 設定
        audio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 訂閱器
        self.audio_sub = self.create_subscription(
            AudioData,
            audio_topic,
            self._audio_callback,
            audio_qos,
            callback_group=self.callback_group,
        )

        # 發佈器
        self.user_text_pub = self.create_publisher(String, self.user_text_topic, 10)
        self.partial_text_pub = self.create_publisher(String, self.partial_text_topic, 10)
        # 訂閱器
        self.playback_status_sub = self.create_subscription(
            Bool,
            "playback_status",
            self._playback_status_callback,
            status_qos,
            callback_group=self.callback_group,
        )

        self.get_logger().info("✓ 語音辨識節點已初始化")
        self.get_logger().info(f"  訂閱話題: {audio_topic}")
        self.get_logger().info(f"  發布話題: {self.user_text_topic}")
        self.get_logger().info(f"  發布話題: {self.partial_text_topic}")
        self.get_logger().info(f"  採樣率: {self.sample_rate} Hz")

    def _init_sherpa_onnx_asr(self) -> None:
        """初始化 sherpa-onnx ASR 模型

        載入 ASR 模型檔案並創建語音辨識流
        """
        try:
            # 使用本地 ASR 模型
            asr_model_dir = get_model_path("asr")
            if asr_model_dir:
                # ★★ 2026-08-17：優先載入 int8 版本 ★★
                #
                # 原本寫死成 encoder/decoder 用 fp32、**只有 joiner 用 int8**：
                #
                #     encoder-epoch-99-avg-1.onnx        fp32  315 MB   ← 幾乎全部的運算量
                #     decoder-epoch-99-avg-1.onnx        fp32   14 MB
                #     joiner-epoch-99-avg-1.int8.onnx    int8  3.1 MB   ← 最小、最不影響效能的那塊
                #
                # 這個組合看起來是意外：**把最不重要的那塊量化了，最重的那塊留在全精度**。
                # sherpa-onnx 官方對資源受限裝置的建議就是全部用 int8
                # （encoder int8 版是 174 MB，比 fp32 小 45%）。
                #
                # 為什麼這件事重要：實測 RTF **0.76**、餘裕只剩 24%，
                # 相機一開就把佇列塞爆（「音訊處理佇列已滿」1405 筆）。
                # encoder 量化是**不換模型、不掉準確率等級**就能拿到的效能，
                # 在 Cortex-A72 上 int8 走 NEON dot-product 路徑，
                # 【推論】約 1.5~2 倍——這比換任何模型都便宜。
                #
                # ⚠ 仍然是【推論】，上機要用 8/18 驗證清單段九實測 RTF 對照。
                # 想退回舊行為（例如懷疑準確率掉了）：`-p prefer_int8_model:=false`
                def _pick(stem):
                    """同一個 stem 有 int8 就用 int8，沒有就退回 fp32。

                    ★ 刻意不加 `-> Path` 型別標註：這個檔沒有 `from pathlib import Path`
                      （`asr_model_dir` 是 `get_model_path()` 回傳的），
                      標註會在 def 當下就 NameError。
                    """
                    if self.prefer_int8_model:
                        cand = asr_model_dir / f"{stem}.int8.onnx"
                        if cand.exists():
                            return cand
                    return asr_model_dir / f"{stem}.onnx"

                encoder_file = _pick("encoder-epoch-99-avg-1")
                decoder_file = _pick("decoder-epoch-99-avg-1")
                joiner_file = _pick("joiner-epoch-99-avg-1")
                tokens_file = asr_model_dir / "tokens.txt"

                if not all(f.exists() for f in [encoder_file, decoder_file, joiner_file, tokens_file]):
                    self.get_logger().warning(f"ASR 模型文件不完整: {asr_model_dir}")
                    return

                self.get_logger().info(f"載入 ASR 模型: {asr_model_dir}")
                # ★ 逐檔印出精度與大小。這是唯一能證明「int8 真的生效」的憑據——
                #   `ros2 param get` 只會告訴你參數是 true，不會告訴你檔案存不存在。
                for label, f in (("encoder", encoder_file), ("decoder", decoder_file),
                                 ("joiner", joiner_file)):
                    prec = "int8" if ".int8." in f.name else "fp32"
                    self.get_logger().info(
                        f"    {label}: {prec}  {f.stat().st_size / 1024 / 1024:.0f} MB  ({f.name})")

                # ★★ 2026-08-14：啟用熱詞（contextual biasing）★★
                #
                # 使用者回報「辨識錯誤」，而中文 ASR 的錯幾乎都是**同音字**
                # （開護/開戶、帶款/貸款、題款/提款）。熱詞就是為這件事設計的：
                # 解碼時對清單裡的詞加分，讓同音候選裡的正確那個勝出。
                #
                # ★ 前提條件本來就已經滿足：熱詞只在 decoding_method 為
                #   "modified_beam_search" 時有效，而這裡本來就是。
                #   （greedy_search 用不了 —— 它每步只留一個候選，沒有東西可以加分。）
                #
                # 檔案格式：一行一個詞，字與字之間空一格（char-based 中文模型的
                # token 就是單字）。詞尾可加 `:分數` 覆寫個別權重。
                # 預設檔在 smartnav_audio/config/hotwords.txt。
                #
                # ⚠ hotwords_score 不要調太高：分數過大會讓模型「聽到什麼都往
                #   熱詞靠」，把不相干的句子也扭曲成銀行詞彙。1.5~2.0 是常用區間，
                #   這裡取 1.8。調高之前先確認錯的真的是同音字而不是漏字。
                hot_file = self.get_parameter("hotwords_file").get_parameter_value().string_value
                hot_score = self.get_parameter("hotwords_score").get_parameter_value().double_value
                if not hot_file:
                    # asr_model_dir = <share>/smartnav_audio/models/asr
                    #   .parent        -> models/
                    #   .parent.parent -> <share>/smartnav_audio/
                    # ★ 不要用 get_model_path("config") —— 那個函式一律接在
                    #   models/ 底下，會解析成 models/config（不存在）。
                    share_dir = asr_model_dir.parent.parent
                    for cand in (share_dir / "config" / "hotwords.txt",
                                 asr_model_dir.parent / "hotwords.txt",
                                 asr_model_dir / "hotwords.txt"):
                        if cand.exists():
                            hot_file = str(cand)
                            break

                kwargs = dict(
                    tokens=str(tokens_file),
                    encoder=str(encoder_file),
                    decoder=str(decoder_file),
                    joiner=str(joiner_file),
                    num_threads=self.num_threads,
                    sample_rate=self.sample_rate,
                    feature_dim=80,
                    decoding_method="modified_beam_search",
                    max_active_paths=self.get_parameter(
                        "max_active_paths").get_parameter_value().integer_value,
                    # ★ 2026-08-17 起這三條 rule **會真的生效**了 —— `recognize_audio()`
                    #   已接上 `recognizer.is_endpoint(stream)`。在那之前它們是死設定，
                    #   斷句 100% 由 voice_trigger 的 VAD（silence_timeout 500 ms）決定。
                    #   現在兩個來源是 OR：誰先判定句尾就誰收句。
                    #   ⚠ 調小 rule2 會讓 sherpa 變成主要斷句者、回應更快但更容易切斷
                    #     講話中間的停頓；調大則退回幾乎全由 VAD 決定。
                    enable_endpoint_detection=True,
                    rule1_min_trailing_silence=self.rule1_min_trailing_silence,
                    rule2_min_trailing_silence=self.rule2_min_trailing_silence,
                    rule3_min_utterance_length=self.rule3_min_utterance_length,
                )

                # ★★ 2026-08-17：sherpa-onnx **不會**忽略 `#` 註解行 ★★
                #
                # hotwords.txt 原本的檔頭註解寫「以 # 開頭的行與空行會被 sherpa-onnx
                # 忽略」，**那是錯的**。它逐行解析，而說明格式用的範例
                # 「詞尾可加 `:分數`，例如 `貴 賓 室 :2.5`」本身就在註解裡，
                # 於是解析器拿 `:2.5` 去 std::stof 而丟出例外：
                #
                #     熱詞載入失敗（沿用無熱詞模式，辨識仍可用）: stof
                #
                # 因為有 try/except 退回無熱詞模式，**辨識照常運作、只是熱詞從沒生效**
                # —— 8/17 實機把「櫃檯」認成「貴」「貴臺」，正是熱詞要治的病。
                # ★ 這種「功能靜默失效但系統看起來正常」是這個專案最常見的失敗型態。
                #
                # 解法：餵給 sherpa 之前先濾掉註解與空行，寫成暫存檔。
                # 這樣原檔可以保留給人看的說明，又不會炸。
                if hot_file:
                    try:
                        import tempfile
                        clean = []
                        # ★ 這個檔案沒有 import pathlib，用內建 open 就好
                        with open(hot_file, "r", encoding="utf-8") as _hf:
                            _lines = _hf.read().splitlines()
                        for raw in _lines:
                            line = raw.strip()
                            if line and not line.startswith("#"):
                                clean.append(line)
                        if clean:
                            tf = tempfile.NamedTemporaryFile(
                                mode="w", suffix=".txt", delete=False, encoding="utf-8")
                            tf.write("\n".join(clean) + "\n")
                            tf.close()
                            self._hotwords_clean = tf.name      # 保留參考，避免被 GC
                            self.get_logger().info(
                                f"熱詞檔已濾除註解：{len(clean)} 個詞（原檔 {hot_file}）")
                            hot_file = tf.name
                        else:
                            self.get_logger().warning(f"熱詞檔濾除註解後是空的：{hot_file}")
                            hot_file = ""
                    except Exception as e:  # noqa: BLE001
                        self.get_logger().warning(f"熱詞檔前處理失敗，改用原檔: {e}")

                recognizer = None
                if hot_file:
                    try:
                        recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
                            hotwords_file=hot_file, hotwords_score=hot_score, **kwargs)
                        self.get_logger().info(
                            f"✓ 熱詞已啟用：{hot_file}（score {hot_score:.1f}）")
                    except Exception as e:  # noqa: BLE001
                        # 不同 sherpa-onnx 版本的參數名不一定相同。熱詞是加分項，
                        # 不能因為它讓整個 ASR 起不來 —— 退回沒有熱詞的版本。
                        self.get_logger().warning(
                            f"熱詞載入失敗（沿用無熱詞模式，辨識仍可用）: {e}")
                        recognizer = None
                if recognizer is None:
                    recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(**kwargs)
                    if not hot_file:
                        self.get_logger().info("未提供熱詞檔（hotwords_file），跳過熱詞")

                self.recognizer = recognizer
                self.stream = recognizer.create_stream()
            else:
                self.get_logger().warning("未找到本地 ASR 模型目錄")
        except Exception as e:
            self.get_logger().error(f"初始化 ASR 模型失敗: {e}")

    def _audio_callback(self, msg: AudioData) -> None:
        """音訊話題回呼函數

        Args:
            msg: 音訊訊息
        """
        with self._playing_lock:
            if self.is_playing:
                return

        try:
            # 驗證採樣率
            if msg.sample_rate != self.sample_rate:
                self.get_logger().error(f"採樣率不匹配: 期望 {self.sample_rate} Hz，收到 {msg.sample_rate} Hz")
                return

            # 解碼音訊數據
            audio = self._decode_audio(msg)
            if audio is None or len(audio) == 0:
                return

            # 進行語音辨識
            self.audio_queue.put_nowait((audio, msg.is_final))
        except queue.Full:
            self.get_logger().warning("音訊處理佇列已滿，丟棄當前音訊塊")
        except Exception as e:
            self.get_logger().error(f"處理音訊失敗: {e}")

    def _process_audio_queue(self) -> None:
        """背景工作執行緒，處理語音辨識推理

        持續從音訊隊列中取出數據並進行語音辨識
        """
        while self.is_running:
            try:
                # 設定 timeout 讓執行緒能定期檢查 self.is_running
                queue_item = self.audio_queue.get(timeout=0.5)
                if queue_item is None:
                    self.audio_queue.task_done()
                    break

                audio, is_final = queue_item

                with self._playing_lock:
                    if self.is_playing:
                        self.audio_queue.task_done()
                        continue

                self.recognize_audio(audio, is_final=is_final)
                self.audio_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"背景辨識執行緒出錯: {e}")

    def _decode_audio(self, msg: AudioData) -> Optional[np.ndarray]:
        """解碼音訊數據

        Args:
            msg: 音訊訊息

        Returns:
            Optional[np.ndarray]: 解碼後的音訊數據 (float32)
        """
        try:
            # 根據格式解碼
            if msg.format == "pcm_s16le":
                audio = AudioCodec.decode_pcm_s16le(bytes(msg.data))
            elif msg.format == "pcm_f32le":
                audio = AudioCodec.decode_pcm_f32le(bytes(msg.data))
            elif msg.format == "pcm_s32le":
                audio = AudioCodec.decode_pcm_s32le(bytes(msg.data))
            else:
                self.get_logger().warning(f"✗ 不支援的音訊格式: {msg.format}")
                return None

            valid, error = validate_audio_data(audio, msg.sample_rate)
            if not valid:
                self.get_logger().warning(f"✗ 無效的音訊數據: {error}")
                return None

            return audio
        except Exception as e:
            self.get_logger().error(f"解碼音訊失敗: {e}")
            return None

    def recognize_audio(self, audio: np.ndarray, is_final: bool = False) -> None:
        """進行語音辨識

        Args:
            audio: 音訊數據陣列 (float32 格式)
            is_final: 是否為最終音訊塊 (true 表示說話已結束)
        """
        with self._playing_lock:
            if self.is_playing:
                return

        if self.recognizer is None or self.stream is None:
            self.get_logger().warning("ASR 模型未初始化")
            return

        if len(audio) == 0:
            self.get_logger().warning("接收到空音訊")
            return

        try:
            # 接受波形數據
            self.stream.accept_waveform(self.sample_rate, audio)

            # ★ 2026-08-14 防記憶體無界成長：VAD 一直不說結束時強制收尾。
            #   `is_final` 來自 voice_trigger 的 VAD，不是 sherpa 自己判斷的，
            #   所以持續噪音會讓同一個 stream 永遠不 reset（見 __init__ 的長註解）。
            self._stream_samples += len(audio)
            if not is_final and self.max_utterance_sec > 0:
                secs = self._stream_samples / float(self.sample_rate)
                if secs >= self.max_utterance_sec:
                    self.get_logger().warning(
                        f"⏱ 單句已達 {secs:.1f} 秒（上限 {self.max_utterance_sec:.0f}），"
                        "強制收尾並重建 stream —— VAD 可能被持續噪音卡在『還在講話』")
                    is_final = True

            # 最終塊，通知引擎結束輸入
            if is_final:
                self.stream.input_finished()

            # 檢查流是否已準備好進行解碼
            while self.recognizer.is_ready(self.stream):
                with self._playing_lock:
                    if self.is_playing:
                        return

                self.recognizer.decode_stream(self.stream)

            # ★★ 2026-08-17：問 sherpa 自己「這裡是不是句尾」★★
            #
            # 必須在 get_result() **之前**問、在 reset() **之後**才失效 ——
            # 端點狀態是解碼器內部的尾隨靜音計數，reset 會把它清掉。
            # 只在 VAD 還沒收句時才問（is_final 已經是句尾，不用再問一次）。
            endpoint = False
            if not is_final and self._sherpa_endpoint_ok:
                try:
                    endpoint = bool(self.recognizer.is_endpoint(self.stream))
                except Exception as exc:
                    # 舊版 sherpa-onnx 可能沒有這個方法 -> 永久退回純 VAD 斷句。
                    # 只警告一次，之後完全靜默（否則每 512 取樣就洗一行）。
                    self._sherpa_endpoint_ok = False
                    self.get_logger().warning(
                        f"sherpa 端點偵測不可用，改由 VAD 單獨斷句（辨識不受影響）: {exc}")

            result = self.recognizer.get_result(self.stream)
            if result and result.strip():
                converted_text = self._convert_simplified_to_traditional(result)
                if converted_text:
                    if is_final or endpoint:
                        src = "VAD" if is_final else "sherpa 端點"
                        self.get_logger().info(f"✓ 最終結果（{src} 斷句）: '{converted_text}'")
                        # ★ 2026-08-14：發出去之前先清洗。回傳 None 代表這句是雜訊，
                        #   直接不發 —— 這就是使用者要的「自動刪除」。
                        #   ⚠ 丟棄要 log，否則現場會變成「我明明有講話卻沒反應」，
                        #     而那跟麥克風壞掉、VAD 沒觸發長得一模一樣。
                        cleaned = self._postprocess(converted_text)
                        if cleaned:
                            self.user_text_pub.publish(String(data=cleaned))
                        # 重置上一次部分結果
                        self.last_partial_result = None
                    else:
                        # 只在部分結果與上一次不同時才發佈
                        if converted_text != self.last_partial_result:
                            self.get_logger().debug(f"▶ 部分結果: '{converted_text}'")
                            self.partial_text_pub.publish(String(data=converted_text))
                            self.last_partial_result = converted_text

            if is_final:
                # 立即重置流，準備下一句識別
                self.stream = self.recognizer.create_stream()
                self._stream_samples = 0   # ★ 重建 stream 一定要歸零，否則計時器會一路累加
            elif endpoint:
                # ★ sherpa 斷句用 `reset()` 而**不是** `create_stream()`：
                #   VAD 還在說「人在講話」，音訊會繼續進來。reset 只清掉解碼假設與
                #   端點計數、保留同一個 stream 物件，下一段音訊接得上；
                #   create_stream 則會把 accept_waveform 之後尚未消化的內部緩衝丟掉，
                #   造成句與句交界處漏字 —— 這正是我們想修的症狀。
                self._reset_stream_for_endpoint()
        except Exception as e:
            self.get_logger().error(f"✗ 語音辨識失敗: {e}")
            # 重置流
            self.stream = self.recognizer.create_stream()

    def _reset_stream_for_endpoint(self) -> None:
        """sherpa 判定句尾後，把解碼器狀態清乾淨但**保留同一個 stream**。

        `OnlineRecognizer.reset(stream)` 是官方端點偵測範例的用法。舊版若沒有這個
        方法，退回 `create_stream()` —— 會在交界處漏一點音訊，但總比整個節點掛掉好。
        """
        try:
            self.recognizer.reset(self.stream)
        except Exception as exc:
            self.get_logger().warning(f"reset(stream) 不可用，改用重建 stream: {exc}")
            self.stream = self.recognizer.create_stream()
        self._stream_samples = 0
        self.last_partial_result = None
        self._endpoint_hits += 1
        # ★ 每 10 句印一次。這個數字是判斷「VAD 是不是根本沒在斷句」的關鍵指標：
        #   若它跟總句數差不多，代表 VAD 幾乎從不收句（被噪音卡住），
        #   該回頭調 voice_trigger 的門檻，而不是繼續在 ASR 這端加工。
        if self._endpoint_hits % 10 == 0:
            self.get_logger().info(f"（sherpa 端點已接手斷句 {self._endpoint_hits} 句）")

    # ── 後處理 ────────────────────────────────────────────────
    #
    # 只放「銀行情境下幾乎不可能是別的意思」的詞。★ 加新條目前先問一句：
    # 「有沒有哪個客人可能真的想講左邊那個詞？」有的話就不要加，交給 LLM 判斷。
    # 例：不要加「保鮮->保險」——真的有人會問保鮮膜；那條放在系統提示詞裡由語意處理。
    ASR_CORRECTIONS = {
        "開護": "開戶",
        "開互": "開戶",
        "題款": "提款",
        "帶款": "貸款",
        "代款": "貸款",
        "櫃台": "櫃檯",
        "服務台": "服務檯",
        "型員": "行員",
    }

    def _postprocess(self, text: str) -> Optional[str]:
        """清洗最終辨識結果。回傳 None 代表這句該丟掉。

        ★ 每一條規則都會 log「改了什麼」，因為**沒有量測就不知道規則是幫忙還是幫倒忙**。
          明天要拿這些 log 統計各條規則的觸發次數，觸發卻改錯的就砍掉。
        """
        raw = text.strip()
        if not raw:
            return None

        # (1) 太短 = 雜訊。VAD 誤觸發時常常吐出一兩個字。
        if len(raw) < self.min_text_length:
            self.get_logger().info(f"⊘ 丟棄（太短 {len(raw)} < {self.min_text_length}）: '{raw}'")
            return None

        # (2) 整句同一個字：「嗯嗯嗯」「啊啊啊啊」。這是持續噪音的典型輸出。
        #
        # ★ 必須排除長度 2 —— 中文的兩字疊詞是合法且高頻的：
        #   **「謝謝」**、「好好」、「慢慢」、「快快」。第一版寫成
        #   `len(set(raw)) == 1` 就把「謝謝」丟掉了，而那是迎賓場景最常聽到的一句。
        #   症狀會是「我明明有講謝謝，它完全沒反應」，跟麥克風壞掉一模一樣。
        #   （寫完當場用假資料跑一遍才發現的 —— 這種規則一定要先餵幾句真實例句。）
        if self.drop_uniform_text and len(raw) >= 3 and len(set(raw)) == 1:
            self.get_logger().info(f"⊘ 丟棄（整句同一個字）: '{raw}'")
            return None

        out = raw

        # (3) 連續重複字摺疊。串流辨識在音訊斷斷續續時會把同一個字吐很多次。
        #     ★ 門檻設 3 而不是 2 —— 中文本來就有「謝謝」「好好」「慢慢」這種疊字，
        #       連續兩個字是正常的，三個以上才幾乎一定是辨識問題。
        if self.collapse_repeats and self.collapse_repeats >= 2:
            chars, run, prev = [], 0, None
            for ch in out:
                run = run + 1 if ch == prev else 1
                prev = ch
                if run <= self.collapse_repeats - 1:
                    chars.append(ch)
            collapsed = "".join(chars)
            if collapsed != out:
                self.get_logger().info(f"✎ 摺疊重複字: '{out}' -> '{collapsed}'")
                out = collapsed

        # (4) 同音修正表（保守，只有幾條）
        if self.enable_corrections:
            for wrong, right in self.ASR_CORRECTIONS.items():
                if wrong in out:
                    out = out.replace(wrong, right)
                    self.get_logger().info(f"✎ 同音修正: '{wrong}' -> '{right}'")

        # 摺疊之後可能又變太短
        if len(out) < self.min_text_length:
            self.get_logger().info(f"⊘ 丟棄（處理後太短）: '{raw}' -> '{out}'")
            return None

        if out != raw:
            self.get_logger().info(f"✓ 後處理: '{raw}' -> '{out}'")
        return out

    def _playback_status_callback(self, msg: Bool) -> None:
        """說話狀態回呼函數"""
        with self._playing_lock:
            self.is_playing = msg.data

        if self.is_playing:
            while not self.audio_queue.empty():
                try:
                    self.audio_queue.get_nowait()
                except queue.Empty:
                    break

            if self.recognizer and self.stream:
                self.stream = self.recognizer.create_stream()
                self._stream_samples = 0   # ★ 重建 stream 一定要歸零，否則計時器會一路累加

            self.last_partial_result = None

    def _convert_simplified_to_traditional(self, text: str) -> str:
        """將簡體中文轉換為繁體中文

        Args:
            text: 原始文字

        Returns:
            str: 轉換後的繁體中文文字
        """
        if not self.opencc:
            return text

        try:
            return self.opencc.convert(text)
        except Exception as e:
            self.get_logger().warning(f"OpenCC 轉換失敗: {e}")
            return text

    def destroy_node(self) -> None:
        """關閉節點並清理執行緒

        停止背景工作執行緒並呼叫父類的 destroy_node
        """
        self.is_running = False
        self.audio_queue.put(None)
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    """語音辨識節點進入點"""
    rclpy.init(args=args)
    node = SpeechRecognizerNode()
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
