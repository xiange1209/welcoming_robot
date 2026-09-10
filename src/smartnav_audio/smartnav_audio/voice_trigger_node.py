#!/usr/bin/env python3
"""語音喚醒觸發節點

使用 VAD 實現語音喚醒
監聽麥克風，在檢測到喚醒詞時發佈音訊
"""

import math
import threading
import sherpa_onnx
import numpy as np
from enum import Enum
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool

from smartnav_msgs.msg import AudioData
from smartnav_audio.voice_utils import AudioCodec, get_model_path, validate_audio_data
from smartnav_audio.audio_recorder import AudioRecorder
from smartnav_audio.mic_array import (
    DEFAULT_COEFFS,
    DEFAULT_GAIN,
    DEFAULT_LAG,
    DEFAULT_PRE,
    DualMicMixer,
)


class TriggerState(Enum):
    """三階段狀態列舉"""

    IDLE = "idle"  # 第一階段：休眠監測
    COMMAND = "command"  # 第三階段：指令解析


class VoiceTriggerNode(Node):
    """語音喚醒觸發節點

    使用 VAD 實現語音喚醒功能

    Publishers:
        /audio_in (smartnav_msgs/AudioData): 音訊數據流
        /voice_triggered (std_msgs/Bool): 喚醒觸發事件
    """

    def __init__(self) -> None:
        """初始化語音喚醒觸發節點

        宣告所有必要參數，初始化音訊處理引擎
        """
        super().__init__("voice_trigger_node")

        # ============ 參數聲明 ============

        # 基本音訊參數
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("chunk_size", 512)
        self.declare_parameter("device", -1)
        self.declare_parameter("dtype", "float32")
        self.declare_parameter("output_format", "pcm_s16le")

        # ★ 2026-08-17 加：Astra S 的雙麥克風處理模式。
        #   可選：left / right / average / beamform / cancel
        #
        #   預設 cancel：離線量到人站正前方時 SNR 比 left 好 +8.2 dB
        #   （噪音 -13.6 dB、人聲 -5.4 dB，差值就是淨賺），
        #   且 ±86 度內每個角度都是正的。係數的泛化跨過一次完整重開機驗證過。
        #
        #   ★ 單聲道退路：mic_mode:=left 就完全回到 2026-08-17 之前的行為
        #     —— 連 InputStream 都會退回開單聲道，不只是繞過 mixer。
        #     實機如果變差，先改這個參數，不要改程式。
        self.declare_parameter(
            "mic_mode", "cancel", ParameterDescriptor(description="雙麥克風處理模式 left/right/average/beamform/cancel")
        )
        # 預設就是驗證過的那組係數（定義在 mic_array.DEFAULT_COEFFS，
        # 讓節點與 ~/maprun/tools_0817 的分析工具共用同一份，不會各改各的）。
        #
        # ★ 絕對不要把預設值寫成空 list []。rclpy 從預設值推型別時，
        #   空 list 會走到 `all(isinstance(v, bytes) for v in [])` == True 這一條，
        #   被推成 BYTE_ARRAY；之後用 -p mic_cancel_coeffs:=[1.0,...] 傳浮點數
        #   會直接丟型別錯誤，而且訊息看起來跟麥克風完全無關。
        #   現在給的是非空的 float list，型別會正確推成 DOUBLE_ARRAY。
        self.declare_parameter(
            "mic_cancel_coeffs",
            list(DEFAULT_COEFFS),
            ParameterDescriptor(description="mic_mode=cancel 用的 FIR 係數，由 mic_snr_cc.py --fit 產生"),
        )
        self.declare_parameter("mic_cancel_pre", DEFAULT_PRE)
        self.declare_parameter("mic_lag", DEFAULT_LAG)
        # ★ 補回 cancel 壓掉的人聲位準（+5.41 dB = x1.86）。
        #   不補的話人聲會比現況小 5.4 dB，VAD threshold 0.25 是在現況位準下
        #   校出來的，可能反而更難觸發。設 1.0 就是不補。
        self.declare_parameter(
            "mic_output_gain",
            DEFAULT_GAIN,
            ParameterDescriptor(description="雙麥克風輸出補回增益（倍率），1.0 = 不補"),
        )

        # VAD 參數
        self.declare_parameter("vad_num_threads", 2)
        # ★ 2026-08-14 加：silero VAD 的判定門檻，越低越靈敏。
        #   sherpa-onnx 預設 0.5 在這台車實測只有 2% 正幀，湊不出連續三幀。
        #   0.25 是實測掃出來的值，推導見 _init_vad 裡的註解。
        self.declare_parameter("vad_threshold", 0.25)

        # 狀態轉移參數 (毫秒)
        self.declare_parameter("speech_start_timeout", 100)
        self.declare_parameter("silence_timeout", 500)
        self.declare_parameter("command_initial_wait_timeout", 3000)

        # Topic 名稱參數
        self.declare_parameter("audio_topic", "/audio_in", ParameterDescriptor(description="音訊數據流話題"))
        self.declare_parameter("triggered_topic", "/voice_triggered", ParameterDescriptor(description="喚醒觸發話題"))

        # ============ 參數獲取 ============

        # 音訊參數
        self.sample_rate: int = self.get_parameter("sample_rate").get_parameter_value().integer_value
        self.chunk_size: int = self.get_parameter("chunk_size").get_parameter_value().integer_value
        self.device: int = self.get_parameter("device").get_parameter_value().integer_value
        self.dtype: str = self.get_parameter("dtype").get_parameter_value().string_value
        self.output_format: str = self.get_parameter("output_format").get_parameter_value().string_value

        # VAD 參數
        self.vad_num_threads: int = self.get_parameter("vad_num_threads").get_parameter_value().integer_value

        # 狀態轉移參數 (毫秒轉幀數)
        speech_start_timeout_ms: int = self.get_parameter("speech_start_timeout").get_parameter_value().integer_value
        silence_timeout_ms: int = self.get_parameter("silence_timeout").get_parameter_value().integer_value
        command_initial_wait_timeout_ms: int = (
            self.get_parameter("command_initial_wait_timeout").get_parameter_value().integer_value
        )

        # Topic 名稱參數
        audio_topic: str = self.get_parameter("audio_topic").get_parameter_value().string_value
        triggered_topic: str = self.get_parameter("triggered_topic").get_parameter_value().string_value

        # 轉換毫秒為幀數 (frames = ms * sample_rate / 1000 / self.chunk_size)
        self.speech_start_frames: int = max(1, int(speech_start_timeout_ms * self.sample_rate / 1000 / self.chunk_size))
        self.silence_frames_threshold: int = max(1, int(silence_timeout_ms * self.sample_rate / 1000 / self.chunk_size))
        self.command_initial_wait_frames: int = max(
            1, int(command_initial_wait_timeout_ms * self.sample_rate / 1000 / self.chunk_size)
        )

        # sherpa-onnx 初始化
        self.vad: Optional[sherpa_onnx.VadModel] = None
        self._init_sherpa_onnx()

        # 三階段狀態機
        self._state = TriggerState.IDLE
        # ★★ 2026-08-19：知道「機器人正在講話」★★
        #
        # 平板固定在車上，所以麥克風**一定**會收到 TTS 的聲音，而且又近又大聲。
        # 後果不是「多辨識一句」那麼單純：
        #   VAD 會被持續的 TTS 聲音一路判成「還在講話」-> COMMAND 狀態不結束
        #   -> 同一個 ASR stream 被餵好幾句 -> 黏句、重複、記憶體成長。
        #
        # ★ 但**不能**直接把音訊丟掉：使用者指出「VIP 或訪客不一定可以觸發語音輸入」。
        #   客人常在機器人講到一半時插話，丟掉音訊等於聽不到他們。
        #
        # 折衷：TTS 開始與結束時**強制切句**（把目前累積的送出去、回到 IDLE），
        # 音訊照收。這樣 TTS 那一段會變成一句獨立的（由 ASR 的文字層回音過濾丟掉），
        # 客人插話的部分則是另一句，不會黏在一起。
        self._tts_active = False
        self.create_subscription(Bool, "playback_status", self._playback_cb, 10)
        self._lock = threading.Lock()

        # Idle 階段：連續 VAD 正幀計數，用於檢測語音起點
        self._vad_positive_frames: int = 0

        # Command 階段：初始等待計數器和說話結束檢測
        self._command_wait_frames: int = 0
        self._in_command_initial_wait: bool = False

        # 麥克風錄製器初始化
        self._audio_buffer = deque(maxlen=15)
        self._recorder: Optional[AudioRecorder] = None
        self._init_recorder()

        # 建立 QoS 配置檔
        audio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.audio_pub = self.create_publisher(AudioData, audio_topic, audio_qos)
        self.triggered_pub = self.create_publisher(Bool, triggered_topic, 10)

        self.get_logger().info("✓ 語音喚醒觸發節點已初始化")
        self.get_logger().info(f"  發布話題: {audio_topic}")
        self.get_logger().info(f"  發布話題: {triggered_topic}")
        self.get_logger().info(f"  採樣率: {self.sample_rate} Hz")
        self.get_logger().info(f"  語音起點檢測: {speech_start_timeout_ms} ms ({self.speech_start_frames} 幀)")
        self.get_logger().info(f"  靜音結束檢測: {silence_timeout_ms} ms ({self.silence_frames_threshold} 幀)")
        self.get_logger().info(
            f"  Command 初始等待: {command_initial_wait_timeout_ms} ms ({self.command_initial_wait_frames} 幀)"
        )

    def _init_sherpa_onnx(self) -> None:
        """初始化 sherpa-onnx 引擎

        載入 VAD 模型，並轉換喚醒關鍵字為音素格式
        """
        # 初始化 VAD
        try:
            vad_model_dir = get_model_path("vad")
            if vad_model_dir:
                vad_model_path = vad_model_dir / "silero_vad.onnx"
                if vad_model_path.exists():
                    vad_config = sherpa_onnx.VadModelConfig()
                    vad_config.silero_vad.model = str(vad_model_path)
                    vad_config.sample_rate = self.sample_rate
                    vad_config.num_threads = self.vad_num_threads

                    # ★★ 2026-08-14：threshold 原本沒設，吃 sherpa-onnx 預設 0.5，
                    #    在這台車的 SNR 下**太嚴，VAD 一次都觸發不了**。
                    #
                    #    拿一段「ASR 確實認得出『你好请问柜台在哪里』」的 15 秒實錄
                    #    離線掃門檻，正幀比例：
                    #
                    #        0.50（預設） 10/468 =  2%
                    #        0.40         14/468 =  3%
                    #        0.30         14/468 =  3%
                    #        0.20         64/468 = 14%
                    #
                    #    而 `speech_start_timeout` 要求**連續 3 幀**才算語音起點 ——
                    #    2% 且分散的正幀湊不出連續三幀，所以狀態機永遠停在 IDLE，
                    #    `/audio_in` 一則都不會發，看起來像「ASR 壞了」，
                    #    實際上模型與收音都是好的。**全程沒有任何錯誤訊息。**
                    #
                    #    ⚠ 這個值跟 SNR 綁在一起。當時的條件是：麥克風增益 33 dB
                    #    （`run_asr_cc.sh` 的 GAIN=66）、距離約 30~50 cm、底噪 -26.5 dBFS、
                    #    人聲高出 6~13 dB。**換場地或改增益要重新掃這個門檻。**
                    #    調太低的代價是誤觸發（把冷氣聲當人聲），要一起看
                    #    `voice_trigger` 有沒有在沒人講話時亂進 COMMAND。
                    vad_threshold = self.get_parameter("vad_threshold").get_parameter_value().double_value
                    try:
                        vad_config.silero_vad.threshold = vad_threshold
                    except Exception as e:
                        self.get_logger().warning(f"✗ 設定 vad_threshold 失敗，沿用預設: {e}")

                    self.vad = sherpa_onnx.VadModel.create(vad_config)
                    self.get_logger().info(f"✓ VAD 模型已載入（threshold {vad_threshold:.2f}）")
                else:
                    self.get_logger().warning(f"✗ VAD 模型文件不存在: {vad_model_path}")
            else:
                self.get_logger().warning("✗ 未找到 VAD 模型目錄")
        except Exception as e:
            self.get_logger().warning(f"✗ 載入 VAD 模型失敗: {e}")

    def _build_mic_mixer(self):
        """依 mic_mode 建立雙麥克風處理器

        Returns:
            (channels, mixer)：channels=1 表示沿用舊的單聲道路徑
        """
        mode = self.get_parameter("mic_mode").get_parameter_value().string_value or "left"
        if mode == "left":
            # ★ 單聲道退路：完全走 2026-08-17 之前的路徑，連 InputStream 都還是
            #   開單聲道，確保行為一個位元組都沒變。
            self.get_logger().info("  雙麥克風模式: left（單聲道擷取，與 2026-08-17 之前完全相同）")
            return 1, None

        # 有預設值了，正常情況不會丟；但參數若被外部覆寫成空陣列還是要接得住。
        try:
            coeffs = list(self.get_parameter("mic_cancel_coeffs").get_parameter_value().double_array_value)
        except Exception:
            coeffs = []
        pre = self.get_parameter("mic_cancel_pre").get_parameter_value().integer_value
        lag = self.get_parameter("mic_lag").get_parameter_value().integer_value
        gain = self.get_parameter("mic_output_gain").get_parameter_value().double_value
        try:
            mixer = DualMicMixer(mode=mode, coeffs=coeffs or None, lag=lag, pre=pre, gain=gain)
        except Exception as e:
            # ★ 任何一步失敗都退回單聲道。這個專題不能因為麥克風處理壞掉就整個沒聲音。
            self.get_logger().error(f"✗ 建立雙麥克風處理器失敗（退回 left 單聲道）: {e}")
            return 1, None

        # ★ 這幾行是實機驗證唯一的憑據（ros2 param get 不算）。
        #   把真正生效的數字全印出來，才能確認參數有吃進去。
        self.get_logger().info(f"  雙麥克風模式: {mode}（雙聲道擷取）")
        if mode == "cancel":
            is_default = (len(coeffs) == len(DEFAULT_COEFFS)
                          and all(abs(a - b) < 1e-9 for a, b in zip(coeffs, DEFAULT_COEFFS)))
            src = "內建驗證過的預設值" if is_default else "外部覆寫"
            self.get_logger().info(
                f"    FIR {len(coeffs)} 抽頭（{src}）  pre={pre}  "
                f"係數最大絕對值 {max(abs(c) for c in coeffs):.2f}"
            )
            self.get_logger().info(
                "    預期：底噪 -13.6 dB、人聲 -5.4 dB -> SNR 淨賺 +8.2 dB（正前方）"
            )
        elif mode == "beamform":
            self.get_logger().info(f"    lag={lag}（2 = 指向車子側面，不是正前方）")
        gain_db = 20.0 * math.log10(gain) if gain > 0 else float("-inf")
        self.get_logger().info(f"    輸出補回增益: x{gain:.2f}（{gain_db:+.2f} dB）")
        return 2, mixer

    def _init_recorder(self) -> None:
        """初始化並啟動麥克風錄製器

        建立 AudioRecorder 實例並啟動音訊捕獲
        """
        try:
            channels, mixer = self._build_mic_mixer()
            self._recorder = AudioRecorder(
                sample_rate=self.sample_rate,
                chunk_size=self.chunk_size,
                device=self.device,
                audio_callback=self.process_audio_chunk,
                logger=self.get_logger(),
                channels=channels,
                mono_mixer=mixer,
            )
            self._recorder.start()
            self.get_logger().info("✓ 麥克風錄製已啟動")
        except Exception as e:
            self.get_logger().error(f"✗ 初始化麥克風錄製器失敗: {e}")

    def _stop_recorder(self) -> None:
        """停止麥克風錄製

        停止音訊捕獲並釋放資源
        """
        if self._recorder:
            try:
                self._recorder.stop()
                self.get_logger().info("✓ 麥克風錄製已停止")
            except Exception as e:
                self.get_logger().error(f"✗ 停止麥克風錄製失敗: {e}")

    def _playback_cb(self, msg: Bool) -> None:
        """機器人開始／結束講話。★ 在**邊界**強制切句，不丟音訊。"""
        active = bool(msg.data)
        if active == self._tts_active:
            return
        self._tts_active = active
        with self._lock:
            in_command = self._state == TriggerState.COMMAND
        if in_command:
            # 邊界切句：讓 ASR 收到 is_final，把目前這段收乾淨，
            # 否則 TTS 的聲音會跟客人的話黏在同一個 stream 裡。
            #
            # ★ `_set_state(IDLE)` 本身**不會**送 is_final（只是重置計數器），
            #   所以要自己補一個帶 is_final 的音訊塊。
            # ★ 不能送空陣列 —— 辨識端 `if len(audio) == 0: return`
            #   會直接丟掉，連 is_final 都不會處理。送一小段靜音。
            self.get_logger().info(
                f"🔀 TTS {'開始' if active else '結束'}，強制切句（音訊照收，客人仍可插話）")
            self._publish_audio(np.zeros(512, dtype=np.float32), is_final=True)
            self._set_state(TriggerState.IDLE)

    def _set_state(self, new_state: TriggerState) -> None:
        """轉移到新狀態

        Args:
            new_state: 目標狀態
        """
        with self._lock:
            if self._state == new_state:
                return

            old_state = self._state
            self._state = new_state

            # 日誌輸出
            transition_str = f"{old_state.value.upper()} -> {new_state.value.upper()}"
            self.get_logger().info(f"[狀態轉移] {transition_str}")

            # 狀態開始初始化
            self._on_state_enter(new_state)

    def _on_state_enter(self, state: TriggerState) -> None:
        """處理進入新狀態時的初始化

        根據新狀態重置相關計數器和狀態變數

        Args:
            state: 新狀態
        """
        if state == TriggerState.IDLE:
            self._vad_positive_frames = 0
            self._silence_frames = 0
            self._command_wait_frames = 0
            self._in_command_initial_wait = False
        elif state == TriggerState.COMMAND:
            self._silence_frames = 0
            self._command_wait_frames = 0
            self._in_command_initial_wait = True

    def _publish_audio(self, audio: np.ndarray, is_final: bool = False) -> None:
        """發佈音訊數據

        Args:
            audio: 音訊數據 (float32 numpy array)
            is_final: 是否為最終語音塊（語音已結束）
        """
        try:
            valid, error = validate_audio_data(audio, self.sample_rate)
            if not valid:
                self.get_logger().warning(f"✗ 無效的音訊數據，無法發佈: {error}")
                return

            if self.output_format == "pcm_s16le":
                audio_bytes = AudioCodec.encode_pcm_s16le(audio)
                format_str = "pcm_s16le"
            elif self.output_format == "pcm_s32le":
                audio_bytes = AudioCodec.encode_pcm_s32le(audio)
                format_str = "pcm_s32le"
            elif self.output_format == "pcm_f32le":
                audio_bytes = AudioCodec.encode_pcm_f32le(audio)
                format_str = "pcm_f32le"
            else:
                self.get_logger().warning(f"✗ 不支援的音訊格式: {self.output_format}")
                return

            msg = AudioData()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "microphone"
            msg.data = audio_bytes
            msg.format = format_str
            msg.sample_rate = self.sample_rate
            msg.channels = 1
            msg.is_final = is_final

            self.audio_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"✗ 發佈音訊失敗: {e}")

    def process_audio_chunk(self, audio: np.ndarray) -> bool:
        """處理音訊塊並進行三階段狀態機轉移

        執行喚醒檢測，根據當前狀態進行相應的處理，發佈音訊或觸發事件

        Args:
            audio: 音訊數據 (float32 numpy array)

        Returns:
            bool: 是否觸發喚醒
        """
        triggered = False

        # 確保 audio 為 float32
        if audio.dtype != np.float32:
            if audio.dtype == np.int16:
                audio = audio.astype(np.float32) / 32768.0
            else:
                audio = audio.astype(np.float32)

        # 執行 VAD
        speech_detected = False
        if self.vad:
            try:
                audio_list = audio.tolist()
                speech_detected = self.vad.is_speech(audio_list)

            except Exception as e:
                self.get_logger().error(f"✗ VAD 檢測失敗: {e}")
                return False

        # ============ 三階段狀態機邏輯 ============

        # Idle 階段
        with self._lock:
            current_state = self._state

        if current_state == TriggerState.IDLE:
            self._audio_buffer.append(audio)
            if speech_detected:
                self._vad_positive_frames += 1
                self.get_logger().debug(f"[IDLE] VAD 正幀計數: {self._vad_positive_frames}/{self.speech_start_frames}")
                if self._vad_positive_frames >= self.speech_start_frames:
                    # VAD 連續正幀達到閾值，轉入 Command 階段
                    self._set_state(TriggerState.COMMAND)

                    for buffered_audio in self._audio_buffer:
                        self._publish_audio(buffered_audio)
                    self._audio_buffer.clear()

                    self.get_logger().info(f"語音起點檢測 (連續 {self._vad_positive_frames} 幀語音)")
            else:
                if self._vad_positive_frames > 0:
                    self.get_logger().debug(f"[IDLE] VAD 正幀計數重置 (從 {self._vad_positive_frames} 幀)")
                self._vad_positive_frames = 0

        # Command 階段
        with self._lock:
            current_state = self._state

        if current_state == TriggerState.COMMAND:
            if self._in_command_initial_wait:
                # 初始等待期：累積幀數直到超過初始等待時長
                self._command_wait_frames += 1
                self._publish_audio(audio)
                if self._command_wait_frames >= self.command_initial_wait_frames:
                    # 初始等待期結束，開始監測說話結束
                    self._in_command_initial_wait = False
                    self._silence_frames = 0
                    self.get_logger().debug(
                        f"[Command] 初始等待期結束 ({self._command_wait_frames} 幀)，"
                        f"開始監測語音結束 (靜音閾值: {self.silence_frames_threshold} 幀)"
                    )
            else:
                # 監測說話結束
                if speech_detected:
                    self._silence_frames = 0
                    self._publish_audio(audio)
                    self.get_logger().debug("[Command] 偵測到語音，靜音計數重置")
                else:
                    self._silence_frames += 1
                    if self._silence_frames >= self.silence_frames_threshold:
                        # 語音結束，發佈最終塊
                        self._publish_audio(audio, is_final=True)
                        # 重置回 IDLE 狀態，供下一次喚醒
                        self._set_state(TriggerState.IDLE)
                        self.get_logger().info("VAD 檢測說話結束，重置為 IDLE")
                    else:
                        self._publish_audio(audio)

        return triggered


def main(args: Optional[list] = None) -> None:
    """語音喚醒觸發節點進入點"""
    rclpy.init(args=args)
    node = VoiceTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_recorder()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
