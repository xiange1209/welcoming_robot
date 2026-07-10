#!/usr/bin/env python3
"""語音喚醒觸發節點

使用 VAD 實現語音喚醒
監聽麥克風，在檢測到喚醒詞時發佈音訊
"""

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

        # VAD 參數
        self.declare_parameter("vad_num_threads", 2)

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

                    self.vad = sherpa_onnx.VadModel.create(vad_config)
                    self.get_logger().info("✓ VAD 模型已載入")
                else:
                    self.get_logger().warning(f"✗ VAD 模型文件不存在: {vad_model_path}")
            else:
                self.get_logger().warning("✗ 未找到 VAD 模型目錄")
        except Exception as e:
            self.get_logger().warning(f"✗ 載入 VAD 模型失敗: {e}")

    def _init_recorder(self) -> None:
        """初始化並啟動麥克風錄製器

        建立 AudioRecorder 實例並啟動音訊捕獲
        """
        try:
            self._recorder = AudioRecorder(
                sample_rate=self.sample_rate,
                chunk_size=self.chunk_size,
                device=self.device,
                audio_callback=self.process_audio_chunk,
                logger=self.get_logger(),
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
