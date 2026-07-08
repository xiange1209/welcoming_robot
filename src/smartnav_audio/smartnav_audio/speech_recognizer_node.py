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

        # 參數獲取
        self.sample_rate = self.get_parameter("sample_rate").get_parameter_value().integer_value
        self.num_threads = self.get_parameter("num_threads").get_parameter_value().integer_value
        self.user_text_topic = self.get_parameter("user_text_topic").get_parameter_value().string_value
        self.partial_text_topic = self.get_parameter("partial_text_topic").get_parameter_value().string_value
        audio_topic = self.get_parameter("audio_topic").get_parameter_value().string_value

        # 初始化 OpenCC 轉換器 (簡體中文 -> 繁體中文)
        self.opencc = None
        try:
            self.opencc = OpenCC("s2twp.json")  # 簡體 -> 繁體
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
                # 查找 ONNX 模型文件
                encoder_file = asr_model_dir / "encoder-epoch-99-avg-1.onnx"
                decoder_file = asr_model_dir / "decoder-epoch-99-avg-1.onnx"
                joiner_file = asr_model_dir / "joiner-epoch-99-avg-1.int8.onnx"
                tokens_file = asr_model_dir / "tokens.txt"

                if not all(f.exists() for f in [encoder_file, decoder_file, joiner_file, tokens_file]):
                    self.get_logger().warning(f"ASR 模型文件不完整: {asr_model_dir}")
                    return

                self.get_logger().info(f"載入 ASR 模型: {asr_model_dir}")

                # 創建 ASR 模型配置
                recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
                    tokens=str(tokens_file),
                    encoder=str(encoder_file),
                    decoder=str(decoder_file),
                    joiner=str(joiner_file),
                    num_threads=self.num_threads,
                    sample_rate=self.sample_rate,
                    feature_dim=80,
                    decoding_method="modified_beam_search",
                    max_active_paths=4,
                    enable_endpoint_detection=True,
                    rule1_min_trailing_silence=2.4,
                    rule2_min_trailing_silence=1.2,
                    rule3_min_utterance_length=20.0,
                )

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

            # 最終塊，通知引擎結束輸入
            if is_final:
                self.stream.input_finished()

            # 檢查流是否已準備好進行解碼
            while self.recognizer.is_ready(self.stream):
                with self._playing_lock:
                    if self.is_playing:
                        return

                self.recognizer.decode_stream(self.stream)

            result = self.recognizer.get_result(self.stream)
            if result and result.strip():
                converted_text = self._convert_simplified_to_traditional(result)
                if converted_text:
                    if is_final:
                        self.get_logger().info(f"✓ 最終結果: '{converted_text}'")
                        self.user_text_pub.publish(String(data=converted_text))
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
        except Exception as e:
            self.get_logger().error(f"✗ 語音辨識失敗: {e}")
            # 重置流
            self.stream = self.recognizer.create_stream()

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
