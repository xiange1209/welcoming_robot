#!/usr/bin/env python3
"""語音合成節點

使用 sherpa-onnx 實現 TTS
訂閱文字話題，將文字轉換為音訊
"""

from typing import Optional
import wave
import numpy as np
import sherpa_onnx

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

from smartnav_msgs.msg import AudioData
from smartnav_audio.voice_utils import AudioCodec, get_model_path, validate_audio_data


class SpeechSynthesizerNode(Node):
    """語音合成節點

    使用 sherpa-onnx 進行 TTS 處理，輸出音訊

    Subscriptions:
        /speech_text (std_msgs/String): 要合成的文字

    Publishers:
        /audio_out (smartnav_msgs/AudioData): 合成的音訊數據
    """

    def __init__(self):
        """初始化語音合成節點"""
        super().__init__("speech_synthesizer_node")

        # 參數聲明
        self.declare_parameter("num_threads", 4)
        self.declare_parameter("output_format", "pcm_s16le")
        self.declare_parameter("text_topic", "/speech_text", ParameterDescriptor(description="要合成的文字話題"))
        self.declare_parameter("audio_topic", "/audio_out", ParameterDescriptor(description="音訊數據流話題"))

        # 參數獲取
        self.num_threads = self.get_parameter("num_threads").get_parameter_value().integer_value
        self.output_format = self.get_parameter("output_format").get_parameter_value().string_value
        text_topic = self.get_parameter("text_topic").get_parameter_value().string_value
        audio_topic = self.get_parameter("audio_topic").get_parameter_value().string_value

        # 初始化 sherpa-onnx TTS 模型
        self.tts = None
        self.sample_rate = None
        self._init_sherpa_onnx_tts()

        # QoS 設定
        audio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # 訂閱器 - 訂閱文字話題
        self.text_sub = self.create_subscription(
            String,
            text_topic,
            self._text_callback,
            10,
        )

        # 發佈器 - 發佈合成的音訊
        self.audio_pub = self.create_publisher(AudioData, audio_topic, audio_qos)

        self.get_logger().info("✓ 語音合成節點已初始化")
        self.get_logger().info(f"  訂閱話題: {text_topic}")
        self.get_logger().info(f"  發布話題: {audio_topic}")
        self.get_logger().info(f"  採樣率: {self.sample_rate} Hz")
        self.get_logger().info(f"  輸出格式: {self.output_format}")

    def _init_sherpa_onnx_tts(self) -> None:
        """初始化 sherpa-onnx TTS 模型

        載入 TTS 模型檔案並創建文字轉語音引擎
        """
        try:
            tts_model_dir = get_model_path("tts")
            if tts_model_dir:
                encoder_file = tts_model_dir / "encoder.int8.onnx"
                decoder_file = tts_model_dir / "decoder.int8.onnx"
                vocoder_file = tts_model_dir / "vocos_24khz.onnx"
                lexicon_file = tts_model_dir / "lexicon.txt"
                tokens_file = tts_model_dir / "tokens.txt"

                if not all(f.exists() for f in [encoder_file, decoder_file, vocoder_file, lexicon_file, tokens_file]):
                    self.get_logger().warning(f"TTS 模型文件不完整: {tts_model_dir}")
                    return

                # 建立 TTS 配置 (Zipvoice 模型)
                zip_model_config = sherpa_onnx.OfflineTtsZipvoiceModelConfig()
                zip_model_config.encoder = str(encoder_file)
                zip_model_config.decoder = str(decoder_file)
                zip_model_config.vocoder = str(vocoder_file)
                zip_model_config.lexicon = str(lexicon_file)
                zip_model_config.tokens = str(tokens_file)
                zip_model_config.data_dir = str(tts_model_dir / "espeak-ng-data")

                model_config = sherpa_onnx.OfflineTtsModelConfig()
                model_config.zipvoice = zip_model_config
                model_config.num_threads = self.num_threads
                model_config.debug = False

                tts_config = sherpa_onnx.OfflineTtsConfig()
                tts_config.model = model_config

                self.tts = sherpa_onnx.OfflineTts(tts_config)
                self.sample_rate = self.tts.sample_rate
                self.get_logger().info(f"✓ TTS 模型已初始化")
            else:
                self.get_logger().warning("✗ 未找到 TTS 模型目錄")
        except Exception as e:
            self.get_logger().error(f"✗ 初始化 TTS 模型失敗: {e}")

    def _text_callback(self, msg: String) -> None:
        """文字輸入回呼函數

        Args:
            msg: 帶有文字內容的訊息
        """
        text = msg.data
        if not text or not text.strip():
            self.get_logger().warning("接收到空文字")
            return

        audio = self.synthesize_text(text)
        if audio is not None:
            # 發佈音訊
            self._publish_audio(audio)

    def synthesize_text(self, text: str) -> Optional[np.ndarray]:
        """進行語音合成

        Args:
            text: 要合成的文字

        Returns:
            Optional[np.ndarray]: 合成的音訊數據 (float32)，失敗時回傳 None
        """
        if self.tts is None:
            self.get_logger().warning("TTS 模型未初始化或已禁用")
            return None

        if not text or not text.strip():
            return None

        try:
            self.get_logger().debug(f"合成文字: '{text}'")

            # 使用 sherpa-onnx TTS 進行合成
            audio = self._sherpa_synthesize(text)

            if audio is not None and len(audio) > 0:
                duration = len(audio) / self.sample_rate if self.sample_rate else 0
                self.get_logger().info(f"合成成功: '{text}' ({len(audio)} 樣本, {duration:.2f}s)")
                return audio
            else:
                self.get_logger().warning(f"合成失敗: '{text}'")
                return None

        except Exception as e:
            self.get_logger().error(f"語音合成失敗: {e}")
            return None

    def _load_reference_audio(self, wav_path: str) -> Optional[np.ndarray]:
        """載入參考音訊檔案

        Args:
            wav_path: WAV 檔案路徑

        Returns:
            Optional[np.ndarray]: 音訊數據 (float32)，載入失敗時回傳 None
        """
        try:
            with wave.open(wav_path, "rb") as f:
                # 讀取原始 byte 數據並轉為 int16，再歸一化為 float32
                data = np.frombuffer(f.readframes(f.getnframes()), dtype=np.int16)
                return data.astype(np.float32) / 32768.0
        except Exception as e:
            self.get_logger().error(f"載入 WAV 檔案失敗 ({wav_path}): {e}")
            return None

    def _sherpa_synthesize(self, text: str) -> Optional[np.ndarray]:
        """使用 sherpa-onnx 進行合成

        Args:
            text: 要合成的文字

        Returns:
            Optional[np.ndarray]: 合成的音訊數據 (float32)，合成失敗時回傳 None
        """
        try:
            if not self.tts:
                return None

            tts_model_dir = get_model_path("tts")
            if not tts_model_dir:
                self.get_logger().error("TTS 參考音訊目錄未找到")
                return None

            reference_file = tts_model_dir / "test_wavs" / "news-female.wav"
            if not reference_file.exists():
                self.get_logger().error(f"參考音訊檔案不存在: {reference_file}")
                return None

            ref_audio = self._load_reference_audio(str(reference_file))
            if ref_audio is None:
                self.get_logger().error(f"無法載入參考音訊: {reference_file}")
                return None

            generated_audio = self.tts.generate(
                text=text,
                prompt_text="各位村民, 大家新年好! 近期, 湖北省武漢市等多个地區",
                prompt_samples=ref_audio.tolist(),
                sample_rate=self.sample_rate,
                speed=1.0,
                num_steps=5,
            )
            if generated_audio is None:
                return None

            # 提取音訊數據和採樣率
            audio_samples = generated_audio.samples

            if audio_samples is None or len(audio_samples) == 0:
                return None

            # 轉換為 numpy 陣列 (float32)
            if isinstance(audio_samples, np.ndarray):
                return audio_samples.astype(np.float32)
            else:
                return np.array(audio_samples, dtype=np.float32)

        except Exception as e:
            self.get_logger().error(f"sherpa-onnx 合成失敗: {e}")
            return None

    def _publish_audio(self, audio: np.ndarray) -> None:
        """發佈音訊數據

        Args:
            audio: 音訊數據陣列 (float32)
        """
        try:
            valid, error = validate_audio_data(audio, self.sample_rate)
            if not valid:
                self.get_logger().warning(f"✗ 無效的音訊數據，無法發佈: {error}")
                return

            # 根據輸出格式編碼
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

            # 建立 AudioData 訊息
            msg = AudioData()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "audio"
            msg.data = audio_bytes
            msg.format = format_str
            msg.sample_rate = self.sample_rate
            msg.channels = 1

            # 發佈
            self.audio_pub.publish(msg)
            self.get_logger().debug(f"已發佈音訊 ({len(audio_bytes)} 位元組)")

        except Exception as e:
            self.get_logger().error(f"發佈音訊失敗: {e}")


def main(args=None):
    """語音合成節點進入點"""
    rclpy.init(args=args)
    node = SpeechSynthesizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
