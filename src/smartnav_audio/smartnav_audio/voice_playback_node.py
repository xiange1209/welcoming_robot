#!/usr/bin/env python3
"""音訊播放節點

訂閱音訊話題，將音訊資料播放至喇叭。
"""

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool

from smartnav_msgs.msg import AudioData
from smartnav_audio.voice_utils import AudioCodec, validate_audio_data
from smartnav_audio.audio_player import AudioPlayer


class AudioPlaybackNode(Node):
    """音訊播放節點

    Subscriptions:
        /audio_out (smartnav_msgs/AudioData): 音訊數據流
    """

    def __init__(self) -> None:
        """初始化音訊播放節點

        宣告參數、初始化播放器並建立訂閱器
        """
        super().__init__("audio_playback_node")

        # 參數宣告
        self.declare_parameter("channels", 1)
        self.declare_parameter("device", -1)
        self.declare_parameter("volume", 1.0)
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("audio_topic", "/audio_out", ParameterDescriptor(description="音訊數據流話題"))

        # 參數獲取
        self.channels = self.get_parameter("channels").get_parameter_value().integer_value
        self.device = self.get_parameter("device").get_parameter_value().integer_value
        self.volume = self.get_parameter("volume").get_parameter_value().double_value
        self.queue_size = self.get_parameter("queue_size").get_parameter_value().integer_value
        audio_topic = self.get_parameter("audio_topic").get_parameter_value().string_value

        self.audio_player: Optional[AudioPlayer] = None
        self.is_playing = False
        self.last_playing_time = self.get_clock().now()
        self._playing_lock = threading.Lock()
        self._init_player()

        self.callback_group = ReentrantCallbackGroup()

        # 建立 QoS 配置檔
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

        self.audio_sub = self.create_subscription(
            AudioData,
            audio_topic,
            self._audio_callback,
            audio_qos,
            callback_group=self.callback_group,
        )

        self.playback_status_pub = self.create_publisher(Bool, "playback_status", status_qos)
        self.playback_status_timer = self.create_timer(
            0.05, self._playback_status_timer_callback, callback_group=self.callback_group
        )

        self.get_logger().info(f"✓ 音訊播放節點已初始化")
        self.get_logger().info(f"  訂閱話題: {audio_topic}")

    def _init_player(self) -> None:
        """初始化並啟動音訊播放器"""
        try:
            self.audio_player = AudioPlayer(
                channels=self.channels,
                device=self.device,
                volume=self.volume,
                queue_size=self.queue_size,
                logger=self.get_logger(),
            )
            self.audio_player.start()
            self.get_logger().info("✓ 音訊播放器已啟動")
        except Exception as e:
            self.get_logger().error(f"✗ 初始化音訊播放器失敗: {e}")

    def _stop_player(self) -> None:
        """停止音訊播放器"""
        if not self.audio_player:
            return

        try:
            self.audio_player.stop()
            self.audio_player = None
            self.get_logger().info("✓ 音訊播放器已停止")

            with self._playing_lock:
                if self.is_playing:
                    self._publish_playback_status(False)
        except Exception as e:
            self.get_logger().error(f"✗ 停止音訊播放器失敗: {e}")

    def _audio_callback(self, msg: AudioData) -> None:
        """音訊話題回呼函數 - 解碼並加入播放隊列

        Args:
            msg: 音訊訊息
        """
        try:
            if msg.format == "pcm_s16le":
                audio = AudioCodec.decode_pcm_s16le(bytes(msg.data))
            elif msg.format == "pcm_f32le":
                audio = AudioCodec.decode_pcm_f32le(bytes(msg.data))
            elif msg.format == "pcm_s32le":
                audio = AudioCodec.decode_pcm_s32le(bytes(msg.data))
            else:
                self.get_logger().warning(f"✗ 不支援的音訊格式: {msg.format}")
                return

            valid, error = validate_audio_data(audio, msg.sample_rate)
            if not valid:
                self.get_logger().warning(f"✗ 無效的音訊數據: {error}")
                return

            if self.audio_player:
                with self._playing_lock:
                    self.last_playing_time = self.get_clock().now()
                    if not self.is_playing:
                        self._publish_playback_status(True)

            # 傳遞已解碼的 float32 音訊到播放器
            if self.audio_player:
                self.audio_player.enqueue_audio(audio, sample_rate=msg.sample_rate)
                self.get_logger().debug(f"已加入播放隊列 ({len(audio)} 個樣本, {msg.sample_rate} Hz)")
        except Exception as e:
            self.get_logger().error(f"音訊播放失敗: {e}")

    def _publish_playback_status(self, status: bool) -> None:
        """發布說話狀態"""
        msg = Bool()
        msg.data = status
        self.is_playing = status
        self.playback_status_pub.publish(msg)

    def _playback_status_timer_callback(self) -> None:
        """定時檢查說話狀態"""
        if not self.audio_player:
            with self._playing_lock:
                if self.is_playing:
                    self._publish_playback_status(False)
            return

        is_playing = self.audio_player.is_playing_audio()
        current_time = self.get_clock().now()

        with self._playing_lock:
            if is_playing:
                self.last_playing_time = current_time
                if not self.is_playing:
                    self._publish_playback_status(True)
            else:
                silence_duration = current_time - self.last_playing_time
                if self.is_playing and silence_duration > Duration(seconds=0.5):
                    self._publish_playback_status(False)


def main(args=None):
    """音訊播放節點進入點"""
    rclpy.init(args=args)
    node = AudioPlaybackNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_player()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
