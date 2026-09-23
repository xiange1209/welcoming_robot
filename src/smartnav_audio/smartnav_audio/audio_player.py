#!/usr/bin/env python3
"""音訊播放工具類

提供音訊播放功能，支持隊列和音量控制
"""

import queue
import threading
import numpy as np
import soxr
import sounddevice as sd
from typing import Optional, Any

from smartnav_audio.voice_utils import get_default_logger


class AudioPlayer:
    """音訊播放工具類

    提供音訊播放功能，使用串流方式處理每個音訊塊
    """

    def __init__(
        self,
        sample_rate: int = 16000,
        channels: int = 1,
        device: int = -1,
        volume: float = 1.0,
        queue_size: int = 50,
        logger: Optional[Any] = None,
    ):
        """初始化音訊播放器

        Args:
            sample_rate: 預設採樣率 (Hz)
            channels: 聲道數
            device: 設備索引 (-1 表示預設設備)
            volume: 音量級別 (0.0-1.0)
            queue_size: 隊列最大容量
            logger: 日誌記錄器，若無則使用預設記錄器
        """
        self.sample_rate = sample_rate
        self.channels = channels
        self.device = device if device >= 0 else None
        self.volume = max(0.0, min(1.0, volume))
        self.logger = logger or get_default_logger(__name__)

        self._playing = False
        self._stream = None
        self._queue = queue.Queue(maxsize=queue_size)
        self._buffer = np.zeros((0, channels), dtype=np.float32)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

    def start(self) -> bool:
        """啟動播放

        Returns:
            bool: 是否成功啟動
        """
        with self._lock:
            if self._playing:
                return False

            try:
                # 清空舊隊列與緩衝區
                self._clear_queue()
                self._buffer = np.zeros((0, self.channels), dtype=np.float32)
                self._stop_event.clear()

                self._stream = sd.OutputStream(
                    samplerate=self.sample_rate,
                    channels=self.channels,
                    device=self.device,
                    callback=self._audio_callback,
                    dtype=np.float32,
                )
                self._stream.start()
                self._playing = True
                return True
            except Exception as e:
                self._playing = False
                if self._stream:
                    try:
                        self._stream.stop()
                        self._stream.close()
                        self._stream = None
                    except Exception as e2:
                        self.logger.error(f"關閉串流失敗: {e2}")
                raise RuntimeError(f"啟動播放失敗: {e}")

    def stop(self) -> None:
        """停止播放器

        停止音訊串流並釋放相關資源，清空隊列與緩衝區
        """
        with self._lock:
            if not self._playing:
                return

            try:
                self._playing = False
                self._stop_event.set()

                # 停止並關閉串流
                if self._stream:
                    self._stream.stop()
                    self._stream.close()
                    self._stream = None

                # 清空舊隊列與緩衝區
                self._clear_queue()
                self._buffer = np.zeros((0, self.channels), dtype=np.float32)
            except Exception as e:
                raise RuntimeError(f"停止播放失敗: {e}")

    def enqueue_audio(self, audio: np.ndarray, sample_rate: Optional[int] = None) -> bool:
        """將音訊添加到播放隊列

        Args:
            audio: 音訊數據陣列 (float32)
            sample_rate: 音訊採樣率，若與播放器不同則會進行重採樣

        Returns:
            bool: 是否成功添加
        """
        if not self._playing:
            return False

        try:
            # 確保音訊為 float32
            audio = audio.astype(np.float32)

            # 轉為 2D 陣列 (samples, channels)
            if audio.ndim == 1:
                # 單聲道音訊
                if self.channels == 1:
                    audio = audio.reshape(-1, 1)
                else:
                    # 單聲道轉多聲道，複製音訊到所有聲道
                    audio = np.repeat(audio.reshape(-1, 1), self.channels, axis=1)
            elif audio.ndim == 2:
                # 檢查聲道數匹配
                if audio.shape[1] != self.channels:
                    self.logger.error(f"音訊聲道數不匹配: 提供={audio.shape[1]}, 播放器={self.channels}")
                    return False
            else:
                self.logger.error(f"無效的音訊維度: {audio.ndim}，期望 1D 或 2D")
                return False

            # 重採樣處理
            if sample_rate is not None and sample_rate != self.sample_rate:
                try:
                    audio = soxr.resample(audio, in_rate=sample_rate, out_rate=self.sample_rate)
                    if audio.ndim == 1:
                        audio = audio.reshape(-1, self.channels)
                except Exception as e:
                    self.logger.error(f"音訊重採樣失敗: {e}")
                    return False

            self._queue.put_nowait(audio)
            return True
        except queue.Full:
            self.logger.warning("播放隊列已滿，丟棄該音訊片段")
            return False
        except Exception as e:
            self.logger.error(f"添加音訊到隊列失敗: {e}")
            return False

    def _audio_callback(self, outdata: np.ndarray, frames: int, time: Any, status: sd.CallbackFlags) -> None:
        """音訊串流回呼函數，處理每個播放的音訊塊

        由 sounddevice 庫在音訊線程中週期性調用，以填充輸出緩衝區

        Args:
            outdata: 需要被填充的音訊數據 (shape: (frames, channels))
            frames: 需要填充的幀數
            time: 時間資訊物件，包含音訊裝置的時間戳
            status: 音訊串流狀態標誌
        """
        try:
            # 檢查是否收到停止訊號
            if self._stop_event.is_set():
                outdata.fill(0.0)
                return

            if status:
                self.logger.warning(f"音訊串流狀態異常: {status}")

            frames_filled = 0

            with self._lock:
                # 從緩衝區填充剩餘的音訊數據
                buffer_len = len(self._buffer)
                if buffer_len > 0:
                    frames_to_take = min(frames, buffer_len)
                    outdata[:frames_to_take, :] = self._buffer[:frames_to_take, :]
                    self._buffer = self._buffer[frames_to_take:, :]
                    frames_filled += frames_to_take

                # 從隊列中取出音訊數據填充
                while frames_filled < frames:
                    try:
                        chunk = self._queue.get_nowait()
                        chunk_len = len(chunk)
                        frames_needed = frames - frames_filled

                        if chunk_len <= frames_needed:
                            outdata[frames_filled : frames_filled + chunk_len, :] = chunk
                            frames_filled += chunk_len
                        else:
                            outdata[frames_filled:frames, :] = chunk[:frames_needed, :]
                            self._buffer = chunk[frames_needed:, :]
                            frames_filled += frames_needed
                    except queue.Empty:
                        break

            # 如果仍有剩餘空間，填充靜音
            if frames_filled < frames:
                outdata[frames_filled:, :] = 0.0

            # 應用音量控制到所有聲道並確保在有效範圍內
            np.multiply(outdata, self.volume, out=outdata)
            np.clip(outdata, -1.0, 1.0, out=outdata)
        except Exception as e:
            self.logger.error(f"音訊回呼處理失敗: {e}")
            outdata.fill(0.0)

    def is_playing_audio(self) -> bool:
        """檢查目前是否有音訊資料正在播放

        Returns:
            bool: 如果正在播放音訊則返回 True，否則返回 False
        """
        if not self._playing:
            return False

        with self._lock:
            has_buffer = len(self._buffer) > 0
            has_queue = not self._queue.empty()
            return has_buffer or has_queue

    def _clear_queue(self) -> None:
        """清空播放隊列

        在啟動播放器前或停止播放器時調用，防止舊的音訊數據在下次播放時被播放
        """
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break
