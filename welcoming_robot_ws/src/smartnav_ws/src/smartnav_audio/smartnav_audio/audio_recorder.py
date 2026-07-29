#!/usr/bin/env python3
"""麥克風錄製工具類

提供麥克風錄製功能，支持串流方式處理音訊數據
"""

import queue
import threading
import numpy as np
import sounddevice as sd
from typing import Callable, Optional, Any

from smartnav_audio.voice_utils import get_default_logger


class AudioRecorder:
    """麥克風錄製工具類

    提供麥克風錄製功能，使用串流方式處理每個音訊塊
    """

    def __init__(
        self,
        sample_rate: int = 16000,
        chunk_size: int = 512,
        device: int = -1,
        dtype: str = "float32",
        queue_size: int = 50,
        audio_callback: Optional[Callable] = None,
        logger: Optional[Any] = None,
    ):
        """初始化麥克風錄製器

        Args:
            sample_rate: 採樣率 (Hz)
            chunk_size: 塊大小 (樣本數)
            device: 設備索引 (-1 表示預設設備)
            dtype: 數據類型 ('float32', 'int16' 等)
            queue_size: 佇列最大容量
            audio_callback: 音訊回調函數，每個塊調用一次
            logger: 日誌記錄器，若無則使用預設記錄器
        """
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.device = device if device >= 0 else None
        self.dtype = dtype
        self.audio_callback = audio_callback
        self.logger = logger or get_default_logger(__name__)

        self._recording = False
        self._stream = None
        self._thread = None
        self._queue = queue.Queue(maxsize=queue_size)
        self._lock = threading.Lock()

    def start(self) -> bool:
        """啟動麥克風錄製

        Returns:
            bool: 是否成功啟動
        """
        with self._lock:
            if self._recording:
                return False

            try:
                # 清空舊佇列
                self._clear_queue()

                # 建立音訊輸入串流 (單聲道，用於模型識別)
                self._stream = sd.InputStream(
                    device=self.device,
                    samplerate=self.sample_rate,
                    channels=1,
                    blocksize=self.chunk_size,
                    dtype=self.dtype,
                    callback=self._audio_callback,
                    latency="low",
                )
                self._stream.start()
                self._recording = True

                # 啟動執行緒以消耗佇列中的音訊數據
                self._thread = threading.Thread(target=self._worker_loop, daemon=True)
                self._thread.start()
                return True
            except Exception as e:
                self._recording = False
                if self._stream:
                    try:
                        self._stream.stop()
                        self._stream.close()
                        self._stream = None
                    except Exception as e2:
                        self.logger.error(f"關閉串流失敗: {e2}")
                raise RuntimeError(f"啟動錄製失敗: {e}")

    def stop(self) -> None:
        """停止麥克風錄製

        停止音訊串流並釋放相關資源，等待背景執行緒結束
        """
        with self._lock:
            if not self._recording:
                return

            try:
                self._recording = False

                # 停止並關閉串流
                if self._stream:
                    self._stream.stop()
                    self._stream.close()
                    self._stream = None

                # 清空舊佇列
                self._clear_queue()
            except Exception as e:
                self.logger.error(f"停止錄製失敗: {e}")

        # 等待執行緒處理完佇列中的所有數據
        if self._thread:
            try:
                self._queue.put(None)  # 送入 None 信號告訴執行緒結束
                self._thread.join(timeout=2.0)
            except Exception as e:
                self.logger.error(f"等待執行緒結束失敗: {e}")
            finally:
                self._thread = None

    def _audio_callback(self, indata: np.ndarray, frames: int, time: Any, status: sd.CallbackFlags) -> None:
        """音訊串流回呼函數，由 sounddevice 在音訊線程中週期性調用

        在音訊線程中同步運行，將音訊數據放入佇列由背景執行緒處理。

        Args:
            indata: 捕獲的音訊數據 (shape: (frames, channels))
            frames: 幀數
            time: 時間資訊物件，包含音訊裝置的時間戳
            status: 音訊串流狀態標誌
        """
        if status:
            self.logger.warning(f"音訊流狀態異常: {status}")

        try:
            # 複製並扁平化音訊數據
            audio_data = indata.flatten().copy()
            # 將音訊數據放入佇列，由執行緒消耗
            self._queue.put_nowait(audio_data)
        except queue.Full:
            self.logger.warning("音訊佇列已滿，丟棄目前音訊塊")
        except Exception as e:
            self.logger.error(f"音訊回呼處理失敗: {e}")

    def _worker_loop(self) -> None:
        """執行緒迴圈，消耗佇列中的音訊數據

        在背景執行緒中運行，持續消耗佇列中的音訊數據並呼叫回調函數處理
        """
        try:
            while True:
                try:
                    # 從佇列取出音訊數據，設定超時避免無限等待
                    audio_data = self._queue.get(timeout=1.0)

                    # 若收到 None 信號則結束
                    if audio_data is None:
                        break

                    # 呼叫回調函數處理音訊數據
                    if self.audio_callback:
                        try:
                            self.audio_callback(audio_data)
                        except Exception as callback_error:
                            self.logger.error(f"音訊回調函數執行失敗: {callback_error}")
                except queue.Empty:
                    pass
                except Exception as e:
                    self.logger.error(f"執行緒處理音訊失敗: {e}")
        except Exception as e:
            self.logger.error(f"執行緒錯誤: {e}")

    def set_audio_callback(self, callback: Callable) -> None:
        """設定音訊回調函數

        Args:
            callback: 音訊回調函數
        """
        with self._lock:
            self.audio_callback = callback

    def _clear_queue(self) -> None:
        """清空音訊佇列

        在啟動錄製前或停止錄製時調用，防止舊的音訊數據在下次錄製時被處理
        """
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break
