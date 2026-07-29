#!/usr/bin/env python3
"""音訊工具模組

提供音訊編碼/解碼與驗證等共用函數
"""

import logging
import numpy as np
from pathlib import Path
from typing import Optional, Tuple

from ament_index_python.packages import get_package_share_directory


class AudioCodec:
    """音訊編碼解碼器"""

    @staticmethod
    def encode_pcm_s16le(audio: np.ndarray) -> bytes:
        """將音訊編碼為 PCM S16LE 位元組

        Args:
            audio: float32 or int16 音訊數據

        Returns:
            bytes: 編碼後的位元組數據
        """
        if np.issubdtype(audio.dtype, np.floating):
            audio = audio.astype(np.float32, copy=False)
            audio = np.clip(np.round(audio * 32768.0), -32768.0, 32767.0).astype(np.int16)
        elif audio.dtype != np.int16:
            audio = np.clip(audio, -32768, 32767).astype(np.int16)

        return np.ascontiguousarray(audio).tobytes()

    @staticmethod
    def decode_pcm_s16le(data: bytes, channels: int = 1) -> np.ndarray:
        """從 PCM S16LE 位元組解碼

        Args:
            data: 編碼的位元組數據
            channels: 聲道數 (預設: 1)

        Returns:
            np.ndarray: float32 音訊數據 ([-1, 1] 範圍)
        """
        audio = np.frombuffer(data, dtype=np.int16)
        audio = audio.astype(np.float32) / 32768.0

        if channels > 1:
            audio = audio.reshape(-1, channels)

        return audio

    @staticmethod
    def encode_pcm_s32le(audio: np.ndarray) -> bytes:
        """將音訊編碼為 PCM S32LE 位元組

        Args:
            audio: float64 or int32 音訊數據

        Returns:
            bytes: 編碼後的位元組數據
        """
        if np.issubdtype(audio.dtype, np.floating):
            audio = audio.astype(np.float64, copy=False)
            audio = np.clip(np.round(audio * 2147483648.0), -2147483648.0, 2147483647.0).astype(np.int32)
        elif audio.dtype != np.int32:
            audio = np.clip(audio, -2147483648, 2147483647).astype(np.int32)

        return np.ascontiguousarray(audio).tobytes()

    @staticmethod
    def decode_pcm_s32le(data: bytes, channels: int = 1) -> np.ndarray:
        """從 PCM S32LE 位元組解碼

        Args:
            data: 編碼的位元組數據
            channels: 聲道數 (預設: 1)

        Returns:
            np.ndarray: float64 音訊數據 ([-1, 1] 範圍)
        """
        audio = np.frombuffer(data, dtype=np.int32)
        audio = audio.astype(np.float64) / 2147483648.0

        if channels > 1:
            audio = audio.reshape(-1, channels)

        return audio

    @staticmethod
    def encode_pcm_f32le(audio: np.ndarray) -> bytes:
        """將音訊編碼為 PCM F32LE 位元組

        Args:
            audio: float32 音訊數據

        Returns:
            bytes: 編碼後的位元組數據
        """
        if audio.dtype == np.float32:
            audio = np.clip(audio, -1.0, 1.0).astype(np.float32, copy=False)
        else:
            audio = np.clip(audio, -1.0, 1.0).astype(np.float32)

        return np.ascontiguousarray(audio).tobytes()

    @staticmethod
    def decode_pcm_f32le(data: bytes, channels: int = 1) -> np.ndarray:
        """從 PCM F32LE 位元組解碼

        Args:
            data: 編碼的位元組數據
            channels: 聲道數 (預設: 1)

        Returns:
            np.ndarray: float32 音訊數據
        """
        audio = np.frombuffer(data, dtype=np.float32).copy()

        if channels > 1:
            audio = audio.reshape(-1, channels)

        return audio


def get_default_logger(module_name: str) -> logging.Logger:
    """取得預設日誌記錄器

    配置標準輸出流處理器與格式化器

    Args:
        module_name: 模組名稱

    Returns:
        logging.Logger: 配置好的 Logger 實例
    """
    logger = logging.getLogger(module_name)
    if not logger.handlers:
        handler = logging.StreamHandler()
        formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
    return logger


def get_model_path(model_type: str) -> Optional[Path]:
    """取得模型文件路徑

    使用 ROS 2 功能包共享路徑搜尋模型文件

    Args:
        model_type: 模型類型 ('vad', 'kws', 'asr', 'tts')

    Returns:
        Path: 模型路徑，如果找不到則回傳 None
    """
    try:
        # 取得 ROS 2 功能包共享目錄
        pkg_share_dir = get_package_share_directory("smartnav_audio")
        model_path = Path(pkg_share_dir) / "models" / model_type

        if model_path.exists():
            logger = get_default_logger(__name__)
            logger.info(f"找到 {model_type} 模型目錄: {model_path}")
            return model_path

        return None
    except Exception as e:
        logger = get_default_logger(__name__)
        logger.warning(f"無法取得模型路徑 ({model_type}): {e}")
        return None


def validate_sample_rate(sample_rate: int) -> bool:
    """驗證採樣率

    Args:
        sample_rate: 採樣率 (Hz)

    Returns:
        bool: 是否為有效的採樣率
    """
    valid_rates = [8000, 16000, 22050, 24000, 44100, 48000]
    return sample_rate in valid_rates


def validate_audio_data(audio: np.ndarray, expected_sr: Optional[int] = None) -> Tuple[bool, str]:
    """驗證音訊數據

    Args:
        audio: 音訊數據
        expected_sr: 預期採樣率

    Returns:
        Tuple[bool, str]: (是否有效, 錯誤訊息)
    """
    if not isinstance(audio, np.ndarray):
        return False, "音訊必須是 numpy 陣列"

    if audio.size == 0:
        return False, "音訊為空"

    if audio.ndim > 2:
        return False, "音訊維度不正確"

    if expected_sr is not None and not validate_sample_rate(expected_sr):
        return False, f"無效的採樣率: {expected_sr}"

    return True, ""
