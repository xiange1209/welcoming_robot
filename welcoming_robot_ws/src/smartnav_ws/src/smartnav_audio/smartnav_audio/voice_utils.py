#!/usr/bin/env python3
"""音訊工具模組

提供音訊編碼/解碼與驗證等共用函數
"""

import logging
import os
import numpy as np
from pathlib import Path
from typing import List, Optional, Tuple

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


def model_path_candidates(model_type: str, override: Optional[str] = None) -> List[Path]:
    """列出會依序嘗試的模型目錄，**不檢查存在與否**。

    找不到模型時要把這張清單原樣印出來——否則現場只會看到「找不到」，
    不知道到底找過哪裡。
    """
    cands: List[Path] = []
    if override:
        cands.append(Path(override).expanduser())
    env_dir = os.environ.get("SMARTNAV_AUDIO_MODELS_DIR")
    if env_dir:
        cands.append(Path(env_dir).expanduser() / model_type)
    try:
        cands.append(Path(get_package_share_directory("smartnav_audio")) / "models" / model_type)
    except Exception:
        pass                       # 套件沒裝好時仍要能走後面的退路
    cands.append(Path.home() / "models" / model_type)
    return cands


def get_model_path(model_type: str, override: Optional[str] = None,
                   logger=None) -> Optional[Path]:
    """取得模型目錄。找不到時**大聲失敗**並列出找過的每一條路徑。

    ★★ 2026-09-18：改寫。原本只找套件 share 目錄，找不到就回 None ★★

    9/17 新實驗室實測到的事故：`smartnav_audio` 套件裡**根本沒有 `models/` 目錄**
    （`setup.py:14-18` 用 `glob.glob("models/**")` 掃套件原始碼，掃不到東西），
    於是 `share/smartnav_audio/models/` 不存在，本函式對 asr 與 vad 都回 None。

    後果是兩個**靜默失效**：
      - `speech_recognizer_node`：只印一行 warning 就繼續，recognizer 從未建立。
        節點正常啟動、正常訂閱 `/audio_in`、**永遠不吐結果**。
      - `voice_trigger_node`：VAD 從不觸發，講話完全沒反應。
    節點活著、話題接上、增益正確，外觀跟「沒人講話」一模一樣。

    ★ 查過 `backup_20260828.tar.gz` 裡也沒有 `models/`，所以不是哪次事故弄壞的，
      是**從來就缺**——表示在家能動的那幾次是靠別的方式找到模型。

    這次的修法（車子端交接建議的 (b) 案）：
      1. 多給幾條退路：呼叫端參數 > 環境變數 `SMARTNAV_AUDIO_MODELS_DIR`
         > 套件 share > `~/models/<type>`。
         Pi 上模型實體就在 `~/models/`，最後那條退路直接讓現行機器可用。
      2. **找不到一律 `logger.error` 並印出找過的完整清單**，不再只是 warning。
         呼叫端仍自行決定要不要 raise（見各節點）。

    Args:
        model_type: 模型類型 ('vad', 'kws', 'asr', 'tts')
        override: 呼叫端指定的路徑（通常來自 ROS 參數），優先於一切
        logger: 有給就用它，否則用模組預設 logger

    Returns:
        Path: 模型目錄；全部找不到時回傳 None
    """
    log = logger or get_default_logger(__name__)
    cands = model_path_candidates(model_type, override)
    for path in cands:
        try:
            if path.exists():
                log.info(f"找到 {model_type} 模型目錄: {path}")
                return path
        except OSError:
            continue               # 路徑不合法或權限不足，換下一條
    tried = "\n".join(f"      {i + 1}. {p}" for i, p in enumerate(cands))
    log.error(
        f"✗ 找不到 {model_type} 模型目錄。已依序嘗試：\n{tried}\n"
        f"    -> 可用 SMARTNAV_AUDIO_MODELS_DIR 指到模型所在的上層目錄，"
        f"或用節點的 {model_type}_model_dir 參數直接指定。")
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
