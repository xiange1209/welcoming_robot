#!/usr/bin/env python3
"""大腦工具模組

提供日誌記錄器初始化與其他共用函數
"""

import logging
import numpy as np


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


def compute_similarity(embedding1: np.ndarray, embedding2: np.ndarray) -> float:
    """計算兩個臉部特徵的相似度

    使用餘弦距離計算，並轉換至 [0, 1] 範圍

    Args:
        embedding1: 第一個臉部特徵向量
        embedding2: 第二個臉部特徵向量

    Returns:
        float: 相似度分數 (0.0-1.0)，越高表示越相似
    """
    try:
        # 確保向量已歸一化
        norm1 = np.linalg.norm(embedding1)
        norm2 = np.linalg.norm(embedding2)

        if norm1 < 1e-6 or norm2 < 1e-6:
            return 0.0

        # 餘弦相似度
        similarity = np.dot(embedding1, embedding2) / (norm1 * norm2)
        # 轉換至 [0, 1] 範圍
        return float((similarity + 1) / 2)
    except Exception as e:
        get_default_logger(__name__).error(f"相似度計算錯誤: {e}")
        return 0.0
