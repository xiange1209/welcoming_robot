#!/usr/bin/env python3
"""LLM 工具模組

提供 LLM 相關的共用函數和類別
"""

import logging
from pathlib import Path
from typing import Optional


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


def get_config_path(config_name: str) -> Optional[Path]:
    """取得配置文件路徑

    使用 ROS 2 功能包共享路徑搜尋配置文件

    Args:
        config_name: 配置文件名稱

    Returns:
        Path: 配置文件路徑，如果找不到則回傳 None
    """
    try:
        # 延遲匯入，讓不依賴 ROS 的模組 (如知識庫檢索) 可以單獨測試
        from ament_index_python.packages import get_package_share_directory

        # 取得 ROS 2 功能包共享目錄
        pkg_share_dir = get_package_share_directory("smartnav_llm")
        config_path = Path(pkg_share_dir) / "config" / config_name

        if config_path.exists():
            logger = get_default_logger(__name__)
            logger.info(f"找到配置文件: {config_path}")
            return config_path

        return None
    except Exception as e:
        logger = get_default_logger(__name__)
        logger.warning(f"無法取得配置文件路徑 ({config_name}): {e}")
        return None


def get_knowledge_dir() -> Optional[Path]:
    """取得銀行知識庫資料夾路徑

    Returns:
        Path: 知識庫資料夾路徑，如果找不到則回傳 None
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        knowledge_dir = Path(get_package_share_directory("smartnav_llm")) / "knowledge"
        if knowledge_dir.is_dir():
            return knowledge_dir

        return None
    except Exception as e:
        logger = get_default_logger(__name__)
        logger.warning(f"無法取得知識庫資料夾路徑: {e}")
        return None
