#!/usr/bin/env python3
"""使用者資料庫管理器

負責管理使用者資訊的持久化存儲
"""

import cv2
import json
import uuid
import shutil
import numpy as np
from enum import IntEnum
from pathlib import Path
from datetime import datetime
from typing import Dict, Optional, Any

from smartnav_brain.brain_utils import get_default_logger


class UserType(IntEnum):
    """使用者類型枚舉

    BLACKLIST 是 2026-08-01 補上的 —— 專題的後半段是「安全通報」，
    但先前的資料模型裡沒有黑名單這個概念，等於通報永遠沒有觸發條件。
    數值接在 ADMIN 之後而不是插在中間：改動既有值會讓已註冊的
    使用者型別全部錯位（SQLite 裡存的是名稱字串，但 msg 用的是 uint8）。
    """

    GUEST = 0
    VIP = 1
    ADMIN = 2
    BLACKLIST = 3


class UserManager:
    """使用者資料庫管理器"""

    def __init__(
        self,
        database_dir: Path = Path.home() / ".smartnav" / "face_database",
        logger: Optional[Any] = None,
    ):
        """初始化資料庫管理器

        Args:
            database_dir: 資料庫根目錄
            logger: 日誌記錄器，若無則使用預設記錄器

        Raises:
            Exception: 當目錄初始化失敗時
        """
        self.database_dir = Path(database_dir)
        self.logger = logger or get_default_logger(__name__)

        # 子目錄
        self.data_dir = self.database_dir / "data"
        self.user_registry_path = self.database_dir / "user_registry.json"

        # 初始化資料庫目錄結構
        try:
            self.database_dir.mkdir(parents=True, exist_ok=True)
            self.data_dir.mkdir(parents=True, exist_ok=True)
            self.logger.info(f"✓ 資料庫目錄已初始化: {self.database_dir}")
        except Exception as e:
            self.logger.error(f"目錄初始化失敗: {e}")
            raise

        # 載入現有索引表
        self.user_registry: Dict[str, Dict[str, Any]] = self._load_user_registry()

    def _load_user_registry(self) -> Dict[str, Dict[str, Any]]:
        """載入現有的使用者資訊索引表

        Returns:
            Dict[str, Dict[str, Any]]: 使用者資訊索引表
        """
        if self.user_registry_path.exists():
            try:
                with open(self.user_registry_path, "r", encoding="utf-8") as f:
                    user_registry = json.load(f)
                self.logger.info(f"✓ 索引表已載入: {len(user_registry)} 個已註冊使用者")
                return user_registry
            except Exception as e:
                self.logger.warning(f"索引表載入失敗，建立新的: {e}")

        return {}

    def _save_user_registry(self) -> bool:
        """將索引表儲存到磁碟

        Returns:
            bool: 成功時為 True，失敗時為 False
        """
        try:
            # 使用暫存檔案確保原子性寫入
            temp_path = self.user_registry_path.with_suffix(".tmp")
            with open(temp_path, "w", encoding="utf-8") as f:
                json.dump(self.user_registry, f, indent=2, ensure_ascii=False)
            temp_path.replace(self.user_registry_path)

        except Exception as e:
            self.logger.error(f"索引表儲存失敗: {e}")
            return False

        return True

    def register_user(
        self, created_at: float, user_name: str, user_type: UserType = UserType.GUEST, description: str = ""
    ) -> str:
        """註冊新使用者

        Args:
            created_at: 創建時間
            user_name: 使用者名稱
            user_type: 使用者類型

        Returns:
            str: 分配的使用者 UUID

        Raises:
            Exception: 當註冊失敗時
        """
        try:
            user_name = user_name.strip()
            if not user_name:
                raise ValueError("使用者名稱不能為空")

            for user in self.user_registry.values():
                if user["user_name"] == user_name:
                    raise ValueError(f"使用者名稱已存在: {user_name}")

            user_uuid = str(uuid.uuid4())

            self.user_registry[user_uuid] = {
                "created_at": datetime.fromtimestamp(created_at).isoformat(),
                "user_name": user_name,
                "user_type": user_type.name,
                "description": description,
                "samples": [],
            }

            self._save_user_registry()
            self.logger.info(f"✓ 使用者已註冊: {user_name} (UUID: {user_uuid})")

        except Exception as e:
            self.logger.error(f"使用者註冊失敗: {e}")
            raise

        return user_uuid

    def add_face_sample(
        self,
        user_uuid: str,
        embedding: np.ndarray,
        image: np.ndarray,
    ) -> bool:
        """為已註冊使用者添加臉部樣本

        特徵向量會儲存在 與 embeddings/ 子目錄中
        特徵向量以 L2 歸一化、float32 格式儲存

        Args:
            user_uuid: 使用者 UUID
            embedding: 臉部特徵向量
            image: 臉部影像

        Returns:
            bool: 成功時為 True，失敗時為 False
        """
        if user_uuid not in self.user_registry:
            self.logger.error(f"使用者 UUID 不存在: {user_uuid}")
            return False

        try:
            # 建立使用者專用目錄
            user_dir = self.data_dir / f"uuid_{user_uuid}"
            embeddings_dir = user_dir / "embeddings"
            images_dir = user_dir / "images"
            embeddings_dir.mkdir(parents=True, exist_ok=True)
            images_dir.mkdir(parents=True, exist_ok=True)

            # 使用時間戳記毫秒 + 索引作為檔案名稱
            timestamp = datetime.now()
            timestamp_ms = timestamp.strftime("%Y%m%d_%H%M%S_%f")[:-3]
            sample_idx = len(self.user_registry[user_uuid]["samples"])
            filename_prefix = f"{timestamp_ms}_{sample_idx}"

            # 生成檔案名稱
            embedding_filename = f"{filename_prefix}.npy"
            image_filename = f"{filename_prefix}.png"

            embedding_path = embeddings_dir / embedding_filename
            image_path = images_dir / image_filename

            # 儲存特徵向量 (L2 歸一化，float32)
            np.save(str(embedding_path), embedding.astype(np.float32))

            # 儲存影像
            cv2.imwrite(str(image_path), image)

            # 更新索引表
            sample_record = {
                "embedding_path": str(embedding_path.relative_to(self.database_dir)),
                "image_path": str(image_path.relative_to(self.database_dir)),
                "timestamp": timestamp.isoformat(),
            }
            self.user_registry[user_uuid]["samples"].append(sample_record)

            self._save_user_registry()
            self.logger.debug(f"✓ 樣本已添加: {self.user_registry[user_uuid]['user_name']} " f"({filename_prefix})")

        except Exception as e:
            self.logger.error(f"樣本添加失敗: {e}")
            return False

        return True

    def get_user_embeddings(self, user_uuid: str) -> Optional[np.ndarray]:
        """取得已註冊使用者的所有特徵向量

        Args:
            user_uuid: 使用者 UUID

        Returns:
            Optional[np.ndarray]: 形狀為 (num_samples, 512) 的特徵向量矩陣，或 None (若無樣本)
        """
        if user_uuid not in self.user_registry:
            self.logger.error(f"使用者 UUID 不存在: {user_uuid}")
            return None

        try:
            samples = self.user_registry[user_uuid]["samples"]
            if not samples:
                return None

            embeddings = []
            for sample in samples:
                embedding_path = self.database_dir / sample["embedding_path"]
                if embedding_path.exists():
                    embedding = np.load(str(embedding_path))
                    embeddings.append(embedding)

            if not embeddings:
                return None

        except Exception as e:
            self.logger.error(f"特徵向量載入失敗: {e}")
            return None

        return np.array(embeddings, dtype=np.float32)

    def get_all_embeddings(self) -> Dict[str, np.ndarray]:
        """取得所有已註冊使用者的特徵向量

        Returns:
            Dict[str, np.ndarray]: 特徵向量字典，格式為 {user_uuid: embeddings_matrix}
        """
        all_embeddings = {}

        for user_uuid in list(self.user_registry.keys()):
            embeddings = self.get_user_embeddings(user_uuid)
            if embeddings is not None:
                all_embeddings[user_uuid] = embeddings

        return all_embeddings

    def get_user_info(self, user_uuid: str) -> Optional[Dict[str, Any]]:
        """取得使用者資訊

        Args:
            user_uuid: 使用者 UUID

        Returns:
            Optional[Dict[str, Any]]: 使用者資訊字典，若使用者不存在，返回 None
        """
        return self.user_registry.get(user_uuid)

    def list_users(self) -> list:
        """列出所有已註冊使用者的摘要資訊

        Returns:
            list: 使用者摘要資訊列表，每個元素為字典，包含
                user_uuid, user_name, user_type, description, created_at, num_samples
        """
        users = []
        for user_uuid, meta in self.user_registry.items():
            users.append(
                {
                    "user_uuid": user_uuid,
                    "user_name": meta.get("user_name", ""),
                    "user_type": meta.get("user_type", "GUEST"),
                    "description": meta.get("description", ""),
                    "created_at": meta.get("created_at", ""),
                    "num_samples": len(meta.get("samples", [])),
                }
            )
        users.sort(key=lambda u: u["user_name"])
        return users

    def update_user(
        self,
        user_uuid: str,
        user_name: Optional[str] = None,
        user_type: Optional[UserType] = None,
        description: Optional[str] = None,
    ) -> bool:
        """修改使用者資料

        Args:
            user_uuid: 使用者 UUID
            user_name: 新名稱，None 或空字串表示不改
            user_type: 新類型，None 表示不改
            description: 新描述，None 表示不改，空字串代表清空

        Returns:
            bool: 成功時為 True，失敗時為 False
        """
        if user_uuid not in self.user_registry:
            self.logger.error(f"使用者 UUID 不存在: {user_uuid}")
            return False

        try:
            if user_name:
                user_name = user_name.strip()
                for user_uuid, meta in self.user_registry.items():
                    if user_uuid != user_uuid and meta["user_name"] == user_name:
                        raise ValueError(f"使用者名稱已存在: {user_name}")
                self.user_registry[user_uuid]["user_name"] = user_name

            if user_type is not None:
                self.user_registry[user_uuid]["user_type"] = UserType(user_type).name

            if description is not None:
                self.user_registry[user_uuid]["description"] = description

            self._save_user_registry()
            self.logger.info(f"✓ 使用者已更新: {self.user_registry[user_uuid]['user_name']}")

        except Exception as e:
            self.logger.error(f"使用者更新失敗: {e}")
            return False

        return True

    def delete_user(self, user_uuid: str) -> bool:
        """刪除指定註冊使用者及其所有樣本

        Args:
            user_uuid: 使用者 UUID

        Returns:
            bool: 成功時為 True，失敗時為 False
        """
        if user_uuid not in self.user_registry:
            self.logger.error(f"使用者 UUID 不存在: {user_uuid}")
            return False

        try:
            user_name = self.user_registry[user_uuid]["user_name"]

            # 刪除整個使用者專用目錄
            user_dir = self.data_dir / f"uuid_{user_uuid}"
            if user_dir.exists():
                shutil.rmtree(user_dir)

            # 從索引表移除使用者
            del self.user_registry[user_uuid]
            self._save_user_registry()

            self.logger.info(f"✓ 使用者已刪除: {user_name}")
            return True

        except Exception as e:
            self.logger.error(f"使用者刪除失敗: {e}")
            return False
