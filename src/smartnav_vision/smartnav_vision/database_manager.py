#!/usr/bin/env python3
"""臉部資料庫管理器

負責管理臉部特徵向量、原始影像與索引表的持久化存儲
"""

import json
import uuid
from pathlib import Path
from typing import Dict, Optional, List, Any
from datetime import datetime
import numpy as np

from smartnav_vision.face_utils import get_default_logger


class DatabaseManager:
    """臉部資料庫管理器 - 處理特徵向量與索引表的 IO 操作"""

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
        self.registry_path = self.database_dir / "registry.json"

        # 初始化目錄結構
        self._init_directories()

        # 載入現有索引表
        self.registry: Dict[str, Dict[str, Any]] = self._load_registry()

    def _init_directories(self) -> None:
        """建立資料庫目錄結構

        Raises:
            Exception: 當目錄建立失敗時
        """
        try:
            self.database_dir.mkdir(parents=True, exist_ok=True)
            self.data_dir.mkdir(parents=True, exist_ok=True)
            self.logger.info(f"✓ 資料庫目錄已初始化: {self.database_dir}")
        except Exception as e:
            self.logger.error(f"目錄初始化失敗: {e}")
            raise

    def _load_registry(self) -> Dict[str, Dict[str, Any]]:
        """載入現有的人員資訊索引表

        Returns:
            Dict[str, Dict[str, Any]]: 人員資訊索引表
        """
        if self.registry_path.exists():
            try:
                with open(self.registry_path, "r", encoding="utf-8") as f:
                    registry = json.load(f)
                self.logger.info(f"✓ 索引表已載入: {len(registry)} 個已註冊人員")
                return registry
            except Exception as e:
                self.logger.warning(f"索引表載入失敗，建立新的: {e}")

        return {}

    def _save_registry(self) -> bool:
        """將索引表儲存到磁碟

        Returns:
            bool: 成功時為 True，失敗時為 False
        """
        try:
            # 使用暫存檔案確保原子性寫入
            temp_path = self.registry_path.with_suffix(".tmp")
            with open(temp_path, "w", encoding="utf-8") as f:
                json.dump(self.registry, f, indent=2, ensure_ascii=False)
            temp_path.replace(self.registry_path)
            return True

        except Exception as e:
            self.logger.error(f"索引表儲存失敗: {e}")
            return False

    def register_person(
        self,
        person_name: str,
        gender: str = "Other",
        person_type: str = "VIP",
    ) -> str:
        """註冊新人員

        Args:
            person_name: 人員名稱
            gender: 性別 (M/F/Other)
            person_type: 人員類型 (VIP/BLACKLIST)

        Returns:
            str: 分配的人員 UUID

        Raises:
            Exception: 當註冊失敗時
        """
        try:
            # 驗證輸入
            if not person_name.strip():
                raise ValueError("人員名稱不能為空")
            if gender not in ["M", "F", "Other"]:
                raise ValueError("性別只能是 M/F/Other")
            if person_type not in ["VIP", "BLACKLIST"]:
                raise ValueError("人員類型只能是 VIP/BLACKLIST")

            person_uuid = str(uuid.uuid4())

            self.registry[person_uuid] = {
                "person_name": person_name,
                "gender": gender,
                "person_type": person_type,
                "created_at": datetime.now().isoformat(),
                "samples": [],
            }

            self._save_registry()
            self.logger.info(
                f"✓ 人員已註冊: {person_name} ({gender}) [{person_type}] (UUID: {person_uuid})"
            )
            return person_uuid

        except Exception as e:
            self.logger.error(f"人員註冊失敗: {e}")
            raise

    def add_face_sample(
        self,
        person_uuid: str,
        image: np.ndarray,
        embedding: np.ndarray,
    ) -> bool:
        """為已註冊人員添加臉部樣本

        影像與特徵向量會分別儲存在 images/ 與 vectors/ 子目錄中
        特徵向量以 L2 歸一化、float32 格式儲存

        Args:
            person_uuid: 人員 UUID
            image: 臉部影像 (OpenCV 格式)
            embedding: 臉部特徵向量

        Returns:
            bool: 成功時為 True，失敗時為 False
        """
        if person_uuid not in self.registry:
            self.logger.error(f"人員 UUID 不存在: {person_uuid}")
            return False

        try:
            import cv2

            # 建立人員專用目錄
            person_dir = self.data_dir / f"uuid_{person_uuid}"
            images_dir = person_dir / "images"
            vectors_dir = person_dir / "vectors"
            images_dir.mkdir(parents=True, exist_ok=True)
            vectors_dir.mkdir(parents=True, exist_ok=True)

            # 使用時間戳記毫秒 + 索引作為檔案名稱
            timestamp = datetime.now()
            timestamp_ms = timestamp.strftime("%Y%m%d_%H%M%S_%f")[:-3]
            sample_idx = len(self.registry[person_uuid]["samples"])
            filename_prefix = f"{timestamp_ms}_{sample_idx}"

            # 生成檔案名稱
            image_filename = f"{filename_prefix}.jpg"
            vector_filename = f"{filename_prefix}.npy"

            image_path = images_dir / image_filename
            vector_path = vectors_dir / vector_filename

            # 儲存影像
            cv2.imwrite(str(image_path), image)

            # 儲存特徵向量 (L2 歸一化，float32)
            np.save(str(vector_path), embedding.astype(np.float32))

            # 更新索引表
            sample_record = {
                "image_path": str(image_path.relative_to(self.database_dir)),
                "vector_path": str(vector_path.relative_to(self.database_dir)),
                "timestamp": timestamp.isoformat(),
            }
            self.registry[person_uuid]["samples"].append(sample_record)

            self._save_registry()

            self.logger.debug(f"✓ 樣本已添加: {self.registry[person_uuid]['person_name']} " f"({filename_prefix})")
            return True

        except Exception as e:
            self.logger.error(f"樣本添加失敗: {e}")
            return False

    def get_person_embeddings(self, person_uuid: str) -> Optional[np.ndarray]:
        """取得已註冊人員的所有特徵向量

        Args:
            person_uuid: 人員 UUID

        Returns:
            Optional[np.ndarray]: 形狀為 (num_samples, 512) 的特徵向量矩陣，或 None (若無樣本)
        """
        if person_uuid not in self.registry:
            self.logger.error(f"人員 UUID 不存在: {person_uuid}")
            return None

        try:
            samples = self.registry[person_uuid]["samples"]
            if not samples:
                return None

            embeddings = []
            for sample in samples:
                vector_path = self.database_dir / sample["vector_path"]
                if vector_path.exists():
                    embedding = np.load(str(vector_path))
                    embeddings.append(embedding)

            if not embeddings:
                return None

            return np.array(embeddings, dtype=np.float32)

        except Exception as e:
            self.logger.error(f"特徵向量載入失敗: {e}")
            return None

    def get_all_embeddings(self) -> Dict[str, np.ndarray]:
        """取得所有已註冊人員的特徵向量

        Returns:
            Dict[str, np.ndarray]: 特徵向量字典，格式為 {person_uuid: embeddings_matrix}
        """
        all_embeddings = {}

        for person_uuid in self.registry.keys():
            embeddings = self.get_person_embeddings(person_uuid)
            if embeddings is not None:
                all_embeddings[person_uuid] = embeddings

        return all_embeddings

    def get_person_info(self, person_uuid: str) -> Optional[Dict[str, Any]]:
        """取得人員資訊

        Args:
            person_uuid: 人員 UUID

        Returns:
            Optional[Dict[str, Any]]: 人員資訊字典，若人員不存在，返回 None
        """
        return self.registry.get(person_uuid)

    def get_all_persons(self) -> Dict[str, Dict[str, Any]]:
        """取得所有已註冊人員的資訊

        Returns:
            Dict[str, Dict[str, Any]]: 所有人員資訊的字典副本
        """
        return self.registry.copy()

    def list_persons(self) -> List[str]:
        """取得所有已註冊人員的名稱清單

        Returns:
            List[str]: 人員名稱清單
        """
        return [info["person_name"] for info in self.registry.values()]

    def delete_person(self, person_uuid: str) -> bool:
        """刪除指定註冊人員及其所有樣本

        Args:
            person_uuid: 人員 UUID

        Returns:
            bool: 成功時為 True，失敗時為 False
        """
        if person_uuid not in self.registry:
            self.logger.error(f"人員 UUID 不存在: {person_uuid}")
            return False

        try:
            import shutil

            person_name = self.registry[person_uuid]["person_name"]

            # 刪除整個人員專用目錄
            person_dir = self.data_dir / f"uuid_{person_uuid}"
            if person_dir.exists():
                shutil.rmtree(person_dir)

            # 從索引表移除人員
            del self.registry[person_uuid]
            self._save_registry()

            self.logger.info(f"✓ 人員已刪除: {person_name}")
            return True

        except Exception as e:
            self.logger.error(f"人員刪除失敗: {e}")
            return False

    def get_statistics(self) -> Dict[str, Any]:
        """取得資料庫統計資訊

        Returns:
            Dict[str, Any]: 人員資訊統計字典
        """
        total_persons = len(self.registry)
        total_samples = sum(len(info["samples"]) for info in self.registry.values())

        vip_count = sum(1 for info in self.registry.values() if info.get("person_type") == "VIP")
        blacklist_count = sum(1 for info in self.registry.values() if info.get("person_type") == "BLACKLIST")

        persons_stats = {}
        for person_uuid, info in self.registry.items():
            persons_stats[person_uuid] = {
                "person_name": info["person_name"],
                "gender": info.get("gender", "Other"),
                "person_type": info.get("person_type", "VIP"),
                "num_samples": len(info["samples"]),
                "created_at": info["created_at"],
            }

        return {
            "total_persons": total_persons,
            "vip_count": vip_count,
            "blacklist_count": blacklist_count,
            "total_samples": total_samples,
            "persons": persons_stats,
        }

    def print_statistics(self) -> None:
        """列印格式化的統計資訊

        輸出包含總人數、總樣本數以及每個已註冊人員的詳細資訊
        """
        stats = self.get_statistics()

        self.logger.info("\n" + "=" * 70)
        self.logger.info("臉部資料庫統計")
        self.logger.info("=" * 70)
        self.logger.info(f"總人數: {stats['total_persons']} (VIP: {stats['vip_count']}, 黑名單: {stats['blacklist_count']})")
        self.logger.info(f"總樣本數: {stats['total_samples']}")

        for person_uuid, person_stats in stats["persons"].items():
            person_type_str = "🟢 VIP" if person_stats["person_type"] == "VIP" else "🔴 黑名單"
            self.logger.info(
                f"  {person_stats['person_name']} ({person_stats['gender']}) [{person_type_str}] ({person_uuid})"
            )
            self.logger.info(f"    - 樣本數: {person_stats['num_samples']}")
            self.logger.info(f"    - 建立時間: {person_stats['created_at']}")

        self.logger.info("=" * 70)
