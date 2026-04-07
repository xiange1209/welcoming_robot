"""
人臉識別模組 - 基於 embedding 相似度的 VIP/黑名單匹配
使用歐氏距離或餘弦相似度進行匹配
"""

import numpy as np
import sqlite3
from pathlib import Path
from typing import Tuple, Optional, Dict
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class FaceRecognizer:
    """基於 embedding 的人臉識別"""

    def __init__(self, db_path: str = "database/ai_bank_robot.db",
                 vip_threshold: float = 0.65,
                 blacklist_threshold: float = 0.60):
        """
        初始化人臉識別器

        Args:
            db_path: SQLite 數據庫路徑
            vip_threshold: VIP 匹配信心度閾值 (0-1, 越高越嚴格)
            blacklist_threshold: 黑名單匹配信心度閾值
        """
        self.db_path = db_path
        self.vip_threshold = vip_threshold
        self.blacklist_threshold = blacklist_threshold
        self.conn = None
        self._load_embeddings_cache()

    def _load_embeddings_cache(self):
        """快取內存中的所有 embedding（加速匹配）"""
        self.conn = sqlite3.connect(self.db_path)
        self.conn.row_factory = sqlite3.Row
        self.vip_cache = {}  # {name: (embedding_array, gender)}
        self.blacklist_cache = {}

        # 加載 VIP 數據
        cursor = self.conn.cursor()
        cursor.execute('SELECT name, gender, face_embedding FROM vip_members WHERE is_active = 1')
        for row in cursor.fetchall():
            try:
                embedding = np.frombuffer(row['face_embedding'], dtype=np.float32)
                gender = row['gender'] if row['gender'] else 'Other'
                self.vip_cache[row['name']] = (embedding, gender)
            except:
                logger.warning(f"無法加載 VIP {row['name']} 的 embedding")

        # 加載黑名單數據
        cursor.execute('SELECT name, gender, face_embedding FROM blacklist WHERE is_active = 1')
        for row in cursor.fetchall():
            try:
                embedding = np.frombuffer(row['face_embedding'], dtype=np.float32)
                gender = row['gender'] if row['gender'] else 'Other'
                self.blacklist_cache[row['name']] = (embedding, gender)
            except:
                logger.warning(f"無法加載黑名單 {row['name']} 的 embedding")

        logger.info(f"✓ 已加載 {len(self.vip_cache)} 個 VIP, {len(self.blacklist_cache)} 個黑名單")

    @staticmethod
    def euclidean_distance(emb1: np.ndarray, emb2: np.ndarray) -> float:
        """歐氏距離（越小越相似）"""
        return float(np.linalg.norm(emb1 - emb2))

    @staticmethod
    def cosine_similarity(emb1: np.ndarray, emb2: np.ndarray) -> float:
        """餘弦相似度（-1 到 1，1 表示完全相同）"""
        norm1 = np.linalg.norm(emb1)
        norm2 = np.linalg.norm(emb2)
        if norm1 == 0 or norm2 == 0:
            return 0.0
        return float(np.dot(emb1, emb2) / (norm1 * norm2))

    @staticmethod
    def similarity_from_distance(distance: float, max_distance: float = 1.5) -> float:
        """將歐氏距離轉換為相似度 (0-1)"""
        # 距離越小，相似度越高
        # distance=0 -> similarity=1.0
        # distance=max_distance -> similarity=0.0
        similarity = max(0.0, 1.0 - (distance / max_distance))
        return min(1.0, similarity)

    def recognize_face(self, face_embedding: np.ndarray) -> Dict:
        """
        識別人臉 - 同時檢查 VIP 和黑名單

        Args:
            face_embedding: 512D 人臉 embedding 向量

        Returns:
            {
                'person_type': 'vip' | 'blacklist' | 'visitor',
                'name': str or None,
                'gender': str or None,  # 'M', 'F', 'Other'
                'confidence': float (0-1),
                'vip_level': str or None,  # 'gold', 'platinum', 'standard'
                'risk_level': str or None,  # 'low', 'medium', 'high'
                'details': dict
            }
        """
        result = {
            'person_type': 'visitor',
            'name': None,
            'gender': None,
            'confidence': 0.0,
            'vip_level': None,
            'risk_level': None,
            'details': {}
        }

        # 1. 先檢查黑名單（安全優先）
        best_blacklist_match = None
        best_blacklist_confidence = 0.0
        best_blacklist_gender = None

        for name, (blk_embedding, gender) in self.blacklist_cache.items():
            distance = self.euclidean_distance(face_embedding, blk_embedding)
            confidence = self.similarity_from_distance(distance)

            if confidence > best_blacklist_confidence:
                best_blacklist_confidence = confidence
                best_blacklist_match = name
                best_blacklist_gender = gender

        if best_blacklist_confidence > self.blacklist_threshold:
            # 檢測到黑名單人員
            cursor = self.conn.cursor()
            cursor.execute('SELECT risk_level, reason FROM blacklist WHERE name = ?', (best_blacklist_match,))
            blk_row = cursor.fetchone()

            result['person_type'] = 'blacklist'
            result['name'] = best_blacklist_match
            result['gender'] = best_blacklist_gender
            result['confidence'] = best_blacklist_confidence
            result['risk_level'] = blk_row['risk_level'] if blk_row else 'medium'
            result['details'] = {
                'reason': blk_row['reason'] if blk_row else 'Unknown',
                'distance': self.euclidean_distance(face_embedding, self.blacklist_cache[best_blacklist_match][0])
            }
            return result

        # 2. 檢查 VIP（如果不是黑名單）
        best_vip_match = None
        best_vip_confidence = 0.0
        best_vip_gender = None

        for name, (vip_embedding, gender) in self.vip_cache.items():
            distance = self.euclidean_distance(face_embedding, vip_embedding)
            confidence = self.similarity_from_distance(distance)

            if confidence > best_vip_confidence:
                best_vip_confidence = confidence
                best_vip_match = name
                best_vip_gender = gender

        if best_vip_confidence > self.vip_threshold:
            # 識別為 VIP
            cursor = self.conn.cursor()
            cursor.execute('SELECT vip_level, phone, email FROM vip_members WHERE name = ?', (best_vip_match,))
            vip_row = cursor.fetchone()

            result['person_type'] = 'vip'
            result['name'] = best_vip_match
            result['gender'] = best_vip_gender
            result['confidence'] = best_vip_confidence
            result['vip_level'] = vip_row['vip_level'] if vip_row else 'standard'
            result['details'] = {
                'phone': vip_row['phone'] if vip_row else None,
                'email': vip_row['email'] if vip_row else None,
                'distance': self.euclidean_distance(face_embedding, self.vip_cache[best_vip_match][0])
            }
            return result

        # 3. 一般訪客
        result['person_type'] = 'visitor'
        result['confidence'] = max(best_vip_confidence, best_blacklist_confidence)
        result['details'] = {
            'closest_match': best_vip_match or best_blacklist_match,
            'closest_confidence': max(best_vip_confidence, best_blacklist_confidence)
        }
        return result

    def reload_cache(self):
        """重新加載快取（如果 VIP/黑名單更新）"""
        if self.conn:
            self.conn.close()
        self._load_embeddings_cache()
        logger.info("✓ 快取已重新加載")

    def add_vip(self, name: str, face_embedding: np.ndarray, gender: str = 'Other', phone: str = None,
                email: str = None, vip_level: str = 'standard') -> bool:
        """添加 VIP 並更新快取"""
        try:
            cursor = self.conn.cursor()
            cursor.execute('''
                INSERT OR REPLACE INTO vip_members (name, gender, face_embedding, phone, email, vip_level)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', (name, gender, face_embedding.tobytes(), phone, email, vip_level))
            self.conn.commit()

            # 更新快取
            self.vip_cache[name] = (face_embedding, gender)
            logger.info(f"✓ VIP '{name}' ({gender}) 已添加")
            return True
        except Exception as e:
            logger.error(f"✗ 添加 VIP 失敗: {e}")
            return False

    def add_blacklist(self, name: str, face_embedding: np.ndarray, gender: str = 'Other', reason: str = None,
                     risk_level: str = 'medium') -> bool:
        """添加黑名單"""
        try:
            cursor = self.conn.cursor()
            cursor.execute('''
                INSERT OR REPLACE INTO blacklist (name, gender, face_embedding, reason, risk_level)
                VALUES (?, ?, ?, ?, ?)
            ''', (name, gender, face_embedding.tobytes(), reason, risk_level))
            self.conn.commit()

            # 更新快取
            self.blacklist_cache[name] = (face_embedding, gender)
            logger.info(f"✓ 黑名單 '{name}' ({gender}) 已添加")
            return True
        except Exception as e:
            logger.error(f"✗ 添加黑名單失敗: {e}")
            return False

    def close(self):
        """關閉數據庫連接"""
        if self.conn:
            self.conn.close()


if __name__ == "__main__":
    # 測試代碼
    recognizer = FaceRecognizer()

    # 創建一個測試 embedding（512D）
    test_embedding = np.random.randn(512).astype(np.float32)

    # 測試識別（應該返回 visitor，因為沒有 VIP）
    result = recognizer.recognize_face(test_embedding)
    print(f"識別結果: {result}")

    recognizer.close()
