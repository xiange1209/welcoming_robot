#!/usr/bin/env python3
"""臉部識別引擎核心類別

封裝 InsightFace 模型的臉部檢測、特徵提取與相似度計算功能
"""

import os
import cv2
import numpy as np
import insightface
from pathlib import Path
from typing import List, Optional, Any
from dataclasses import dataclass

from smartnav_vision.face_utils import get_default_logger


@dataclass
class FaceResult:
    """臉部檢測結果資料類別"""

    bbox: np.ndarray
    confidence: float
    embedding: np.ndarray
    keypoints: np.ndarray
    age: Optional[float] = None
    gender: Optional[str] = None
    person_name: str = "Unknown"
    person_type: str = "VISITOR"  # VIP / BLACKLIST / VISITOR


class FaceEngine:
    """臉部識別引擎 - 封裝 InsightFace 模型與特徵處理"""

    def __init__(
        self,
        model_name: str = "buffalo_sc",
        confidence_threshold: float = 0.5,
        enable_gpu: bool = True,
        ctx_id: int = 0,
        logger: Optional[Any] = None,
    ):
        """初始化臉部引擎

        Args:
            model_name: InsightFace 模型名稱 (buffalo_l, buffalo_sc, buffalo_m 等)
            confidence_threshold: 檢測信心閾值 (0.0-1.0)
            enable_gpu: 若可用，啟用 GPU 加速
            ctx_id: GPU 設備 ID (0 為第一個 GPU，-1 為 CPU)
            logger: 日誌記錄器，若無則使用預設記錄器

        Raises:
            RuntimeError: 當 InsightFace 模型初始化失敗時
        """
        self.model_name = model_name
        self.confidence_threshold = confidence_threshold
        self.enable_gpu = enable_gpu
        self.ctx_id = ctx_id if enable_gpu else -1
        self.logger = logger or get_default_logger(__name__)

        self.face_model: Optional[insightface.app.FaceAnalysis] = None
        self._init_model()

    def _init_model(self) -> None:
        """初始化 InsightFace FaceAnalysis 模型

        Raises:
            RuntimeError: 當模型載入失敗時
        """
        try:
            os.environ["INSIGHTFACE_HOME"] = str(Path.home() / ".insightface")
            providers = self._get_providers()

            self.face_model = insightface.app.FaceAnalysis(
                name=self.model_name,
                providers=providers,
            )

            self.face_model.prepare(ctx_id=self.ctx_id, det_thresh=self.confidence_threshold)

            msg = f"✓ InsightFace {self.model_name} 模型載入成功 " f"(提供者: {', '.join(providers)})"
            self.logger.info(msg)
        except Exception as e:
            raise RuntimeError(f"InsightFace 初始化失敗: {e}\n" "請確保已安裝: pip install insightface onnxruntime")

    def _get_providers(self) -> List[str]:
        """取得 ONNX Runtime 的可用執行提供者

        Returns:
            List[str]: 按優先順序的提供者清單
        """
        providers = []

        if self.enable_gpu:
            try:
                import onnxruntime

                available_providers = onnxruntime.get_available_providers()

                if "CUDAExecutionProvider" in available_providers:
                    providers.append("CUDAExecutionProvider")
                    self.logger.info("✓ GPU 加速 (CUDA) 可用")
                else:
                    self.logger.warning("⚠ 已請求 GPU 但 CUDA 提供者不可用，改用 CPU")
            except ImportError:
                self.logger.warning("⚠ onnxruntime 未找到，改用 CPU")

        providers.append("CPUExecutionProvider")
        return providers

    def detect_and_extract(self, image: np.ndarray) -> List[FaceResult]:
        """偵測影像中的臉部並提取特徵

        Args:
            image: BGR 格式的輸入影像 (OpenCV 格式)

        Returns:
            List[FaceResult]: 偵測到的臉部清單
        """
        if image is None or image.size == 0:
            return []

        try:
            if self.face_model is None:
                self.logger.error("臉部模型未初始化")
                return []

            faces = self.face_model.get(image)

            results = []
            for face in faces:
                face_data = FaceResult(
                    bbox=face.bbox.astype(int),
                    confidence=float(face.det_score),
                    embedding=face.embedding.astype(np.float32),
                    keypoints=face.kps.astype(np.float32),
                )

                age = getattr(face, "age", None)
                if age is not None:
                    face_data.age = float(age)

                gender = getattr(face, "gender", None)
                if gender is not None:
                    face_data.gender = "M" if gender == 1 else "F"

                results.append(face_data)

            return results
        except Exception as e:
            self.logger.error(f"臉部檢測錯誤: {e}")
            return []

    def compute_similarity(self, embedding1: np.ndarray, embedding2: np.ndarray) -> float:
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
            self.logger.error(f"相似度計算錯誤: {e}")
            return 0.0

    def extract_face_region(
        self, image: np.ndarray, bbox: np.ndarray, expand_ratio: float = 0.2
    ) -> Optional[np.ndarray]:
        """從影像中提取臉部區域並做簡單擴展

        Args:
            image: BGR 格式的輸入影像
            bbox: 邊界框座標 [x1, y1, x2, y2]
            expand_ratio: 邊界框擴展比例 (0.2 = 20% 擴展)

        Returns:
            Optional[np.ndarray]: 裁剪的臉部影像，若提取失敗則返回 None
        """
        try:
            h, w = image.shape[:2]
            x1, y1, x2, y2 = bbox

            width = x2 - x1
            height = y2 - y1
            expand_x = int(width * expand_ratio / 2)
            expand_y = int(height * expand_ratio / 2)

            x1 = max(0, x1 - expand_x)
            y1 = max(0, y1 - expand_y)
            x2 = min(w, x2 + expand_x)
            y2 = min(h, y2 + expand_y)

            face_region = image[y1:y2, x1:x2]

            if face_region.size == 0:
                return None

            return face_region
        except Exception as e:
            self.logger.debug(f"臉部區域提取錯誤: {e}")
            return None

    def draw_detections(
        self,
        image: np.ndarray,
        faces: List[FaceResult],
        draw_keypoints: bool = True,
        draw_info: bool = True,
    ) -> np.ndarray:
        """在影像上繪製臉部檢測結果

        繪製內容包括邊界框、人員名稱、性別、檢測信心、年齡、身份類型與臉部關鍵點

        邊界框顏色編碼：
        - 🟢 綠色 (VIP)
        - 🔴 紅色 (黑名單)
        - 🔵 藍色 (訪客)

        Args:
            image: BGR 格式的輸入影像
            faces: 從 detect_and_extract() 獲得的檢測結果清單
            draw_keypoints: 是否繪製臉部關鍵點
            draw_info: 是否繪製信心分數、年齡、性別

        Returns:
            np.ndarray: 繪製檢測結果的影像
        """
        image_copy = image.copy()

        for i, face in enumerate(faces):
            x1, y1, x2, y2 = map(int, face.bbox)
            confidence = face.confidence
            person_name = face.person_name
            person_type = getattr(face, "person_type", "VISITOR")
            gender = getattr(face, "gender", "")

            # 根據身份類型選擇顏色
            if person_type == "BLACKLIST":
                color = (0, 0, 255)  # 紅色 (BGR)
                type_label = "🔴 黑名單"
            elif person_type == "VIP":
                color = (0, 255, 0)  # 綠色 (BGR)
                type_label = "🟢 VIP"
            else:  # VISITOR
                color = (255, 0, 0)  # 藍色 (BGR)
                type_label = "🔵 訪客"

            # 繪製邊界框
            cv2.rectangle(image_copy, (x1, y1), (x2, y2), color, 2)

            if draw_info:
                # 主標籤：人員名稱、性別、身份類型
                if person_name != "Unknown":
                    gender_str = f"_{gender}" if gender else ""
                    text = f"{person_name}{gender_str} ({confidence:.2f})"
                else:
                    text = f"訪客 ({confidence:.2f})"

                cv2.putText(
                    image_copy,
                    text,
                    (x1, max(y1 - 30, 25)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )

                # 身份類型標籤
                cv2.putText(
                    image_copy,
                    type_label,
                    (x1, max(y1 - 10, 25)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                )

                # 年齡和性別資訊 (偵測到時顯示)
                info_text = []
                if face.age is not None:
                    info_text.append(f"Age: {int(face.age)}")
                if face.gender is not None and person_name == "Unknown":
                    # 未識別時才顯示自動偵測的性別
                    info_text.append(f"Det: {face.gender}")

                if info_text:
                    info_str = ", ".join(info_text)
                    cv2.putText(
                        image_copy,
                        info_str,
                        (x1, y2 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        1,
                    )

            if draw_keypoints and face.keypoints is not None:
                keypoints = face.keypoints.astype(int)
                for kp in keypoints:
                    cv2.circle(image_copy, tuple(kp), 2, (0, 0, 255), -1)

        return image_copy
