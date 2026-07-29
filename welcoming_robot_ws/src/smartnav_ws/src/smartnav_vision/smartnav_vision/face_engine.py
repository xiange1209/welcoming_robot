#!/usr/bin/env python3
"""臉部識別引擎核心類別

封裝 InsightFace 模型的臉部偵測、特徵提取功能
"""

import os
import numpy as np
import insightface
from pathlib import Path
from typing import List, Optional, Any
from dataclasses import dataclass

from smartnav_vision.face_utils import get_default_logger


@dataclass
class FaceDetectResult:
    """臉部偵測結果資料類別"""

    bbox: np.ndarray
    embedding: np.ndarray
    age: Optional[float] = None
    gender: Optional[str] = None


class FaceEngine:
    """臉部識別引擎 - 封裝 InsightFace 模型與特徵處理"""

    def __init__(
        self,
        model_name: str = "buffalo_sc",
        ctx_id: int = 0,
        det_thresh: float = 0.5,
        enable_gpu: bool = True,
        logger: Optional[Any] = None,
    ):
        """初始化臉部引擎

        Args:
            model_name: InsightFace 模型名稱 (buffalo_sc, buffalo_m, buffalo_l 等)
            ctx_id: GPU 設備 ID (0 為第一個 GPU，-1 為 CPU)
            det_thresh: 偵測信心閾值 (0.0-1.0)
            enable_gpu: 若可用，啟用 GPU 加速
            logger: 日誌記錄器，若無則使用預設記錄器
        """
        self.model_name = model_name
        self.det_thresh = det_thresh
        self.enable_gpu = enable_gpu
        self.ctx_id = ctx_id if enable_gpu else -1
        self.logger = logger or get_default_logger(__name__)

        self.face_model: Optional[insightface.app.FaceAnalysis] = None
        self._init_model()

    def _init_model(self) -> None:
        """初始化 InsightFace FaceAnalysis 模型"""
        try:
            os.environ["INSIGHTFACE_HOME"] = str(Path.home() / ".insightface")
            providers = self._get_providers()

            self.face_model = insightface.app.FaceAnalysis(
                name=self.model_name,
                providers=providers,
            )

            self.face_model.prepare(ctx_id=self.ctx_id, det_thresh=self.det_thresh)
            self.logger.info(f"✓ InsightFace {self.model_name} 模型載入成功")
        except Exception as e:
            raise RuntimeError(f"InsightFace 初始化失敗: {e}")

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
                    self.logger.warning("⚠ GPU 加速 (CUDA) 不可用，將使用 CPU")
            except ImportError:
                self.logger.warning("⚠ onnxruntime 未找到，將使用 CPU")

        providers.append("CPUExecutionProvider")
        return providers

    def detect_and_extract(self, image: np.ndarray) -> Optional[FaceDetectResult]:
        """偵測影像中的臉部並提取特徵

        Args:
            image: BGR 格式的輸入影像 (OpenCV 格式)

        Returns:
            Optional[FaceDetectResult]: 提取到的臉部結果，若沒有提取到則返回 None
        """
        if image is None or image.size == 0:
            return None

        try:
            if self.face_model is None:
                self.logger.error("臉部模型未初始化")
                return None

            faces = self.face_model.get(image, max_num=1)

            if len(faces) == 0:
                return None

            face = FaceDetectResult(
                bbox=faces[0].bbox.astype(int),
                embedding=faces[0].embedding.astype(np.float32),
            )

            age = getattr(faces[0], "age", None)
            if age is not None:
                face.age = float(age)

            gender = getattr(faces[0], "gender", None)
            if gender is not None:
                face.gender = "M" if gender == 1 else "F"
        except Exception as e:
            self.logger.error(f"臉部偵測並提取錯誤: {e}")
            return None

        return face
