"""
使用 MediaPipe 的人臉檢測 - 簡化版（無需下載模型）
適合 Phase 2 快速原型開發
"""

import cv2
import numpy as np
from typing import List, Dict
import logging

try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MediaPipeFaceDetector:
    """使用 MediaPipe 的人臉檢測器 - 無需下載模型"""

    def __init__(self):
        """初始化 MediaPipe 人臉檢測"""
        if not MEDIAPIPE_AVAILABLE:
            logger.error("✗ MediaPipe 未安裝")
            logger.info("安裝指令: pip install mediapipe")
            self.detector = None
            return

        # 初始化 MediaPipe Face Detection
        mp_face_detection = mp.solutions.face_detection
        self.detector = mp_face_detection.FaceDetection(
            model_selection=0,  # 0=近距離, 1=遠距離
            min_detection_confidence=0.5
        )
        logger.info("✓ MediaPipe Face Detector 已加載")

    def detect(self, image: np.ndarray) -> List[Dict]:
        """
        檢測人臉
        Args:
            image: 輸入圖像 (BGR, numpy array)
        Returns:
            檢測結果列表
        """
        if self.detector is None:
            return []

        # 轉換為 RGB（MediaPipe 需要 RGB）
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        h, w = image.shape[:2]

        # 運行檢測
        results = self.detector.process(rgb_image)

        detections = []
        if results.detections:
            for detection in results.detections:
                bbox = detection.location_data.relative_bounding_box

                # 轉換為像素座標
                x_min = max(0, int(bbox.xmin * w))
                y_min = max(0, int(bbox.ymin * h))
                x_max = min(w, int((bbox.xmin + bbox.width) * w))
                y_max = min(h, int((bbox.ymin + bbox.height) * h))

                detections.append({
                    'box': [x_min, y_min, x_max, y_max],
                    'confidence': detection.score[0],
                    'landmarks': []  # MediaPipe 提供關鍵點，可選添加
                })

        return detections

    def draw_detections(self, image: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """繪製檢測結果"""
        result = image.copy()
        for det in detections:
            x1, y1, x2, y2 = det['box']
            confidence = det['confidence']

            # 繪製邊界框
            cv2.rectangle(result, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # 繪製置信度
            label = f"{confidence:.2f}"
            cv2.putText(result, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return result


class InsightFaceFaceDetector:
    """使用 InsightFace 的檢測器 - 更多功能（Phase 3+）"""

    def __init__(self):
        """初始化 InsightFace"""
        try:
            from insightface.app import FaceAnalysis
            self.face_analysis = FaceAnalysis()
            self.face_analysis.prepare(ctx_id=0, det_model='retinaface')
            logger.info("✓ InsightFace 已加載")
            self.available = True
        except ImportError:
            logger.warning("⚠ InsightFace 未安裝")
            logger.info("安裝指令: pip install insightface")
            self.available = False
        except Exception as e:
            logger.error(f"✗ InsightFace 初始化失敗: {e}")
            self.available = False

    def detect(self, image: np.ndarray) -> List[Dict]:
        """檢測人臉"""
        if not self.available:
            return []

        faces = self.face_analysis.get(image)

        detections = []
        for face in faces:
            bbox = face.bbox.astype(int)
            x1, y1, x2, y2 = bbox

            detections.append({
                'box': [x1, y1, x2, y2],
                'confidence': float(face.det_score),
                'embedding': face.embedding  # 128D 人臉嵌入向量
            })

        return detections


# 選擇推薦檢測器
def get_face_detector():
    """獲取推薦的人臉檢測器"""
    logger.info("嘗試加載人臉檢測器...")

    # Phase 2：優先使用 MediaPipe（無需下載）
    detector = MediaPipeFaceDetector()
    if detector.detector is not None:
        logger.info("✓ 使用 MediaPipe 檢測器（推薦 Phase 2）")
        return detector

    # Phase 3+：使用 InsightFace（功能更多）
    detector = InsightFaceFaceDetector()
    if detector.available:
        logger.info("✓ 使用 InsightFace 檢測器（Phase 3+）")
        return detector

    logger.error("✗ 無可用的人臉檢測器")
    return None


if __name__ == "__main__":
    # 測試
    detector = get_face_detector()
    if detector:
        logger.info("✓ 檢測器已就緒，可以進行即時推論")
