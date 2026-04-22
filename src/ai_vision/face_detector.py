"""
人臉檢測模組 - 使用 RetinaFace INT8 ONNX 模型
輸出：人臉邊界框、置信度、5 點關鍵點
"""

import cv2
import numpy as np
from typing import List, Tuple, Dict
import logging
from .inference_engine import InferenceEngine

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class FaceDetector:
    """RetinaFace 人臉檢測器"""

    def __init__(self, config_path: str = "config/inference_config.yaml", model_name: str = 'face_detector'):
        """
        初始化人臉檢測器
        Args:
            config_path: 配置檔路徑
            model_name: 推理引擎中的模型名稱
        """
        self.engine = InferenceEngine(config_path)
        self.model_name = model_name
        self.config = self.engine.config
        self.threshold = self.config['inference_params']['face_detect_threshold']
        self.nms_threshold = self.config['inference_params']['face_detect_nms_threshold']

        # 加載模型
        model_path = self.config['model_paths'][model_name]
        self.session = self.engine.load_model(model_name, model_path)

        if self.session is None:
            logger.error("✗ 人臉檢測器初始化失敗")

    def _preprocess(self, image: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int]]:
        """
        前處理圖像
        Args:
            image: 輸入圖像 (BGR, numpy array)
        Returns:
            (processed_image, original_size)
        """
        h, w = image.shape[:2]
        # RetinaFace 通常輸入 640x480 或 320x240（INT8 版本）
        target_size = 320
        scale = target_size / max(h, w)
        new_h, new_w = int(h * scale), int(w * scale)

        resized = cv2.resize(image, (new_w, new_h))
        # 填充到方形
        padded = np.zeros((target_size, target_size, 3), dtype=np.float32)
        padded[:new_h, :new_w] = resized

        # 標準化
        padded = (padded - 127.5) / 128.0

        # HWC -> CHW
        processed = np.transpose(padded, (2, 0, 1))
        processed = np.expand_dims(processed, 0).astype(np.float32)

        return processed, (h, w)

    def _postprocess(self, outputs: List[np.ndarray], original_size: Tuple[int, int]) -> List[Dict]:
        """
        後處理模型輸出
        Args:
            outputs: 模型輸出列表
            original_size: 原始圖像尺寸 (h, w)
        Returns:
            檢測結果列表 [{'box': [...], 'confidence': ..., 'landmarks': [...]}]
        """
        h, w = original_size
        detections = []

        # 解析 RetinaFace 輸出
        # 通常輸出格式：[batch, num_anchors, (x1, y1, x2, y2, score, lx, ly, rx, ry, nose_x, nose_y, lmouth_x, lmouth_y, rmouth_x, rmouth_y)]
        if isinstance(outputs, list):
            face_outputs = outputs[0]  # 邊界框和置信度
        else:
            face_outputs = outputs

        for detection in face_outputs:
            if detection[4] < self.threshold:  # 置信度檢查
                continue

            x1, y1, x2, y2 = detection[:4]
            # 縮放回原始尺寸
            x1, y1, x2, y2 = int(x1 * w / 320), int(y1 * h / 320), int(x2 * w / 320), int(y2 * h / 320)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)

            confidence = float(detection[4])
            # 5 點關鍵點（眼睛、鼻子、嘴角）
            landmarks = detection[5:15].reshape(5, 2) if len(detection) > 15 else []

            detections.append({
                'box': [x1, y1, x2, y2],
                'confidence': confidence,
                'landmarks': landmarks,
                'box_area': (x2 - x1) * (y2 - y1)
            })

        # 非極大值抑制 (NMS)
        detections = self._nms(detections, self.nms_threshold)
        return detections

    @staticmethod
    def _nms(detections: List[Dict], nms_threshold: float) -> List[Dict]:
        """
        非極大值抑制
        Args:
            detections: 檢測結果列表
            nms_threshold: IoU 閾值
        Returns:
            過濾後的檢測結果
        """
        if not detections:
            return []

        # 按置信度排序
        detections = sorted(detections, key=lambda x: x['confidence'], reverse=True)

        keep = []
        while detections:
            current = detections.pop(0)
            keep.append(current)

            if not detections:
                break

            # 移除與當前框 IoU 過高的檢測
            remaining = []
            for det in detections:
                iou = FaceDetector._compute_iou(current['box'], det['box'])
                if iou < nms_threshold:
                    remaining.append(det)
            detections = remaining

        return keep

    @staticmethod
    def _compute_iou(box1: List[int], box2: List[int]) -> float:
        """計算 IoU (Intersection over Union)"""
        x1_min, y1_min, x1_max, y1_max = box1
        x2_min, y2_min, x2_max, y2_max = box2

        inter_x_min = max(x1_min, x2_min)
        inter_y_min = max(y1_min, y2_min)
        inter_x_max = min(x1_max, x2_max)
        inter_y_max = min(y1_max, y2_max)

        if inter_x_max < inter_x_min or inter_y_max < inter_y_min:
            return 0.0

        inter_area = (inter_x_max - inter_x_min) * (inter_y_max - inter_y_min)
        box1_area = (x1_max - x1_min) * (y1_max - y1_min)
        box2_area = (x2_max - x2_min) * (y2_max - y2_min)
        union_area = box1_area + box2_area - inter_area

        return inter_area / union_area if union_area > 0 else 0.0

    def detect(self, image: np.ndarray) -> List[Dict]:
        """
        檢測圖像中的人臉
        Args:
            image: 輸入圖像 (BGR, numpy array)
        Returns:
            檢測結果列表 [{'box': [x1, y1, x2, y2], 'confidence': ..., 'landmarks': [...]}]
        """
        if self.session is None:
            logger.error("✗ 模型未加載，無法進行檢測")
            return []

        processed, original_size = self._preprocess(image)
        outputs = self.engine.predict(self.model_name, processed)

        if outputs is None:
            return []

        detections = self._postprocess(outputs, original_size)
        return detections

    def draw_detections(self, image: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """
        在圖像上繪製檢測結果
        Args:
            image: 輸入圖像
            detections: 檢測結果列表
        Returns:
            繪製後的圖像
        """
        result = image.copy()

        for det in detections:
            x1, y1, x2, y2 = det['box']
            confidence = det['confidence']

            # 繪製邊界框
            cv2.rectangle(result, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # 繪製置信度
            label = f"{confidence:.2f}"
            cv2.putText(result, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 繪製 5 點關鍵點
            if len(det['landmarks']) > 0:
                landmarks = det['landmarks']
                for lm in landmarks:
                    # 縮放關鍵點座標
                    lm_x, lm_y = int(lm[0]), int(lm[1])
                    cv2.circle(result, (lm_x, lm_y), 3, (0, 0, 255), -1)

        return result

    def benchmark(self, image_size: Tuple[int, int] = (320, 240), num_runs: int = 10) -> Dict[str, float]:
        """
        基準測試檢測性能
        Args:
            image_size: 測試圖像尺寸
            num_runs: 測試運行次數
        Returns:
            {min_ms, max_ms, avg_ms, fps}
        """
        return self.engine.benchmark(self.model_name, (1, 3, image_size[1], image_size[0]), num_runs)


if __name__ == "__main__":
    # 測試人臉檢測
    detector = FaceDetector()
    print("✓ 人臉檢測器初始化成功")

    # 測試模型性能
    benchmark = detector.benchmark()
    print(f"基準測試結果：{benchmark}")
