#!/usr/bin/env python3
"""
即時人臉檢測 - YOLOv8 ONNX 版本（無需 PyTorch）
支持：Raspberry Pi 4 + Picamera2
特性：
  - YOLOv8 Nano ONNX 模型（~6MB）
  - ONNX Runtime CPU 推理（快速且輕量）
  - 支持側臉檢測
  - 跳幀優化（提高 FPS）
  - 按 Q 退出，S 保存，空格暫停
"""

import cv2
import numpy as np
import sys
import time
from pathlib import Path
from picamera2 import Picamera2
import logging
import urllib.request
import os

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

try:
    import onnxruntime as ort
    ONNX_AVAILABLE = True
except ImportError:
    ONNX_AVAILABLE = False


class RealtimeFaceDetectorYOLOv8ONNX:
    """使用 Picamera2 + YOLOv8 ONNX 的即時人臉檢測（無需 PyTorch）"""

    # YOLOv8n-face ONNX 模型下載源
    MODEL_URL = "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n-face.onnx"
    MODEL_PATH = "models/yolov8n-face.onnx"

    def __init__(self, model_path: str = None):
        """初始化檢測器"""
        if not ONNX_AVAILABLE:
            logger.error("✗ ONNX Runtime 未安裝。請執行：pip install onnxruntime")
            sys.exit(1)

        if model_path is None:
            model_path = self.MODEL_PATH

        logger.info("初始化 Picamera2...")
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "XBGR8888"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)
        logger.info("✓ Picamera2 已初始化 (640x480)")

        # 檢查並下載模型
        self.model_path = model_path
        self._ensure_model()

        # 初始化 ONNX 推理引擎
        logger.info(f"加載 YOLOv8 ONNX 模型: {self.model_path}")
        try:
            # 使用 CPU 執行程序
            sess_options = ort.SessionOptions()
            sess_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL

            self.session = ort.InferenceSession(
                self.model_path,
                sess_options=sess_options,
                providers=['CPUExecutionProvider']
            )

            # 獲取輸入/輸出信息
            self.input_name = self.session.get_inputs()[0].name
            self.output_names = [output.name for output in self.session.get_outputs()]

            logger.info("✓ YOLOv8 Nano ONNX 已加載")
        except Exception as e:
            logger.error(f"✗ 模型加載失敗: {e}")
            sys.exit(1)

        # 跳幀設置
        self.detect_interval = 2  # 每 2 幀檢測一次
        self.frame_since_detect = 0
        self.last_detections = []

        # 統計信息
        self.frame_count = 0
        self.max_faces = 0
        self.start_time = time.time()
        self.save_dir = Path("/tmp")
        self.paused = False

    def _ensure_model(self):
        """檢查模型是否存在，不存在則下載"""
        model_file = Path(self.model_path)

        if model_file.exists():
            logger.info(f"✓ 模型已存在: {self.model_path}")
            return

        logger.info(f"📥 模型不存在，準備下載...")
        logger.info(f"   源: {self.MODEL_URL}")

        # 創建目錄
        model_file.parent.mkdir(parents=True, exist_ok=True)

        try:
            logger.info(f"   下載中... (這可能需要 1-2 分鐘)")
            urllib.request.urlretrieve(self.MODEL_URL, self.model_path)
            logger.info(f"✓ 模型下載完成: {self.model_path}")
        except Exception as e:
            logger.error(f"✗ 模型下載失敗: {e}")
            logger.info("   請手動下載或檢查網絡連接")
            sys.exit(1)

    def detect_faces_yolo_onnx(self, frame):
        """使用 YOLOv8 ONNX 檢測人臉"""
        h, w = frame.shape[:2]

        # 預處理：縮放到 640x640
        blob = cv2.dnn.blobFromImage(
            frame, 1.0/255.0, (640, 640),
            [0, 0, 0], swapRB=False, crop=False
        )

        # 推理
        try:
            outputs = self.session.run(self.output_names, {self.input_name: blob})
        except Exception as e:
            logger.error(f"推理失敗: {e}")
            return []

        # 解析輸出
        detections = []
        predictions = outputs[0]  # [1, 25200, 15] - batch, proposals, (x, y, w, h, conf, classes...)

        if predictions.shape[0] == 0:
            return detections

        predictions = predictions[0]  # 取第一個 batch

        for pred in predictions:
            # YOLOv8 輸出格式: [x, y, w, h, conf, ...]
            x, y, w_pred, h_pred = pred[:4]
            confidence = float(pred[4])

            # 過濾置信度
            if confidence < 0.5:
                continue

            # 轉換座標（從 640x640 縮放回原始尺寸）
            scale_x = w / 640
            scale_y = h / 640

            x1 = int((x - w_pred / 2) * scale_x)
            y1 = int((y - h_pred / 2) * scale_y)
            x2 = int((x + w_pred / 2) * scale_x)
            y2 = int((y + h_pred / 2) * scale_y)

            # 邊界檢查
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(w, x2)
            y2 = min(h, y2)

            if x2 > x1 and y2 > y1:
                box_w = x2 - x1
                box_h = y2 - y1

                detections.append({
                    'box': (x1, y1, box_w, box_h),
                    'confidence': confidence,
                    'type': 'face'
                })

        # 應用 NMS 去除重複
        detections = self._nms(detections, nms_threshold=0.4)

        return detections

    @staticmethod
    def _nms(detections, nms_threshold=0.4):
        """非最大值抑制 - 去除重疊檢測"""
        if len(detections) < 2:
            return detections

        # 計算面積
        for det in detections:
            x, y, w, h = det['box']
            det['area'] = w * h

        # 按面積降序排列
        detections = sorted(detections, key=lambda x: x['area'], reverse=True)

        keep = []
        while detections:
            current = detections.pop(0)
            keep.append(current)

            remaining = []
            for det in detections:
                iou = RealtimeFaceDetectorYOLOv8ONNX._iou(current['box'], det['box'])
                if iou < nms_threshold:
                    remaining.append(det)
            detections = remaining

        return keep

    @staticmethod
    def _iou(box1, box2):
        """計算兩個邊界框的 IoU"""
        x1, y1, w1, h1 = box1
        x2, y2, w2, h2 = box2

        xi1 = max(x1, x2)
        yi1 = max(y1, y2)
        xi2 = min(x1 + w1, x2 + w2)
        yi2 = min(y1 + h1, y2 + h2)

        inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)

        box1_area = w1 * h1
        box2_area = w2 * h2
        union_area = box1_area + box2_area - inter_area

        return inter_area / union_area if union_area > 0 else 0

    def draw_detections(self, frame, detections):
        """繪製檢測結果"""
        result = frame.copy()
        for det in detections:
            x, y, w, h = det['box']
            confidence = det['confidence']

            # 繪製邊界框（綠色）
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 繪製標籤
            label = f"Face {confidence:.2f}"
            cv2.putText(result, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return result

    def run(self):
        """運行即時檢測"""
        logger.info("=" * 70)
        logger.info("🎥 即時人臉檢測 (Picamera2 + YOLOv8 ONNX)")
        logger.info("=" * 70)
        logger.info("控制：")
        logger.info("  Q - 退出")
        logger.info("  S - 保存當前幀")
        logger.info("  空格 - 暫停/繼續")
        logger.info("=" * 70)

        try:
            while True:
                # 讀取幀
                frame_xbgr = self.picam2.capture_array()
                if frame_xbgr is None:
                    logger.error("✗ 讀取幀失敗")
                    break

                # 色彩轉換：XBGR8888 → BGR
                frame_bgr = cv2.cvtColor(frame_xbgr, cv2.COLOR_RGBA2BGR)

                if not self.paused:
                    self.frame_count += 1
                    self.frame_since_detect += 1

                    # 每 N 幀進行一次檢測
                    if self.frame_since_detect >= self.detect_interval:
                        self.last_detections = self.detect_faces_yolo_onnx(frame_bgr)
                        self.frame_since_detect = 0
                        self.max_faces = max(self.max_faces, len(self.last_detections))

                    detections = self.last_detections
                    result = self.draw_detections(frame_bgr, detections)
                else:
                    result = frame_bgr
                    detections = []

                # 添加統計信息
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                status = "⏸️  PAUSED" if self.paused else "▶️  RUNNING"
                info_text = f"Frame: {self.frame_count:4d} | FPS: {fps:5.1f} | Faces: {len(detections)} | {status}"
                cv2.putText(result, info_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # 控制提示
                hint_text = "Press [Q]uit [S]ave [Space]Pause"
                cv2.putText(result, hint_text, (10, 460),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                # 顯示幀
                cv2.imshow("Face Detection (YOLOv8 ONNX)", result)

                # 按鍵處理
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q'):
                    logger.info("\n< 退出中... >")
                    break
                elif key == ord('s') or key == ord('S'):
                    filename = self.save_dir / f"face_detection_{self.frame_count:05d}.jpg"
                    cv2.imwrite(str(filename), result)
                    logger.info(f"✓ 幀已保存：{filename}")
                elif key == 32:  # 空格鍵
                    self.paused = not self.paused
                    status_msg = "⏸️  已暫停" if self.paused else "▶️  已繼續"
                    logger.info(f"{status_msg}")

        except KeyboardInterrupt:
            logger.info("\n< 收到中斷信號 >")
        finally:
            self.cleanup()

    def cleanup(self):
        """清理資源"""
        self.picam2.stop()
        self.picam2.close()
        cv2.destroyAllWindows()

        logger.info("=" * 70)
        logger.info("✓ 程序已結束")
        logger.info(f"  總幀數: {self.frame_count}")
        logger.info(f"  最多人臉數: {self.max_faces}")
        if self.frame_count > 0:
            logger.info(f"  平均 FPS: {self.frame_count / (time.time() - self.start_time):.1f}")
        logger.info(f"  運行時長: {time.time() - self.start_time:.1f}s")
        logger.info("=" * 70)


def main():
    """主程序"""
    detector = RealtimeFaceDetectorYOLOv8ONNX()
    detector.run()


if __name__ == "__main__":
    main()
