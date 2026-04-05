#!/usr/bin/env python3
"""
樹莓派即時人臉檢測 - 使用 OpenCV DNN（無需額外模型下載）
適用於：樹莓派 Camera Module 或 USB 攝像頭
輸出：人臉檢測框 + 保存幀到 /tmp

使用方法：
  python3 scripts/realtime_detection_opencv_dnn.py

操作：
  S - 保存當前幀
  Q - 退出
"""

import cv2
import numpy as np
from pathlib import Path
import logging
import sys

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)


class OpenCVDNNFaceDetector:
    """使用 OpenCV DNN 的人臉檢測器（無需下載模型）"""

    # OpenCV 內建的 Caffe 模型路徑
    CAFFE_PATHS = [
        "/usr/share/opencv4/opencv_face_detector.pbtxt",
        "/usr/local/share/opencv4/opencv_face_detector.pbtxt",
    ]
    MODEL_PATHS = [
        "/usr/share/opencv4/opencv_face_detector_uint8.pb",
        "/usr/local/share/opencv4/opencv_face_detector_uint8.pb",
    ]

    def __init__(self, confidence_threshold: float = 0.5):
        self.confidence_threshold = confidence_threshold
        self.net = None
        self.use_cascade = False
        self.cascade = None
        self._load_model()

    def _load_model(self):
        """加載 OpenCV DNN 模型"""
        # 嘗試加載 Caffe 模型
        for proto_path, model_path in zip(self.CAFFE_PATHS, self.MODEL_PATHS):
            if Path(proto_path).exists() and Path(model_path).exists():
                try:
                    logger.info(f"✓ 加載 Caffe DNN 模型")
                    self.net = cv2.dnn.readNetFromCaffe(proto_path, model_path)
                    logger.info(f"  Proto: {proto_path}")
                    logger.info(f"  Model: {model_path}")
                    return
                except Exception as e:
                    logger.warning(f"Caffe 加載失敗：{e}")

        # 備選：使用 Haar Cascade（更快，但精度較低）
        logger.info("✓ 使用 Haar Cascade 分類器（快速模式）")
        try:
            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            self.cascade = cv2.CascadeClassifier(cascade_path)
            self.use_cascade = True
            logger.info(f"  Cascade: {cascade_path}")
        except Exception as e:
            logger.error(f"Cascade 加載失敗：{e}")

    def detect(self, frame: np.ndarray) -> list:
        """
        檢測人臉
        Args:
            frame: 輸入圖像 (BGR)
        Returns:
            檢測結果列表 [{'box': [x, y, w, h], 'confidence': ...}]
        """
        h, w = frame.shape[:2]
        detections = []

        if self.net is not None:
            # 使用 Caffe DNN
            blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), [104, 117, 123], False, False)
            self.net.setInput(blob)
            detections_raw = self.net.forward()

            for i in range(min(detections_raw.shape[2], 10)):  # 限制最多 10 個檢測
                confidence = float(detections_raw[0, 0, i, 2])
                if confidence < self.confidence_threshold:
                    continue

                x1 = int(detections_raw[0, 0, i, 3] * w)
                y1 = int(detections_raw[0, 0, i, 4] * h)
                x2 = int(detections_raw[0, 0, i, 5] * w)
                y2 = int(detections_raw[0, 0, i, 6] * h)

                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w, x2), min(h, y2)

                if x2 > x1 and y2 > y1:
                    detections.append({
                        'box': [x1, y1, x2 - x1, y2 - y1],
                        'confidence': confidence
                    })
        elif self.use_cascade and self.cascade is not None:
            # 使用 Haar Cascade
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.cascade.detectMultiScale(gray, 1.1, 4, minSize=(20, 20))

            for (x, y, w, h) in faces:
                detections.append({
                    'box': [x, y, w, h],
                    'confidence': 0.95
                })

        return detections

    @staticmethod
    def draw_detections(frame: np.ndarray, detections: list) -> np.ndarray:
        """在圖像上繪製檢測結果"""
        result = frame.copy()
        for det in detections:
            x, y, w, h = det['box']
            confidence = det['confidence']

            # 繪製邊界框（綠色）
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 繪製置信度標籤
            label = f"{confidence:.2f}"
            cv2.putText(result, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return result


def main():
    """主程序"""
    logger.info("=" * 60)
    logger.info("🎥 樹莓派即時人臉檢測（OpenCV DNN）")
    logger.info("=" * 60)

    # 初始化檢測器
    detector = OpenCVDNNFaceDetector(confidence_threshold=0.5)

    # 打開攝像頭
    camera_id = 0
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        logger.error(f"✗ 無法打開攝像頭 (ID: {camera_id})")
        logger.info("\n啟用樹莫派攝像頭步驟：")
        logger.info("  1. sudo raspi-config")
        logger.info("  2. 選擇 Interfacing Options → Camera → Enable")
        logger.info("  3. sudo reboot")
        return

    # 設定攝像頭參數
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 10)

    logger.info(f"✓ 攝像頭已打開 (ID: {camera_id})")
    logger.info(f"  解析度: 640x480")
    logger.info(f"  幀率: 10 FPS")

    logger.info("\n控制：")
    logger.info("  S - 保存當前幀到 /tmp/face_detection_*.jpg")
    logger.info("  Q - 退出")
    logger.info("=" * 60)

    frame_count = 0
    face_count = 0
    save_dir = Path("/tmp")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                logger.error("讀取幀失敗")
                break

            frame_count += 1

            # 檢測人臉
            detections = detector.detect(frame)
            face_count = max(face_count, len(detections))

            # 添加信息文字
            h, w = frame.shape[:2]
            info_text = f"Frame: {frame_count} | Faces: {len(detections)} | {w}x{h}"
            cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # 繪製檢測結果
            result = detector.draw_detections(frame, detections)

            # 嘗試顯示（如果環境支援）
            display_available = False
            try:
                cv2.imshow("Face Detection", result)
                display_available = True
                key = cv2.waitKey(1) & 0xFF
            except cv2.error as e:
                # 沒有 GUI 支援，但仍然可以保存和處理
                key = -1
                # 每 30 幀輸出一次日誌
                if frame_count % 30 == 0:
                    logger.info(f"[Frame {frame_count}] Detected {len(detections)} face(s)")

            # 按鍵處理
            if key == ord('q') or key == ord('Q'):
                logger.info("< 退出 >")
                break
            elif key == ord('s') or key == ord('S'):
                filename = save_dir / f"face_detection_{frame_count:05d}.jpg"
                cv2.imwrite(str(filename), result)
                logger.info(f"✓ 幀已保存：{filename}")
            elif key != -1 and key != 255:  # 有按鍵但不是無操作
                logger.info(f"  按下: {chr(key)} (ASCII: {key})")

    except KeyboardInterrupt:
        logger.info(f"\n< 收到中斷信號 >")
    finally:
        cap.release()
        try:
            if display_available:
                cv2.destroyAllWindows()
        except:
            pass

        logger.info("=" * 60)
        logger.info(f"✓ 程序已結束")
        logger.info(f"  總幀數: {frame_count}")
        logger.info(f"  最多人臉數: {face_count}")
        logger.info(f"  啟用 X11 轉發查看即時影像：")
        logger.info(f"  ssh -X xiange@192.168.1.125")
        logger.info("=" * 60)


if __name__ == "__main__":
    main()
