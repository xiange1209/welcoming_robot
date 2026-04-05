#!/usr/bin/env python3
"""
即時人臉檢測 - YOLOv8 Nano 版本
支持：Raspberry Pi 4 + Picamera2
特性：
  - 更高精度（YOLOv8）
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

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

# 嘗試導入 YOLOv8
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class RealtimeFaceDetectorYOLOv8:
    """使用 Picamera2 + YOLOv8 Nano 的即時人臉檢測"""

    def __init__(self, model_path: str = 'yolov8n-face.pt'):
        """初始化檢測器"""
        if not YOLO_AVAILABLE:
            logger.error("✗ ultralytics 未安裝。請執行：pip install ultralytics")
            sys.exit(1)

        logger.info("初始化 Picamera2...")
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "XBGR8888"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)
        logger.info("✓ Picamera2 已初始化 (640x480)")

        # 初始化 YOLOv8 模型
        logger.info(f"加載 YOLOv8 模型: {model_path}")
        try:
            self.model = YOLO(model_path)
            # 設置為 CPU 推理（樹莫派沒有 GPU）
            self.model.to('cpu')
            logger.info("✓ YOLOv8 Nano 已加載")
        except Exception as e:
            logger.error(f"✗ 模型加載失敗: {e}")
            logger.info("  嘗試自動下載模型...")
            self.model = YOLO('yolov8n-face.pt')  # 自動下載
            logger.info("✓ 模型已下載並加載")

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

    def detect_faces_yolo(self, frame):
        """使用 YOLOv8 檢測人臉"""
        # 運行推理（注意：YOLOv8 輸入不需要特殊預處理，模型內部處理）
        results = self.model(frame, verbose=False, conf=0.5)

        detections = []
        if results and len(results) > 0:
            result = results[0]

            # 提取邊界框
            if result.boxes is not None:
                for box in result.boxes:
                    # 獲取座標
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    confidence = float(box.conf[0])

                    # 轉換為 (x, y, w, h) 格式
                    x = x1
                    y = y1
                    w = x2 - x1
                    h = y2 - y1

                    detections.append({
                        'box': (x, y, w, h),
                        'confidence': confidence,
                        'type': 'face'
                    })

        return detections

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
        logger.info("🎥 即時人臉檢測 (Picamera2 + YOLOv8 Nano)")
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
                        self.last_detections = self.detect_faces_yolo(frame_bgr)
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
                cv2.imshow("Face Detection (YOLOv8 Nano)", result)

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
    detector = RealtimeFaceDetectorYOLOv8()
    detector.run()


if __name__ == "__main__":
    main()
