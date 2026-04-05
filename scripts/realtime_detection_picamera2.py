#!/usr/bin/env python3
"""
即時人臉檢測 - 改進版 (Picamera2 + Haar Cascade)
支持：Raspberry Pi Camera Module (v1/v2/v3)
特性：
  - 改進的 Haar Cascade 參數（更準確）
  - 正確的色彩格式
  - 支持按 Q 暫停/退出
  - 支持按 S 保存幀
  - 支持按空格暫停/繼續
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


class RealtimeFaceDetectorHaarCascade:
    """使用 Picamera2 + Haar Cascade 的即時人臉檢測應用"""

    def __init__(self, confidence_threshold: float = 0.5):
        """初始化檢測器"""
        logger.info("初始化 Picamera2...")
        self.picam2 = Picamera2()

        # 配置攝像頭參數
        config = self.picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "XBGR8888"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)  # 等待攝像頭穩定

        logger.info("✓ Picamera2 已初始化 (640x480)")

        # 初始化 Haar Cascade 分類器
        logger.info("初始化人臉檢測器...")
        self.cascade_frontal = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        self.cascade_profile = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_profileface.xml'
        )
        logger.info("✓ 人臉檢測器已加載 (正臉 + 側臉)")

        # 跳幀設置：每 3 幀檢測一次（減少 CPU 開銷）
        self.detect_interval = 3
        self.frame_since_detect = 0
        self.last_detections = []

        self.confidence_threshold = confidence_threshold

        # 統計信息
        self.frame_count = 0
        self.max_faces = 0
        self.start_time = time.time()
        self.save_dir = Path("/tmp")
        self.paused = False

    def detect_faces_haar(self, frame_gray):
        """快速人臉檢測（正臉 + 側臉，優化性能）"""
        detections = []

        # 1️⃣ 檢測正臉（優化參數以提高速度）
        frontal_faces = self.cascade_frontal.detectMultiScale(
            frame_gray,
            scaleFactor=1.15,     # 更大 = 更快但精度降低
            minNeighbors=7,       # 更高 = 更少誤檢
            flags=cv2.CASCADE_SCALE_IMAGE,
            minSize=(60, 60),     # 更大最小尺寸 = 避免誤檢
            maxSize=(300, 300)
        )
        for (x, y, w, h) in frontal_faces:
            detections.append({
                'box': (x, y, w, h),
                'confidence': 0.95,
                'type': 'frontal'
            })

        # 2️⃣ 檢測側臉（只在完整檢測幀進行）
        profile_faces = self.cascade_profile.detectMultiScale(
            frame_gray,
            scaleFactor=1.15,
            minNeighbors=7,
            flags=cv2.CASCADE_SCALE_IMAGE,
            minSize=(60, 60),
            maxSize=(300, 300)
        )
        for (x, y, w, h) in profile_faces:
            detections.append({
                'box': (x, y, w, h),
                'confidence': 0.90,
                'type': 'profile'
            })

        # 應用 NMS 去除重複
        detections = self._nms(detections, nms_threshold=0.4)

        return detections

    @staticmethod
    def _nms(detections, nms_threshold=0.3):
        """非最大值抑制 - 去除重複檢測"""
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
            # 保留面積最大的
            current = detections.pop(0)
            keep.append(current)

            # 移除與當前框重疊超過閾值的
            remaining = []
            for det in detections:
                iou = RealtimeFaceDetectorHaarCascade._iou(current['box'], det['box'])
                if iou < nms_threshold:
                    remaining.append(det)
            detections = remaining

        return keep

    @staticmethod
    def _iou(box1, box2):
        """計算兩個邊界框的 IoU（Intersection over Union）"""
        x1, y1, w1, h1 = box1
        x2, y2, w2, h2 = box2

        # 計算交集
        xi1 = max(x1, x2)
        yi1 = max(y1, y2)
        xi2 = min(x1 + w1, x2 + w2)
        yi2 = min(y1 + h1, y2 + h2)

        inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)

        # 計算並集
        box1_area = w1 * h1
        box2_area = w2 * h2
        union_area = box1_area + box2_area - inter_area

        return inter_area / union_area if union_area > 0 else 0

    def draw_detections(self, frame, detections):
        """繪製檢測結果（顯示人臉類型）"""
        result = frame.copy()
        for det in detections:
            x, y, w, h = det['box']
            confidence = det['confidence']
            face_type = det.get('type', 'unknown')

            # 根據類型選擇顏色
            if face_type == 'frontal':
                color = (0, 255, 0)      # 綠色：正臉
            elif face_type == 'profile':
                color = (0, 165, 255)    # 橙色：側臉
            else:
                color = (0, 255, 0)

            # 繪製邊界框
            cv2.rectangle(result, (x, y), (x + w, y + h), color, 2)

            # 繪製標籤
            label = f"{face_type}"
            cv2.putText(result, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return result

    def run(self):
        """運行即時檢測"""
        logger.info("=" * 70)
        logger.info("🎥 即時人臉檢測 (Picamera2 + Haar Cascade - 改進版)")
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
                # 注意：picamera2 的 XBGR8888 需要特殊處理
                frame_bgr = cv2.cvtColor(frame_xbgr, cv2.COLOR_RGBA2BGR)

                if not self.paused:
                    self.frame_count += 1
                    self.frame_since_detect += 1

                    # 每 N 幀進行一次檢測（跳幀以提高 FPS）
                    if self.frame_since_detect >= self.detect_interval:
                        frame_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
                        self.last_detections = self.detect_faces_haar(frame_gray)
                        self.frame_since_detect = 0
                        self.max_faces = max(self.max_faces, len(self.last_detections))

                    # 使用最近的檢測結果
                    detections = self.last_detections

                    # 繪製檢測結果
                    result = self.draw_detections(frame_bgr, detections)
                else:
                    result = frame_bgr
                    detections = []

                # 添加統計信息
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                status = " PAUSED" if self.paused else " RUNNING"
                info_text = f"Frame: {self.frame_count:4d} | FPS: {fps:5.1f} | Faces: {len(detections)} | {status}"
                cv2.putText(result, info_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # 添加控制提示
                hint_text = "Press [Q]uit [S]ave [Space]Pause"
                cv2.putText(result, hint_text, (10, 460),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                # 顯示幀
                cv2.imshow("Face Detection (Picamera2 + Haar Cascade)", result)

                # 按鍵處理（重要：waitKey 要等待按鍵）
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q'):
                    logger.info("\n< 退出中... >")
                    break
                elif key == ord('s') or key == ord('S'):
                    filename = self.save_dir / f"face_detection_{self.frame_count:05d}.jpg"
                    cv2.imwrite(str(filename), result)
                    logger.info(f"✓ 幀已保存：{filename}")
                elif key == 32:  # 空格鍵 (ASCII 32)
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
    detector = RealtimeFaceDetectorHaarCascade()
    detector.run()


if __name__ == "__main__":
    main()
