#!/usr/bin/env python3
"""
即時人臉檢測 - InsightFace 版本
支持：Raspberry Pi 4 + Picamera2
特性：
  - InsightFace RetinaFace ONNX 模型（內置，~6MB）
  - 高準確度人臉檢測 + 側臉支持
  - 同時生成人臉 embedding（後續用於識別）
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

try:
    from insightface.app import FaceAnalysis
    INSIGHTFACE_AVAILABLE = True
except ImportError:
    INSIGHTFACE_AVAILABLE = False


class RealtimeFaceDetectorInsightFace:
    """使用 Picamera2 + InsightFace 的即時人臉檢測"""

    def __init__(self):
        """初始化檢測器"""
        if not INSIGHTFACE_AVAILABLE:
            logger.error("✗ InsightFace 未安裝。請執行：pip install insightface")
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

        # 初始化 InsightFace
        logger.info("初始化 InsightFace...")
        try:
            # 使用 'buffalo_sc' 模型（輕量，樹莓派友善）
            # 可選：'buffalo_sc' (小), 'buffalo_m' (中), 'buffalo_l' (大)
            self.face_app = FaceAnalysis(
                name='buffalo_sc',
                root='./insightface_models',  # 模型保存位置
                providers=['CPUExecutionProvider']  # 樹莫派 CPU
            )
            self.face_app.prepare(ctx_id=-1, det_size=(512, 512))
            logger.info("✓ InsightFace 已初始化 (RetinaFace + ArcFace)")
        except Exception as e:
            logger.error(f"✗ InsightFace 初始化失敗: {e}")
            sys.exit(1)

        # 跳幀設置
        self.detect_interval = 3  # 每 3 幀檢測一次（樹莫派優化，降低 CPU 壓力）
        self.frame_since_detect = 0
        self.last_detections = []

        # 統計信息
        self.frame_count = 0
        self.max_faces = 0
        self.start_time = time.time()
        self.save_dir = Path("/tmp")
        self.paused = False

    def detect_faces_insightface(self, frame):
        """使用 InsightFace 檢測人臉"""
        detections = []

        try:
            # 執行人臉檢測和識別
            faces = self.face_app.get(frame)

            for face in faces:
                # 獲取邊界框
                bbox = face.bbox.astype(np.int32)  # [x1, y1, x2, y2]
                x1, y1, x2, y2 = bbox

                # 轉換為 (x, y, w, h) 格式
                x = x1
                y = y1
                w = x2 - x1
                h = y2 - y1

                # 獲取置信度（face.det_score）
                confidence = float(face.det_score)

                # 獲取人臉 embedding（128D，用於識別）
                embedding = face.embedding  # (512,) 維向量

                detections.append({
                    'box': (x, y, w, h),
                    'confidence': confidence,
                    'embedding': embedding,
                    'type': 'face'
                })

        except Exception as e:
            logger.error(f"檢測失敗: {e}")

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
        logger.info("🎥 即時人臉檢測 (Picamera2 + InsightFace)")
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
                        self.last_detections = self.detect_faces_insightface(frame_bgr)
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
                status = " PAUSED" if self.paused else " RUNNING"
                info_text = f"Frame: {self.frame_count:4d} | FPS: {fps:5.1f} | Faces: {len(detections)} | {status}"
                cv2.putText(result, info_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # 控制提示
                hint_text = "Press [Q]uit [S]ave [Space]Pause"
                cv2.putText(result, hint_text, (10, 460),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                # 嘗試顯示（如果環境支援）
                try:
                    cv2.imshow("Face Detection (InsightFace)", result)
                    key = cv2.waitKey(1) & 0xFF
                except cv2.error:
                    # 不支援 GUI，模擬按鍵檢查用其他方法
                    key = -1
                    # 每 30 幀輸出一次日誌
                    if self.frame_count % 30 == 0:
                        logger.info(f"[Frame {self.frame_count:4d}] Detected {len(detections)} face(s)")

                # 按鍵處理（如果支援 GUI）
                if key == ord('q') or key == ord('Q'):
                    logger.info("\n< 退出中... >")
                    break
                elif key == ord('s') or key == ord('S'):
                    filename = self.save_dir / f"face_detection_{self.frame_count:05d}.jpg"
                    cv2.imwrite(str(filename), result)
                    logger.info(f"✓ 幀已保存：{filename}")

        except KeyboardInterrupt:
            logger.info("\n< 收到中斷信號 >")
        finally:
            self.cleanup()

    def cleanup(self):
        """清理資源"""
        self.picam2.stop()
        self.picam2.close()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            # 沒有 GUI 支援，忽略錯誤
            pass

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
    detector = RealtimeFaceDetectorInsightFace()
    detector.run()


if __name__ == "__main__":
    main()
