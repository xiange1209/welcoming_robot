#!/usr/bin/env python3
"""
即時人臉檢測 - 使用樹莓派攝像頭/USB 攝像頭
按 Q 退出，按空格保存幀
"""

import cv2
import numpy as np
import sys
import time
from pathlib import Path

# 導入自定義模組
sys.path.insert(0, str(Path(__file__).parent.parent))
from vision_ai.inference_engine import InferenceEngine
from vision_ai.face_detector import FaceDetector

# 注意：目前只測試推理框架，還沒有模型文件
# 所以會有 "Model not found" 錯誤，這是預期的

class RealtimeFaceDetector:
    """即時人臉檢測應用"""

    def __init__(self, camera_id: int = 0, config_path: str = "config/inference_config.yaml"):
        """
        初始化即時檢測器
        Args:
            camera_id: 攝像頭 ID (0=主攝像頭, 1=USB攝像頭等)
            config_path: 配置檔路徑
        """
        self.camera_id = camera_id
        self.config_path = config_path

        # 初始化攝像頭
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            print(f"✗ 無法打開攝像頭 {camera_id}")
            sys.exit(1)

        # 設置攝像頭參數
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 10)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 單幀緩衝，降低延遲

        print(f"✓ 攝像頭已打開 (ID: {camera_id})")

        # 初始化人臉檢測器
        try:
            self.detector = FaceDetector(config_path)
            print("✓ 人臉檢測器已加載")
            self.detector_ready = True
        except Exception as e:
            print(f"⚠ 人臉檢測器初始化失敗: {e}")
            print("  （模型文件可能未下載，但可繼續測試框架）")
            self.detector_ready = False

        # 統計信息
        self.frame_count = 0
        self.detection_count = 0
        self.start_time = time.time()

    def run(self):
        """運行即時檢測"""
        print("\n" + "=" * 50)
        print("🎥 即時人臉檢測")
        print("=" * 50)
        print("按鍵控制：")
        print("  Q - 退出")
        print("  SPACE - 保存當前幀")
        print("=" * 50 + "\n")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("✗ 無法讀取幀")
                break

            self.frame_count += 1

            # 進行人臉檢測
            if self.detector_ready:
                try:
                    detections = self.detector.detect(frame)
                    self.detection_count += len(detections)

                    # 繪製檢測結果
                    annotated_frame = self.detector.draw_detections(frame, detections)
                except Exception as e:
                    print(f"✗ 檢測失敗: {e}")
                    annotated_frame = frame
            else:
                annotated_frame = frame
                detections = []

            # 添加統計信息
            elapsed = time.time() - self.start_time
            fps = self.frame_count / elapsed if elapsed > 0 else 0

            stats_text = f"Frame: {self.frame_count} | FPS: {fps:.1f} | Faces: {len(detections)}"
            cv2.putText(annotated_frame, stats_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 顯示幀
            cv2.imshow("Real-time Face Detection", annotated_frame)

            # 按鍵處理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n✓ 退出即時檢測")
                break
            elif key == ord(' '):
                # 保存幀
                filename = f"frame_{self.frame_count}.png"
                cv2.imwrite(filename, annotated_frame)
                print(f"✓ 幀已保存: {filename}")

        # 清理
        self.cap.release()
        cv2.destroyAllWindows()

        # 輸出統計
        print("\n" + "=" * 50)
        print("📊 統計信息")
        print("=" * 50)
        print(f"總幀數: {self.frame_count}")
        print(f"檢測到的人臉: {self.detection_count}")
        print(f"平均 FPS: {self.frame_count / (time.time() - self.start_time):.1f}")
        print(f"運行時長: {time.time() - self.start_time:.1f}s")


def main():
    # 嘗試打開攝像頭 0 (樹莓派預設攝像頭或 USB 攝像頭)
    detector = RealtimeFaceDetector(camera_id=0)
    detector.run()


if __name__ == "__main__":
    main()
