#!/usr/bin/env python3
"""
簡化版模型下載 - 使用可靠的開源模型源
適合樹莓派 INT8 推理
"""

import os
from pathlib import Path

class ModelGuide:
    """模型來源指南"""

    @staticmethod
    def print_guide():
        print("\n" + "=" * 70)
        print("📦 樹莓派人臉識別模型獲取指南")
        print("=" * 70)

        print("\n【推薦方案】使用 MediaPipe 或 OpenCV 內建模型")
        print("-" * 70)

        print("\n🔹 方案 A: MediaPipe Face Detection (推薦 Phase 2)")
        print("   特點：")
        print("   - ✅ 已集成在 Raspberry Pi 中")
        print("   - ✅ 不需下載，pip 即可安裝")
        print("   - ✅ 預訓練完成，可直接使用")
        print("   - ✅ 性能優秀 (~50-100ms)")
        print("   ")
        print("   安裝：")
        print("   $ pip install mediapipe")
        print("   ")
        print("   代碼示例：")
        print("   from mediapipe.tasks import python")
        print("   from mediapipe.tasks.python import vision")
        print("   ")
        print("   detector = vision.FaceDetector.create_from_model_path(model_path)")

        print("\n" + "-" * 70)
        print("\n🔹 方案 B: OpenCV DNN 內建模型")
        print("   特點：")
        print("   - ✅ OpenCV 已安裝")
        print("   - ✅ 無需額外下載")
        print("   - ✅ 支持 Caffe、TensorFlow、ONNX")
        print("   ")
        print("   可用模型（OpenCV 已包含）：")
        print("   - deploy.prototxt (Caffe)")
        print("   - res10_300x300_ssd_iter_140000.caffemodel")

        print("\n" + "-" * 70)
        print("\n🔹 方案 C: 手動下載 ONNX 模型")
        print("   ")
        print("   RetinaFace INT8 替代來源：")
        print("   1. Ultralytics YOLOv8: https://github.com/ultralytics/yolov8")
        print("   2. MediaPipe Solutions: https://developers.google.com/mediapipe")
        print("   3. ONNX Model Zoo: https://github.com/onnx/models")
        print("   ")
        print("   ArcFace INT8 替代來源：")
        print("   1. InsightFace: https://github.com/deepinsight/insightface")
        print("   2. PyPI: pip install insightface")

        print("\n" + "=" * 70)
        print("【推薦選擇】")
        print("=" * 70)
        print("\n✅ Phase 2 (當前階段)：")
        print("   使用 MediaPipe Face Detection")
        print("   → 簡單、快速、無需下載模型")
        print("   ")
        print("✅ Phase 3：")
        print("   集成 ArcFace 人臉識別")
        print("   → 從 InsightFace 下載或 pip 安裝")
        print("\n" + "=" * 70)


def install_mediapipe():
    """安裝 MediaPipe"""
    print("\n🚀 安裝 MediaPipe...")
    os.system("pip install --no-cache-dir mediapipe -q")
    print("✓ MediaPipe 安裝完成")


def download_opencv_models():
    """下載 OpenCV DNN 模型"""
    print("\n📥 下載 OpenCV 預訓練模型...")

    models_dir = Path("models")
    models_dir.mkdir(exist_ok=True)

    # OpenCV 模型 URL
    models = {
        'deploy.prototxt': 'https://raw.githubusercontent.com/opencv/opencv_3rdparty/dnn_samples_face_detector_20170830/opencv_face_detector.prototxt',
        'res10_300x300_ssd_iter_140000.caffemodel': 'https://raw.githubusercontent.com/opencv/opencv_3rdparty/dnn_samples_face_detector_20170830/opencv_face_detector_uint8.caffemodel'
    }

    import urllib.request
    for model_name, url in models.items():
        filepath = models_dir / model_name
        if filepath.exists():
            print(f"✓ {model_name} 已存在")
            continue

        print(f"📥 下載 {model_name}...")
        try:
            urllib.request.urlretrieve(url, str(filepath))
            print(f"✓ {model_name} 下載完成")
        except Exception as e:
            print(f"✗ 下載失敗: {e}")


def main():
    ModelGuide.print_guide()

    print("\n選擇操作:")
    print("1. 安裝 MediaPipe（推薦 Phase 2）")
    print("2. 下載 OpenCV DNN 模型")
    print("3. 僅顯示指南")

    choice = input("\n輸入選擇 (1-3): ").strip()

    if choice == '1':
        install_mediapipe()
    elif choice == '2':
        download_opencv_models()
    else:
        print("\n✓ 已顯示指南")


if __name__ == "__main__":
    main()
