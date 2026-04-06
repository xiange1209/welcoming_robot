#!/usr/bin/env python3
"""
測試 VIP 識別是否工作
用已註冊的 VIP 照片進行實測
"""

import sys
import cv2
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from vision_ai.face_recognizer import FaceRecognizer
from insightface.app import FaceAnalysis

def test_vip_recognition(test_image_path: str):
    """
    測試識別指定照片中的人物
    """
    print(f"\n🧪 測試 VIP 識別: {test_image_path}")
    print("="*50)

    # 初始化識別器
    recognizer = FaceRecognizer(
        db_path="database/ai_bank_robot.db",
        vip_threshold=0.65,
        blacklist_threshold=0.60
    )
    print(f"✓ 已加載 {len(recognizer.vip_cache)} 個 VIP")

    # 初始化人臉檢測
    face_app = FaceAnalysis(
        name='buffalo_sc',
        root='./insightface_models',
        providers=['CPUExecutionProvider']
    )
    face_app.prepare(ctx_id=-1, det_size=(512, 512))

    # 讀取測試圖像
    image = cv2.imread(test_image_path)
    if image is None:
        print(f"✗ 無法讀取圖像: {test_image_path}")
        return False

    # 檢測人臉
    print(f"檢測人臉...")
    faces = face_app.get(image)

    if len(faces) == 0:
        print("✗ 圖像中未檢測到人臉")
        return False

    print(f"✓ 檢測到 {len(faces)} 張人臉")

    # 逐個識別
    for i, face in enumerate(faces):
        embedding = face.embedding.astype(np.float32)
        det_conf = face.det_score

        print(f"\n👤 人臉 #{i+1} (檢測信心度: {det_conf:.4f})")

        # 進行識別
        rec_result = recognizer.recognize_face(embedding)

        if rec_result['type'] == 'vip':
            print(f"  ✅ VIP: {rec_result['name']}")
            print(f"     信心度: {rec_result['confidence']:.4f}")
        elif rec_result['type'] == 'blacklist':
            print(f"  ⛔ 黑名單: {rec_result['name']}")
            print(f"     信心度: {rec_result['confidence']:.4f}")
        else:
            print(f"  🟢 訪客 (不認識)")
            print(f"     最相似 VIP: {rec_result.get('closest_vip_name', 'N/A')} ({rec_result.get('closest_vip_conf', 0):.4f})")

        return True

if __name__ == "__main__":
    # 測試照片路徑（使用第一個命令行參數或預設）
    test_image = sys.argv[1] if len(sys.argv) > 1 else "test_photos/lee.jpg"

    if Path(test_image).exists():
        success = test_vip_recognition(test_image)
        if success:
            print("\n✅ 測試完成")
        else:
            print("\n❌ 測試失敗")
    else:
        print(f"❌ 找不到測試圖像: {test_image}")
        print("\n使用方式:")
        print(f"  python3 {sys.argv[0]} test_photos/lee.jpg")
