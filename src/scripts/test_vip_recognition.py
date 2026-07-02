#!/usr/bin/env python3
"""單張照片人臉辨識驗證工具。

用途：
1. 驗證單張照片是否能被目前資料庫中的 VIP / 黑名單正確辨識。
2. 顯示前幾名相似候選，方便調整閾值或判斷誤判來源。
3. 作為 Phase 2b 端對端驗證前的快速檢查工具。
"""

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    cv2 = None
    CV2_AVAILABLE = False

try:
    from insightface.app import FaceAnalysis
    INSIGHTFACE_AVAILABLE = True
except ImportError:
    FaceAnalysis = None
    INSIGHTFACE_AVAILABLE = False


_SRC_DIR = Path(__file__).parent.parent
_PROJECT_ROOT = _SRC_DIR.parent
sys.path.insert(0, str(_SRC_DIR))

from ai_vision.face_recognizer import FaceRecognizer


def _build_face_app():
    if not INSIGHTFACE_AVAILABLE:
        raise RuntimeError("InsightFace 未安裝，請先安裝 insightface")

    face_app = FaceAnalysis(
        name="buffalo_sc",
        root=str(_PROJECT_ROOT / "insightface_models"),
        providers=["CPUExecutionProvider"],
    )
    face_app.prepare(ctx_id=-1, det_size=(512, 512))
    return face_app


def _load_image(image_path: Path):
    if not CV2_AVAILABLE:
        raise RuntimeError("OpenCV 未安裝，請先安裝 opencv-python")

    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError(f"無法讀取圖像：{image_path}")
    return image


def _rank_matches(recognizer: FaceRecognizer, embedding: np.ndarray, cache: dict, top_k: int):
    ranked = []
    for name, (known_embedding, gender) in cache.items():
        similarity = recognizer.cosine_similarity(embedding, known_embedding)
        ranked.append((name, gender, similarity))

    ranked.sort(key=lambda item: item[2], reverse=True)
    return ranked[:top_k]


def _print_ranked(title: str, ranked: list[tuple[str, str, float]]):
    print(f"  {title} 候選：")
    if not ranked:
        print("    - 無資料")
        return

    for idx, (name, gender, similarity) in enumerate(ranked, start=1):
        print(f"    {idx}. {name} ({gender}) -> {similarity:.4f}")


def _expectation_matches(result: dict, expect_type: str | None, expect_name: str | None) -> bool:
    if expect_type and result["person_type"] != expect_type:
        return False
    if expect_name and result.get("name") != expect_name:
        return False
    return True


def test_vip_recognition(
    image_path: Path,
    db_path: Path,
    vip_threshold: float,
    blacklist_threshold: float,
    expect_type: str | None,
    expect_name: str | None,
    top_k: int,
) -> bool:
    print(f"\n🧪 測試照片：{image_path}")
    print(f"📁 資料庫：{db_path}")
    print("=" * 60)

    recognizer = FaceRecognizer(
        db_path=str(db_path),
        vip_threshold=vip_threshold,
        blacklist_threshold=blacklist_threshold,
    )
    print(
        f"✓ 已載入資料庫：VIP {len(recognizer.vip_cache)} 人 / 黑名單 {len(recognizer.blacklist_cache)} 人"
    )
    print(f"✓ 閾值：VIP {vip_threshold:.2f} / 黑名單 {blacklist_threshold:.2f}")

    face_app = _build_face_app()
    image = _load_image(image_path)

    print("檢測人臉中...")
    faces = face_app.get(image)
    if len(faces) == 0:
        print("✗ 圖像中未檢測到人臉")
        return False

    print(f"✓ 檢測到 {len(faces)} 張人臉")
    matched_expectation = False

    for index, face in enumerate(faces, start=1):
        bbox = [int(value) for value in face.bbox.tolist()]
        embedding = face.embedding.astype(np.float32)
        detection_confidence = float(face.det_score)
        result = recognizer.recognize_face(embedding)

        print()
        print(f"👤 人臉 #{index}")
        print(f"  bbox: {bbox}")
        print(f"  檢測信心度: {detection_confidence:.4f}")
        print(f"  辨識結果: {result['person_type']}")
        print(f"  名稱: {result.get('name') or '-'}")
        print(f"  性別: {result.get('gender') or '-'}")
        print(f"  信心度: {result['confidence']:.4f}")

        if result.get("vip_level"):
            print(f"  VIP 等級: {result['vip_level']}")
        if result.get("risk_level"):
            print(f"  風險等級: {result['risk_level']}")

        _print_ranked("VIP", _rank_matches(recognizer, embedding, recognizer.vip_cache, top_k))
        _print_ranked("黑名單", _rank_matches(recognizer, embedding, recognizer.blacklist_cache, top_k))

        if _expectation_matches(result, expect_type, expect_name):
            matched_expectation = True

    if expect_type or expect_name:
        print()
        if matched_expectation:
            print("✅ 與預期相符")
        else:
            print("❌ 與預期不符")
        return matched_expectation

    print()
    print("✅ 測試完成")
    return True


def build_parser():
    parser = argparse.ArgumentParser(description="單張照片人臉辨識驗證工具")
    parser.add_argument("image", help="待測照片路徑")
    parser.add_argument(
        "--db-path",
        default=str(_PROJECT_ROOT / "database" / "ai_bank_robot.db"),
        help="SQLite 資料庫路徑",
    )
    parser.add_argument("--vip-threshold", type=float, default=0.25, help="VIP 閾值")
    parser.add_argument("--blacklist-threshold", type=float, default=0.20, help="黑名單閾值")
    parser.add_argument(
        "--expect-type",
        choices=["vip", "blacklist", "visitor"],
        help="預期辨識類型，用於自動驗證",
    )
    parser.add_argument("--expect-name", help="預期人名，用於自動驗證")
    parser.add_argument("--top-k", type=int, default=3, help="顯示前幾名候選")
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    image_path = Path(args.image)
    if not image_path.exists():
        print(f"❌ 找不到測試圖像：{image_path}")
        raise SystemExit(1)

    db_path = Path(args.db_path)
    if not db_path.exists():
        print(f"❌ 找不到資料庫：{db_path}")
        raise SystemExit(1)

    success = test_vip_recognition(
        image_path=image_path,
        db_path=db_path,
        vip_threshold=args.vip_threshold,
        blacklist_threshold=args.blacklist_threshold,
        expect_type=args.expect_type,
        expect_name=args.expect_name,
        top_k=max(1, args.top_k),
    )
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
