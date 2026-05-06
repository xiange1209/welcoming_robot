#!/usr/bin/env python3
"""
VIP 資料庫管理工具
初始化、添加 VIP、添加黑名單、查詢等
"""

import sys
import os
import time
import argparse
import numpy as np
import cv2
from pathlib import Path

# 添加 src/ 到路徑
_SRC_DIR = Path(__file__).parent.parent
_PROJECT_ROOT = _SRC_DIR.parent
sys.path.insert(0, str(_SRC_DIR))

DB_PATH = str(_PROJECT_ROOT / "database" / "ai_bank_robot.db")

from database.schema import DatabaseSchema
from ai_vision.face_recognizer import FaceRecognizer

try:
    import onnxruntime as _ort
    _ort.set_default_logger_severity(3)
except Exception:
    pass

try:
    from insightface.app import FaceAnalysis
    INSIGHTFACE_AVAILABLE = True
except ImportError:
    INSIGHTFACE_AVAILABLE = False


def init_database():
    """初始化數據庫"""
    print("初始化數據庫...")
    schema = DatabaseSchema(DB_PATH)
    schema.initialize()
    print("✓ 數據庫已初始化")


def extract_embedding_from_image(image_path: str) -> np.ndarray:
    """
    從圖像提取人臉 embedding

    Args:
        image_path: 圖像文件路徑

    Returns:
        512D embedding 向量
    """
    if not INSIGHTFACE_AVAILABLE:
        print("✗ InsightFace 未安裝。請執行：pip install insightface")
        return None

    if not Path(image_path).exists():
        print(f"✗ 圖像文件不存在：{image_path}")
        return None

    try:
        # 讀取圖像
        image = cv2.imread(image_path)
        if image is None:
            print(f"✗ 無法讀取圖像：{image_path}")
            return None

        print(f"  讀取圖像：{image_path} ({image.shape})")

        # 初始化 InsightFace
        print("  初始化 InsightFace...")
        face_app = FaceAnalysis(
            name='buffalo_sc',
            root='./insightface_models',
            providers=['CPUExecutionProvider']
        )
        face_app.prepare(ctx_id=-1, det_size=(512, 512))

        # 檢測人臉
        print("  檢測人臉...")
        faces = face_app.get(image)

        if len(faces) == 0:
            print("✗ 圖像中未檢測到人臉")
            return None

        if len(faces) > 1:
            print(f"⚠ 檢測到 {len(faces)} 張人臉，使用第一張")

        # 提取第一個人臉的 embedding
        embedding = faces[0].embedding.astype(np.float32)
        confidence = faces[0].det_score

        print(f"✓ 成功提取 embedding (檢測信心度: {confidence:.4f})")
        return embedding

    except Exception as e:
        print(f"✗ 提取 embedding 失敗: {e}")
        return None


def capture_face_embeddings_from_camera(n_samples: int = 5,
                                        window_title: str = "Face Registration") -> list:
    """
    開啟攝影機，讓使用者按空白鍵拍攝 n_samples 張臉，
    回傳已 L2 正規化的 embedding 列表。
    按 Q 或 ESC 取消，回傳空列表。
    """
    if not os.environ.get("DISPLAY"):
        os.environ["DISPLAY"] = ":1"

    if not INSIGHTFACE_AVAILABLE:
        print("✗ InsightFace 未安裝")
        return []

    print("  初始化 InsightFace...")
    face_app = FaceAnalysis(name='buffalo_sc', root='./insightface_models',
                            providers=['CPUExecutionProvider'])
    face_app.prepare(ctx_id=-1, det_size=(512, 512))

    # 嘗試 Picamera2，失敗則用 USB
    picam2 = None
    cap = None
    try:
        sys.path.insert(0, '/usr/local/lib/python3.12/site-packages')
        from picamera2 import Picamera2
        picam2 = Picamera2()
        cfg = picam2.create_preview_configuration(main={"size": (640, 480), "format": "XBGR8888"})
        picam2.configure(cfg)
        picam2.start()
        time.sleep(1)
        print("  ✓ 攝影機已啟動 (Picamera2)")
    except Exception:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("✗ 無法開啟攝影機")
            return []
        print("  ✓ 攝影機已啟動 (USB)")

    embeddings = []
    print(f"\n  [空白鍵] 拍攝樣本 | [Q/ESC] 取消 | 目標 {n_samples} 張\n")

    def _read_frame():
        if picam2:
            raw = picam2.capture_array()
            return cv2.cvtColor(raw, cv2.COLOR_RGBA2BGR)
        else:
            ret, f = cap.read()
            return f if ret else None

    # 預熱：先顯示純攝影機畫面讓視窗渲染出來，再開始 InsightFace
    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_title, 640, 480)
    print("  攝影機預熱中...")
    for _ in range(20):
        f = _read_frame()
        if f is not None:
            cv2.putText(f, "Warming up... (InsightFace loading)",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
            cv2.imshow(window_title, f)
        cv2.waitKey(30)
    print("  ✓ 預熱完成，開始人臉偵測")

    try:
        while len(embeddings) < n_samples:
            frame = _read_frame()
            if frame is None:
                break

            faces = face_app.get(frame)
            display = frame.copy()

            for face in faces:
                x1, y1, x2, y2 = face.bbox.astype(int)
                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(display, f"{face.det_score:.2f}", (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            bar_clr = (0, 255, 255) if faces else (100, 100, 200)
            cv2.putText(display,
                        f"Samples {len(embeddings)}/{n_samples}  "
                        f"Faces:{len(faces)}  [SPACE]=Capture [Q]=Cancel",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, bar_clr, 2)

            cv2.imshow(window_title, display)
            key = cv2.waitKey(30) & 0xFF

            if key in (ord('q'), ord('Q'), 27):
                print("  已取消")
                break
            elif key == ord(' '):
                if faces:
                    emb = faces[0].embedding.astype(np.float32)
                    norm = np.linalg.norm(emb)
                    if norm > 0:
                        emb = emb / norm
                    embeddings.append(emb)
                    print(f"  ✓ 樣本 {len(embeddings)}/{n_samples} "
                          f"(信心度: {faces[0].det_score:.2f})")
                    # 閃爍提示
                    fb = display.copy()
                    cv2.putText(fb, f"CAPTURED {len(embeddings)}/{n_samples}",
                                (160, 250), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                    cv2.imshow(window_title, fb)
                    cv2.waitKey(400)
                else:
                    print("  ⚠ 未偵測到人臉，請調整位置")
    finally:
        cv2.destroyAllWindows()
        if picam2:
            picam2.stop()
            picam2.close()
        if cap:
            cap.release()

    return embeddings


def _average_embeddings(embeddings: list) -> np.ndarray:
    """L2 正規化後平均，再次正規化"""
    arr = np.stack(embeddings, axis=0)
    avg = arr.mean(axis=0).astype(np.float32)
    norm = np.linalg.norm(avg)
    return avg / norm if norm > 0 else avg


def add_vip_from_camera():
    """攝影機即時拍攝多張樣本後新增 VIP"""
    print("\n=== 攝影機即時註冊 VIP ===")
    name = input("VIP 名字: ").strip()
    if not name:
        print("✗ 未輸入名字")
        return
    gender    = input("性別 [M/F/Other] (預設 Other): ").strip() or "Other"
    phone     = input("電話 (選填): ").strip() or None
    email     = input("郵箱 (選填): ").strip() or None
    vip_level = input("等級 [standard/gold/platinum] (預設 standard): ").strip() or "standard"

    print(f"\n準備拍攝 {name} 的人臉 (5 張樣本)，按空白鍵逐張拍攝...")
    embeddings = capture_face_embeddings_from_camera(
        n_samples=5, window_title="Register VIP")

    if len(embeddings) < 3:
        print(f"✗ 樣本不足（需至少 3 張，實際 {len(embeddings)} 張），已取消")
        return

    avg = _average_embeddings(embeddings)
    recognizer = FaceRecognizer(DB_PATH)
    if recognizer.add_vip(name, avg, gender, phone, email, vip_level):
        print(f"✓ VIP '{name}' ({gender}) 已從 {len(embeddings)} 個樣本平均後儲存")
    recognizer.close()


def add_blacklist_from_camera():
    """攝影機即時拍攝多張樣本後新增黑名單"""
    print("\n=== 攝影機即時註冊黑名單 ===")
    name       = input("姓名: ").strip()
    if not name:
        print("✗ 未輸入名字")
        return
    gender     = input("性別 [M/F/Other] (預設 Other): ").strip() or "Other"
    reason     = input("原因 (選填): ").strip() or None
    risk_level = input("風險級別 [low/medium/high] (預設 medium): ").strip() or "medium"

    print(f"\n準備拍攝 {name} 的人臉 (5 張樣本)，按空白鍵逐張拍攝...")
    embeddings = capture_face_embeddings_from_camera(
        n_samples=5, window_title="Register Blacklist")

    if len(embeddings) < 3:
        print(f"✗ 樣本不足（需至少 3 張，實際 {len(embeddings)} 張），已取消")
        return

    avg = _average_embeddings(embeddings)
    recognizer = FaceRecognizer(DB_PATH)
    if recognizer.add_blacklist(name, avg, gender, reason, risk_level):
        print(f"✓ 黑名單 '{name}' ({gender}) 已從 {len(embeddings)} 個樣本平均後儲存")
    recognizer.close()


def add_vip_interactive():
    """交互式添加 VIP"""
    print("\n=== 添加 VIP ===")
    name = input("VIP 名字: ").strip()
    gender = input("性別 [M/F/Other] (預設: Other): ").strip() or "Other"
    phone = input("電話 (選填): ").strip() or None
    email = input("郵箱 (選填): ").strip() or None
    vip_level = input("等級 [standard/gold/platinum] (預設: standard): ").strip() or "standard"
    image_path = input("人臉照片路徑 (或留空生成測試用): ").strip()

    # 生成或加載 embedding
    if image_path and Path(image_path).exists():
        print(f"\n提取 embedding 中...")
        embedding = extract_embedding_from_image(image_path)
        if embedding is None:
            return
    else:
        # 生成隨機 embedding（用於測試）
        embedding = np.random.randn(512).astype(np.float32)
        embedding = embedding / (np.linalg.norm(embedding) + 1e-6)  # 正規化
        print("⚠ 生成隨機測試 embedding（實際應使用真實人臉圖像）")

    # 添加到資料庫
    recognizer = FaceRecognizer(DB_PATH)
    if recognizer.add_vip(name, embedding, gender, phone, email, vip_level):
        print(f"✓ VIP '{name}' ({gender}) 已添加")
    else:
        print(f"✗ 添加失敗")

    recognizer.close()


def add_vip_from_image(name: str, image_path: str, gender: str = 'Other', phone: str = None,
                       email: str = None, vip_level: str = 'standard'):
    """從圖像添加 VIP"""
    print(f"\n=== 從圖像添加 VIP: {name} ({gender}) ===")

    # 提取 embedding
    print("提取 embedding 中...")
    embedding = extract_embedding_from_image(image_path)
    if embedding is None:
        return False

    # 添加到資料庫
    recognizer = FaceRecognizer(DB_PATH)
    success = recognizer.add_vip(name, embedding, gender, phone, email, vip_level)
    recognizer.close()

    return success


def add_blacklist_interactive():
    """交互式添加黑名單"""
    print("\n=== 添加黑名單 ===")
    name = input("姓名: ").strip()
    gender = input("性別 [M/F/Other] (預設: Other): ").strip() or "Other"
    reason = input("原因 (選填): ").strip() or None
    risk_level = input("風險級別 [low/medium/high] (預設: medium): ").strip() or "medium"
    image_path = input("人臉照片路徑 (或留空生成測試用): ").strip()

    # 生成或加載 embedding
    if image_path and Path(image_path).exists():
        print(f"\n提取 embedding 中...")
        embedding = extract_embedding_from_image(image_path)
        if embedding is None:
            return
    else:
        embedding = np.random.randn(512).astype(np.float32)
        embedding = embedding / (np.linalg.norm(embedding) + 1e-6)
        print("⚠ 生成隨機測試 embedding")

    # 添加到資料庫
    recognizer = FaceRecognizer(DB_PATH)
    if recognizer.add_blacklist(name, embedding, gender, reason, risk_level):
        print(f"✓ 黑名單 '{name}' ({gender}) 已添加")
    else:
        print(f"✗ 添加失敗")

    recognizer.close()


def add_blacklist_from_image(name: str, image_path: str, gender: str = 'Other', reason: str = None,
                            risk_level: str = 'medium'):
    """從圖像添加黑名單"""
    print(f"\n=== 從圖像添加黑名單: {name} ({gender}) ===")

    # 提取 embedding
    print("提取 embedding 中...")
    embedding = extract_embedding_from_image(image_path)
    if embedding is None:
        return False

    # 添加到資料庫
    recognizer = FaceRecognizer(DB_PATH)
    success = recognizer.add_blacklist(name, embedding, gender, reason, risk_level)
    recognizer.close()

    return success


def list_vips():
    """列出所有 VIP"""
    import sqlite3

    if not Path(DB_PATH).exists():
        print("✗ 數據庫不存在，請先執行 init")
        return

    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()

    cursor.execute('SELECT id, name, gender, vip_level, phone, email, visit_count FROM vip_members WHERE is_active = 1')
    rows = cursor.fetchall()

    if not rows:
        print("未找到 VIP")
        return

    print("\n=== VIP 列表 ===")
    print(f"{'ID':<4} {'名字':<15} {'性別':<4} {'等級':<10} {'電話':<15} {'郵箱':<20} {'造訪次數':<5}")
    print("-" * 85)

    for row in rows:
        gender_display = row['gender'] if row['gender'] else 'Other'
        print(f"{row['id']:<4} {row['name']:<15} {gender_display:<4} {row['vip_level']:<10} "
              f"{row['phone'] or '-':<15} {row['email'] or '-':<20} {row['visit_count']:<5}")

    conn.close()


def list_blacklist():
    """列出所有黑名單"""
    import sqlite3

    if not Path(DB_PATH).exists():
        print("✗ 數據庫不存在，請先執行 init")
        return

    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()

    cursor.execute('SELECT id, name, gender, risk_level, reason FROM blacklist WHERE is_active = 1')
    rows = cursor.fetchall()

    if not rows:
        print("未找到黑名單記錄")
        return

    print("\n=== 黑名單 ===")
    print(f"{'ID':<4} {'名字':<15} {'性別':<4} {'風險級別':<10} {'原因':<30}")
    print("-" * 70)

    for row in rows:
        gender_display = row['gender'] if row['gender'] else 'Other'
        print(f"{row['id']:<4} {row['name']:<15} {gender_display:<4} {row['risk_level']:<10} {row['reason'] or '-':<30}")

    conn.close()


def test_recognition():
    """測試識別功能"""
    print("\n=== 測試識別 ===")

    if not Path(DB_PATH).exists():
        print("✗ 數據庫不存在，請先初始化")
        return

    recognizer = FaceRecognizer(DB_PATH)

    if len(recognizer.vip_cache) == 0:
        print("⚠ 未添加任何 VIP")
        return

    # 測試：查詢第一個 VIP 的 embedding
    first_vip_name = list(recognizer.vip_cache.keys())[0]
    test_embedding, _ = recognizer.vip_cache[first_vip_name]  # cache: (embedding, gender)

    print(f"測試識別 VIP: {first_vip_name}")
    result = recognizer.recognize_face(test_embedding)

    print(f"識別結果:")
    print(f"  類型: {result['person_type']}")
    print(f"  名字: {result['name']}")
    print(f"  信心度: {result['confidence']:.4f}")
    print(f"  詳情: {result['details']}")

    # 測試：添加噪聲的 embedding
    print(f"\n測試識別（添加噪聲）:")
    noisy_embedding = test_embedding + np.random.randn(512).astype(np.float32) * 0.1
    result = recognizer.recognize_face(noisy_embedding)

    print(f"識別結果:")
    print(f"  類型: {result['person_type']}")
    print(f"  名字: {result['name']}")
    print(f"  信心度: {result['confidence']:.4f}")

    recognizer.close()


def create_test_data():
    """創建測試數據"""
    print("創建測試數據...")

    # 初始化數據庫
    init_database()

    # 創建識別器
    recognizer = FaceRecognizer()

    # 添加測試 VIP
    test_vips = [
        ("李美琪", "0900123456", "lee@example.com", "platinum"),
        ("王建民", "0911234567", "wang@example.com", "gold"),
        ("黃小明", "0922234567", "huang@example.com", "standard"),
    ]

    print("\n添加測試 VIP...")
    for name, phone, email, level in test_vips:
        embedding = np.random.randn(512).astype(np.float32)
        embedding = embedding / (np.linalg.norm(embedding) + 1e-6)
        recognizer.add_vip(name, embedding, 'Other', phone, email, level)

    # 添加測試黑名單
    test_blacklist = [
        ("詐騙犯 A", "Known fraud case", "high"),
        ("盜竊嫌疑人", "Suspected theft", "medium"),
    ]

    print("\n添加測試黑名單...")
    for name, reason, risk in test_blacklist:
        embedding = np.random.randn(512).astype(np.float32)
        embedding = embedding / (np.linalg.norm(embedding) + 1e-6)
        recognizer.add_blacklist(name, embedding, 'Other', reason, risk)

    recognizer.close()
    print("✓ 測試數據已創建")


def delete_vip_interactive():
    """交互式刪除 VIP"""
    import sqlite3
    list_vips()
    print()
    name = input("請輸入要刪除的 VIP 名字（留空取消）: ").strip()
    if not name:
        print("已取消")
        return
    confirm = input(f"確認刪除 VIP '{name}'? [y/N]: ").strip().lower()
    if confirm != 'y':
        print("已取消")
        return
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("UPDATE vip_members SET is_active=0 WHERE name=? AND is_active=1", (name,))
    if cursor.rowcount > 0:
        conn.commit()
        print(f"✓ VIP '{name}' 已刪除")
    else:
        print(f"✗ 找不到 VIP '{name}'")
    conn.close()


def delete_blacklist_interactive():
    """交互式刪除黑名單"""
    import sqlite3
    list_blacklist()
    print()
    name = input("請輸入要刪除的黑名單姓名（留空取消）: ").strip()
    if not name:
        print("已取消")
        return
    confirm = input(f"確認刪除黑名單 '{name}'? [y/N]: ").strip().lower()
    if confirm != 'y':
        print("已取消")
        return
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("UPDATE blacklist SET is_active=0 WHERE name=? AND is_active=1", (name,))
    if cursor.rowcount > 0:
        conn.commit()
        print(f"✓ 黑名單 '{name}' 已刪除")
    else:
        print(f"✗ 找不到黑名單 '{name}'")
    conn.close()


def main():
    parser = argparse.ArgumentParser(description="VIP 資料庫管理工具")
    subparsers = parser.add_subparsers(dest="command", help="命令")

    # 初始化
    subparsers.add_parser("init", help="初始化數據庫")

    # 添加 VIP
    subparsers.add_parser("add-vip", help="添加 VIP（照片路徑，交互式）")

    # 攝影機即時註冊 VIP
    subparsers.add_parser("add-vip-camera", help="攝影機即時拍攝多張樣本後新增 VIP")

    # 從圖像添加 VIP
    add_vip_img = subparsers.add_parser("add-vip-image", help="從圖像添加 VIP")
    add_vip_img.add_argument("name", help="VIP 名字")
    add_vip_img.add_argument("image", help="人臉照片路徑")
    add_vip_img.add_argument("--gender", default="Other", help="性別 [M/F/Other]")
    add_vip_img.add_argument("--phone", help="電話 (選填)")
    add_vip_img.add_argument("--email", help="郵箱 (選填)")
    add_vip_img.add_argument("--level", default="standard", help="等級 [standard/gold/platinum]")

    # 添加黑名單
    subparsers.add_parser("add-blacklist", help="添加黑名單（照片路徑，交互式）")

    # 攝影機即時註冊黑名單
    subparsers.add_parser("add-blacklist-camera", help="攝影機即時拍攝多張樣本後新增黑名單")

    # 從圖像添加黑名單
    add_bl_img = subparsers.add_parser("add-blacklist-image", help="從圖像添加黑名單")
    add_bl_img.add_argument("name", help="姓名")
    add_bl_img.add_argument("image", help="人臉照片路徑")
    add_bl_img.add_argument("--gender", default="Other", help="性別 [M/F/Other]")
    add_bl_img.add_argument("--reason", help="原因 (選填)")
    add_bl_img.add_argument("--risk", default="medium", help="風險級別 [low/medium/high]")

    # 列表
    subparsers.add_parser("list-vips", help="列出所有 VIP")
    subparsers.add_parser("list-blacklist", help="列出所有黑名單")
    subparsers.add_parser("delete-vip", help="刪除 VIP（交互式）")
    subparsers.add_parser("delete-blacklist", help="刪除黑名單（交互式）")

    # 測試
    subparsers.add_parser("test", help="測試識別功能")

    # 創建測試數據
    subparsers.add_parser("create-test-data", help="創建測試數據（隨機 embedding）")

    args = parser.parse_args()

    if args.command == "init":
        init_database()
    elif args.command == "add-vip":
        add_vip_interactive()
    elif args.command == "add-vip-camera":
        add_vip_from_camera()
    elif args.command == "add-vip-image":
        add_vip_from_image(args.name, args.image, args.gender, args.phone, args.email, args.level)
    elif args.command == "add-blacklist":
        add_blacklist_interactive()
    elif args.command == "add-blacklist-camera":
        add_blacklist_from_camera()
    elif args.command == "add-blacklist-image":
        add_blacklist_from_image(args.name, args.image, args.gender, args.reason, args.risk)
    elif args.command == "list-vips":
        list_vips()
    elif args.command == "list-blacklist":
        list_blacklist()
    elif args.command == "delete-vip":
        delete_vip_interactive()
    elif args.command == "delete-blacklist":
        delete_blacklist_interactive()
    elif args.command == "test":
        test_recognition()
    elif args.command == "create-test-data":
        create_test_data()
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
