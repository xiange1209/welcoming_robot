#!/usr/bin/env python3
"""
VIP 資料庫管理工具
初始化、添加 VIP、添加黑名單、查詢等
"""

import sys
import argparse
import numpy as np
from pathlib import Path
from database.schema import DatabaseSchema
from vision_ai.face_recognizer import FaceRecognizer


def init_database():
    """初始化數據庫"""
    print("初始化數據庫...")
    schema = DatabaseSchema()
    schema.initialize()
    print("✓ 數據庫已初始化")


def add_vip_interactive():
    """交互式添加 VIP"""
    print("\n=== 添加 VIP ===")
    name = input("VIP 名字: ").strip()
    phone = input("電話 (選填): ").strip() or None
    email = input("郵箱 (選填): ").strip() or None
    vip_level = input("等級 [standard/gold/platinum] (預設: standard): ").strip() or "standard"
    embedding_path = input("Face embedding 文件路徑 (或留空生成測試用): ").strip()

    # 生成或加載 embedding
    if embedding_path and Path(embedding_path).exists():
        try:
            embedding = np.load(embedding_path).astype(np.float32)
            if embedding.shape != (512,):
                print(f"✗ Embedding 維度錯誤：{embedding.shape}，應為 (512,)")
                return
        except Exception as e:
            print(f"✗ 加載 embedding 失敗: {e}")
            return
    else:
        # 生成隨機 embedding（用於測試）
        embedding = np.random.randn(512).astype(np.float32)
        embedding = embedding / (np.linalg.norm(embedding) + 1e-6)  # 正規化
        print("⚠ 生成隨機測試 embedding（實際應使用真實人臉圖像）")

    # 添加到資料庫
    recognizer = FaceRecognizer()
    if recognizer.add_vip(name, embedding, phone, email, vip_level):
        print(f"✓ VIP '{name}' 已添加")
    else:
        print(f"✗ 添加失敗")

    recognizer.close()


def add_blacklist_interactive():
    """交互式添加黑名單"""
    print("\n=== 添加黑名單 ===")
    name = input("姓名: ").strip()
    reason = input("原因 (選填): ").strip() or None
    risk_level = input("風險級別 [low/medium/high] (預設: medium): ").strip() or "medium"
    embedding_path = input("Face embedding 文件路徑 (或留空生成測試用): ").strip()

    # 生成或加載 embedding
    if embedding_path and Path(embedding_path).exists():
        try:
            embedding = np.load(embedding_path).astype(np.float32)
            if embedding.shape != (512,):
                print(f"✗ Embedding 維度錯誤：{embedding.shape}，應為 (512,)")
                return
        except Exception as e:
            print(f"✗ 加載 embedding 失敗: {e}")
            return
    else:
        embedding = np.random.randn(512).astype(np.float32)
        embedding = embedding / (np.linalg.norm(embedding) + 1e-6)
        print("⚠ 生成隨機測試 embedding")

    # 添加到資料庫
    recognizer = FaceRecognizer()
    if recognizer.add_blacklist(name, embedding, reason, risk_level):
        print(f"✓ 黑名單 '{name}' 已添加")
    else:
        print(f"✗ 添加失敗")

    recognizer.close()


def list_vips():
    """列出所有 VIP"""
    import sqlite3

    db_path = "database/ai_bank_robot.db"
    if not Path(db_path).exists():
        print("✗ 數據庫不存在")
        return

    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()

    cursor.execute('SELECT id, name, vip_level, phone, email, visit_count FROM vip_members WHERE is_active = 1')
    rows = cursor.fetchall()

    if not rows:
        print("未找到 VIP")
        return

    print("\n=== VIP 列表 ===")
    print(f"{'ID':<4} {'名字':<15} {'等級':<10} {'電話':<15} {'郵箱':<20} {'造訪次數':<5}")
    print("-" * 75)

    for row in rows:
        print(f"{row['id']:<4} {row['name']:<15} {row['vip_level']:<10} "
              f"{row['phone'] or '-':<15} {row['email'] or '-':<20} {row['visit_count']:<5}")

    conn.close()


def list_blacklist():
    """列出所有黑名單"""
    import sqlite3

    db_path = "database/ai_bank_robot.db"
    if not Path(db_path).exists():
        print("✗ 數據庫不存在")
        return

    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()

    cursor.execute('SELECT id, name, risk_level, reason FROM blacklist WHERE is_active = 1')
    rows = cursor.fetchall()

    if not rows:
        print("未找到黑名單記錄")
        return

    print("\n=== 黑名單 ===")
    print(f"{'ID':<4} {'名字':<15} {'風險級別':<10} {'原因':<30}")
    print("-" * 60)

    for row in rows:
        print(f"{row['id']:<4} {row['name']:<15} {row['risk_level']:<10} {row['reason'] or '-':<30}")

    conn.close()


def test_recognition():
    """測試識別功能"""
    print("\n=== 測試識別 ===")

    db_path = "database/ai_bank_robot.db"
    if not Path(db_path).exists():
        print("✗ 數據庫不存在，請先初始化")
        return

    recognizer = FaceRecognizer(db_path)

    if len(recognizer.vip_cache) == 0:
        print("⚠ 未添加任何 VIP")
        return

    # 測試：查詢第一個 VIP 的 embedding
    first_vip_name = list(recognizer.vip_cache.keys())[0]
    test_embedding = recognizer.vip_cache[first_vip_name]

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
        recognizer.add_vip(name, embedding, phone, email, level)

    # 添加測試黑名單
    test_blacklist = [
        ("詐騙犯 A", "Known fraud case", "high"),
        ("盜竊嫌疑人", "Suspected theft", "critical"),
    ]

    print("\n添加測試黑名單...")
    for name, reason, risk in test_blacklist:
        embedding = np.random.randn(512).astype(np.float32)
        embedding = embedding / (np.linalg.norm(embedding) + 1e-6)
        recognizer.add_blacklist(name, embedding, reason, risk)

    recognizer.close()
    print("✓ 測試數據已創建")


def main():
    parser = argparse.ArgumentParser(description="VIP 資料庫管理工具")
    subparsers = parser.add_subparsers(dest="command", help="命令")

    # 初始化
    subparsers.add_parser("init", help="初始化數據庫")

    # 添加 VIP
    subparsers.add_parser("add-vip", help="添加 VIP（交互式）")

    # 添加黑名單
    subparsers.add_parser("add-blacklist", help="添加黑名單（交互式）")

    # 列表
    subparsers.add_parser("list-vips", help="列出所有 VIP")
    subparsers.add_parser("list-blacklist", help="列出所有黑名單")

    # 測試
    subparsers.add_parser("test", help="測試識別功能")

    # 創建測試數據
    subparsers.add_parser("create-test-data", help="創建測試數據")

    args = parser.parse_args()

    if args.command == "init":
        init_database()
    elif args.command == "add-vip":
        add_vip_interactive()
    elif args.command == "add-blacklist":
        add_blacklist_interactive()
    elif args.command == "list-vips":
        list_vips()
    elif args.command == "list-blacklist":
        list_blacklist()
    elif args.command == "test":
        test_recognition()
    elif args.command == "create-test-data":
        create_test_data()
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
