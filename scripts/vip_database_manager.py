#!/usr/bin/env python3
"""
VIP 數據庫管理工具
用途：
  - 初始化 SQLite 數據庫
  - 添加 VIP 成員
  - 添加黑名單人員
  - 查詢和管理數據
"""

import sqlite3
import numpy as np
import sys
import json
from pathlib import Path
from datetime import datetime


class VIPDatabaseManager:
    """VIP 和黑名單數據庫管理器"""

    def __init__(self, db_path: str = "database/ai_bank_robot.db"):
        """初始化數據庫連接"""
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self.conn = None

    def connect(self):
        """連接到數據庫"""
        self.conn = sqlite3.connect(str(self.db_path))
        self.conn.row_factory = sqlite3.Row
        return self.conn

    def close(self):
        """關閉連接"""
        if self.conn:
            self.conn.close()

    def init_database(self):
        """初始化數據庫架構"""
        self.connect()
        cursor = self.conn.cursor()

        # VIP 成員表
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS vip_members (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL UNIQUE,
                phone TEXT,
                email TEXT,
                vip_level TEXT DEFAULT 'standard',
                face_embedding BLOB NOT NULL,
                embedding_version TEXT DEFAULT '1.0',
                registered_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                last_visit TIMESTAMP,
                visit_count INTEGER DEFAULT 0,
                notes TEXT,
                is_active BOOLEAN DEFAULT 1
            )
        ''')

        # 黑名單表
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS blacklist (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL UNIQUE,
                face_embedding BLOB NOT NULL,
                reason TEXT,
                risk_level TEXT DEFAULT 'medium',
                added_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                added_by TEXT,
                is_active BOOLEAN DEFAULT 1,
                notes TEXT
            )
        ''')

        # 訪客日誌表
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS visit_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                person_id INTEGER,
                person_name TEXT,
                person_type TEXT,
                face_embedding BLOB,
                emotion TEXT,
                confidence REAL,
                visit_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                visit_duration INTEGER,
                location TEXT,
                device_status TEXT,
                frame_image_path TEXT,
                notes TEXT
            )
        ''')

        # 安全告警表
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS security_alerts (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                alert_type TEXT NOT NULL,
                person_id INTEGER,
                person_name TEXT,
                severity TEXT DEFAULT 'medium',
                message TEXT,
                alert_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                resolved BOOLEAN DEFAULT 0,
                resolved_time TIMESTAMP,
                resolved_by TEXT,
                action_taken TEXT,
                device_status TEXT
            )
        ''')

        # 索引
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_vip_name ON vip_members(name)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_blacklist_name ON blacklist(name)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_visit_time ON visit_logs(visit_time DESC)')

        self.conn.commit()
        print("✓ 數據庫初始化完成")

    def add_vip(self, name: str, embedding: np.ndarray, phone: str = None,
                email: str = None, vip_level: str = 'standard', notes: str = None):
        """添加 VIP 成員"""
        cursor = self.conn.cursor()
        try:
            cursor.execute('''
                INSERT OR REPLACE INTO vip_members
                (name, face_embedding, phone, email, vip_level, notes, registered_at)
                VALUES (?, ?, ?, ?, ?, ?, ?)
            ''', (name, embedding.tobytes(), phone, email, vip_level, notes, datetime.now()))
            self.conn.commit()
            print(f"✓ VIP '{name}' 已添加 (等級: {vip_level})")
            return True
        except Exception as e:
            print(f"✗ 添加失敗: {e}")
            return False

    def add_blacklist(self, name: str, embedding: np.ndarray, reason: str = None,
                      risk_level: str = 'medium', notes: str = None):
        """添加黑名單人員"""
        cursor = self.conn.cursor()
        try:
            cursor.execute('''
                INSERT OR REPLACE INTO blacklist
                (name, face_embedding, reason, risk_level, notes, added_at)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', (name, embedding.tobytes(), reason, risk_level, notes, datetime.now()))
            self.conn.commit()
            print(f"✓ 黑名單 '{name}' 已添加 (風險等級: {risk_level})")
            return True
        except Exception as e:
            print(f"✗ 添加失敗: {e}")
            return False

    def list_vips(self):
        """列出所有 VIP"""
        cursor = self.conn.cursor()
        cursor.execute('SELECT id, name, vip_level, phone, email, visit_count FROM vip_members WHERE is_active = 1')
        rows = cursor.fetchall()

        print("\n" + "=" * 80)
        print("VIP 成員列表")
        print("=" * 80)
        if not rows:
            print("（無 VIP 記錄）")
        else:
            for row in rows:
                print(f"ID: {row['id']:3} | 名稱: {row['name']:15} | 等級: {row['vip_level']:10} | "
                      f"電話: {row['phone'] or 'N/A':15} | 訪問次數: {row['visit_count']}")
        print("=" * 80 + "\n")

    def list_blacklist(self):
        """列出所有黑名單"""
        cursor = self.conn.cursor()
        cursor.execute('SELECT id, name, risk_level, reason FROM blacklist WHERE is_active = 1')
        rows = cursor.fetchall()

        print("\n" + "=" * 80)
        print("黑名單列表")
        print("=" * 80)
        if not rows:
            print("（無黑名單記錄）")
        else:
            for row in rows:
                print(f"ID: {row['id']:3} | 名稱: {row['name']:15} | 風險等級: {row['risk_level']:10} | "
                      f"原因: {row['reason'] or 'N/A'}")
        print("=" * 80 + "\n")

    def get_vip_count(self):
        """獲取 VIP 數量"""
        cursor = self.conn.cursor()
        cursor.execute('SELECT COUNT(*) as cnt FROM vip_members WHERE is_active = 1')
        return cursor.fetchone()['cnt']

    def get_blacklist_count(self):
        """獲取黑名單數量"""
        cursor = self.conn.cursor()
        cursor.execute('SELECT COUNT(*) as cnt FROM blacklist WHERE is_active = 1')
        return cursor.fetchone()['cnt']

    def delete_vip(self, name: str):
        """刪除 VIP（邏輯刪除）"""
        cursor = self.conn.cursor()
        cursor.execute('UPDATE vip_members SET is_active = 0 WHERE name = ?', (name,))
        self.conn.commit()
        print(f"✓ VIP '{name}' 已刪除")

    def delete_blacklist(self, name: str):
        """刪除黑名單（邏輯刪除）"""
        cursor = self.conn.cursor()
        cursor.execute('UPDATE blacklist SET is_active = 0 WHERE name = ?', (name,))
        self.conn.commit()
        print(f"✓ 黑名單 '{name}' 已刪除")


def create_test_data():
    """創建測試數據"""
    print("\n正在創建測試 VIP 和黑名單...")

    manager = VIPDatabaseManager()
    manager.connect()

    # 創建 3 個虛擬 embedding
    test_vips = [
        {
            'name': '李美琪',
            'embedding': np.random.randn(512).astype(np.float32),
            'phone': '0912345678',
            'email': 'li.meiqi@bank.com',
            'level': 'platinum',
            'notes': 'CEO - 最高 VIP'
        },
        {
            'name': '王小明',
            'embedding': np.random.randn(512).astype(np.float32),
            'phone': '0987654321',
            'email': 'wang.xiaoming@bank.com',
            'level': 'gold',
            'notes': '資深客戶'
        },
        {
            'name': '張三',
            'embedding': np.random.randn(512).astype(np.float32),
            'phone': '0911111111',
            'email': 'zhang.san@bank.com',
            'level': 'standard',
            'notes': '普通 VIP'
        }
    ]

    test_blacklist = [
        {
            'name': '嫌疑人 A',
            'embedding': np.random.randn(512).astype(np.float32),
            'risk_level': 'high',
            'reason': '詐欺罪嫌'
        },
        {
            'name': '嫌疑人 B',
            'embedding': np.random.randn(512).astype(np.float32),
            'risk_level': 'medium',
            'reason': '禁止進入'
        }
    ]

    # 添加 VIP
    for vip in test_vips:
        manager.add_vip(vip['name'], vip['embedding'], vip['phone'], vip['email'], vip['level'], vip['notes'])

    # 添加黑名單
    for bl in test_blacklist:
        manager.add_blacklist(bl['name'], bl['embedding'], bl['reason'], bl['risk_level'])

    manager.list_vips()
    manager.list_blacklist()

    print(f"✓ 測試數據創建完成 ({manager.get_vip_count()} VIP, {manager.get_blacklist_count()} 黑名單)")
    manager.close()


if __name__ == "__main__":
    # 初始化數據庫
    manager = VIPDatabaseManager()
    manager.init_database()
    manager.connect()

    # 創建測試數據
    if len(sys.argv) > 1 and sys.argv[1] == '--create-test':
        create_test_data()
    else:
        manager.list_vips()
        manager.list_blacklist()
        print("\n📝 提示:")
        print("  python3 vip_database_manager.py --create-test    # 創建測試數據")

    manager.close()
