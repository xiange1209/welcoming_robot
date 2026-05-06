"""
SQLite 數據庫架構定義
- vip_members: VIP 名單 + 人臉 embedding
- blacklist: 黑名單人員
- visit_logs: 訪客日誌
- security_alerts: 安全告警
- robot_status_log: 機器人健康狀態
"""

import sqlite3
import json
from datetime import datetime
from pathlib import Path

_DEFAULT_DB = str(Path(__file__).parent.parent.parent / "database" / "ai_bank_robot.db")

class DatabaseSchema:
    """數據庫初始化和管理"""

    def __init__(self, db_path: str = None):
        self.db_path = Path(db_path or _DEFAULT_DB)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self.conn = None

    def connect(self):
        """連接到 SQLite 數據庫"""
        self.conn = sqlite3.connect(str(self.db_path))
        self.conn.row_factory = sqlite3.Row
        return self.conn

    def close(self):
        """關閉連接"""
        if self.conn:
            self.conn.close()

    def initialize(self):
        """初始化所有表"""
        conn = self.connect()
        cursor = conn.cursor()

        # 1. VIP 成員表
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS vip_members (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                gender TEXT,                                 -- 'M', 'F', 'Other'
                phone TEXT,
                email TEXT,
                vip_level TEXT DEFAULT 'standard',          -- gold, platinum, standard
                face_embedding BLOB NOT NULL,               -- 512D embedding 向量 (numpy float32)
                embedding_version TEXT DEFAULT '1.0',
                registered_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                last_visit TIMESTAMP,
                visit_count INTEGER DEFAULT 0,
                notes TEXT,
                is_active BOOLEAN DEFAULT 1,
                UNIQUE(name)
            )
        ''')

        # 2. 黑名單表
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS blacklist (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                gender TEXT,                                 -- 'M', 'F', 'Other'
                face_embedding BLOB NOT NULL,
                reason TEXT,                                 -- 加入黑名單理由
                risk_level TEXT DEFAULT 'medium',           -- low, medium, high
                added_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                added_by TEXT,
                is_active BOOLEAN DEFAULT 1,
                notes TEXT,
                UNIQUE(name)
            )
        ''')

        # 3. 訪客日誌表
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS visit_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                person_id INTEGER,                          -- VIP member id or NULL
                person_name TEXT,
                person_type TEXT NOT NULL,                  -- 'vip', 'blacklist', 'visitor'
                face_embedding BLOB,
                emotion TEXT,                               -- happy, angry, sad, neutral, etc.
                confidence REAL,                            -- 臉部識別信心度 (0-1)
                visit_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                visit_duration INTEGER,                     -- 秒數
                location TEXT,                              -- 機器人位置/區域
                device_status TEXT,                         -- 設備狀態 (JSON)
                frame_image_path TEXT,                      -- 可選：保存人臉幀
                notes TEXT
            )
        ''')

        # 4. 安全告警表
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS security_alerts (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                alert_type TEXT NOT NULL,                   -- 'blacklist_detected', 'unauthorized_access', etc.
                person_id INTEGER,
                person_name TEXT,
                severity TEXT DEFAULT 'medium',             -- low, medium, high, critical
                message TEXT,
                alert_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                resolved BOOLEAN DEFAULT 0,
                resolved_time TIMESTAMP,
                resolved_by TEXT,
                action_taken TEXT,                          -- e.g. 'lora_sent', 'email_sent'
                device_status TEXT                          -- JSON
            )
        ''')

        # 5. 機器人狀態日誌表
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS robot_status_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                cpu_usage REAL,                             -- CPU 百分比
                memory_usage REAL,                          -- 內存百分比
                disk_usage REAL,                            -- 磁盤百分比
                temperature REAL,                           -- 樹莓派溫度 (°C)
                inference_latency_ms REAL,                  -- 推理延遲
                fps REAL,                                   -- 實際幀率
                device_status TEXT,                         -- JSON: 各項硬件狀態
                error_log TEXT
            )
        ''')

        # 建立索引以提高查詢效率
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_visit_logs_time ON visit_logs(visit_time DESC)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_visit_logs_person_type ON visit_logs(person_type)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_security_alerts_time ON security_alerts(alert_time DESC)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_security_alerts_resolved ON security_alerts(resolved)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_robot_status_time ON robot_status_log(timestamp DESC)')

        conn.commit()
        conn.close()
        print("✓ 數據庫初始化完成")

    @staticmethod
    def add_vip_member(conn, name: str, face_embedding, gender: str = 'Other', phone: str = None, email: str = None, vip_level: str = 'standard'):
        """添加 VIP 成員"""
        cursor = conn.cursor()
        cursor.execute('''
            INSERT INTO vip_members (name, gender, face_embedding, phone, email, vip_level)
            VALUES (?, ?, ?, ?, ?, ?)
        ''', (name, gender, face_embedding, phone, email, vip_level))
        conn.commit()
        return cursor.lastrowid

    @staticmethod
    def query_vip_by_name(conn, name: str):
        """根據名字查詢 VIP"""
        cursor = conn.cursor()
        cursor.execute('SELECT * FROM vip_members WHERE name = ? AND is_active = 1', (name,))
        return cursor.fetchone()

    @staticmethod
    def log_visit(conn, person_type: str, person_name: str, face_embedding=None, emotion: str = None, confidence: float = None):
        """記錄訪問"""
        cursor = conn.cursor()
        cursor.execute('''
            INSERT INTO visit_logs (person_type, person_name, face_embedding, emotion, confidence)
            VALUES (?, ?, ?, ?, ?)
        ''', (person_type, person_name, face_embedding, emotion, confidence))
        conn.commit()

    @staticmethod
    def log_security_alert(conn, alert_type: str, person_name: str = None, severity: str = 'medium', message: str = None):
        """記錄安全告警"""
        cursor = conn.cursor()
        cursor.execute('''
            INSERT INTO security_alerts (alert_type, person_name, severity, message)
            VALUES (?, ?, ?, ?)
        ''', (alert_type, person_name, severity, message))
        conn.commit()

    @staticmethod
    def log_robot_status(conn, cpu_usage: float = None, memory_usage: float = None, disk_usage: float = None,
                        temperature: float = None, inference_latency_ms: float = None, fps: float = None):
        """記錄機器人狀態"""
        cursor = conn.cursor()
        cursor.execute('''
            INSERT INTO robot_status_log (cpu_usage, memory_usage, disk_usage, temperature, inference_latency_ms, fps)
            VALUES (?, ?, ?, ?, ?, ?)
        ''', (cpu_usage, memory_usage, disk_usage, temperature, inference_latency_ms, fps))
        conn.commit()


if __name__ == "__main__":
    # 初始化數據庫
    schema = DatabaseSchema()
    schema.initialize()
