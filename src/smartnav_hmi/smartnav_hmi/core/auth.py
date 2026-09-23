"""管理者登入。

★ 這是「操作閘門」不是資安機制：共用平板上避免訪客誤按到使用者管理或導航，
  並讓管理端點需要登入才能呼叫。權杖存在記憶體、走明文 HTTP，同一個區網內
  有心人仍可側錄——不要把它當成真正的存取控制。
"""

import os
import secrets
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Protocol
from dotenv import load_dotenv
from ament_index_python.packages import get_package_share_directory


class _Logger(Protocol):
    """只用到這兩支，不綁 rclpy 的 logger 型別"""

    def info(self, msg: str) -> None: ...

    def warn(self, msg: str) -> None: ...


class AdminAuth:
    """記憶體中的權杖表 + 帳密比對"""

    def __init__(self, username: str, password: str, session_sec: float):
        self.username = username
        self.password = password
        self.session_sec = session_sec
        self._lock = threading.Lock()
        self._tokens: Dict[str, float] = {}

    def issue(self) -> str:
        token = secrets.token_urlsafe(24)
        now = time.time()
        with self._lock:
            # 順手清掉過期的，免得長時間執行後無限增長
            self._tokens = {t: exp for t, exp in self._tokens.items() if exp > now}
            self._tokens[token] = now + self.session_sec
        return token

    def valid(self, token: str) -> bool:
        if not token:
            return False
        with self._lock:
            expiry = self._tokens.get(token)
            if expiry is None:
                return False
            if expiry <= time.time():
                del self._tokens[token]
                return False
        return True

    def revoke(self, token: str) -> None:
        with self._lock:
            self._tokens.pop(token, None)

    def check(self, username: str, password: str) -> bool:
        # compare_digest 避免用字串比較洩漏長度/前綴資訊
        return secrets.compare_digest(username, self.username) and secrets.compare_digest(password, self.password)


def token_from_header(authorization: str) -> str:
    """從 `Authorization: Bearer <token>` 取出權杖，格式不符時回空字串"""
    if not authorization.lower().startswith("bearer "):
        return ""
    return authorization[7:].strip()


def resolve_password(param_value: str, username: str, logger: Optional[_Logger] = None) -> str:
    """設定管理者密碼，順序：launch 參數 > 環境變數 > config/.env > 隨機生成"""
    if param_value:
        return param_value

    env = os.getenv("SMARTNAV_ADMIN_PASSWORD", "")
    if env:
        if logger:
            logger.info("管理者密碼來自環境變數 SMARTNAV_ADMIN_PASSWORD")
        return env

    # 從 config/.env 讀取
    if load_dotenv and get_package_share_directory:
        try:
            share_dir = Path(get_package_share_directory("smartnav_hmi"))
            env_file = share_dir / "config" / ".env"
            if env_file.exists():
                load_dotenv(env_file)
                pw = os.getenv("SMARTNAV_ADMIN_PASSWORD", "")
                if pw:
                    if logger:
                        logger.info("管理者密碼來自 config/.env")
                    return pw
        except Exception:
            pass

    # token_urlsafe 會產生 URL 安全字元，不會有需要跳脫的符號，平板上好輸入
    pw = secrets.token_urlsafe(6)
    if logger:
        logger.warn(
            "\n"
            "════════════════════════════════════════════\n"
            " 未指定管理者密碼，已隨機生成一組：\n"
            f"     帳號 {username}    密碼 {pw}\n"
            "\n"
            " 這組密碼每次重啟都會變。要固定的話擇一：\n"
            "   ros2 launch ... admin_password:=你的密碼\n"
            "   export SMARTNAV_ADMIN_PASSWORD=你的密碼\n"
            "   在 config/.env 設定 SMARTNAV_ADMIN_PASSWORD\n"
            "════════════════════════════════════════════"
        )
    return pw
