"""依人臉身分決定能不能帶位、能去哪裡（系統裡唯一的一份規則）。

規則檔 ~/.smartnav/access_policy.json（沒有就用 DEFAULT，每次都重讀，改了不用重啟）。
LLM 與 HMI 都不再自己判斷：它們只能透過 nav_arbiter 的 RequestNavigation／ListGuideDestinations 取得結果。

    一般訪客／認不出來／身分過期   → guest：一般地點
    VIP（含 ADMIN 身分的員工）      → vip：一般地點＋貴賓專屬地點
    黑名單                         → blocked：不提供帶位，只能對話（通報行員由 bank_reception 處理）

全部是純函式（不碰 ROS），方便單獨測試。
"""

import json
import time
from pathlib import Path
from typing import Optional

POLICY_PATH = Path.home() / ".smartnav" / "access_policy.json"
DEFAULT = {
    # 只有 VIP 能去的地點（地點名稱，跟地圖上標的一樣）
    "vip_only_waypoints": ["貴賓室"],
    "blacklist_can_navigate": False,
    # 不列在迎賓頁按鈕上的地點（工作人員用的，例如起點／充電位）；只影響迎賓頁，管理者頁照常
    "guest_menu_exclude": ["起點"],
    # 身分多久沒更新就當成「沒有人／一般訪客」（人臉每 2 秒推論一次，人走掉 user_auth 也會清掉身分）
    "identity_fresh_sec": 10.0,
    # ★ 2026-11-24（使用者同意，先 30 秒）：認出貴賓後這麼多秒內，「認不出」或「沒人」仍算同一位貴賓。
    #   11/24 貴賓室帶位被拒 4 次 —— 等級 VIP↔一般 一小時跳 73 次，客人一轉頭就掉回一般。0 = 關閉。
    "vip_grace_sec": 30.0,
}

TIER_GUEST = "guest"
TIER_VIP = "vip"
TIER_BLOCKED = "blocked"

# ★ 對客人說的話一律中性：不說「黑名單」、不說「辨識」，也不說原因
BLOCKED_REPLY = "不好意思，這項服務目前沒辦法提供；需要協助的話，行員馬上會過來為您服務。"
POSE_DENIED_REPLY = "不好意思，這項服務目前沒辦法提供；需要協助的話，請洽櫃檯行員。"


def vip_only_reply(waypoint_name: str) -> str:
    return f"{waypoint_name}是貴賓專屬的區域，需要的話請洽櫃檯行員協助。"


def load_policy() -> dict:
    policy = dict(DEFAULT)
    try:
        policy.update(json.loads(POLICY_PATH.read_text(encoding="utf-8")))
    except (OSError, ValueError):
        pass
    return policy


def tier_of(identity: Optional[dict], policy: dict, now: Optional[float] = None) -> str:
    """identity：{"recognized", "user_type"（字串）, "stamp"（time.monotonic()）}"""
    now = time.monotonic() if now is None else now
    if not identity or not identity.get("recognized"):
        return TIER_GUEST
    if now - float(identity.get("stamp", 0.0)) > float(policy["identity_fresh_sec"]):
        return TIER_GUEST
    kind = identity.get("user_type")
    if kind == "BLACKLIST" and not policy["blacklist_can_navigate"]:
        return TIER_BLOCKED
    if kind in ("VIP", "ADMIN"):
        return TIER_VIP
    return TIER_GUEST


def with_vip_grace(identity: Optional[dict], last_vip: Optional[dict], policy: dict,
                   now: Optional[float] = None) -> Optional[dict]:
    """認出貴賓後 vip_grace_sec 內，把「認不出／沒人」換成那位貴賓（給 tier_of 用）。

    認得出是別人（其他註冊者、黑名單）時以這一刻為準 —— 寬限只補「認不出」，不蓋掉「認得出」。
    last_vip：最後一次認出 VIP／ADMIN 的那則身分（含 stamp＝收到時的 time.monotonic()）。
    """
    now = time.monotonic() if now is None else now
    grace = float(policy.get("vip_grace_sec", 0) or 0)
    if grace <= 0 or not last_vip:
        return identity
    if identity and identity.get("recognized"):
        return identity
    if now - float(last_vip.get("stamp", -1e9)) > grace:
        return identity
    return {**last_vip, "stamp": now, "vip_grace": True}


def navigation_denial(tier: str, waypoint_name: str, policy: dict) -> Optional[str]:
    """這位客人不能去這個地點就回要對他說的話；可以去回 None"""
    if tier == TIER_BLOCKED:
        return BLOCKED_REPLY
    if waypoint_name in policy["vip_only_waypoints"] and tier != TIER_VIP:
        return vip_only_reply(waypoint_name)
    return None
