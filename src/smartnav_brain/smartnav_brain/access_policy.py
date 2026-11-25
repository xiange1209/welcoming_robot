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
    # ★ 2026-11-24（使用者同意，先 30 秒）：認出貴賓後這麼多秒內，「認不出」仍算同一位貴賓。0 = 關閉。
    #   11/24 貴賓室帶位被拒 4 次 —— 等級 VIP↔一般 一小時跳 73 次，客人一轉頭就掉回一般。
    # ★ 2026-11-25：預設改 0（關閉）。程式分不出「貴賓角度不好」和「貴賓走開、陌生人馬上接上」，
    #   寬限期內陌生人會被帶去貴賓室；而那 4 次被拒離上次認出 13～177 秒，8 秒一次都救不回來、30 秒只救回 1 次。
    #   要用就在 ~/.smartnav/access_policy.json 寫秒數，並接受上述風險。
    "vip_grace_sec": 0.0,
    # ★ 2026-11-25：寬限只補「同一張臉暫時認不出」。鏡頭前沒有人（user_auth 發的「無人」）就結束寬限；
    #   false = 沒有人也繼續算（11/24 原本的行為）。
    "vip_grace_clear_on_absent": True,
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
    return {**last_vip, "stamp": now, "vip_grace": True, "vip_grace_age": now - float(last_vip.get("stamp", now))}


def next_last_vip(last_vip: Optional[dict], identity: Optional[dict], absent: bool, policy: dict) -> Optional[dict]:
    """收到一則身分之後，「寬限用的最後一位貴賓」該變成什麼 —— 寬限不可以跨人。

    認出貴賓／員工 → 換成這一位；認出別人（一般註冊者、黑名單）→ 清掉；
    沒有人 → 依 vip_grace_clear_on_absent 清掉；有臉但認不出 → 不變。
    ★ 2026-11-25：原本從來不清，「貴賓 → 黑名單 → 黑名單轉頭認不出」會變回貴賓。
    """
    if identity and identity.get("recognized"):
        return dict(identity) if identity.get("user_type") in ("VIP", "ADMIN") else None
    if absent and policy.get("vip_grace_clear_on_absent", True):
        return None
    return last_vip


def navigation_denial(tier: str, waypoint_name: str, policy: dict) -> Optional[str]:
    """這位客人不能去這個地點就回要對他說的話；可以去回 None"""
    if tier == TIER_BLOCKED:
        return BLOCKED_REPLY
    if waypoint_name in policy["vip_only_waypoints"] and tier != TIER_VIP:
        return vip_only_reply(waypoint_name)
    return None
