"""依身分決定「能不能帶位、能去哪裡」（★ 2026-10-22）。

身分來自人臉辨識（/user_identity），規則檔 ~/.smartnav/access_policy.json（沒有就用 DEFAULT）。
LLM 節點（smartnav_llm/access_policy.py）讀同一份檔、用同一套判斷——平板點的和嘴巴說的結果才會一致。

    一般訪客／認不出來／身分過期   → guest：一般地點
    VIP（含 ADMIN 身分的員工）      → vip：一般地點＋貴賓專屬地點
    黑名單                         → blocked：不提供帶位（只能對話；通報行員是 bank_reception 的事）
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
    # 不列在迎賓頁按鈕上的地點（工作人員用的，例如起點／充電位）；只影響平板顯示，管理者頁照常
    "guest_menu_exclude": ["起點"],
    # 身分多久沒更新就當成「沒有人／一般訪客」（人臉每 2 秒推論一次，人走掉 user_auth 也會清掉身分）
    "identity_fresh_sec": 10.0,
}


def load_policy() -> dict:
    """每次都重讀：檔案很小，改了不用重啟。壞掉就用預設值。"""
    policy = dict(DEFAULT)
    try:
        policy.update(json.loads(POLICY_PATH.read_text(encoding="utf-8")))
    except (OSError, ValueError):
        pass
    return policy


def tier_of(identity: Optional[dict], policy: dict, now: Optional[float] = None) -> str:
    """identity：HMI 狀態裡的那份（user_type 是字串、updated_at 是 time.time()）"""
    now = time.time() if now is None else now
    if not identity or not identity.get("recognized"):
        return "guest"
    if now - float(identity.get("updated_at", 0.0)) > float(policy["identity_fresh_sec"]):
        return "guest"
    kind = identity.get("user_type")
    if kind == "BLACKLIST" and not policy["blacklist_can_navigate"]:
        return "blocked"
    if kind in ("VIP", "ADMIN"):
        return "vip"
    return "guest"


def allowed(tier: str, waypoint_name: str, policy: dict) -> bool:
    if tier == "blocked":
        return False
    return not (waypoint_name in policy["vip_only_waypoints"] and tier != "vip")
