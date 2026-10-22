"""依人臉身分決定能不能帶位、能去哪裡（★ 2026-10-22）。

跟 HMI 的 smartnav_hmi/modules/guide/policy.py 讀同一份 ~/.smartnav/access_policy.json、用同一套判斷——
客人用說的（這裡）和在平板上點的（HMI）結果必須一致。全部是純函式（不碰 ROS）。

    一般訪客／認不出來／身分過期   → guest：一般地點
    VIP（含 ADMIN 身分的員工）      → vip：一般地點＋貴賓專屬地點；LLM 會拿到姓名與備註
    黑名單                         → blocked：不提供帶位，只能對話（通報行員由 bank_reception 處理）
"""

import json
import time
from pathlib import Path
from typing import Optional

POLICY_PATH = Path.home() / ".smartnav" / "access_policy.json"
DEFAULT = {
    "vip_only_waypoints": ["貴賓室"],
    "blacklist_can_navigate": False,
    "identity_fresh_sec": 10.0,
}
# UserType.msg：GUEST=0、VIP=1、ADMIN=2、BLACKLIST=3
TYPE_NAMES = {0: "GUEST", 1: "VIP", 2: "ADMIN", 3: "BLACKLIST"}

# ★ 對客人說的話一律中性：不說「黑名單」、不說「辨識」，也不說原因
BLOCKED_REPLY = "不好意思，這項服務目前沒辦法提供；需要協助的話，行員馬上會過來為您服務。"


def load_policy() -> dict:
    policy = dict(DEFAULT)
    try:
        policy.update(json.loads(POLICY_PATH.read_text(encoding="utf-8")))
    except (OSError, ValueError):
        pass
    return policy


def tier_of(identity: Optional[dict], policy: dict, now: Optional[float] = None) -> str:
    """identity：{"recognized", "user_type"（字串）, "user_name", "description", "stamp"（time.time()）}"""
    now = time.time() if now is None else now
    if not identity or not identity.get("recognized"):
        return "guest"
    if now - float(identity.get("stamp", 0.0)) > float(policy["identity_fresh_sec"]):
        return "guest"
    kind = identity.get("user_type")
    if kind == "BLACKLIST" and not policy["blacklist_can_navigate"]:
        return "blocked"
    if kind in ("VIP", "ADMIN"):
        return "vip"
    return "guest"


def nav_denial(tier: str, waypoint_name: str, policy: dict) -> Optional[str]:
    """不能去就回要對客人說的話；可以去回 None"""
    if tier == "blocked":
        return BLOCKED_REPLY
    if waypoint_name in policy["vip_only_waypoints"] and tier != "vip":
        return f"{waypoint_name}是貴賓專屬的區域，需要的話我請行員來協助您。"
    return None


def prompt_context(tier: str, identity: Optional[dict]) -> str:
    """附在系統提示最後的「目前的客人」。只有 VIP 會拿到姓名；黑名單只給中性的服務限制、不說原因。"""
    if tier == "vip" and identity:
        name = (identity.get("user_name") or "").strip()
        desc = (identity.get("description") or "").strip()
        lines = ["", "---", "", "# 目前在機器人前面的客人（人臉辨識，會隨時更新）", f"- 這位是本行貴賓：{name or '（未提供姓名）'}"]
        if desc:
            lines.append(f"- 備註：{desc}")
        lines.append("- 請用姓名稱呼（例如「" + (name or "貴賓") + "您好」），語氣更親切，並主動提供貴賓服務（例如帶位到貴賓室、請專員協助）。")
        lines.append("- 不要提到相似度、辨識、資料庫這些系統細節。")
        return "\n".join(lines)
    if tier == "blocked":
        return ("\n\n---\n\n# 目前的服務限制\n- 這位客人目前不提供帶位服務。客人要求帶路時，說「需要協助的話，行員馬上會過來為您服務」。\n"
                "- 不可以提到原因、身分、名單或辨識；其他一般問題照常回答。")
    return ""
