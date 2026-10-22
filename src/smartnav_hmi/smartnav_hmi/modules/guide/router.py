"""訪客帶位（★ 2026-10-22）：迎賓頁不用登入就能點「帶我去」，能去哪裡依人臉辨識到的身分決定。

規則見 policy.py。登入的管理者走原本的 /api/navigate（地圖與導航頁），不受這裡限制。
★ 回給平板的訊息一律中性：平板是對著客人的，不能出現「黑名單」字樣（通報行員走 Telegram）。
"""

from fastapi import APIRouter
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from smartnav_msgs.action import Navigate
from smartnav_msgs.srv import ListWaypoints

from ...api.dependencies import guard
from .policy import allowed, load_policy, tier_of

BLOCKED_MSG = "需要協助的話，請洽櫃檯行員，我們馬上為您服務。"


class GuideRequest(BaseModel):
    waypoint_id: str


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    def _tier(policy: dict) -> str:
        return tier_of(node.state.snapshot().get("identity") or {}, policy)

    def _waypoints() -> list:
        res = node.jobs.call_service("list_waypoints", ListWaypoints.Request())
        if not res.success:
            raise RuntimeError(res.message or "讀不到地點")
        return [(w.waypoint_id, w.waypoint_name) for w in res.waypoints_info if w.waypoint_name]

    def _navigating() -> bool:
        jobs = node.state.snapshot().get("jobs") or {}
        jobs = jobs.values() if isinstance(jobs, dict) else jobs
        return any(j.get("action") == "navigate" and j.get("status") in ("pending", "running") for j in jobs)

    @router.get("/api/guide/destinations")
    async def api_guide_destinations() -> JSONResponse:
        def work():
            policy = load_policy()
            tier = _tier(policy)
            if tier == "blocked":
                return {"success": True, "can_navigate": False, "tier": "guest", "message": BLOCKED_MSG,
                        "destinations": []}
            dests = [
                {"waypoint_id": wid, "name": name, "vip_only": name in policy["vip_only_waypoints"]}
                for wid, name in _waypoints()
                if allowed(tier, name, policy) and name not in policy.get("guest_menu_exclude", [])
            ]
            return {"success": True, "can_navigate": True, "tier": tier, "message": "", "destinations": dests,
                    "busy": _navigating()}

        return await guard(node, work)

    @router.post("/api/guide")
    async def api_guide(req: GuideRequest) -> JSONResponse:
        def work():
            policy = load_policy()
            tier = _tier(policy)  # ★ 送出前重新判斷：清單是幾秒前拿的，身分可能已經換人
            if tier == "blocked":
                return {"success": False, "message": BLOCKED_MSG}
            name = dict(_waypoints()).get(req.waypoint_id)
            if name is None:
                return {"success": False, "message": "找不到這個地點，請重新整理後再試"}
            if not allowed(tier, name, policy):
                node.get_logger().info(f"帶位拒絕：{name} 是貴賓專屬，目前身分 {tier}")
                return {"success": False, "message": f"{name}是貴賓專屬區域，需要的話請洽櫃檯行員"}
            if _navigating():
                return {"success": False, "message": "我正在帶位中，請稍等一下"}
            goal = Navigate.Goal()
            goal.waypoint_id = req.waypoint_id
            goal.target_name = name
            job_id = node.jobs.start_action("navigate", goal, f"帶位到 {name}（迎賓頁，{tier}）")
            node.get_logger().info(f"🧭 迎賓頁帶位：{name}（身分 {tier}）")
            return {"success": True, "message": f"好的，帶您到{name}", "job_id": job_id}

        return await guard(node, work)

    return router
