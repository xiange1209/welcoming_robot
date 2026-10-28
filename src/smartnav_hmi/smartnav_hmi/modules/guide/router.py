"""訪客帶位（★ 2026-10-22）：迎賓頁不用登入就能點「帶我去」，能去哪裡依人臉辨識到的身分決定。

身分規則只在 smartnav_brain 的 nav_arbiter：這裡只負責顯示它回的清單，以及把請求送給它。
登入的管理者走原本的 /api/navigate（地圖與導航頁，來源 OPERATOR），不受身分限制。
★ 回給平板的訊息一律中性：平板是對著客人的，不能出現「黑名單」字樣（通報行員走 Telegram）。
"""

from fastapi import APIRouter
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from smartnav_msgs.action import RequestNavigation
from smartnav_msgs.srv import ListGuideDestinations

from ...api.dependencies import guard

# 送出後最多等 nav_arbiter 核准或拒絕這麼久；超過就先回「已送出」，結果由 job 狀態更新
AUTHORIZE_WAIT_SEC = 8.0


class GuideRequest(BaseModel):
    waypoint_id: str


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    @router.get("/api/guide/destinations")
    async def api_guide_destinations() -> JSONResponse:
        def work():
            res = node.jobs.call_service("list_guide_destinations", ListGuideDestinations.Request())
            if not res.success:
                raise RuntimeError(res.message or "讀不到可帶位的地點")
            return {
                "success": True,
                "can_navigate": bool(res.can_navigate),
                "tier": res.tier,
                "message": res.message,
                "busy": bool(res.busy),
                "destinations": [
                    {"waypoint_id": d.waypoint_id, "name": d.name, "vip_only": bool(d.vip_only)}
                    for d in res.destinations
                ],
            }

        return await guard(node, work)

    @router.post("/api/guide")
    async def api_guide(req: GuideRequest) -> JSONResponse:
        def work():
            goal = RequestNavigation.Goal()
            goal.source = RequestNavigation.Goal.SOURCE_GUEST_UI
            goal.waypoint_id = req.waypoint_id
            job_id = node.jobs.start_action("navigate", goal, f"帶位到 {req.waypoint_id}（迎賓頁）")
            job = node.jobs.wait_started(job_id, AUTHORIZE_WAIT_SEC)
            status = job.get("status")
            if status in ("pending", "running", "succeeded"):
                node.get_logger().info(f"🧭 迎賓頁帶位：{req.waypoint_id}")
                # ★ 10/27：按平板的人就站在車旁／車前，出發會擋住；約三成路徑先倒車，所以請客人站旁邊（同 LLM 帶位的說法）
                return {"success": True, "message": "好的，請跟我來。請先站到我旁邊，等我開動再跟上。", "job_id": job_id}
            return {"success": False, "message": job.get("message") or "目前沒辦法帶位，請洽櫃檯行員"}

        return await guard(node, work)

    return router
