"""地圖管理（切換、建圖、刪除、狀態）。

由 modules/navigation/router.py 組裝。搬自 api/routes/maps.py，邏輯本就薄
（大多是轉發 ROS service/action），只把 node._call_service/_start_action
改成 node.jobs.call_service/start_action，node._query_nav_mode 改成
node.system_control.query_nav_mode()。
"""

from fastapi import APIRouter
from fastapi.responses import JSONResponse
from std_srvs.srv import Trigger

from smartnav_msgs.action import CreateMap
from smartnav_msgs.srv import DeleteMap, ListMaps, SwitchMap

from ...core.schemas.common import CreateMapRequest, SwitchMapRequest
from ...api.dependencies import guard


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    # ── 地圖管理 ──────────────────────────────────────

    @router.get("/api/maps", dependencies=admin_only)
    async def api_maps() -> JSONResponse:
        def work():
            res = node.jobs.call_service("list_maps", ListMaps.Request())
            return {
                "success": bool(res.success),
                "message": res.message,
                "maps": [{"map_id": m.map_id, "map_name": m.map_name} for m in res.maps_info],
            }

        return await guard(node, work)

    @router.post("/api/maps/switch", dependencies=admin_only)
    async def api_switch_map(req: SwitchMapRequest) -> JSONResponse:
        def work():
            request = SwitchMap.Request()
            request.map_id = req.map_id
            res = node.jobs.call_service("switch_map", request)
            return {"success": bool(res.success), "message": res.message}

        return await guard(node, work)

    @router.post("/api/maps/create", dependencies=admin_only)
    async def api_create_map(req: CreateMapRequest) -> JSONResponse:
        if not req.map_name.strip():
            return JSONResponse({"success": False, "message": "請提供地圖名稱"}, status_code=400)
        goal = CreateMap.Goal()
        goal.map_name = req.map_name.strip()
        job_id = node.jobs.start_action("create_map", goal, f"建立地圖「{goal.map_name}」")
        return JSONResponse({"success": True, "message": "建圖已開始", "job_id": job_id})

    # ── 地點與導航 ────────────────────────────────────

    @router.post("/api/maps/finish", dependencies=admin_only)
    async def api_finish_map() -> JSONResponse:
        """結束建圖並存檔（遙控建圖用）

        自動探索模式下 map_service_cc 會自己在收到 /exploration_complete
        之後存檔；遙控建圖沒有那個事件，要由操作者按下按鈕才知道走完了。
        """
        client = node.service_clients.get("finish_map")
        if client is None or not client.wait_for_service(timeout_sec=3.0):
            return JSONResponse(
                {"success": False, "message": "/finish_map 服務不存在（導航堆疊沒啟動？）"},
                status_code=503,
            )
        try:
            res = node.jobs.await_future(client.call_async(Trigger.Request()), timeout=30.0)
        except Exception as exc:  # noqa: BLE001
            return JSONResponse({"success": False, "message": f"存檔失敗：{exc}"}, status_code=500)
        ok = bool(res and res.success)
        return JSONResponse(
            {"success": ok, "message": (res.message if res else "無回應")},
            status_code=200 if ok else 500,
        )

    @router.delete("/api/maps/{map_id}", dependencies=admin_only)
    async def api_delete_map(map_id: str) -> JSONResponse:
        """刪除地圖（建壞的、建到一半失敗的都可以刪）

        安全檢查刻意放在 map_service_cc 那端而不是這裡：
        它才知道現在是不是在建圖、目前載入的是哪張。
        HMI 這層只負責轉發與呈現結果。
        """
        client = node.service_clients.get("delete_map")
        if client is None or not client.wait_for_service(timeout_sec=3.0):
            return JSONResponse(
                {"success": False, "message": "delete_map 服務不存在（導航堆疊沒啟動？）"},
                status_code=503,
            )
        req = DeleteMap.Request()
        req.map_id = map_id
        try:
            res = node.jobs.await_future(client.call_async(req), timeout=15.0)
        except Exception as exc:  # noqa: BLE001
            return JSONResponse({"success": False, "message": f"刪除失敗：{exc}"}, status_code=500)
        ok = bool(res and res.success)
        return JSONResponse(
            {"success": ok, "message": (res.message if res else "無回應")},
            status_code=200 if ok else 409,
        )

    @router.get("/api/maps/status", dependencies=admin_only)
    async def api_map_status() -> JSONResponse:
        """建圖頁面用的綜合狀態

        把「操作者想知道的事」湊成一包，前端只要打這一支就好：
        現在是建圖還是定位、載入哪張圖、圖多大、建了多少格、有沒有建圖作業在跑。
        """
        meta = node.state.map_meta_copy()
        mode = node.system_control.query_nav_mode()
        jobs = node.state.jobs_copy()
        # 欄位名是 "action" 不是 "kind"（jobs_copy() 產出的結構）。
        # 之前寫成 kind，篩選永遠回 None：畫面上「建圖作業」一直顯示
        # 「尚未開始」，使用者連按三次都失敗卻毫不知情，走完才發現存不了。
        mapping_job = next(
            (j for j in jobs if j.get("action") == "create_map" and j.get("status") == "running"),
            None,
        )
        # 沒有進行中的作業時，把最近一次失敗帶出去，讓畫面能講出原因，
        # 而不是只顯示「尚未開始」——失敗和沒開始是兩件完全不同的事。
        last_failed = None
        if mapping_job is None:
            fails = [j for j in jobs
                     if j.get("action") == "create_map" and j.get("status") == "failed"]
            if fails:
                last_failed = max(fails, key=lambda j: j.get("started_at") or 0)
        known = None
        if meta and meta.get("known_cells") is not None:
            known = meta.get("known_cells")
        return JSONResponse(
            {
                "mode": mode,                       # mapping / localization / unknown
                "current_map": (node.state.snapshot().get("system") or {}).get("map_id"),
                "mapping_job": mapping_job,         # None 表示沒有建圖作業在跑
                "last_failed_job": last_failed,     # 最近一次失敗，讓畫面講得出原因
                "map_meta": meta,                   # 寬高、解析度、原點
                "known_cells": known,
            }
        )

    return router
