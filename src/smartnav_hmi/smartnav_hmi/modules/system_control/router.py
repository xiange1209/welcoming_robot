"""節點開關與測試情境。

由 api.main 組裝。/api/system/... 搬自 api/routes/system.py，內容不變。

★ /api/scenarios 與 /api/scenarios/{key} 是搬遷時「順手修好」的一個既有
  bug：SystemControlManager.scenario_list()/scenario_apply()（測試情境，
  2026-09-18 新增）本身邏輯完好，但唯一曾經掛過這兩支端點的地方是
  hmi_server_node.py 裡從未被呼叫的舊版 _build_app()——也就是說這個功能
  自從加進去就沒有真的在線上出現過，前端也從未呼叫過 /api/scenarios。
  這裡把它接回真正在用的 router，恢復成可用狀態。
"""

from fastapi import APIRouter
from fastapi.responses import JSONResponse


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    # ── 系統節點開關 ─────────────────────────────────

    @router.get("/api/system/status", dependencies=admin_only)
    async def api_system_status() -> JSONResponse:
        return JSONResponse({"units": node.system_control.system_status()})

    @router.get("/api/system/operations/{operation_id}", dependencies=admin_only)
    async def api_system_operation(operation_id: str) -> JSONResponse:
        operation = node.system_control.operation(operation_id)
        if operation is None:
            return JSONResponse({"success": False, "message": "找不到操作"}, status_code=404)
        return JSONResponse({"success": True, "operation": operation})

    @router.post("/api/system/{unit}/{action:path}", dependencies=admin_only)
    async def api_system_control(unit: str, action: str) -> JSONResponse:
        # action 可能是 "start"、"stop"，或 "start:mapping" 這種帶啟動方式的形式。
        # 用 {action:path} 而不是 {action}，冒號才不會被路由切掉。
        ok, result = node.system_control.system_control(unit, action)
        return JSONResponse(
            {"success": ok, **result}, status_code=202 if ok else 400
        )

    # ── 測試情境 ─────────────────────────────────────
    # 用 /api/scenarios 而不是 /api/system/scenario/... ——上面那條是
    # {unit}/{action:path} 的萬用路由，會把 scenario 吃掉當成單元名。

    @router.get("/api/scenarios", dependencies=admin_only)
    async def api_scenarios() -> JSONResponse:
        return JSONResponse({"scenarios": node.system_control.scenario_list()})

    @router.post("/api/scenarios/{key}", dependencies=admin_only)
    async def api_scenario_apply(key: str) -> JSONResponse:
        ok, result = node.system_control.scenario_request(key)
        return JSONResponse({"success": ok, **result}, status_code=202 if ok else 400)

    return router
