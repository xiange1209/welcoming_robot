"""節點開關與測試情境。

由 api.main 組裝。/api/system/... 搬自 api/routes/system.py，內容不變。

★ /api/scenarios 與 /api/scenarios/{key} 是搬遷時「順手修好」的一個既有
  bug：SystemControlManager.scenario_list()/scenario_apply()（測試情境，
  2026-09-18 新增）本身邏輯完好，但唯一曾經掛過這兩支端點的地方是
  hmi_server_node.py 裡從未被呼叫的舊版 _build_app()——也就是說這個功能
  自從加進去就沒有真的在線上出現過，前端也從未呼叫過 /api/scenarios。
  這裡把它接回真正在用的 router，恢復成可用狀態。
"""

import asyncio

from fastapi import APIRouter
from fastapi.responses import JSONResponse


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    # ── 系統節點開關 ─────────────────────────────────

    @router.get("/api/system/status", dependencies=admin_only)
    async def api_system_status() -> JSONResponse:
        return JSONResponse({"units": node.system_control.system_status()})

    @router.post("/api/system/{unit}/{action:path}", dependencies=admin_only)
    async def api_system_control(unit: str, action: str) -> JSONResponse:
        # action 可能是 "start"、"stop"，或 "start:mapping" 這種帶啟動方式的形式。
        # 用 {action:path} 而不是 {action}，冒號才不會被路由切掉。
        ok, msg = node.system_control.system_control(unit, action)
        return JSONResponse({"success": ok, "message": msg}, status_code=200 if ok else 400)

    # ── 測試情境 ─────────────────────────────────────
    # 用 /api/scenarios 而不是 /api/system/scenario/... ——上面那條是
    # {unit}/{action:path} 的萬用路由，會把 scenario 吃掉當成單元名。

    @router.get("/api/scenarios", dependencies=admin_only)
    async def api_scenarios() -> JSONResponse:
        return JSONResponse({"scenarios": node.system_control.scenario_list()})

    @router.post("/api/scenarios/{key}", dependencies=admin_only)
    async def api_scenario_apply(key: str) -> JSONResponse:
        # ★ 一定要丟到執行緒：停止走的是 subprocess.run(timeout=60)，
        #   一個情境最多停 6 個單元 = 最壞 6 分鐘。在事件迴圈裡跑會把整個 HMI 凍住，
        #   而平板那端看起來就像網頁當掉。
        ok, steps = await asyncio.to_thread(node.system_control.scenario_apply, key)
        return JSONResponse(
            {"success": ok, "steps": steps, "message": "\n".join(steps)},
            status_code=200 if ok else 400,
        )

    # ── 一鍵驗證（2026-09-24）────────────────────────
    # 同樣避開 /api/system/{unit}/{action:path} 萬用路由，用獨立前綴。

    @router.get("/api/verify", dependencies=admin_only)
    async def api_verify_list() -> JSONResponse:
        return JSONResponse({"items": node.system_control.verify_list()})

    @router.post("/api/verify/{key}", dependencies=admin_only)
    async def api_verify_run(key: str) -> JSONResponse:
        # 一樣丟執行緒——V1 量幀率就要 12 秒，在事件迴圈裡跑會把 HMI 凍住
        ok, out = await asyncio.to_thread(node.system_control.verify_run, key)
        # 前端的 api() 在非 2xx 時會把 message 跳成 toast。沒給的話畫面只會出現
        # 「機器人回報錯誤 (HTTP 400)」，看不出是哪裡壞。短的錯誤直接給，
        # 長的（整段腳本輸出）就指回下方輸出框。
        msg = "" if ok else (out if len(out) < 200 and "\n" not in out else "驗證腳本回報失敗，詳見下方輸出")
        return JSONResponse(
            {"success": ok, "output": out, "message": msg},
            status_code=200 if ok else 400,
        )

    return router
