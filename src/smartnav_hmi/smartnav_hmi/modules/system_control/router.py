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
import subprocess
import threading
import time

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

    # ── 電源（★ 2026-10-22）──────────────────────────
    # 展示收尾不用再 SSH：先把所有單元依序關掉（同「全部關閉」情境），再交給 systemd。
    # 立刻回 202、在背景做——單元一個一個優雅關閉要十幾秒，HTTP 不能等那麼久。
    # dry_run=true 只回報會做什麼（測試用）。HMI 服務跑在 user 底下，靠 sudo -n（免密碼）。

    @router.post("/api/power/{action}", dependencies=admin_only)
    async def api_power(action: str, dry_run: bool = False) -> JSONResponse:
        if action not in ("poweroff", "reboot"):
            return JSONResponse({"success": False, "message": "只接受 poweroff 或 reboot"}, status_code=400)
        cmd = ["sudo", "-n", "/usr/bin/systemctl", action]
        label = "關機" if action == "poweroff" else "重新開機"
        if dry_run:
            return JSONResponse({"success": True, "dry_run": True,
                                 "message": f"（測試）會先關閉所有單元，再執行 {' '.join(cmd)}"})

        def work():
            node.get_logger().warn(f"⏻ 管理者要求{label}：先關閉所有單元")
            ok, steps = node.system_control.scenario_apply("all_off")
            for s in steps:
                node.get_logger().info(f"  {s}")
            node.get_logger().warn(f"⏻ 單元關閉{'完成' if ok else '有失敗（照樣繼續）'}，執行 {' '.join(cmd)}")
            time.sleep(1.0)
            subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        threading.Thread(target=work, daemon=True).start()
        return JSONResponse({"success": True, "message": f"正在關閉所有功能，接著{label}"}, status_code=202)

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
