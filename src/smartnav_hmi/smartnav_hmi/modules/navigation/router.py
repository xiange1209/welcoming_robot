"""導航總彙：地圖切換/建圖、地點/導航、教導路徑、遙控與緊急停止。

由 api.main 組裝。原本分散在 api/routes/maps.py、waypoints.py、paths.py、
teleop.py 四個檔案，搬過來合成一個 modules/navigation/ 業務模組——它們都在
操作「讓車子移動」的來源，緊急停止（/api/estop）正是要一次停掉全部。
"""

from fastapi import APIRouter
from fastapi.responses import JSONResponse

from ...core.schemas.common import RearMaskRequest, TeleopRequest
from . import maps as maps_routes
from . import paths as paths_routes
from . import waypoints as waypoints_routes


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    # ── 遙控建圖 ─────────────────────────────────────

    @router.post("/api/teleop", dependencies=admin_only)
    async def api_teleop(req: TeleopRequest) -> JSONResponse:
        """收一次遙控指令。前端要持續送，停止送 = 車子停。

        刻意不做成「按下去開始、放開才停」的狀態機：那樣一旦放開的那則
        請求沒送達（手機遙控很常見），車子就會一直跑下去。
        改成前端持續心跳、後端逾時自動歸零，斷線就是最安全的狀態。
        """
        node.teleop.apply(req.linear, req.angular)
        return JSONResponse({"success": True})

    @router.post("/api/teleop/stop", dependencies=admin_only)
    async def api_teleop_stop() -> JSONResponse:
        """明確的停止。看門狗本來就會停，這條只是讓停止更即時。"""
        node.teleop.apply(0.0, 0.0)
        return JSONResponse({"success": True, "message": "已停止"})

    # ★★ 拿掉 admin_only —— 緊急停止不可以需要登入。★★
    #
    # 其他端點保護的是「不要讓路人亂開車」，這一顆的作用是**讓車停下來**。
    # 登入 12 小時後過期，而展示當天平板可能整天開著 —— 真的要急停時
    # 跳出登入框，是最糟的失敗方式。濫用風險評估過：最壞情況是有人惡意
    # 讓車停下來，那正是安全的方向，對照「該停停不下來」完全值得。
    @router.post("/api/estop")
    async def api_estop() -> JSONResponse:
        """全域緊急停止：**一個動作停下所有會讓車子移動的來源**

        先前唯一的停止鍵在遙控頁，而且只停遙控——導航中或重播教導路徑
        時按它完全沒用，操作者得先切到地圖分頁、找到作業清單、按取消，
        而那期間車子還在開。展示時評審站在旁邊，這個延遲不能接受。

        這裡一次處理三個來源，而且**順序有意義**：
          1. 先送零速（最快讓輪子停下，不必等任何服務回應）
          2. 再取消所有進行中的作業（導航、建圖、教導路徑重播）
          3. 最後回報哪些被取消了

        即使取消服務沒回應，第 1 步的零速加上遙控看門狗（0.6 秒）
        也會讓車停住——安全動作不可以依賴任何一個會失敗的呼叫。
        """
        node.teleop.apply(0.0, 0.0)

        cancelled, failed = [], []
        for job in node.state.jobs_copy():
            if job.get("status") in ("running", "pending"):
                jid = job.get("job_id", "")
                (cancelled if node.jobs.cancel_job(jid) else failed).append(
                    job.get("label") or jid
                )

        # ★ 登記簿之外的目標也要停（語音發起的導航就在這裡）
        broadcast = node.jobs.cancel_all_action_goals()

        node.get_logger().warn(
            f"緊急停止：已送零速，取消 {len(cancelled)} 個作業"
            + (f"，另對 {len(broadcast)} 個動作送出取消全部（{'、'.join(broadcast)}）" if broadcast else "")
            + (f"，{len(failed)} 個取消失敗" if failed else "")
        )
        if cancelled:
            msg = "已緊急停止，並取消：" + "、".join(cancelled)
        else:
            msg = "已緊急停止（當時沒有進行中的作業）"
        if failed:
            msg += f"　⚠ 這些取消失敗：{'、'.join(failed)}"
        return JSONResponse({"success": True, "message": msg,
                             "cancelled": cancelled, "failed": failed})

    @router.post("/api/teleop/rearmask", dependencies=admin_only)
    async def api_teleop_rearmask(req: RearMaskRequest) -> JSONResponse:
        """開關雷達後方扇形遮罩。

        遙控建圖時操作者常常走在車子後方，會被雷達掃進去變成移動的假障礙，
        污染地圖也干擾 scan matching。lslidar 驅動本身支援
        angle_disable_min/max（單位 0.01 度），直接在驅動裡裁掉最便宜。
        """
        ok, msg = node.teleop.set_lidar_rear_mask(req.enabled, req.half_angle_deg)
        return JSONResponse({"success": ok, "message": msg}, status_code=200 if ok else 500)

    # ── 地圖 / 地點 / 教導路徑 ─────────────────────────

    router.include_router(maps_routes.build_router(node, admin_only))
    router.include_router(waypoints_routes.build_router(node, admin_only))
    router.include_router(paths_routes.build_router(node, admin_only))

    return router
