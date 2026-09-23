"""到訪統計。

由 api.main 組裝。原本是 api/routes/media.py 的 /api/stats。
"""

import asyncio

from fastapi import APIRouter
from fastapi.responses import JSONResponse

from .visit_stats import read_visit_stats


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    @router.get("/api/stats")
    async def api_stats(days: int = 7) -> JSONResponse:
        """到訪統計。★ 與 /api/hardware 一樣刻意不要求登入 ——
        這是展示時要投影出來的頁面，登入畫面會打斷展示節奏；
        而且內容只有姓名與時間，沒有比迎賓頁的即時影像更敏感。

        ★ sqlite 查詢是阻塞 I/O，一定要丟到 executor。
          直接在協程裡查會卡住整個事件迴圈，連即時影像都會停格。
        """
        days = max(1, min(31, int(days)))
        data = await asyncio.get_running_loop().run_in_executor(
            None, read_visit_stats, days, node.visit_log_path)
        return JSONResponse(data, headers={"Cache-Control": "no-store"})

    return router
