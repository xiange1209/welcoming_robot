"""系統健康檢查。

由 api.main 組裝。原本是 api/routes/pages.py 的 /api/health，搬過來與
node_status()／resource_tick() 放在一起，理由都是「系統/節點資訊」。
"""

import asyncio
import socket

from fastapi import APIRouter
from fastapi.responses import JSONResponse


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    @router.get("/api/health")
    async def api_health() -> JSONResponse:
        has_frame = node.video.latest_jpeg() is not None
        # 影像訂閱按需建立之後，「手上沒有影格」不再等於「相機沒影像」——
        # 沒人在看的時候本來就沒有。改問話題上有沒有發布者，這其實比
        # 原本的判斷更準：它回答的是「相機節點在不在發」。
        if not has_frame:
            has_frame = node.count_publishers(node.image_topic) > 0
        has_map = node.map_render.latest_png() is not None
        services = {name: c.service_is_ready() for name, c in node.service_clients.items()}
        actions = {name: c.server_is_ready() for name, c in node.action_clients.items()}
        # 查 lifecycle 會阻塞，丟去執行緒池免得卡住事件迴圈（影像串流也在上面跑）
        nodes = await asyncio.to_thread(node.health.node_status)
        return JSONResponse(
            {
                "ok": True,
                "node": node.get_name(),
                "urls": node.access_urls(),
                "camera": has_frame,
                "map": has_map,
                "services": services,
                "actions": actions,
                "nodes": nodes,
                "host": socket.gethostname(),
            }
        )

    return router
