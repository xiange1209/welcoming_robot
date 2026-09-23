"""影像串流、地圖 PNG、拍照。

由 api.main 組裝。原本分散在 api/routes/media.py（/video, /api/map.png,
/api/map/meta, /api/map/watch）與 api/routes/users.py（/api/frame.jpg），
搬過來與影像/地圖渲染邏輯放在一起。
"""

import asyncio
import time

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse, Response, StreamingResponse


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    @router.get("/video")
    async def video(request: Request) -> StreamingResponse:
        return StreamingResponse(
            node.video.mjpeg_generator(request),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @router.get("/api/frame.jpg")
    async def api_frame() -> Response:
        """抓當下這一張影格。前端「從畫面拍照」用這支

        影像訂閱是按需建立的，所以這裡要先舉手說「我要影格」，再給它一點時間
        把訂閱建起來、收到第一張。正常情況下拍照時 /video 已經開著，這段等待
        會直接跳過。
        """
        node.video.note_interest()
        deadline = time.monotonic() + 2.0
        while True:
            jpeg = node.video.latest_jpeg()
            if jpeg is not None or time.monotonic() >= deadline:
                break
            await asyncio.sleep(0.1)
        if jpeg is None:
            return JSONResponse({"success": False, "message": "目前沒有相機影像"}, status_code=404)
        return Response(content=jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @router.get("/api/map.png")
    async def api_map_png() -> Response:
        node.pose_tracker.note_watch()
        png = node.map_render.latest_png()
        if png is None:
            return JSONResponse({"error": "尚未收到地圖"}, status_code=404)
        # 前端用 ?v=<version> 破快取，這裡明確禁止快取避免拿到舊圖
        return Response(content=png, media_type="image/png", headers={"Cache-Control": "no-store"})

    @router.get("/api/map/meta")
    async def api_map_meta() -> JSONResponse:
        node.pose_tracker.note_watch()
        meta = node.state.map_meta_copy()
        if meta is None:
            return JSONResponse({"error": "尚未收到地圖"}, status_code=404)
        return JSONResponse(meta)

    @router.post("/api/map/watch")
    async def api_map_watch() -> JSONResponse:
        """地圖分頁的心跳

        位姿訂閱（amcl_pose／TF）與地圖 PNG 渲染都很貴，而且只有地圖分頁
        看得到。前端在地圖分頁時每 5 秒打一次這支，後端才開這些來源；
        切走或關掉頁面就不再有心跳，十幾秒後自動全部收掉。

        刻意不設成管理端點：它不吐任何資料，只是一個「有人在看」的訊號，
        要求權杖只會讓沒登入的頁面看不到地圖更新。
        """
        node.pose_tracker.note_watch()
        return JSONResponse({"success": True})

    return router
