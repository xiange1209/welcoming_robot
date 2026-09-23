"""狀態快照與 WebSocket 推播。

由 api.main 組裝。搬自 api/routes/pages.py（GET / 留在 api/main.py 跟靜態檔
掛載放一起；GET /api/health 搬去 modules/system/router.py），這裡剩下
/api/state 與 /ws。
"""

import asyncio
import json

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    @router.get("/api/state")
    async def api_state() -> JSONResponse:
        return JSONResponse(node.state.snapshot())

    @router.websocket("/ws")
    async def ws_endpoint(websocket: WebSocket) -> None:
        """狀態推播

        ★ 加上「斷線就收掉」。原本這個迴圈只有在 send 失敗時才會結束，而 send
          只在狀態版本變動時才發生——狀態不動的期間（迎賓機閒置時的常態）
          連線已經死了也沒人知道，迴圈就以 10 Hz 空轉下去，而且沒有上限：
          平板每重新整理一次就多留一條。

          兩道防線：
          1. **協定層 ping/pong**（uvicorn 的 ws_ping_interval/timeout，
             見 hmi_server_node._run_server）。沒有 pong 就由伺服器主動關閉連線。
          2. 這裡的 receive 監看任務。連線一被關閉（不論是對方關的、
             還是第 1 條判死的），它馬上收到 websocket.disconnect，
             推播迴圈下一輪就結束——不必等到有東西要送。
        """
        await websocket.accept()
        node.note_client()
        with node._client_lock:
            node._ws_clients += 1
        last_version = -1
        last_messages_version = None  # None 代表第一次，會帶上完整對話

        async def watch_disconnect() -> None:
            """只為了偵測斷線而收訊息。前端目前不送任何東西，這是刻意的：
            能不能活由協定層的 pong 決定，不依賴前端要記得送心跳
            （JS 計時器在背景分頁會被節流，拿它當存活判準會誤踢）。"""
            while True:
                message = await websocket.receive()
                if message.get("type") == "websocket.disconnect":
                    return
                # 收到什麼都算一次「還活著」，以後前端要加心跳也不必改這裡
                node.note_client()

        watcher = asyncio.ensure_future(watch_disconnect())
        try:
            while not watcher.done():
                current = node.state.version
                if current != last_version:
                    last_version = current
                    payload = node.state.snapshot(since_messages_version=last_messages_version)
                    last_messages_version = payload["messages_version"]
                    await websocket.send_text(json.dumps(payload, ensure_ascii=False))
                await asyncio.sleep(0.1)
        except WebSocketDisconnect:
            pass
        except Exception as e:
            node.get_logger().debug(f"WebSocket 結束: {e}")
        finally:
            watcher.cancel()
            with node._client_lock:
                node._ws_clients = max(0, node._ws_clients - 1)

    return router
