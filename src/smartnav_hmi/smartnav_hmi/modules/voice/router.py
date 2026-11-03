"""語音設定 API 與平板麥克風串流（★ 2026-11-03）。

  GET  /api/voice/settings   公開：迎賓頁要知道該不該唸、要不要顯示麥克風鈕
  POST /api/voice/settings   要登入：會改變整台機器人的行為
  WS   /ws/mic               公開（跟 /api/say 一樣：迎賓平板通常沒登入）：
                             平板送 16 kHz、單聲道、pcm_s16le 的二進位音訊塊，
                             轉成 smartnav_msgs/AudioData 發到 remote_mic_audio。
                             只在 input=browser 時收；同一時間只收一台（新連線取代舊的）。
"""

import asyncio
import itertools
import json

from fastapi import APIRouter, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

# 一塊最多 1 秒（16000 取樣 × 2 bytes）。正常是 0.1 秒一塊，超過代表前端送錯格式
MAX_CHUNK_BYTES = 32000
_ids = itertools.count(1)


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    @router.get("/api/voice/settings")
    async def api_voice_settings() -> JSONResponse:
        return JSONResponse({"success": True, **node.voice_payload()})

    @router.post("/api/voice/settings", dependencies=admin_only)
    async def api_voice_settings_set(req: Request) -> JSONResponse:
        try:
            body = await req.json()
        except Exception:
            body = {}
        try:
            data = node.set_voice_modes(output=body.get("output"), input_=body.get("input"))
        except ValueError as exc:
            return JSONResponse({"success": False, "message": str(exc)}, status_code=400)
        return JSONResponse({"success": True, **data})

    @router.post("/api/voice/robot_tts")
    async def api_robot_tts(req: Request) -> JSONResponse:
        """機器人朗讀開關。★ 蓄意不需要登入（跟平板的朗讀鈕一樣，迎賓平板通常沒登入）：
        只能讓機器人閉嘴或開口，不能讓它講任何指定的話，也不會讓車子動。"""
        try:
            body = await req.json()
        except Exception:
            body = {}
        on = body.get("on")
        if not isinstance(on, bool):
            return JSONResponse({"success": False, "message": "on 要是 true / false"}, status_code=400)
        data = node.set_voice_modes(robot_tts=on)
        return JSONResponse({"success": True, **data})

    @router.websocket("/ws/voice_monitor")
    async def ws_voice_monitor(websocket: WebSocket) -> None:
        """機器人端收到的語音輸入波形（★ 2026-11-03，語音頁的測試用）。

        每塊約 32 ms 一個點：[seq, 峰值 0~1, VAD 判定有聲 0/1, 正在收句 0/1]。不管來源是
        機器人麥克風還是平板錄音都會經過 voice_trigger，所以看得出「平板有送、機器人有沒有收到」。
        唯讀、不含音訊內容（只有音量），所以跟 /ws 一樣不需要登入。
        """
        await websocket.accept()
        last = 0
        closed = asyncio.Event()

        async def watch() -> None:
            try:
                while True:
                    if (await websocket.receive()).get("type") == "websocket.disconnect":
                        break
            finally:
                closed.set()

        watcher = asyncio.ensure_future(watch())
        try:
            while not closed.is_set():
                pts = node.voice_monitor_since(last)
                if pts:
                    last = pts[-1][0]
                    await websocket.send_text(json.dumps({"pts": [[round(p, 3) if isinstance(p, float) else p
                                                                   for p in m] for m in pts]}))
                await asyncio.sleep(0.1)
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            watcher.cancel()

    @router.websocket("/ws/mic")
    async def ws_mic(websocket: WebSocket) -> None:
        await websocket.accept()
        if node.voice.get()["input"] != "browser":
            await websocket.close(code=4003, reason="voice input is not browser")
            return
        my_id = next(_ids)
        old = node.claim_mic(my_id, websocket)
        if old is not None:
            # 新的平板取代舊的：同時收兩支麥克風會把兩段話混在同一個 VAD 裡
            try:
                await old.close(code=4001, reason="replaced by another tablet")
            except Exception:
                pass
        node.get_logger().info(f"平板麥克風已連線（#{my_id}）")
        try:
            while True:
                message = await websocket.receive()
                if message.get("type") == "websocket.disconnect":
                    break
                if not node.mic_owner_is(my_id):
                    break                              # 被別台取代了
                if node.voice.get()["input"] != "browser":
                    await websocket.close(code=4003, reason="voice input changed")
                    break
                data = message.get("bytes")
                if not data:
                    continue                           # 文字訊息（例如前端心跳）忽略
                if len(data) > MAX_CHUNK_BYTES or len(data) % 2:
                    continue
                await asyncio.to_thread(node.publish_remote_mic, data)
        except WebSocketDisconnect:
            pass
        except Exception as exc:  # noqa: BLE001
            node.get_logger().debug(f"/ws/mic 結束：{exc}")
        finally:
            node.release_mic(my_id)
            node.get_logger().info(f"平板麥克風已斷線（#{my_id}）")

    return router
