"""對話與語音。

由 api.main 組裝。搬自 api/routes/chat.py，內容不變——這幾支端點本來就薄，
只是把使用者輸入轉發到對應的 ROS 話題。
"""

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse
from std_msgs.msg import Bool, Empty, String

from ...core.schemas.common import SayRequest


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    # ── 對話 ──────────────────────────────────────────

    @router.post("/api/say")
    async def api_say(req: SayRequest) -> JSONResponse:
        """把網頁打的字丟進 user_text，等同於對機器人講話"""
        text = req.text.strip()
        if not text:
            return JSONResponse({"success": False, "message": "內容不可為空"}, status_code=400)
        node.user_text_pub.publish(String(data=text))
        return JSONResponse({"success": True, "message": "已送出"})

    @router.post("/api/tts_state")
    async def api_tts_state(req: Request) -> JSONResponse:
        """平板回報 speechSynthesis 的起訖。★ 蓄意不需要登入：

        它不會讓車子動、不改任何狀態，只是把「我正在唸」告訴 ASR。
        要求登入反而會讓未登入的展示平板變成「機器人聽自己講話」。
        """
        try:
            body = await req.json()
        except Exception:
            body = {}
        active = bool(body.get("active"))
        node.playback_pub.publish(Bool(data=active))
        node.state.set_system(speaking=active)
        return JSONResponse({"ok": True, "active": active})

    @router.post("/api/speak")
    async def api_speak(req: SayRequest) -> JSONResponse:
        """跳過 LLM，直接讓機器人念出這段文字（測試 TTS 用）"""
        text = req.text.strip()
        if not text:
            return JSONResponse({"success": False, "message": "內容不可為空"}, status_code=400)
        node.speech_text_pub.publish(String(data=text))
        return JSONResponse({"success": True, "message": "已送出"})

    @router.post("/api/chat/clear")
    async def api_chat_clear() -> JSONResponse:
        """清除所有對話

        畫面上的對話紀錄與 LLM 的對話記憶要一起清，否則模型還會沿用
        上一位客戶的上下文回答下一位客戶。
        """
        node.state.clear_messages()
        node.clear_conversation_pub.publish(Empty())
        node.get_logger().info("🧹 已清除對話紀錄")
        return JSONResponse({"success": True, "message": "已清除對話"})

    return router
