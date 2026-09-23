"""管理者登入。

由 api.main 組裝。搬自 api/routes/session.py，內容不變。
"""

from fastapi import APIRouter, Header
from fastapi.responses import JSONResponse

from ...core.auth import token_from_header
from ...core.schemas.common import LoginRequest


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    @router.post("/api/login")
    async def api_login(req: LoginRequest) -> JSONResponse:
        if not node.auth.check(req.username, req.password):
            node.get_logger().warn(f"管理者登入失敗（帳號 {req.username!r}）")
            return JSONResponse(
                {"success": False, "message": "帳號或密碼錯誤"}, status_code=401
            )
        token = node.auth.issue()
        node.get_logger().info("管理者登入成功")
        return JSONResponse(
            {
                "success": True,
                "message": "登入成功",
                "token": token,
                "expires_in": node.admin_session_sec,
            }
        )

    @router.post("/api/logout")
    async def api_logout(authorization: str = Header(default="")) -> JSONResponse:
        token = token_from_header(authorization)
        node.auth.revoke(token)
        return JSONResponse({"success": True, "message": "已登出"})

    @router.get("/api/session")
    async def api_session(authorization: str = Header(default="")) -> JSONResponse:
        """前端重新整理後用這支確認手上的權杖還有效"""
        token = token_from_header(authorization)
        return JSONResponse({"success": True, "admin": node.auth.valid(token)})

    return router
