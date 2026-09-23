"""路由共用的相依與小工具。"""

import asyncio
from fastapi import HTTPException, Depends, Header
from fastapi.responses import JSONResponse

from ..core.auth import token_from_header


def make_admin_only(node) -> list:
    """產生「這支端點要管理者登入」的相依串列。

    做成工廠而不是模組層級的函式，是因為它要綁到某一個節點的權杖表上；
    FastAPI 的 Depends 只認函式物件，所以把 node 包進閉包裡。
    """

    def require_admin(authorization: str = Header(default="")) -> None:
        """保護管理端點。權杖從 Authorization: Bearer <token> 取得"""
        if not node.auth.valid(token_from_header(authorization)):
            raise HTTPException(status_code=401, detail="需要管理者登入")

    return [Depends(require_admin)]


async def guard(node, work) -> JSONResponse:
    """把阻塞的服務呼叫丟到執行緒池，並把例外轉成前端看得懂的 JSON

    服務未就緒回 503（對應節點沒啟動），逾時回 504，其餘 500。
    前端只要看 success 欄位即可，不必解析錯誤字串。
    """
    try:
        payload = await asyncio.to_thread(work)
        return JSONResponse(payload)
    except TimeoutError as e:
        return JSONResponse({"success": False, "message": str(e)}, status_code=504)
    except RuntimeError as e:
        return JSONResponse({"success": False, "message": str(e)}, status_code=503)
    except Exception as e:  # noqa: BLE001 - 最後一道防線，不能讓例外冒到 ASGI 層
        node.get_logger().error(f"HTTP 處理錯誤: {e}")
        return JSONResponse({"success": False, "message": f"伺服器錯誤: {e}"}, status_code=500)
