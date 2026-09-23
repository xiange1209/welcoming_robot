"""周邊硬體偵測。

由 api.main 組裝。原本是 api/routes/media.py 的 /api/hardware。

★ 搬遷時順便修掉一個既有 bug：原本呼叫 `node.hardware_status`，但
  HmiServerNode 從未定義過這個方法（建構子只建了 `self._hardware =
  HardwareProbe(...)`），這支端點原本一定會丟 AttributeError -> 500。
  現在改呼叫 node.hardware.status()（見 hmi_server_node.py 把
  HardwareProbe 實例掛在 self.hardware 底下）。
"""

import asyncio

from fastapi import APIRouter
from fastapi.responses import JSONResponse


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    @router.get("/api/hardware")
    async def api_hardware() -> JSONResponse:
        """周邊硬體偵測。★ 刻意不要求登入 ——
        這是純唯讀的診斷資訊，而「東西壞了看不出來」比「被人看到有幾個 USB」嚴重。"""
        items = await asyncio.get_running_loop().run_in_executor(None, node.hardware.status)
        return JSONResponse({"success": True, "items": items})

    return router
