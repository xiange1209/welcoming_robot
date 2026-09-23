"""使用者管理與註冊。

由 api.main 組裝。搬自 api/routes/users.py（GET /api/frame.jpg 已搬去
modules/imaging/router.py，跟其餘影像端點放一起）。
"""

import time

from fastapi import APIRouter
from fastapi.responses import JSONResponse

from smartnav_msgs.msg import UserType
from smartnav_msgs.srv import DeleteUser, ListUsers, RegisterFacePhoto, UpdateUser

from ...core.constants import MAX_PHOTOS, USER_TYPE_NAMES
from ..imaging.imaging import decode_photo
from ...core.schemas.common import RegisterPhotoRequest, RegisterRequest, UpdateUserRequest


from ...api.dependencies import guard


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    # ── 使用者管理 ────────────────────────────────────

    @router.get("/api/users", dependencies=admin_only)
    async def api_users() -> JSONResponse:
        def work():
            res = node.jobs.call_service("list_users", ListUsers.Request())
            users = [
                {
                    "user_uuid": u.user_uuid,
                    "user_name": u.user_name,
                    "user_type": int(u.user_type.type),
                    "user_type_name": USER_TYPE_NAMES.get(u.user_type.type, "GUEST"),
                    "description": u.description,
                    "created_at": u.created_at,
                    "num_samples": int(u.num_samples),
                }
                for u in res.users
            ]
            return {"success": bool(res.success), "message": res.message, "users": users}

        return await guard(node, work)

    @router.post("/api/users/register", dependencies=admin_only)
    async def api_register(req: RegisterRequest) -> JSONResponse:
        """立刻回應並在背景採樣

        不用 guard 阻塞 20～30 秒，是因為註冊進度要讓「所有」裝置看得到：
        狀態寫進 node.state 後由 WebSocket 廣播，發起註冊的那台平板
        並不特別。
        """
        name = req.user_name.strip()
        if not name:
            return JSONResponse({"success": False, "message": "請提供使用者名稱"}, status_code=400)
        if node.state.registration_active():
            return JSONResponse(
                {"success": False, "message": "已有註冊進行中，請等它結束"}, status_code=409
            )

        num_samples = max(1, int(req.num_samples))
        now = time.time()
        # 先擺一個 running 讓所有裝置立刻有反應；真正的進度與期限由
        # RegistrationRunner.run() 覆寫。
        node.state.set_registration(
            {
                "status": "running",
                "user_name": name,
                "num_samples": num_samples,
                "target": num_samples,
                "collected": 0,
                "started_at": now,
                "deadline": now + node.registration.REG_TIMEOUT_SEC,
                "message": f"正在為「{name}」採樣，請正對鏡頭",
            }
        )
        node.registration.start_background(name, int(req.user_type), req.description, num_samples)
        return JSONResponse({"success": True, "message": f"開始為「{name}」註冊"})

    @router.post("/api/users/register-photo", dependencies=admin_only)
    async def api_register_photo(req: RegisterPhotoRequest) -> JSONResponse:
        """用現成照片註冊。與即時採樣不同，這支是同步的，做完才回應"""
        name = req.user_name.strip()
        if not name:
            return JSONResponse({"success": False, "message": "請提供使用者名稱"}, status_code=400)
        if not req.photos:
            return JSONResponse({"success": False, "message": "請至少提供一張照片"}, status_code=400)
        if len(req.photos) > MAX_PHOTOS:
            return JSONResponse(
                {"success": False, "message": f"一次最多 {MAX_PHOTOS} 張照片"}, status_code=400
            )
        if node.state.registration_active():
            return JSONResponse(
                {"success": False, "message": "已有註冊進行中，請等它結束"}, status_code=409
            )

        try:
            photos = [decode_photo(p) for p in req.photos]
        except ValueError as e:
            return JSONResponse({"success": False, "message": str(e)}, status_code=400)

        def work():
            request = RegisterFacePhoto.Request()
            request.user_name = name
            request.user_type = UserType(type=int(req.user_type))
            request.description = req.description
            request.photos = photos
            # 每張照片都要跑一次 InsightFace，Pi 上不快，給比預設寬鬆的時限
            res = node.jobs.call_service(
                "register_face_photo", request, timeout=30.0 + 5.0 * len(photos)
            )
            return {
                "success": bool(res.success),
                "message": res.message,
                "accepted": int(res.accepted),
                "rejected": int(res.rejected),
            }

        return await guard(node, work)

    @router.put("/api/users/{user_uuid}", dependencies=admin_only)
    async def api_update_user(user_uuid: str, req: UpdateUserRequest) -> JSONResponse:
        def work():
            request = UpdateUser.Request()
            request.user_uuid = user_uuid
            request.user_name = req.user_name
            request.user_type = UserType(type=int(req.user_type))
            request.description = req.description
            res = node.jobs.call_service("update_user", request)
            return {"success": bool(res.success), "message": res.message}

        return await guard(node, work)

    @router.delete("/api/users/{user_uuid}", dependencies=admin_only)
    async def api_delete_user(user_uuid: str) -> JSONResponse:
        def work():
            request = DeleteUser.Request()
            request.user_uuid = user_uuid
            res = node.jobs.call_service("delete_user", request)
            return {"success": bool(res.success), "message": res.message}

        return await guard(node, work)

    return router
