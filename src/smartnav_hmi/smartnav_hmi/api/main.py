"""HTTP 介面組裝層"""

from pathlib import Path
from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from starlette.exceptions import HTTPException as StarletteHTTPException
from ament_index_python.packages import get_package_share_directory

from .dependencies import make_admin_only
from ..modules.chat import router as chat_router
from ..modules.hardware import router as hardware_router
from ..modules.imaging import router as imaging_router
from ..modules.jobs import router as jobs_router
from ..modules.navigation import router as navigation_router
from ..modules.realtime import router as realtime_router
from ..modules.session import router as session_router
from ..modules.stats import router as stats_router
from ..modules.system import router as system_router
from ..modules.system_control import router as system_control_router
from ..modules.users import router as users_router


def build_app(node) -> FastAPI:
    app = FastAPI(title="SmartNav HMI", docs_url=None, redoc_url=None)

    web_dir = Path(get_package_share_directory("smartnav_hmi")) / "frontend" / "dist"
    if (web_dir / "assets").is_dir():
        app.mount(
            "/assets",
            StaticFiles(directory=str(web_dir / "assets"), follow_symlink=True),
            name="assets",
        )

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def index() -> HTMLResponse:
        index_path = web_dir / "index.html"
        if not index_path.exists():
            return HTMLResponse(
                "<h1>找不到 index.html</h1>",
                status_code=500,
            )

        return HTMLResponse(
            index_path.read_text(encoding="utf-8"),
            headers={"Cache-Control": "no-store, must-revalidate"},
        )

    @app.get("/favicon.svg", include_in_schema=False)
    async def favicon():
        favicon_path = web_dir / "favicon.svg"
        if favicon_path.is_file():
            return FileResponse(favicon_path)
        raise StarletteHTTPException(status_code=404, detail="Favicon not found")

    @app.middleware("http")
    async def track_client(request: Request, call_next):
        """全域更新用戶端活動狀態，避免個別端點漏呼叫導致 ROS 訂閱被誤關"""
        node.note_client()
        return await call_next(request)

    @app.exception_handler(StarletteHTTPException)
    async def http_error(_request: Request, exc: StarletteHTTPException) -> JSONResponse:
        """前端只認 success/message 兩個欄位，別讓 FastAPI 丟原生的 detail"""
        return JSONResponse({"success": False, "message": str(exc.detail)}, status_code=exc.status_code)

    admin_only = make_admin_only(node)

    app.include_router(session_router.build_router(node, admin_only))
    app.include_router(realtime_router.build_router(node, admin_only))
    app.include_router(imaging_router.build_router(node, admin_only))
    app.include_router(hardware_router.build_router(node, admin_only))
    app.include_router(stats_router.build_router(node, admin_only))
    app.include_router(system_router.build_router(node, admin_only))
    app.include_router(system_control_router.build_router(node, admin_only))
    app.include_router(navigation_router.build_router(node, admin_only))
    app.include_router(jobs_router.build_router(node, admin_only))
    app.include_router(users_router.build_router(node, admin_only))
    app.include_router(chat_router.build_router(node, admin_only))

    return app
