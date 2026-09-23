"""長時間動作的作業清單。

由 api.main 組裝。原本是 api/routes/jobs.py。
"""

from fastapi import APIRouter
from fastapi.responses import JSONResponse


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    # ── 作業（長時間動作）─────────────────────────────

    @router.get("/api/jobs", dependencies=admin_only)
    async def api_jobs() -> JSONResponse:
        return JSONResponse({"jobs": node.state.jobs_copy()})

    @router.delete("/api/jobs/{job_id}", dependencies=admin_only)
    async def api_cancel_job(job_id: str) -> JSONResponse:
        ok = node.jobs.cancel_job(job_id)
        return JSONResponse(
            {"success": ok, "message": "已送出取消要求" if ok else "找不到進行中的作業"},
            status_code=200 if ok else 404,
        )

    return router
