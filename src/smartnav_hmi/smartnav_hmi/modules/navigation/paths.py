"""教導-重現路徑。

由 modules/navigation/router.py 組裝。搬自 api/routes/paths.py。
"""

import asyncio
import math

from fastapi import APIRouter
from fastapi.responses import JSONResponse
from geometry_msgs.msg import Pose

from ...core.schemas.common import FollowPathRequest, PlanPathRequest, RecordPathRequest

# 教導-重現路徑這五個介面是一組的：缺任何一個就把整組視為不可用，
# 避免出現「列表看得到但按了重播沒反應」這種更難查的半殘狀態。
try:
    from smartnav_msgs.srv import DeleteTaughtPath, ListTaughtPaths, PlanTaughtPath, RecordPath
    from smartnav_msgs.action import FollowTaughtPath
    TAUGHT_PATH_AVAILABLE = True
except ImportError:  # pragma: no cover
    DeleteTaughtPath = ListTaughtPaths = PlanTaughtPath = RecordPath = None
    FollowTaughtPath = None
    TAUGHT_PATH_AVAILABLE = False


async def _call_taught_path_service(node, name: str, request, timeout: float = None):
    """呼叫教導路徑服務，把 call_service 的阻塞呼叫丟到執行緒池並收斂例外。

    ★ 這裡在修一個既有 bug：原本每支端點都寫 `if resp is None: ...503...`，
      但 node.jobs.call_service()（前身 node._call_service）從來不會回傳
      None——服務沒就緒是丟 RuntimeError、逾時是丟 TimeoutError，兩者都會
      直接穿出 asyncio.to_thread 變成未攔截的例外，FastAPI 接手後端點就是
      毫無結構的 500，那個 `is None` 分支其實永遠不會被走到。
      這裡比照 api/dependencies.py 的 guard()，把例外轉成跟其餘端點一致的
      {success, message} JSON + 對應狀態碼（503/504/500）。

    回傳 (resp, error)：resp 非 None 時呼叫成功可以照舊往下處理；
    否則 error 是可以直接回傳給呼叫端的 JSONResponse。
    """
    try:
        if timeout is None:
            resp = await asyncio.to_thread(node.jobs.call_service, name, request)
        else:
            resp = await asyncio.to_thread(node.jobs.call_service, name, request, timeout)
        return resp, None
    except TimeoutError as e:
        return None, JSONResponse({"success": False, "message": str(e)}, status_code=504)
    except RuntimeError as e:
        return None, JSONResponse({"success": False, "message": str(e)}, status_code=503)
    except Exception as e:  # noqa: BLE001 - 最後一道防線，不能讓例外冒到 ASGI 層
        node.get_logger().error(f"HTTP 處理錯誤: {e}")
        return None, JSONResponse({"success": False, "message": f"伺服器錯誤: {e}"}, status_code=500)


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    # ── 教導-重現路徑 ──────────────────────────────────
    #
    # 這台車最可靠的移動方式：錄下人開過的位姿，之後用純追蹤重播。
    # 路徑是人開過的所以物理上保證可行，繞開了自動規劃在窄走廊的
    # 各種麻煩（MPPI 視野不足、容差疊加、K-turn 做不出來）。

    def _taught_unavailable() -> JSONResponse:
        return JSONResponse(
            {
                "success": False,
                "message": "教導路徑需要重建 smartnav_msgs（colcon build --packages-select "
                           "smartnav_msgs smartnav_navigation_cc）後重啟所有節點",
            },
            status_code=501,
        )

    @router.get("/api/paths", dependencies=admin_only)
    async def api_list_paths() -> JSONResponse:
        if not TAUGHT_PATH_AVAILABLE:
            return _taught_unavailable()
        req = ListTaughtPaths.Request()
        req.current_map_only = True
        resp, err = await _call_taught_path_service(node, "list_taught_paths", req)
        if err is not None:
            return err
        return JSONResponse({
            "success": bool(resp.success),
            "message": resp.message,
            "paths": [
                {
                    "path_id": p.path_id, "name": p.name, "map_id": p.map_id,
                    "num_points": int(p.num_points), "length_m": round(float(p.length_m), 2),
                    "num_cusps": int(p.num_cusps), "source": p.source,
                    "created_at": p.created_at,
                }
                for p in resp.paths
            ],
        })

    @router.post("/api/paths/record", dependencies=admin_only)
    async def api_record_path(req: RecordPathRequest) -> JSONResponse:
        """錄製控制：start / stop / cancel

        錄製期間節點只是被動記錄位姿，不送任何速度指令，所以跟遙控
        完全不衝突——使用者照常用方向鍵或鍵盤把車開一遍。
        """
        if not TAUGHT_PATH_AVAILABLE:
            return _taught_unavailable()
        actions = {
            "start": RecordPath.Request.START,
            "stop": RecordPath.Request.STOP,
            "cancel": RecordPath.Request.CANCEL,
        }
        if req.action not in actions:
            return JSONResponse(
                {"success": False, "message": f"未知的動作 {req.action}"}, status_code=400
            )
        if req.action == "stop" and not (req.name or "").strip():
            return JSONResponse(
                {"success": False, "message": "請先輸入路徑名稱再結束錄製"}, status_code=400
            )
        r = RecordPath.Request()
        r.action = actions[req.action]
        r.name = (req.name or "").strip()
        resp, err = await _call_taught_path_service(node, "record_path", r)
        if err is not None:
            return err
        out = {"success": bool(resp.success), "message": resp.message}
        if req.action == "stop" and resp.success:
            out.update({"path_id": resp.path_id, "num_points": int(resp.num_points),
                        "length_m": round(float(resp.length_m), 2)})
        return JSONResponse(out, status_code=200 if resp.success else 400)

    @router.delete("/api/paths/{path_id}", dependencies=admin_only)
    async def api_delete_path(path_id: str) -> JSONResponse:
        if not TAUGHT_PATH_AVAILABLE:
            return _taught_unavailable()
        r = DeleteTaughtPath.Request()
        r.path_id = path_id
        resp, err = await _call_taught_path_service(node, "delete_taught_path", r)
        if err is not None:
            return err
        return JSONResponse({"success": bool(resp.success), "message": resp.message},
                            status_code=200 if resp.success else 400)

    @router.post("/api/paths/follow", dependencies=admin_only)
    async def api_follow_path(req: FollowPathRequest) -> JSONResponse:
        """重播教導路徑。與導航一樣走作業機制，進度在作業清單裡看。"""
        if not TAUGHT_PATH_AVAILABLE:
            return _taught_unavailable()
        goal = FollowTaughtPath.Goal()
        goal.path_id = req.path_id
        goal.reverse = bool(req.reverse)
        goal.speed_scale = float(req.speed_scale or 0.0)
        label = ("反向重播「%s」" if req.reverse else "重播「%s」") % (req.name or req.path_id)
        job_id = node.jobs.start_action("follow_taught_path", goal, label)
        return JSONResponse({"success": True, "message": label + "：已送出", "job_id": job_id})

    @router.post("/api/paths/plan", dependencies=admin_only)
    async def api_plan_path(req: PlanPathRequest) -> JSONResponse:
        """把地圖上點選的一串位置規劃成教導路徑

        逐段呼叫 nav2 的 /compute_path_to_pose。刻意不自己做曲線內插——
        SmacPlannerHybrid + REEDS_SHEPP 本來就會產生符合最小轉彎半徑、
        含折返點的阿克曼可行路徑。規劃器沒問題，出問題的是 MPPI 控制器。
        """
        if not TAUGHT_PATH_AVAILABLE:
            return _taught_unavailable()
        if not (req.name or "").strip():
            return JSONResponse({"success": False, "message": "請先輸入路徑名稱"},
                                status_code=400)
        if not req.points:
            return JSONResponse({"success": False, "message": "請先在地圖上點選至少一個位置"},
                                status_code=400)
        r = PlanTaughtPath.Request()
        r.name = req.name.strip()
        r.start_from_robot = bool(req.start_from_robot)
        for pt in req.points:
            pose = Pose()
            pose.position.x = float(pt.get("x", 0.0))
            pose.position.y = float(pt.get("y", 0.0))
            yaw = float(pt.get("yaw", 0.0))
            pose.orientation.z = math.sin(yaw * 0.5)
            pose.orientation.w = math.cos(yaw * 0.5)
            r.waypoints.append(pose)
        # 規劃要逐段呼叫 nav2，段數多時會慢，逾時放寬
        resp, err = await _call_taught_path_service(node, "plan_taught_path", r, 60.0)
        if err is not None:
            return err
        out = {"success": bool(resp.success), "message": resp.message}
        if resp.success:
            out.update({"path_id": resp.path_id, "num_points": int(resp.num_points),
                        "length_m": round(float(resp.length_m), 2),
                        "num_cusps": int(resp.num_cusps)})
        return JSONResponse(out, status_code=200 if resp.success else 400)

    return router
