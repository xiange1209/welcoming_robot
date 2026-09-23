"""地點與導航。

由 modules/navigation/router.py 組裝。搬自 api/routes/waypoints.py。
"""

import time

from fastapi import APIRouter
from fastapi.responses import JSONResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger

from smartnav_msgs.action import GlobalLocalization, Navigate
from smartnav_msgs.srv import CreateWaypoint, ListWaypoints

from ..imaging.geometry import make_pose, quaternion_to_yaw
from ...core.schemas.common import CreateWaypointRequest, NavigateRequest, SetPoseRequest

# DeleteWaypoint 是後加的介面，smartnav_msgs 不一定重建過——容許它不存在，
# 只讓刪除地點那支端點回報「需要重建」，而不是整個節點 import 就掛掉。
try:
    from smartnav_msgs.srv import DeleteWaypoint
except ImportError:  # pragma: no cover
    DeleteWaypoint = None


from ...api.dependencies import guard


def build_router(node, admin_only) -> APIRouter:
    router = APIRouter()

    @router.get("/api/waypoints", dependencies=admin_only)
    async def api_waypoints() -> JSONResponse:
        # 會打這支就代表地圖分頁被打開了。舊版前端沒有 /api/map/watch 心跳，
        # 靠這裡至少還能讓地圖先渲染出來。
        node.pose_tracker.note_watch()

        def work():
            res = node.jobs.call_service("list_waypoints", ListWaypoints.Request())
            waypoints = []
            for w in res.waypoints_info:
                q = w.pose.orientation
                waypoints.append(
                    {
                        "waypoint_id": w.waypoint_id,
                        "waypoint_name": w.waypoint_name,
                        "map_id": w.map_id,
                        "x": w.pose.position.x,
                        "y": w.pose.position.y,
                        "yaw": quaternion_to_yaw(q.x, q.y, q.z, q.w),
                    }
                )
            return {"success": bool(res.success), "message": res.message, "waypoints": waypoints}

        return await guard(node, work)

    @router.post("/api/waypoints", dependencies=admin_only)
    async def api_create_waypoint(req: CreateWaypointRequest) -> JSONResponse:
        if not req.waypoint_name.strip():
            return JSONResponse({"success": False, "message": "請提供地點名稱"}, status_code=400)

        def work():
            request = CreateWaypoint.Request()
            request.waypoint_name = req.waypoint_name.strip()
            request.use_given_pose = bool(req.use_given_pose)
            if req.use_given_pose:
                request.pose = make_pose(req.x, req.y, req.yaw)
            res = node.jobs.call_service("create_waypoint", request)
            return {"success": bool(res.success), "message": res.message}

        return await guard(node, work)

    @router.delete("/api/waypoints/{waypoint_id}", dependencies=admin_only)
    async def api_delete_waypoint(waypoint_id: str) -> JSONResponse:
        """刪除地點

        走 waypoint_service 的 delete_waypoint 服務而不是自己動 json 檔：
        地點資料庫是那個節點的記憶體狀態，繞過它直接改檔案，下一次建點時
        它會用自己那份記憶體整份覆寫回去，剛刪掉的又活過來。
        """
        if DeleteWaypoint is None:
            return JSONResponse(
                {
                    "success": False,
                    "message": "刪除地點需要重建 smartnav_msgs（colcon build --packages-select "
                               "smartnav_msgs smartnav_navigation_cc）後重啟",
                },
                status_code=501,
            )

        def work():
            request = DeleteWaypoint.Request()
            request.waypoint_id = waypoint_id
            res = node.jobs.call_service("delete_waypoint", request)
            return {"success": bool(res.success), "message": res.message}

        return await guard(node, work)

    @router.post("/api/navigate", dependencies=admin_only)
    async def api_navigate(req: NavigateRequest) -> JSONResponse:
        goal = Navigate.Goal()
        if req.waypoint_id:
            goal.waypoint_id = req.waypoint_id
            label = f"導航到地點 {req.target_name or req.waypoint_id}"
        else:
            goal.use_target_pose = True
            goal.target_pose = make_pose(req.x, req.y, req.yaw)
            goal.target_name = req.target_name or "指定位置"
            label = f"導航到 ({req.x:.2f}, {req.y:.2f})"
        job_id = node.jobs.start_action("navigate", goal, label)
        return JSONResponse({"success": True, "message": "導航已開始", "job_id": job_id})

    @router.post("/api/localize", dependencies=admin_only)
    async def api_localize() -> JSONResponse:
        """全域定位：把粒子撒滿整張圖再收斂。

        ⚠️ 走廊裡不要用這支，用下面的 /api/localize/here。
        理由見 SetPoseRequest 的說明（沿軸歧義 ＋ 會繞圈撞牆）。
        """
        job_id = node.jobs.start_action("global_localization", GlobalLocalization.Goal(), "全域定位")
        return JSONResponse({"success": True, "message": "全域定位已開始", "job_id": job_id})

    @router.post("/api/localize/here", dependencies=admin_only)
    async def api_localize_here(req: SetPoseRequest) -> JSONResponse:
        """把定位設到一個已知位置，不做全域搜尋（2026-08-06 新增）。

        兩步：發 /initialpose 給 AMCL 一個先驗，再用 /align_pose 做
        不需移動的掃描對齊。走廊裡這是唯一可靠的定位重設方式——
        全域定位會沿長軸收斂到錯的那一段（8/06 實測錯 3 公尺，
        而掃描吻合度還顯示 90.4%）。
        """

        def work():
            # ── 1. 決定目標位姿 ──
            if req.waypoint_id:
                res = node.jobs.call_service("list_waypoints", ListWaypoints.Request())
                hit = None
                for w in res.waypoints_info:
                    if w.waypoint_id == req.waypoint_id:
                        hit = w
                        break
                if hit is None:
                    return {"success": False,
                            "message": f"找不到地點 {req.waypoint_id}"}
                pose = hit.pose
                where = hit.waypoint_name or req.waypoint_id
            else:
                pose = make_pose(req.x, req.y, req.yaw)
                where = f"({req.x:.2f}, {req.y:.2f})"

            # ── 2. 發初始位姿 ──
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose = pose
            # 對角線：x/y 各 0.25（≈0.5 m 標準差）、yaw 0.0685（≈15 度）。
            # 這是 RViz「2D Pose Estimate」的預設值——表達「知道大概在哪
            # 但不精確」。設太小 AMCL 會拒絕修正自己的誤差，設太大等於沒設。
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.0685
            node.initialpose_pub.publish(msg)

            if not req.align:
                return {"success": True, "message": f"已把定位設到「{where}」"}

            # ── 3. 掃描對齊 ──
            # 等 AMCL 把 initialpose 吸收進粒子群再對齊，否則對齊的是舊位姿。
            time.sleep(0.5)
            client = node.service_clients.get("align_pose")
            if client is None or not client.service_is_ready():
                return {"success": True,
                        "message": f"已把定位設到「{where}」；"
                                   "/align_pose 服務不在，略過掃描對齊"}
            ares = node.jobs.call_service("align_pose", Trigger.Request())
            return {"success": True,
                    "message": f"已把定位設到「{where}」；掃描對齊：{ares.message}"}

        return await guard(node, work)

    return router
