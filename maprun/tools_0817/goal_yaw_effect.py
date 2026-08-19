#!/usr/bin/env python3
"""同一個終點座標、只改「目標朝向」，比較規劃出來的折返點數。

★ 為什麼要問這個（2026-08-19）
從門口／大廳那一帶規劃回起點，一直得到 **14~16 個折返點**的路徑（6.97 m 的路），
而從起點往外規劃只要 1~2 個。純幾何算一下就看出原因：

    門口 (0.133, 2.609) -> 起點 (4.598, 0.999) 的行進方向 = atan2(-1.61, 4.465) = -19.8 度
    起點這個 waypoint 存的目標朝向                        = +160.8 度
                                                    差 = 180.6 度

**每次到達起點都要在 0.99 m 走廊裡掉頭 180 度**，而轉彎半徑下限 0.95 m —— 幾何上做不到，
規劃器只能用一連串折返湊。這支就是要證實或推翻它：只改朝向，其他全部不動。

用法：
    python3 goal_yaw_effect.py 起點
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from smartnav_msgs.srv import ListWaypoints, PlanTaughtPath

NAME = sys.argv[1] if len(sys.argv) > 1 else "起點"


def main():
    rclpy.init()
    n = Node("goal_yaw_effect")
    wp = n.create_client(ListWaypoints, "/list_waypoints")
    pl = n.create_client(PlanTaughtPath, "/plan_taught_path")
    if not wp.wait_for_service(timeout_sec=10.0) or not pl.wait_for_service(timeout_sec=10.0):
        print("✗ 服務不在"); return 1
    f = wp.call_async(ListWaypoints.Request())
    rclpy.spin_until_future_complete(n, f, timeout_sec=15.0)
    tgt = {w.waypoint_name: w for w in f.result().waypoints_info}.get(NAME)
    if tgt is None:
        print(f"✗ 沒有地點 {NAME}"); return 1

    q = tgt.pose.orientation
    base_yaw = 2.0 * math.atan2(q.z, q.w)
    print(f"  終點 {NAME}  x={tgt.pose.position.x:+.3f} y={tgt.pose.position.y:+.3f}"
          f"  原本朝向 {math.degrees(base_yaw):+.1f}°")
    print(f"  {'目標朝向':>10} {'點數':>5} {'長度':>8} {'折返':>5}  結果")
    print("  " + "-" * 56)

    for delta_deg in (0, 90, 180, -90):
        yaw = base_yaw + math.radians(delta_deg)
        p = Pose()
        p.position.x = tgt.pose.position.x
        p.position.y = tgt.pose.position.y
        p.orientation.z = math.sin(yaw / 2)
        p.orientation.w = math.cos(yaw / 2)
        req = PlanTaughtPath.Request()
        req.name = f"_yawtest_{delta_deg}"
        req.start_from_robot = True
        req.waypoints = [p]
        fut = pl.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=180.0)
        r = fut.result()
        shown = (math.degrees(yaw) + 180) % 360 - 180
        if r is None:
            print(f"  {shown:+9.1f}°     —        —      —   ✗ 逾時")
        elif r.success:
            print(f"  {shown:+9.1f}° {r.num_points:5d} {r.length_m:7.2f}m "
                  f"{r.num_cusps:5d}  ✓")
        else:
            print(f"  {shown:+9.1f}°     —        —      —   ✗ {r.message[:34]}")
        time.sleep(1.0)

    print("  " + "-" * 56)
    print("  ★ 只做規劃，車子沒有動過。")
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
