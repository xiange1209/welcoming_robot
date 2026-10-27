#!/usr/bin/env python3
"""多候選規劃擇優的離線測試（★ 2026-10-23 車上 agent；只用離線 planner_server，不動車、不改資料庫）。

每一對地點：終點照原樣規劃一次（C0），再把終點做小擾動規劃 6 次——
  沿車頭 ±6 cm、橫向 ±6 cm（到點容許 20 cm，不影響抵達判定）、朝向 ±2.8°（終點對齊容許 5°，不會多一次三點對齊）——
用 plan_metrics.score（做不到的弧長、折角折返、微折返、折返數、倒車長、路徑長）挑分數最低的。
輸出 C0 與擇優兩組路徑（paths.json，key「c0|起|終」「best|起|終」）給 eval_tracking.py 比。
先開：ros2 launch ~/maprun/tools_1020/plan_bench/plan_bench.launch.py
用法：python3 plan_candidates_bench.py [--radius 1.20] [--out ~/maprun/verify_out/plan_cand_<HHMM>]
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plan_metrics as pm  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--radius", type=float, default=1.20)
ap.add_argument("--shift", type=float, default=0.06)
ap.add_argument("--dyaw-deg", type=float, default=2.8)
ap.add_argument("--out", default=os.path.expanduser(f"~/maprun/verify_out/plan_cand_{time.strftime('%H%M')}"))
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)

home = Path.home() / ".smartnav"
map_id = json.loads((home / "nav_state.json").read_text())["last_active_map"]
db = json.loads((home / "waypoint_database" / "waypoints.json").read_text())["waypoints_db"]
wps = [(v["name"], v["pose"]) for v in db.values() if v.get("map_id") == map_id]


def pose_of(p):
    return p["position"]["x"], p["position"]["y"], 2.0 * math.atan2(p["orientation"]["z"], p["orientation"]["w"])


def ps(x, y, yaw):
    m = PoseStamped()
    m.header.frame_id = "map"
    m.pose.position.x, m.pose.position.y = x, y
    m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return m


rclpy.init()
n = Node("plan_candidates_bench_cc")
cli = ActionClient(n, ComputePathToPose, "compute_path_to_pose")
setp = n.create_client(SetParameters, "/planner_server/set_parameters")
if not cli.wait_for_server(timeout_sec=120) or not setp.wait_for_service(timeout_sec=30):
    raise SystemExit("✗ 等不到 planner_server（先開 plan_bench.launch.py）")
req = SetParameters.Request()
req.parameters = [Parameter(name="GridBased.minimum_turning_radius",
                            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=a.radius))]
f = setp.call_async(req)
rclpy.spin_until_future_complete(n, f, timeout_sec=10)
time.sleep(1.0)


def plan(start, goal):
    g = ComputePathToPose.Goal()
    g.start, g.goal, g.planner_id, g.use_start = ps(*start), ps(*goal), "GridBased", True
    fut = cli.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
    gh = fut.result()
    if not (gh and gh.accepted):
        return None
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=20)
    r = rf.result()
    if not (r and r.result.path.poses):
        return None
    return pm.annotate([[q.pose.position.x, q.pose.position.y,
                         2.0 * math.atan2(q.pose.orientation.z, q.pose.orientation.w), 1] for q in r.result.path.poses])


rows, paths, t_all = [], {}, []
d, dy = a.shift, math.radians(a.dyaw_deg)
for sn, sp in wps:
    for gn, gp in wps:
        if sn == gn:
            continue
        s, (gx, gy, gyaw) = pose_of(sp), pose_of(gp)
        c, si = math.cos(gyaw), math.sin(gyaw)
        cands = [("C0", (gx, gy, gyaw)), ("前+", (gx + d * c, gy + d * si, gyaw)), ("前-", (gx - d * c, gy - d * si, gyaw)),
                 ("左", (gx - d * si, gy + d * c, gyaw)), ("右", (gx + d * si, gy - d * c, gyaw)),
                 ("yaw+", (gx, gy, gyaw + dy)), ("yaw-", (gx, gy, gyaw - dy))]
        t0 = time.perf_counter()
        scored = []
        for name, goal in cands:
            pts = plan(s, goal)
            if pts:
                sc, det = pm.score(pts)
                scored.append((sc, name, pts, det))
        t_all.append(time.perf_counter() - t0)
        if not scored:
            rows.append(dict(起點=sn, 終點=gn, 成功=0))
            continue
        c0 = next((x for x in scored if x[1] == "C0"), None)
        best = min(scored, key=lambda x: x[0])
        if c0:
            paths[f"c0|{sn}|{gn}"] = c0[2]
        paths[f"best|{sn}|{gn}"] = best[2]
        rows.append(dict(起點=sn, 終點=gn, 成功=1, 候選成功=len(scored), 選中=best[1],
                         C0分=round(c0[0], 2) if c0 else None, 擇優分=round(best[0], 2),
                         C0=c0[3] if c0 else None, 擇優=best[3]))
        print(f"{sn}→{gn}: 選 {best[1]}  分數 {c0[0] if c0 else float('nan'):.1f} → {best[0]:.1f}  "
              f"C0 {c0[3] if c0 else ''}  擇優 {best[3]}", flush=True)


def agg(key):
    g = [r[key] for r in rows if r.get(key)]
    return {k: round(sum(x[k] for x in g if x[k] is not None), 2) for k in ("長", "倒車", "做不到m", "折返", "折角", "微折返")}


summary = dict(半徑=a.radius, 對數=len(rows), C0=agg("C0"), 擇優=agg("擇優"),
               換掉C0的對數=sum(1 for r in rows if r.get("選中") not in (None, "C0")),
               每對規劃秒數中位=round(sorted(t_all)[len(t_all) // 2], 3), 每對規劃秒數最大=round(max(t_all), 3))
print(json.dumps(summary, ensure_ascii=False))
json.dump(dict(summary=summary, rows=rows), open(os.path.join(a.out, "candidates.json"), "w", encoding="utf-8"),
          ensure_ascii=False, indent=1)
json.dump(paths, open(os.path.join(a.out, "paths.json"), "w"))
req.parameters[0].value.double_value = 0.95
f = setp.call_async(req)
rclpy.spin_until_future_complete(n, f, timeout_sec=10)
print("輸出：", a.out)
n.destroy_node()
rclpy.shutdown()
