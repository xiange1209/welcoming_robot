#!/usr/bin/env python3
"""規劃器最小迴轉半徑 A/B（★ 2026-10-23 車上 agent，只用離線 planner_server，不動車、不改資料庫）。

背景：path_teach 每條規劃路徑都報「有 N 處車子做不到」——規劃器用對稱 0.95 m，
      車子 8/29 實測左 1.03／右 1.18（控制器箝制 1.05／1.20、可行性門檻 ×0.95 = 1.00／1.14）。
做法：同一個 planner_server，依序把 GridBased.minimum_turning_radius 設成各個值（SetParameters 服務），
      目前地圖的地點兩兩規劃（use_start=True、終點用存的朝向），每條記：成功、長度、折返、倒車長、
      「做不到」的處數（path_teach._check_feasibility 同一套算法）、最緊半徑；完整路徑存 JSON 給 track_sim 用。
先開：ros2 launch ~/maprun/tools_1020/plan_bench/plan_bench.launch.py
用法：python3 plan_radius_bench.py [--radii 0.95,1.05,1.20] [--out ~/maprun/verify_out/plan_radius_<HHMM>]
"""
import argparse
import json
import math
import os
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

R_LEFT, R_RIGHT, MARGIN, WIN = 1.05, 1.20, 0.95, 0.15   # path_teach 的預設值

ap = argparse.ArgumentParser()
ap.add_argument("--radii", default="0.95,1.05,1.20")
ap.add_argument("--configs", default="",
                help="取代 --radii：名稱:參數=值,參數=值;名稱:…（例 r120ns:GridBased.minimum_turning_radius=1.20,GridBased.smooth_path=false）")
ap.add_argument("--out", default=os.path.expanduser(f"~/maprun/verify_out/plan_radius_{time.strftime('%H%M')}"))
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
def _val(v):
    if v.lower() in ("true", "false"):
        return v.lower() == "true"
    try:
        return int(v) if v.isdigit() else float(v)
    except ValueError:
        return v


if a.configs:
    configs = []
    for c in a.configs.split(";"):
        name, _, kvs = c.partition(":")
        configs.append((name, [(k, _val(v)) for k, v in (kv.split("=") for kv in kvs.split(","))]))
else:
    configs = [(f"{r:.2f}", [("GridBased.minimum_turning_radius", r)]) for r in (float(x) for x in a.radii.split(","))]

home = Path.home() / ".smartnav"
map_id = json.loads((home / "nav_state.json").read_text())["last_active_map"]
db = json.loads((home / "waypoint_database" / "waypoints.json").read_text())["waypoints_db"]
wps = [(v["name"], v["pose"]) for v in db.values() if v.get("map_id") == map_id]


def norm(x):
    return math.atan2(math.sin(x), math.cos(x))


def to_ps(p):
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.pose.position.x, ps.pose.position.y = p["position"]["x"], p["position"]["y"]
    ps.pose.orientation.z, ps.pose.orientation.w = p["orientation"]["z"], p["orientation"]["w"]
    return ps


def points(path):
    """nav2 Path → [x, y, yaw, d]；d 用 path_teach._annotate_directions 同一套規則。"""
    pts = [[p.pose.position.x, p.pose.position.y, 2.0 * math.atan2(p.pose.orientation.z, p.pose.orientation.w), 1]
           for p in path.poses]
    for i in range(len(pts) - 1):
        fwd = (pts[i + 1][0] - pts[i][0]) * math.cos(pts[i][2]) + (pts[i + 1][1] - pts[i][1]) * math.sin(pts[i][2])
        if abs(fwd) > 1e-3:
            pts[i][3] = 1 if fwd > 0 else -1
        elif i > 0:
            pts[i][3] = pts[i - 1][3]
    if len(pts) >= 2:
        pts[-1][3] = pts[-2][3]
    return pts


def feasibility(pts):
    """path_teach._check_feasibility 的算法：回傳 (做不到的處數, 最緊半徑, 做不到的總弧長)"""
    segs, s = [], 0
    for i in range(1, len(pts)):
        if pts[i][3] != pts[i - 1][3]:
            segs.append((s, i))
            s = i
    segs.append((s, len(pts)))
    bad, worst, bad_arc = 0, float("inf"), 0.0
    for a_, b_ in segs:
        for i in range(a_, b_ - 1):
            arc, j = 0.0, i + 1
            while j < b_ and arc < WIN:
                arc += math.hypot(pts[j][0] - pts[j - 1][0], pts[j][1] - pts[j - 1][1])
                j += 1
            if arc < WIN * 0.5:
                break
            d = norm(pts[j - 1][2] - pts[i][2])
            if abs(d) < math.radians(1.0):
                continue
            r = arc / abs(d)
            worst = min(worst, r)
            left = (d > 0) if pts[i][3] >= 0 else (d < 0)
            if r < (R_LEFT if left else R_RIGHT) * MARGIN:
                bad += 1
                bad_arc += math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
    return bad, worst, bad_arc


def cusp_quality(pts):
    """(折返點數, 幾何朝向跳變 > 15° 的折返點數, 微折返段數：兩個折返點之間 < 0.30 m)"""
    cusps, kinked = [], 0
    for c in range(1, len(pts) - 1):
        if pts[c][3] == pts[c - 1][3]:
            continue
        cusps.append(c)
        din = math.atan2(pts[c][1] - pts[c - 1][1], pts[c][0] - pts[c - 1][0])
        dout = math.atan2(pts[c + 1][1] - pts[c][1], pts[c + 1][0] - pts[c][0])
        hin = din if pts[c - 1][3] > 0 else din + math.pi
        hout = dout if pts[c][3] > 0 else dout + math.pi
        kinked += abs(math.degrees(norm(hout - hin))) > 15.0
    micro = 0
    for c0, c1 in zip(cusps, cusps[1:]):
        seg = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(c0, c1))
        micro += seg < 0.30
    return len(cusps), kinked, micro


def stats(pts):
    length = rev = 0.0
    for p, q in zip(pts, pts[1:]):
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        length += d
        if p[3] < 0:
            rev += d
    cusps = sum(1 for p, q in zip(pts, pts[1:]) if p[3] != q[3])
    return length, rev, cusps


rclpy.init()
n = Node("plan_radius_bench_cc")
cli = ActionClient(n, ComputePathToPose, "compute_path_to_pose")
setp = n.create_client(SetParameters, "/planner_server/set_parameters")
print("等 compute_path_to_pose …", flush=True)
if not cli.wait_for_server(timeout_sec=120) or not setp.wait_for_service(timeout_sec=30):
    raise SystemExit("✗ 等不到 planner_server（先開 plan_bench.launch.py）")
time.sleep(3.0)


def set_params(kvs):
    req = SetParameters.Request()
    for k, v in kvs:
        if isinstance(v, bool):
            pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
        elif isinstance(v, int):
            pv = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v)
        elif isinstance(v, float):
            pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)
        else:
            pv = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=v)
        req.parameters.append(Parameter(name=k, value=pv))
    fut = setp.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
    res = fut.result()
    return bool(res and res.results and all(r.successful for r in res.results)), \
        [r.reason for r in (res.results if res else []) if not r.successful]


def plan(sp, gp):
    goal = ComputePathToPose.Goal()
    goal.goal, goal.start = to_ps(gp), to_ps(sp)
    goal.planner_id, goal.use_start = "GridBased", True
    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
    gh = fut.result()
    if not (gh and gh.accepted):
        return None
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=20)
    return rf.result()


summary, all_paths = [], {}
DEFAULTS = [("GridBased.minimum_turning_radius", 0.95), ("GridBased.smooth_path", True)]
for cname, kvs in configs:
    set_params(DEFAULTS)                     # 每組都從車上的預設值開始
    ok_set = set_params(kvs)
    time.sleep(1.0)
    rows = []
    for i, (sn, sp) in enumerate(wps):
        for j, (gn, gp) in enumerate(wps):
            if i == j:
                continue
            t0 = time.perf_counter()
            res = plan(sp, gp)
            ms = (time.perf_counter() - t0) * 1000
            row = {"起點": sn, "終點": gn, "成功": 0, "ms": round(ms)}
            if res and res.result.path.poses:
                pts = points(res.result.path)
                length, rev, cusps = stats(pts)
                bad, worst, bad_arc = feasibility(pts)
                _nc, kinked, micro = cusp_quality(pts)
                row.update(成功=1, 長=round(length, 2), 倒車=round(rev, 2), 折返=cusps, 做不到=bad, 折角折返=kinked, 微折返=micro,
                           最緊半徑=round(worst, 3) if worst < 1e9 else None, 做不到弧長=round(bad_arc, 2))
                all_paths[f"{cname}|{sn}|{gn}"] = pts
            rows.append(row)
    ok = [x for x in rows if x["成功"]]
    tot = sum(x["長"] for x in ok) or 1.0
    s = {
        "設定": cname, "參數": kvs, "參數設定成功": ok_set, "成功": f"{len(ok)}/{len(rows)}",
        "實際最緊半徑中位": round(statistics.median([x["最緊半徑"] for x in ok if x["最緊半徑"]]), 3) if ok else None,
        "有做不到的路徑": sum(x["做不到"] > 0 for x in ok), "做不到處數": sum(x["做不到"] for x in ok),
        "做不到弧長m": round(sum(x["做不到弧長"] for x in ok), 2),
        "折返總數": sum(x["折返"] for x in ok), "零折返路徑": sum(x["折返"] == 0 for x in ok),
        "折角折返(>15°)": sum(x["折角折返"] for x in ok), "微折返段(<0.30m)": sum(x["微折返"] for x in ok),
        "總長m": round(tot, 1), "倒車m": round(sum(x["倒車"] for x in ok), 1),
        "倒車%": round(100 * sum(x["倒車"] for x in ok) / tot, 1),
        "規劃ms中位": round(statistics.median([x["ms"] for x in rows])),
        "失敗": [f"{x['起點']}→{x['終點']}" for x in rows if not x["成功"]],
    }
    summary.append(s)
    print(json.dumps(s, ensure_ascii=False), flush=True)
    with open(os.path.join(a.out, f"plans_{cname}.json"), "w", encoding="utf-8") as f:
        json.dump(rows, f, ensure_ascii=False, indent=1)

set_params(DEFAULTS)   # 收尾設回車上的值（離線節點，關掉也就沒了）
with open(os.path.join(a.out, "summary.json"), "w", encoding="utf-8") as f:
    json.dump(summary, f, ensure_ascii=False, indent=1)
with open(os.path.join(a.out, "paths.json"), "w", encoding="utf-8") as f:
    json.dump(all_paths, f)
print("輸出：", a.out)
n.destroy_node()
rclpy.shutdown()
