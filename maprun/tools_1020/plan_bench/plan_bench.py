#!/usr/bin/env python3
"""離線規劃基準的用戶端：目前地圖的地點兩兩規劃（ComputePathToPose，use_start=True），量規劃時間與路徑品質。

先開：ros2 launch ~/maprun/tools_1020/plan_bench/plan_bench.launch.py
再跑：python3 plan_bench.py [--repeat 3] [--out ~/maprun/verify_out/plan_bench_<HHMM>]

每一對（起點→終點）記：成功與否、規劃時間（規劃器回報＋牆鐘）、路徑長、直線距離、繞行比、
折返次數（Reeds-Shepp 前進↔倒車切換）、倒車長度、路徑上最小曲率半徑、半徑 < 0.95 m／< 1.03 m 的點數。
（0.95＝規劃器 minimum_turning_radius；1.03＝實車量到做得到的迴轉半徑，見 taught-path 筆記）
"""
import argparse
import csv
import json
import math
import os
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

ap = argparse.ArgumentParser()
ap.add_argument("--repeat", type=int, default=3)
ap.add_argument("--map", default="")
ap.add_argument("--goal-yaw", choices=["stored", "flip", "best2"], default="stored",
                help="終點朝向：stored＝地點存的朝向；flip＝反過來 180°；best2＝兩種都規劃、挑倒車最少的")
ap.add_argument("--out", default=os.path.expanduser(f"~/maprun/verify_out/plan_bench_{time.strftime('%H%M')}"))
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)

home = Path.home() / ".smartnav"
map_id = a.map or json.loads((home / "nav_state.json").read_text())["last_active_map"]
db = json.loads((home / "waypoint_database" / "waypoints.json").read_text())["waypoints_db"]
wps = [(v["name"], v["pose"]) for v in db.values() if v.get("map_id") == map_id]


def to_ps(p, flip=False):
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.pose.position.x, ps.pose.position.y = p["position"]["x"], p["position"]["y"]
    z, w = p["orientation"]["z"], p["orientation"]["w"]
    if flip:  # 繞 z 轉 180°：(z, w) → (w, -z)
        z, w = w, -z
    ps.pose.orientation.z, ps.pose.orientation.w = z, w
    return ps


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


def wrap(a_):
    return (a_ + math.pi) % (2 * math.pi) - math.pi


def analyse(path):
    pts = [(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation)) for p in path.poses]
    length = rev_len = 0.0
    dirs = []
    for (x0, y0, h0), (x1, y1, _h1) in zip(pts, pts[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        if d < 1e-6:
            continue
        length += d
        backward = abs(wrap(math.atan2(y1 - y0, x1 - x0) - h0)) > math.pi / 2
        dirs.append(backward)
        if backward:
            rev_len += d
    cusps = sum(1 for u, v in zip(dirs, dirs[1:]) if u != v)
    # 曲率半徑：取間距 ≥ 0.10 m 的三點算外接圓（太密的點會被量化誤差放大）
    sparse = [pts[0]] if pts else []
    for p in pts[1:]:
        if math.hypot(p[0] - sparse[-1][0], p[1] - sparse[-1][1]) >= 0.10:
            sparse.append(p)
    radii = []
    for (x0, y0, _), (x1, y1, _), (x2, y2, _) in zip(sparse, sparse[1:], sparse[2:]):
        a_ = math.hypot(x1 - x0, y1 - y0)
        b_ = math.hypot(x2 - x1, y2 - y1)
        c_ = math.hypot(x2 - x0, y2 - y0)
        area2 = abs((x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0))
        if area2 < 1e-9 or c_ < 0.5 * (a_ + b_):   # 共線，或折返點（方向反轉，不是轉彎）
            continue
        radii.append(a_ * b_ * c_ / (2.0 * area2))
    return {
        "路徑長m": round(length, 2), "折返次數": cusps, "倒車長m": round(rev_len, 2), "點數": len(pts),
        "最小半徑m": round(min(radii), 2) if radii else "",
        "半徑<0.95點數": sum(r < 0.95 for r in radii), "半徑<1.03點數": sum(r < 1.03 for r in radii),
    }


rclpy.init()
n = Node("plan_bench_cc")
cli = ActionClient(n, ComputePathToPose, "compute_path_to_pose")
print("等 compute_path_to_pose …")
if not cli.wait_for_server(timeout_sec=120):
    raise SystemExit("✗ 等不到 planner_server（先開 plan_bench.launch.py）")
time.sleep(3.0)  # 讓全域成本地圖收到地圖
print(f"地圖 {map_id}：{[w[0] for w in wps]}；每對 {a.repeat} 次")

rows = []
for rep in range(1, a.repeat + 1):
    for i, (sn, sp) in enumerate(wps):
        for j, (gn, gp) in enumerate(wps):
            if i == j:
                continue
            def plan(flip):
                goal = ComputePathToPose.Goal()
                goal.goal, goal.start = to_ps(gp, flip), to_ps(sp)
                goal.planner_id, goal.use_start = "GridBased", True
                fut = cli.send_goal_async(goal)
                rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
                gh = fut.result()
                if gh and gh.accepted:
                    rf = gh.get_result_async()
                    rclpy.spin_until_future_complete(n, rf, timeout_sec=15)
                    return rf.result()
                return None

            t0 = time.perf_counter()
            flips = {"stored": [False], "flip": [True], "best2": [False, True]}[a.goal_yaw]
            cands = []
            for fl in flips:
                rr = plan(fl)
                if rr and rr.result.path.poses:
                    m = analyse(rr.result.path)
                    cands.append(((m["倒車長m"], m["折返次數"], m["路徑長m"]), fl, rr))
                elif not cands and fl == flips[-1]:
                    cands.append(((1e9, 0, 0), fl, rr))
            cands.sort(key=lambda c: c[0])
            _, chosen_flip, res = cands[0]
            wall = time.perf_counter() - t0
            straight = math.hypot(gp["position"]["x"] - sp["position"]["x"], gp["position"]["y"] - sp["position"]["y"])
            row = {"輪": rep, "起點": sn, "終點": gn, "直線m": round(straight, 2), "牆鐘ms": round(wall * 1000, 1),
                   "終點朝向": "反向" if chosen_flip else "原朝向"}
            r = res.result if res else None
            ok = bool(r and r.path.poses)
            row["成功"] = int(ok)
            row["錯誤碼"] = getattr(r, "error_code", "") if r else "無回應"
            if r is not None:
                row["規劃ms"] = round(r.planning_time.sec * 1000 + r.planning_time.nanosec / 1e6, 1)
            if ok:
                row.update(analyse(r.path))
                row["繞行比"] = round(row["路徑長m"] / straight, 2) if straight > 0.05 else ""
            rows.append(row)
            print(f"[{rep}] {sn}→{gn}: {'✓' if ok else '✗'} {row.get('規劃ms', '—')} ms "
                  f"長 {row.get('路徑長m', '—')} m 折返 {row.get('折返次數', '—')} 最小半徑 {row.get('最小半徑m', '—')}")

keys = []
for r in rows:
    keys += [k for k in r if k not in keys]
with open(os.path.join(a.out, "plans.csv"), "w", newline="", encoding="utf-8-sig") as fh:
    w = csv.DictWriter(fh, fieldnames=keys)
    w.writeheader()
    w.writerows(rows)

ok = [r for r in rows if r["成功"]]
pt = [r["規劃ms"] for r in ok if "規劃ms" in r]
lines = [
    f"離線規劃基準 {time.strftime('%Y-%m-%d %H:%M')}：地圖 {map_id}，{len(wps)} 個地點兩兩 × {a.repeat} 輪 = {len(rows)} 次（終點朝向 {a.goal_yaw}）",
    f"成功 {len(ok)}/{len(rows)}；規劃時間 中位 {statistics.median(pt):.0f} ms、平均 {statistics.mean(pt):.0f} ms、最大 {max(pt):.0f} ms"
    if pt else "沒有成功的規劃",
]
if ok:
    lines += [
        f"路徑長 {min(r['路徑長m'] for r in ok):.2f}~{max(r['路徑長m'] for r in ok):.2f} m；"
        f"繞行比中位 {statistics.median(r['繞行比'] for r in ok if r['繞行比'] != ''):.2f}",
        f"有折返的路徑 {sum(r['折返次數'] > 0 for r in ok)}/{len(ok)}；"
        f"最小曲率半徑 < 0.95 m 的路徑 {sum(r['半徑<0.95點數'] > 0 for r in ok)}/{len(ok)}，< 1.03 m {sum(r['半徑<1.03點數'] > 0 for r in ok)}/{len(ok)}",
    ]
    tot = sum(r["路徑長m"] for r in ok)
    rv = sum(r["倒車長m"] for r in ok)
    lines.append(f"含倒車的路徑 {sum(r['倒車長m'] > 0.05 for r in ok)}/{len(ok)}；總路程 {tot:.1f} m 其中倒車 {rv:.1f} m（{100 * rv / tot:.0f}%）")
    fails = sorted({f"{r['起點']}→{r['終點']}" for r in rows if not r["成功"]})
    if fails:
        lines.append("失敗的組合：" + "、".join(fails))
open(os.path.join(a.out, "summary.txt"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
print("\n".join(lines))
print("輸出：", a.out)
n.destroy_node()
rclpy.shutdown()
