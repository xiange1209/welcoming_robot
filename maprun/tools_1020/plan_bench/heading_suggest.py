#!/usr/bin/env python3
"""替每個地點找「倒車最少」的終點朝向（★ 2026-10-23，只分析、不改資料庫）。

背景：10/22 plan_bench 量到 6 個地點兩兩規劃，28/30 條要倒車、總路程 33% 是倒車；8/10 實測多折返全敗。
做法：每個地點當終點時試 8 個方向（每 45°）＋目前存的朝向，從其他每個地點（用它們目前的朝向）規劃過去，
      挑「倒車長度 → 折返次數 → 路徑長」最小的；最後把所有地點都換成建議朝向，整張表重算一次比較。
先開：ros2 launch ~/maprun/tools_1020/plan_bench/plan_bench.launch.py
用法：python3 heading_suggest.py [--out ~/maprun/verify_out/heading_suggest_<HHMM>]
"""
import argparse
import csv
import json
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

ap = argparse.ArgumentParser()
ap.add_argument("--out", default=os.path.expanduser(f"~/maprun/verify_out/heading_suggest_{time.strftime('%H%M')}"))
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)

home = Path.home() / ".smartnav"
map_id = json.loads((home / "nav_state.json").read_text())["last_active_map"]
db = json.loads((home / "waypoint_database" / "waypoints.json").read_text())["waypoints_db"]
wps = {v["name"]: (v["pose"]["position"]["x"], v["pose"]["position"]["y"],
                   2.0 * math.atan2(v["pose"]["orientation"]["z"], v["pose"]["orientation"]["w"]))
       for v in db.values() if v.get("map_id") == map_id}


def ps(x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def wrap(v):
    return (v + math.pi) % (2 * math.pi) - math.pi


def cost(path):
    pts = [(q.pose.position.x, q.pose.position.y, 2.0 * math.atan2(q.pose.orientation.z, q.pose.orientation.w))
           for q in path.poses]
    length = rev = 0.0
    dirs = []
    for (x0, y0, h0), (x1, y1, _h) in zip(pts, pts[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        if d < 1e-6:
            continue
        back = abs(wrap(math.atan2(y1 - y0, x1 - x0) - h0)) > math.pi / 2
        length += d
        rev += d if back else 0.0
        dirs.append(back)
    cusps = sum(1 for u, v in zip(dirs, dirs[1:]) if u != v)
    return rev, cusps, length


rclpy.init()
n = Node("heading_suggest_cc")
cli = ActionClient(n, ComputePathToPose, "compute_path_to_pose")
if not cli.wait_for_server(timeout_sec=120):
    raise SystemExit("✗ 等不到 planner_server（先開 plan_bench.launch.py）")
time.sleep(3.0)


def plan(start, goal):
    g = ComputePathToPose.Goal()
    g.start, g.goal, g.planner_id, g.use_start = ps(*start), ps(*goal), "GridBased", True
    f = cli.send_goal_async(g)
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    gh = f.result()
    if not gh or not gh.accepted:
        return None
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=15)
    r = rf.result()
    return cost(r.result.path) if r and r.result.path.poses else None


def total(headings):
    """所有起點→終點（起點朝向也用 headings）的倒車、折返、長度、失敗數"""
    rv = cu = ln = fails = 0
    for s, (sx, sy, _) in wps.items():
        for g, (gx, gy, _) in wps.items():
            if s == g:
                continue
            c = plan((sx, sy, headings[s]), (gx, gy, headings[g]))
            if c is None:
                fails += 1
                continue
            rv, cu, ln = rv + c[0], cu + c[1], ln + c[2]
    return rv, cu, ln, fails


current = {k: v[2] for k, v in wps.items()}
rows, suggested = [], dict(current)
for g, (gx, gy, cur) in wps.items():
    cands = sorted({round(cur, 3)} | {round(wrap(math.radians(d)), 3) for d in range(0, 360, 45)})
    scores = []
    for h in cands:
        rv = cu = ln = fails = 0
        for s, (sx, sy, sh) in wps.items():
            if s == g:
                continue
            c = plan((sx, sy, sh), (gx, gy, h))
            if c is None:
                fails += 1
                continue
            rv, cu, ln = rv + c[0], cu + c[1], ln + c[2]
        scores.append((fails, round(rv, 2), cu, round(ln, 2), h))
    scores.sort()
    best = scores[0]
    cur_score = next(x for x in scores if abs(x[4] - round(cur, 3)) < 1e-6)
    suggested[g] = best[4]
    rows.append({"地點": g, "目前朝向°": round(math.degrees(cur)), "建議朝向°": round(math.degrees(best[4])),
                 "目前：倒車m": cur_score[1], "目前：折返": cur_score[2], "建議：倒車m": best[1], "建議：折返": best[2],
                 "建議：失敗": best[0]})
    print(rows[-1], flush=True)

before, after = total(current), total(suggested)
with open(os.path.join(a.out, "per_waypoint.csv"), "w", newline="", encoding="utf-8-sig") as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)
json.dump({k: round(math.degrees(v), 1) for k, v in suggested.items()},
          open(os.path.join(a.out, "suggested_headings_deg.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
pct = lambda t: 100 * t[0] / t[2] if t[2] else 0  # noqa: E731
lines = [
    f"地點朝向建議 {time.strftime('%Y-%m-%d %H:%M')}（地圖 {map_id}，{len(wps)} 個地點兩兩 {len(wps) * (len(wps) - 1)} 條）",
    f"目前朝向：倒車 {before[0]:.1f} m／總長 {before[2]:.1f} m（{pct(before):.0f}%）、折返 {before[1]} 次、失敗 {before[3]}",
    f"建議朝向：倒車 {after[0]:.1f} m／總長 {after[2]:.1f} m（{pct(after):.0f}%）、折返 {after[1]} 次、失敗 {after[3]}",
    "★ 只是規劃器上的比較；終點朝向也影響「車停好時面向哪裡」（例如大廳要面向客人），採用前要現場確認。",
]
open(os.path.join(a.out, "summary.txt"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
print("\n".join(lines))
print("輸出：", a.out)
n.destroy_node()
rclpy.shutdown()
