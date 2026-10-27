#!/usr/bin/env python3
"""掃 ~/.smartnav/path_database/paths.json 裡所有「規劃路徑」（meta.source == plan），
用 path_teach_cc._check_feasibility 同一套算法（0.15 m 滑動視窗、折返兩側不跨段、左右門檻＝控制器半徑 × 0.95）
找出車子做不到的地方，並依位置分類：起點（前 2 點）、終點（後 3 點）、折返旁（±2 點）、中段。
只讀，不改資料庫。（★ 2026-10-23 車上 agent）"""
import json, math, sys
from collections import Counter

R_LEFT, R_RIGHT, MARGIN, WIN = 1.05, 1.20, 0.95, 0.15

def norm(a):
    return math.atan2(math.sin(a), math.cos(a))

def bad_windows(pts):
    segs, s = [], 0
    for i in range(1, len(pts)):
        if pts[i]["d"] != pts[i - 1]["d"]:
            segs.append((s, i)); s = i
    segs.append((s, len(pts)))
    out = []
    for a, b in segs:
        for i in range(a, b - 1):
            arc, j = 0.0, i + 1
            while j < b and arc < WIN:
                arc += math.hypot(pts[j]["x"] - pts[j-1]["x"], pts[j]["y"] - pts[j-1]["y"]); j += 1
            if arc < WIN * 0.5:
                break
            d = norm(pts[j-1]["yaw"] - pts[i]["yaw"])
            if abs(d) < math.radians(1.0):
                continue
            r = arc / abs(d)
            left = (d > 0) if pts[i]["d"] >= 0 else (d < 0)
            lim = (R_LEFT if left else R_RIGHT) * MARGIN
            if r < lim:
                out.append((i, j - 1, r, "左" if left else "右", math.degrees(abs(d)), arc))
    return out, segs

db = json.load(open(sys.argv[1] if len(sys.argv) > 1 else "/home/user/.smartnav/path_database/paths.json"))
db = db.get("paths", db)
loc, side, rbin, n_paths, n_bad_paths = Counter(), Counter(), Counter(), 0, 0
kinks = []
for pid, p in db.items():
    if p["meta"].get("source") != "plan":
        continue
    pts = p["points"]; n_paths += 1
    bad, segs = bad_windows(pts)
    if bad: n_bad_paths += 1
    cusp_idx = [a for a, _ in segs[1:]]
    for (i, j, r, s, deg, arc) in bad:
        if i <= 1: where = "起點"
        elif j >= len(pts) - 3: where = "終點"
        elif any(abs(i - c) <= 2 or abs(j - c) <= 2 for c in cusp_idx): where = "折返旁"
        else: where = "中段"
        loc[where] += 1; side[s] += 1
        rbin["< 0.60（折角）" if r < 0.6 else "0.60~0.90" if r < 0.9 else "0.90~0.96（規劃器 0.95）" if r < 0.96 else "0.96~1.14"] += 1
        if r < 0.6: kinks.append((pid, p["meta"]["name"], i, j, round(r, 3), s, round(deg, 1), round(arc, 3), where, len(pts)))
print(f"規劃路徑 {n_paths} 條，其中 {n_bad_paths} 條有做不到的地方；共 {sum(loc.values())} 處")
print("  位置：", dict(loc)); print("  轉向：", dict(side)); print("  需要半徑：", dict(rbin))
print(f"  折角（< 0.60 m）{len(kinks)} 處，前 12 筆：")
for k in kinks[:12]: print("   ", k)
