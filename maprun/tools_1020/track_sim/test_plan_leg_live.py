#!/usr/bin/env python3
"""path_teach_cc 的 _plan_leg（2026-10-24 多候選擇優＋降級）對**真的** planner_server 測（不動車、不寫路徑庫）。
先開：ros2 launch ~/maprun/tools_1020/plan_bench/plan_bench.launch.py
用法（先 source ~/maprun/env.sh）：python3 test_plan_leg_live.py [<plan_cand_r140 目錄>]
  ① 規劃半徑設 1.40（＝新 yaml），30 對逐一呼叫節點的 _plan_leg：都成功、選中的分數 ≤ 原樣、終點離真終點 ≤ 0.10 m、
     最後一點朝向＝真終點朝向；與離線 plan_candidates_bench 的選擇對照（長度 0 的候選要被擋掉）
  ② 降級：規劃半徑暫設 3.0（大部分規劃不出來）→ _plan_leg 要自己降到 1.20 規劃出來，事後設回 3.0
  ③ plan_candidates:=false → 只規劃一次（舊行為）
"""
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from smartnav_navigation_cc.path_teach_cc_node import PathTeachNode, yaw_from_quat

DIR = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/maprun/verify_out/plan_cand_r140_0030")
passed = failed = 0


def check(name, cond, detail=""):
    global passed, failed
    passed += bool(cond)
    failed += not cond
    print(f"  {'✓' if cond else '✗'} {name}" + ("" if cond else f"  {detail}"), flush=True)


rclpy.init(args=["--ros-args", "-r", "__node:=path_teach_cc_plan_test"])
node = PathTeachNode()
ex = MultiThreadedExecutor(num_threads=4)
ex.add_node(node)
threading.Thread(target=ex.spin, daemon=True).start()

home = Path.home() / ".smartnav"
map_id = json.loads((home / "nav_state.json").read_text())["last_active_map"]
db = json.loads((home / "waypoint_database" / "waypoints.json").read_text())["waypoints_db"]
wps = [(v["name"], v["pose"]) for v in db.values() if v.get("map_id") == map_id]
offline = {f"{r['起點']}|{r['終點']}": r for r in json.load(open(os.path.join(DIR, "candidates.json")))["rows"]}


def pose(p):
    return node._make_pose(p["position"]["x"], p["position"]["y"],
                           2.0 * math.atan2(p["orientation"]["z"], p["orientation"]["w"]))


assert node._planner_client.wait_for_server(timeout_sec=60), "等不到 planner_server（先開 plan_bench.launch.py）"
time.sleep(2.0)
orig = node._planner_radius()
print(f"planner_server 目前規劃半徑 {orig}", flush=True)

print("① 規劃半徑 1.40：30 對")
check("設成 1.40", node._set_planner_radius(1.40) and abs(node._planner_radius() - 1.40) < 1e-9)
times, mism = [], []
for sn, sp in wps:
    for gn, gp in wps:
        if sn == gn:
            continue
        s, g = pose(sp), pose(gp)
        t0 = time.perf_counter()
        pts = node._plan_leg(s, g)
        times.append(time.perf_counter() - t0)
        if not pts:
            check(f"{sn}→{gn} 規劃成功", False)
            continue
        gyaw = yaw_from_quat(g.orientation)
        err = math.hypot(pts[-1].x - g.position.x, pts[-1].y - g.position.y)
        node._annotate_directions(pts)
        sc, det = node._path_score(pts)
        off = offline.get(f"{sn}|{gn}")
        ok = err <= 0.10 + 1e-9 and abs(pts[-1].yaw - gyaw) < 1e-9 and len(pts) >= 2
        if off and off.get("C0分") is not None:
            ok = ok and sc <= off["C0分"] + 0.01           # 不會比原樣差（離線檔分數四捨五入到小數兩位）
        if off and abs(sc - off["擇優分"]) > 0.01:          # 離線檔的分數四捨五入到小數兩位
            mism.append(f"{sn}→{gn}（離線選「{off['選中']}」{off['擇優分']:.2f}，節點 {sc:.2f}、長 {det['length_m']:.2f}）")
        if not ok:
            check(f"{sn}→{gn}", False, f"終點差 {err:.3f} m、分數 {sc:.2f}、原樣 {off and off.get('C0分')}")
check(f"30 對都成功、終點 ≤ 0.10 m、朝向＝真終點、分數不比原樣差（{len(times)} 對）", failed == 0)
times.sort()
print(f"     每對 _plan_leg 時間：中位 {times[len(times) // 2]:.2f} s、最大 {times[-1]:.2f} s")
print(f"     與離線選擇不同的 {len(mism)} 對（預期：長度 0 的候選被擋，其餘應相同）：")
for m in mism:
    print("      ", m)
check("與離線選擇不同的只有離線選到長度 0 路徑的那幾對",
      all(any(k in m for k in ("門口→貴賓室", "貴賓室→門口")) for m in mism), str(mism))

print("② 降級：模擬「1.40 在這裡規劃不出來」（規劃器半徑 >= 1.39 時 _plan_once 一律回失敗），其餘全走真的 planner_server")
st = {"r": node._planner_radius()}
_orig_set, _orig_once, _orig_fb = node._set_planner_radius, node._plan_once, node._plan_fallback_radii
fb_calls, set_calls = [], []


def fake_set(r, wait=True):
    set_calls.append((r, wait))
    ok = _orig_set(r, wait)
    st["r"] = r
    return ok


def fake_once(start_pose, goal_pose, fail_all=False):
    if fail_all or st["r"] >= 1.39:
        return None, False
    return _orig_once(start_pose, goal_pose)


node._set_planner_radius = fake_set
node._plan_once = fake_once
node._plan_fallback_radii = lambda a_, b_, rr: (fb_calls.append(list(rr)), _orig_fb(a_, b_, rr))[1]
s, g = pose(dict(wps)["起點"]), pose(dict(wps)["門口"])
t0 = time.perf_counter()
pts = node._plan_leg(s, g)
dt = time.perf_counter() - t0
check(f"原樣與 6 個候選都失敗 → 呼叫降級 {fb_calls}", fb_calls == [[1.2, 0.95]], str(fb_calls))
check(f"降到 1.20 規劃出來（{dt:.1f} s，含 Smac 重建約 5 秒）", bool(pts), "規劃不出來")
if pts:
    node._annotate_directions(pts)
    _sc, det = node._path_score(pts)
    check(f"那條路徑是用 1.20 規劃的（三點外接圓最緊 {det['worst_r']:.2f} m；量法約一成離散誤差，介於 0.95 與 1.40 之間）", 1.0 < det["worst_r"] < 1.3, str(det["worst_r"]))
check(f"改參數的順序＝降到 1.20、非同步設回 1.40：{set_calls}",
      len(set_calls) == 2 and abs(set_calls[0][0] - 1.20) < 1e-9 and set_calls[0][1] is True
      and abs(set_calls[1][0] - 1.40) < 1e-9 and set_calls[1][1] is False, str(set_calls))
time.sleep(8.0)                                     # 設回是非同步的（Smac 重建查找表約 5 秒）
check("planner_server 真的回到 1.40", abs((_orig_get := node._planner_radius() or 0) - 1.40) < 1e-9, str(_orig_get))
print("   降級也全失敗：")
set_calls.clear()
fb_calls.clear()
node._plan_once = lambda a_, b_: fake_once(a_, b_, fail_all=True)
t0 = time.perf_counter()
pts = node._plan_leg(s, g)
dt = time.perf_counter() - t0
check(f"回 None（{dt:.1f} s）", pts is None)
check(f"還是設回 1.40：{set_calls}", set_calls and abs(set_calls[-1][0] - 1.40) < 1e-9 and set_calls[-1][1] is False,
      str(set_calls))
time.sleep(8.0)
check("planner_server 真的回到 1.40", abs((node._planner_radius() or 0) - 1.40) < 1e-9, str(node._planner_radius()))
node._set_planner_radius, node._plan_once, node._plan_fallback_radii = _orig_set, _orig_once, _orig_fb

print("③ plan_candidates:=false")
node._set_planner_radius(1.40)
node.set_parameters([Parameter("plan_candidates", Parameter.Type.BOOL, False)])
n_calls = []
_orig_once = node._plan_once
node._plan_once = lambda st, go: (n_calls.append(1), _orig_once(st, go))[1]
pts = node._plan_leg(pose(dict(wps)["櫃台"]), pose(dict(wps)["門口"]))
check(f"只規劃一次（{len(n_calls)} 次）且成功", len(n_calls) == 1 and bool(pts))
node._plan_once = _orig_once

print("④ 多段規劃（_plan_cb：起點→櫃台→門口，不從車子開始）：接點要連續（★ 10/24 改成下一段從上一段實際終點接）")
from smartnav_msgs.srv import PlanTaughtPath
node._save_db = lambda: True                      # 不寫真的路徑庫
w = dict(wps)
mreq = PlanTaughtPath.Request(name="_test_multi", start_from_robot=False,
                              waypoints=[pose(w["起點"]), pose(w["櫃台"]), pose(w["門口"])])
mres = node._plan_cb(mreq, PlanTaughtPath.Response())
check(f"多段規劃成功（{mres.message}）", mres.success, mres.message)
if mres.success:
    mpts = node._paths.pop(mres.path_id)["points"]
    gaps = [math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(mpts, mpts[1:])]
    check(f"相鄰點距最大 {max(gaps):.3f} m（< 0.30 m，沒有接點跳躍）", max(gaps) < 0.30)
    g_last = w["門口"]["position"]
    check("終點離門口 ≤ 0.10 m", math.hypot(mpts[-1].x - g_last["x"], mpts[-1].y - g_last["y"]) <= 0.10 + 1e-9)

node._set_planner_radius(orig if orig else 0.95)
print(f"收尾：規劃半徑設回 {orig}")
print(f"\n{passed}/{passed + failed} 通過", flush=True)
ex.shutdown()
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if failed == 0 else 1)
