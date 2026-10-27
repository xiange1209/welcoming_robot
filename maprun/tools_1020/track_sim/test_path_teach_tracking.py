#!/usr/bin/env python3
"""path_teach_cc 2026-10-24 改動的測試：直接用**節點本身的方法**（不是模擬器裡的重寫），不需要車、不需要 ROS 在跑。
  ① 單元：_project_on_path 的正負號（前進／倒車）、_lookahead_point_interp、_xte_to_segments、_pick_target 開關
  ② 評分一致：節點的 _path_score／_geometric_tight_spots 與離線的 plan_metrics.score 在 R1.40 的 30 條上一字不差
  ③ 可行性：geometric=True 不再把 Smac 的 yaw 假折角報成「需要半徑 0.4 m」
  ④ 閉迴路：track_sim 的車子模型 ＋ 節點的 _closest_index／_pick_target／_pure_pursuit／_xi_step，
     三種組態（舊／內插／內插＋積分）× 6 種干擾 × 2 次，結果要和 track_sim 自己的 base／pi 同級
用法（先 source ~/maprun/env.sh）：python3 test_path_teach_tracking.py [<plan_cand 目錄>]
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plan_metrics as pm  # noqa: E402
import track_sim as ts  # noqa: E402
from smartnav_navigation_cc.path_teach_cc_node import PathPoint, PathTeachNode  # noqa: E402

DIR = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/maprun/verify_out/plan_cand_r140_0030")
passed = failed = 0


def check(name, cond, detail=""):
    global passed, failed
    passed += bool(cond)
    failed += not cond
    print(f"  {'✓' if cond else '✗'} {name}" + ("" if cond else f"  {detail}"))


class _P:
    def __init__(self, v):
        self.value = v


def stub(interp=True, xi=True):
    """不跑 __init__ 的節點替身：只設這些方法會讀到的屬性（值＝節點預設值）。"""
    n = object.__new__(PathTeachNode)
    n.min_radius_left, n.min_radius_right = 1.05, 1.20
    n.rev_front_axle, n.axle_spacing = False, 0.322
    n.la_min, n.la_max, n.la_k, n.la_rev_scale, n.la_rev_ahead = 0.30, 0.80, 1.2, 1.8, 0.24
    n._la_interp, n._xi_on, n._xi_dist, n._xi_max, n._xi_gate = interp, xi, 1.5, 0.15, 0.25
    params = {"feasibility_min_radius": 0.76, "feasibility_window_m": 0.15}
    n.get_parameter = lambda k: _P(params[k])
    return n


def to_pp(pts):
    return [PathPoint(p[0], p[1], p[2], p[3]) for p in pts]


class NodeController:
    """track_sim.run 用的控制器介面，裡面呼叫節點的方法（照 _follow_loop 的順序）。"""

    def __init__(self, interp, xi, v_fwd=0.15, v_rev=0.10, rate=15.4):
        self.n = stub(interp, xi)
        self.dt, self.v_fwd, self.v_rev = 1.0 / rate, v_fwd, v_rev
        self.xi, self.prev_xy, self._pp, self._key = {1: 0.0, -1: 0.0}, None, None, None

    def closest_index(self, pts, x, y, hint):
        return PathTeachNode._closest_index(self._pts(pts), x, y, hint)

    def _pts(self, pts):
        if self._key is not id(pts):
            self._key, self._pp = id(pts), to_pp(pts)
        return self._pp

    def command(self, pts, idx, segs, pose, direction):
        n, pp = self.n, self._pts(pts)
        v = self.v_fwd if direction > 0 else self.v_rev
        ld = max(n.la_min, min(n.la_max, (n.la_k * v + n.la_min) * (n.la_rev_scale if direction < 0 else 1.0)))
        rx, ry = n._track_ref(pose, direction)
        ki = (2.0 / (ld * ld)) / n._xi_dist
        xi_curv = -ki * self.xi[direction] if n._xi_on else 0.0
        _i, tgt = n._pick_target(pp, idx, rx, ry, ld)
        lin, ang = n._pure_pursuit(pose, tgt, direction, v, 0.0, xi_curv)
        if n._xi_on:
            self.prev_xy = n._xi_step(self.xi, self.prev_xy, pp, idx, rx, ry, direction, lin, ang, "following", 0.0, ki)
        return lin, ang


def main():
    print("① 單元")
    n = stub()
    line = [PathPoint(0.18 * i, 0.0, 0.0, 1) for i in range(20)]
    check("在線上兩點中間：線段偏離 0、舊算法（到最近點）0.09",
          abs(PathTeachNode._xte_to_segments(line, 0, 20, 0.27, 0.0)) < 1e-9
          and abs(min(math.hypot(p.x - 0.27, p.y) for p in line) - 0.09) < 1e-9)
    e, si = PathTeachNode._project_on_path(line, 3, 0.60, 0.10)
    check("前進：在路徑左邊 = +0.10、線段起點索引 3", abs(e - 0.10) < 1e-9 and si == 3, f"{e} {si}")
    rev = [PathPoint(0.18 * i, 0.0, math.pi, -1) for i in range(20)]      # 車頭朝 -x、倒車往 +x
    e, _ = PathTeachNode._project_on_path(rev, 3, 0.60, 0.10)
    check("倒車：行進方向（+x）的左邊仍是 +0.10（與純追蹤鏡射後同一座標系）", abs(e - 0.10) < 1e-9, f"{e}")
    i, tp = PathTeachNode._lookahead_point_interp(line, 3, 0.55, 0.0, 0.48)
    check("內插前視：剛好 0.48 m", abs(math.hypot(tp.x - 0.55, tp.y) - 0.48) < 1e-9 and i == 6, f"{tp.x} {i}")
    # 照 _annotate_directions 的慣例：折返點本身（0.72）標新的方向（-1），前進段的折線畫到它為止
    cusp = ([PathPoint(0.18 * i, 0.0, 0.0, 1) for i in range(4)] + [PathPoint(0.72, 0.0, 0.0, -1)]
            + [PathPoint(0.72 - 0.18 * k, 0.0, 0.0, -1) for k in range(1, 5)])
    _i, tp = PathTeachNode._lookahead_point_interp(cusp, 1, 0.20, 0.0, 0.80)
    check("內插前視不跨折返：停在折返點（0.72, 0）", abs(tp.x - 0.72) < 1e-9 and tp.direction == -1, f"{tp.x}")
    _i, tp = PathTeachNode._lookahead_point_interp(line, 17, 3.30, 0.0, 0.48)
    check("快到終點：回傳最後一點", abs(tp.x - line[-1].x) < 1e-9)
    n0 = stub(interp=False)
    i0, t0 = n0._pick_target(line, 3, 0.55, 0.0, 0.48)
    i1, t1 = n._pick_target(line, 3, 0.55, 0.0, 0.48)
    check("_pick_target：關＝舊的頂點（距離 0.53）、開＝內插（0.48）",
          abs(math.hypot(t0.x - 0.55, t0.y) - 0.53) < 1e-9 and abs(math.hypot(t1.x - 0.55, t1.y) - 0.48) < 1e-9)
    v, w = n._pure_pursuit((0.0, 0.0, 0.0), PathPoint(1.0, 0.0, 0.0, 1), 1, 0.15, 0.0, 5.0)
    check("extra_curv 也照方向箝制（要 5.0 只給左上限 1/1.05）", abs(w / v - 1.0 / 1.05) < 1e-9, f"{w / v}")

    print("② 節點評分 ＝ 離線評分（R1.40 的 30 條擇優路徑）")
    paths = json.load(open(os.path.join(DIR, "paths.json")))
    diffs = []
    for k, pts in paths.items():
        s_off, d_off = pm.score([list(p) for p in pts])
        s_node, d_node = n._path_score(to_pp(pts))
        diffs.append(abs(s_off - s_node))
        if abs(s_off - s_node) > 1e-3:
            print("     差異", k, s_off, s_node, d_off, d_node)
    # 容差 1e-3：節點把每個窗的弧長四捨五入到 0.1 mm（arc_m 是給人看的），離線版沒有
    check(f"{len(diffs)} 條分數一致（最大差 {max(diffs):.2e}，容差 1e-3）", max(diffs) < 1e-3)

    print("③ 可行性檢查（10/21 實車規劃的「_auto_櫃台_3」，R0.95）")
    db = json.load(open(os.path.expanduser("~/.smartnav/path_database/paths.json")))["paths"]
    p = db.get("path_bec655584465")
    if p:
        pts = [PathPoint(q["x"], q["y"], q["yaw"], q["d"]) for q in p["points"]]
        ok_y, note_y, bad_y = n._check_feasibility(pts)
        ok_g, note_g, bad_g = n._check_feasibility(pts, geometric=True)
        print("     yaw ：", note_y[:90])
        print("     幾何：", note_g[:90])
        check("幾何法照樣抓到 0.95 m 的弧（車子確實轉不過去）", not ok_g and bad_g and bad_g[0]["radius_m"] > 0.8)
        check("幾何法最緊處不再是 yaw 假折角（< 0.6 m）", all(b["radius_m"] > 0.6 for b in bad_g))
    else:
        print("     （路徑庫裡沒有 path_bec655584465，略過）")

    print("④ 閉迴路：節點的方法 vs track_sim（R1.40 擇優路徑、6 種干擾 × 2 次）")
    best = {k: v for k, v in paths.items() if k.startswith("best|")}
    res = {}
    for label, mk in (("舊（頂點、無積分）", lambda: NodeController(False, False)),
                      ("內插", lambda: NodeController(True, False)),
                      ("內插＋積分", lambda: NodeController(True, True)),
                      ("track_sim base", lambda: ts.Controller("base")),
                      ("track_sim pi", lambda: ts.Controller("pi"))):
        ok = tot = 0
        p95 = []
        for scen, sc in __import__("eval_tracking").SCENARIOS.items():
            for k, pts in best.items():
                for seed in range(2):
                    r = ts.run(pts, mk(), veh_kw=sc["veh"], loc_kw=sc["loc"], seed=seed)
                    ok += r["ok"]
                    tot += 1
                    e = sorted(r["errs"])
                    p95.append(e[int(0.95 * (len(e) - 1))])
        res[label] = (ok, tot, sorted(p95)[len(p95) // 2] * 100)
        print(f"     {label:<18} 成功 {ok}/{tot}、各趟 p95 偏離的中位 {res[label][2]:.1f} cm")
    check("節點「舊」＝ track_sim base（成功數差 ≤ 4）", abs(res["舊（頂點、無積分）"][0] - res["track_sim base"][0]) <= 4)
    check("節點「內插＋積分」＝ track_sim pi（成功數差 ≤ 6）", abs(res["內插＋積分"][0] - res["track_sim pi"][0]) <= 6)
    check("節點「內插＋積分」比「舊」好（成功更多、偏離更小）",
          res["內插＋積分"][0] > res["舊（頂點、無積分）"][0] and res["內插＋積分"][2] < res["舊（頂點、無積分）"][2])
    print(f"\n{passed}/{passed + failed} 通過")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
