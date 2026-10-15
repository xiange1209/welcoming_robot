# -*- coding: utf-8 -*-
"""tools/curvature/curvature_lib.py 的測試（合成圓弧／直線／S 形、倒車旗標、折返切段、分箱與 Spearman、對照 corner_check）。

跑法：python3 tests/curvature/test_curvature.py        （純標準函式庫＋numpy，與 tests/nav_stub 同風格：✓／✗、最後印 N/M 通過）
測試只證明它檢查的東西：曲率估計器在合成幾何上的偏差、統計函式的定義；不證明真車的行為。
"""
import math
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

REPO = Path(os.environ.get("SMARTNAV_REPO") or Path(__file__).resolve().parents[2])
sys.path.insert(0, str(REPO / "tools" / "curvature"))
sys.path.insert(0, str(REPO / "maprun"))
import curvature_lib as C  # noqa: E402

results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


def near(a, b, tol):
    return a is not None and abs(a - b) <= tol


def arc(R, ang, sp=0.05, d=1, ccw=True, x0=0.0, y0=0.0, noise=0.0, seed=1):
    """半徑 R、轉角 ang(rad) 的圓弧，起點 (x0,y0) 朝 +x。ccw=True 往左彎。"""
    rnd = np.random.default_rng(seed)
    n = int(R * ang / sp) + 1
    th = np.linspace(0, ang, n)
    sg = 1 if ccw else -1
    pts = []
    for t in th:
        x = x0 + R * math.sin(t)
        y = y0 + sg * R * (1 - math.cos(t))
        pts.append({"x": x + rnd.normal(0, noise), "y": y + rnd.normal(0, noise), "yaw": sg * t, "d": d})
    return pts


def kmed(rows):
    return float(np.median([r[1] for r in rows])) if rows else None


print("── 1. 曲率估計：圓／直線／S 形 ──")
for R in (0.75, 1.0, 2.0):
    for sp in (0.05, 0.186):
        rows = C.path_profile(arc(R, math.pi / 2 + 0.3 * math.pi, sp))[0]
        k = kmed(rows)
        tol = 0.03 if R < 1.5 else 0.02
        check(near(k, 1 / R, tol * (1 / R) + 1e-3), f"左彎 R={R} 間距 {sp}：κ={k:.4f}（期望 {1 / R:.4f}，相對 {abs(k * R - 1) * 100:.1f}%）")
rows = C.path_profile(arc(1.0, 2.5, 0.05, ccw=False))[0]
check(near(kmed(rows), -1.0, 0.03), f"右彎 R=1：κ 為負（{kmed(rows):.3f}）")
line = [{"x": i * 0.05, "y": 0.0, "yaw": 0.0, "d": 1} for i in range(100)]
rows = C.path_profile(line)[0]
check(max(abs(r[1]) for r in rows) < 0.01, f"直線：|κ|<0.01（最大 {max(abs(r[1]) for r in rows):.4f}）")
# S 形：先左後右各 R=1
a1 = arc(1.0, 1.2, 0.05, ccw=True)
x1, y1, h1 = a1[-1]["x"], a1[-1]["y"], a1[-1]["yaw"]
a2 = []
for t in np.linspace(0, 1.2, 25)[1:]:
    # 從 (x1,y1) 朝 h1 方向開始往右彎
    lx, ly = math.sin(t), -(1 - math.cos(t))
    a2.append({"x": x1 + lx * math.cos(h1) - ly * math.sin(h1), "y": y1 + lx * math.sin(h1) + ly * math.cos(h1), "yaw": h1 - t, "d": 1})
rows = C.path_profile(a1 + a2)[0]
ks = [r[1] for r in rows]
check(max(ks) > 0.9 and min(ks) < -0.9, f"S 形：同時出現 +1 與 -1 附近的 κ（{max(ks):.2f}／{min(ks):.2f}）")
check(abs(max(ks) - 1) < 0.08 and abs(min(ks) + 1) < 0.08, "S 形：峰值與 1/R 差 <8%（接點處平滑會削峰，這是已知偏差）")
noisy = arc(1.0, 3.0, 0.05, noise=0.005)
k = kmed(C.path_profile(noisy)[0])
check(near(k, 1.0, 0.1), f"座標抖動 σ=5 mm 的圓（R=1）：κ 中位數 {k:.3f}（期望 1，±10%）")
sd = float(np.std([r[1] for r in C.path_profile(noisy)[0]]))
check(sd < 0.12, f"同上：κ 標準差 {sd:.3f} < 0.12（平滑把雜訊壓住）")

print("── 2. 倒車旗標：輪子打哪邊 ──")
fwd = C.path_profile(arc(1.0, 2.0, 0.05, d=1, ccw=True))[0]
rev = C.path_profile(arc(1.0, 2.0, 0.05, d=-1, ccw=True))[0]
check(all(r[3] == 1 for r in fwd) and all(r[3] == -1 for r in rev), "同一條向左彎的幾何：前進＝左輪、倒車＝右輪（side 翻轉）")
check(near(kmed(fwd), kmed(rev), 1e-9), "倒車不改變幾何曲率 κ_p 的符號（只改 side）")
lim_meas = C.LIMITS[C.MEASURED]    # 左 0.971 / 右 0.845
r_hard = 1 / 0.90                  # κ=0.90：介於右限 0.845 與左限 0.971 之間
a_f = C.path_profile(arc(r_hard, 3.0, 0.05, d=1, ccw=True))[0]
a_r = C.path_profile(arc(r_hard, 3.0, 0.05, d=-1, ccw=True))[0]
check(C.over_stats(a_f, lim_meas)["frac_all"] == 0.0, "κ=0.90 前進左彎：低於左限 0.971 -> 不超限")
check(C.over_stats(a_r, lim_meas)["frac_all"] > 0.9, "同一彎倒車（輪子打右）：高於右限 0.845 -> 幾乎全段超限")

print("── 3. 切段：d 旗標與幾何折返 ──")
p = arc(1.0, 1.5, 0.05, d=1) + [dict(q, d=-1) for q in arc(1.0, 1.5, 0.05, d=-1, x0=0.0, y0=0.0)]
check(len(C.split_runs(p)) == 2, "d 由 +1 變 -1：切成 2 段")
back = [{"x": i * 0.05, "y": 0.0, "yaw": 0.0, "d": 1} for i in range(20)] + [{"x": 0.95 - i * 0.05, "y": 0.0, "yaw": 0.0, "d": 1} for i in range(1, 15)]
check(len(C.split_runs(back)) == 2, "d 沒變但位移反向（教導路徑微退漏標）：仍切成 2 段")
rows = C.path_profile(back)[0]
check(all(abs(r[1]) < 0.05 for r in rows), "切開後不會在折返點產生假的尖峰 κ")
tiny = [{"x": 0, "y": 0, "yaw": 0, "d": 1}, {"x": 0.05, "y": 0, "yaw": 0, "d": 1}, {"x": 0.1, "y": 0, "yaw": 0, "d": 1}]
rows, comp, skip, tot = C.path_profile(tiny)
check(rows == [] and near(skip, 0.1, 1e-6), "長度 <0.2 m 的 run 不估計，弧長計入 skipped")
short = C.path_profile(arc(0.8, 0.5, 0.05))   # 弧長 0.4 m：用首／中／末三點
check(len(short[0]) == 1 and short[0][0][8] is True and near(short[0][0][1], 1 / 0.8, 0.05), f"0.2–0.55 m 的短 run：單一 κ（{short[0][0][1]:.3f}，期望 1.25）")

print("── 4. 超限統計 ──")
arc90 = C.path_profile(arc(0.9, math.pi / 2, 0.05) + [], )[0]   # R=0.9 -> κ=1.11
st = C.over_stats(arc90, C.LIMITS[C.MEASURED])
check(st["frac_L"] > 0.99 and st["frac_R"] == 0.0 and st["len_R"] == 0.0, "R=0.9 左彎 vs 實測左限 0.971：左側全段超限、右側無樣本")
st2 = C.over_stats(arc90, C.LIMITS["planner_R0.95"])
check(st2["frac_all"] > 0.99, "R=0.9 vs 規劃器 0.95：超限")
st3 = C.over_stats(C.path_profile(arc(1.2, math.pi / 2, 0.05))[0], C.LIMITS[C.MEASURED])
check(st3["frac_all"] == 0.0 and st3["longest_all"] == 0.0, "R=1.2 左彎 vs 實測左限：不超限")
st4 = C.over_stats(C.path_profile(arc(1.1, math.pi, 0.05, ccw=False))[0], C.LIMITS[C.MEASURED])
check(st4["frac_R"] > 0.99, "R=1.1 右彎 vs 實測右限 R=1.183：超限（右邊比左邊嚴）")
check(near(st["longest_L"], st["over_L"], 1e-9), "整段連續超限：最長連續＝超限總弧長")
segs = C.over_segments(arc90, C.LIMITS[C.MEASURED])
check(len(segs) == 1 and segs[0]["side"] == 1 and near(segs[0]["kmax"], 1 / 0.9, 0.05) and segs[0]["length"] > 0.8, f"over_segments：1 段、左輪、κmax≈1.11、長度 {segs[0]['length']:.2f} m")
check(near(segs[0]["x0"], 0.29, 0.05), "不可行路段起點座標 ＝ 估計區間起點（扣掉邊緣 m=0.30 m）")
# 兩段被不超限區隔開
two = arc(0.9, 1.2, 0.05) 
lastx, lasty = two[-1]["x"], two[-1]["y"]
line2 = [{"x": lastx + 0.05 * i * math.cos(1.2), "y": lasty + 0.05 * i * math.sin(1.2), "yaw": 1.2, "d": 1} for i in range(1, 40)]
rows = C.path_profile(two + line2)[0]
check(len(C.over_segments(rows, C.LIMITS[C.MEASURED])) == 1, "彎後接直線：只一段超限")
# 去雜訊
spike = [(0.1 * i, (3.0 if 5 <= i <= 7 else 0.1), 1, 1, 0, 0, 0, 0.05, False) for i in range(20)]
kept, dl, ns = C.clean_rows(spike)
check(len(kept) == 17 and near(dl, 0.15, 1e-9) and ns == 1, "clean_rows：峰值 3.0 (>2.0) 的連續超硬限一段被整段丟掉")
mild = [(0.1 * i, (1.5 if 5 <= i <= 7 else 0.1), 1, 1, 0, 0, 0, 0.05, False) for i in range(20)]
check(len(C.clean_rows(mild)[0]) == 20, "峰值 1.5（<2.0）不丟：可能是真實的緊彎")

print("── 5. 分箱、Spearman、事件 ──")
samples = [(0.1, 0.05), (0.1, 0.07), (0.3, 0.2), (0.6, 0.3), (0.6, 0.5), (0.9, 0.4), (1.2, 0.9), (1.2, 0.7), (float("nan"), 9.9)]
bt = C.bin_table(samples)
check([b["n"] for b in bt] == [2, 1, 2, 1, 2], f"分箱計數 {[b['n'] for b in bt]}（nan 不進任何箱）")
check(near(bt[0]["median"], 0.06, 1e-9) and near(bt[4]["median"], 0.8, 1e-9), "分箱中位數正確")
check(bt[4]["bin"] == ">0.95" and bt[0]["bin"] == "0–0.2", "箱標籤")
x = list(range(30)); y = [v * 2 + 1 for v in x]
check(near(C.spearman(x, y), 1.0, 1e-12) and near(C.spearman(x, y[::-1]), -1.0, 1e-12), "Spearman：嚴格單調 ±1")
check(near(C.spearman([1, 2, 2, 3], [1, 3, 3, 5]), 1.0, 1e-9), "Spearman：有同分（平均名次）仍為 1")
rho, pv = C.spearman_perm_p(x, y, n_perm=999)
check(rho == 1.0 and pv < 0.01, f"排列檢定 p={pv:.4f} 對完全單調很小")
rng = np.random.default_rng(3); rx = rng.normal(size=40); ry = rng.normal(size=40)
rho, pv = C.spearman_perm_p(rx, ry, n_perm=999)
check(abs(rho) < 0.4 and pv > 0.02, f"獨立隨機資料：ρ={rho:.2f}、p={pv:.2f}（不顯著）")
check(C.spearman([1, 1, 1, 1], [1, 2, 3, 4]) != C.spearman([1, 1, 1, 1], [1, 2, 3, 4]), "x 全相同：回 nan 而不是假的 0")
rows = [{"state": s, "index": i, "ct": c, "t": 0, "total": 10, "msg": ""} for s, i, c in
        [("following", 1, .05), ("blocked", 2, .1), ("blocked", 2, .1), ("escaping", 2, 0.0), ("escaping", 2, 0.0), ("blocked", 3, .2), ("slowing", 4, float("nan"))]]
check(C.events(rows) == [("blocked", 2), ("escaping", 2), ("blocked", 3)], "events：連續同狀態只算一次進入")
check([C.usable(r) for r in rows] == [True, True, True, False, False, True, False], "usable：排除 escaping（cross_track 欄位混用）與 nan")

print("── 6. 路徑索引 → 需要的 κ ──")
pts = arc(1.0, 2.0, 0.05)
rows = C.path_profile(pts)[0]
kap = C.kappa_at_points(pts, rows)
check(len(kap) == len(pts), "每個路徑點都有對應")
mid = len(pts) // 2
check(near(kap[mid][0], 1.0, 0.05) and kap[mid][2] == 1, f"弧中點：|κ|≈1、左輪（{kap[mid][0]:.3f}）")
check(kap[0][0] == kap[0][0], "起點（邊緣）以最近取樣外推，不是 nan")
revpts = arc(1.0, 2.0, 0.05, d=-1)
check(C.kappa_at_points(revpts, C.path_profile(revpts)[0])[mid][2] == -1, "倒車路徑：side=-1")

print("── 7. 與 maprun/corner_check_cc.py 一致 ──")
import corner_check_cc as cc  # noqa: E402
for R in (0.75, 1.03, 1.183):
    k = abs(kmed(C.path_profile(arc(R, 2.5, 0.05))[0]))
    w_est, w_ref = cc.geometry(1 / k)[0], cc.geometry(R)[0]
    check(near(w_est, w_ref, 0.02), f"R={R}：用估計的 κ 反推半徑代入 corner_check 的最小淨寬 {w_est:.3f} ≈ 直接代入 {w_ref:.3f}")
check(near(cc.geometry(1.03)[0], 0.893, 0.002) and near(cc.geometry(1.183)[0], 0.983, 0.002), "corner_check 表值（左 1.03→0.893、右 1.183→0.983 m）")

if not (REPO / "docs/cloud/evidence/replay").exists():
    # 第 8 節要用雲端工作區的目錄結構（docs/cloud/evidence/replay、tools/curvature/run_t6.py）；
    # 本檔放在 車子/maprun/tools_1015/curvature/ 時沒有這些，跳過而不是失敗（完整 55 項請在雲端工作區跑）。
    print("── 8. 真實資料整合：跳過（需要雲端工作區的目錄結構）──")
    print(f"\n{sum(results)}/{len(results)} 通過（第 8 節跳過）")
    sys.exit(0 if all(results) else 1)

print("── 8. 真實資料整合（路徑庫＋重播）──")
paths = C.load_paths(REPO / ".smartnav/path_database/paths.json")
check(len(paths) == 122, f"路徑庫 {len(paths)} 條")
rp = sorted((REPO / "docs/cloud/evidence/replay").glob("*.csv"))
check(len(rp) == 3, f"重播 CSV {len(rp)} 份")
r0 = C.load_replay(rp[0])
check(sum(1 for r in r0 if r["state"] == "escaping") == 956 and sum(1 for r in r0 if C.usable(r)) == 610, "第 1 趟：escaping 956 列被排除、可用 610 列")
with tempfile.TemporaryDirectory() as td:
    out = subprocess.run([sys.executable, str(REPO / "tools/curvature/run_t6.py"), "--out", td], capture_output=True, text=True)
    check(out.returncode == 0, "run_t6.py 可執行" + ("" if out.returncode == 0 else out.stderr[-300:]))
    csvp = Path(td) / "T6_per_path_summary.csv"
    lines = csvp.read_text(encoding="utf-8-sig").splitlines() if csvp.exists() else []
    check(len(lines) == 123, f"per-path 摘要：1 列標題＋122 列（得 {len(lines)}）")
    check((Path(td) / "T6_infeasible_segments.csv").exists() and (Path(td) / "T6_results.json").exists(), "不可行路段與 results.json 都有輸出")

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
