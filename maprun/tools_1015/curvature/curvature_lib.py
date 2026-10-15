# -*- coding: utf-8 -*-
"""T6：路徑曲率剖面與超限統計（純資料分析，不 import rclpy，只用標準函式庫＋numpy）。

★ 約定（與 path_teach_cc_node.py 一致）
  - 路徑點 {x, y, yaw, d}：d=+1 前進、-1 倒車；點的順序＝車子走過的順序。
  - κ_p：沿「行進順序」的有號幾何曲率，>0 = 路徑向左彎（逆時針）。車身偏航率 ω = |v|·κ_p（前進、倒車皆同，
    因為倒車時車頭朝向＝切線+π，朝向變化率仍等於切線轉向率）。
  - 輪子打哪邊：前進 κ_p>0 → 輪子打左；倒車 κ_p>0 → 輪子打右（ω>0 的倒車要打右，見 T3 §3 與
    path_teach._clamp_curv）。所以 side = sign(κ_p·d)：>0 用「左」上限、<0 用「右」上限。
  - 只在同一個 d 的連續一段（run）內估曲率；折返點是原地停車換向，不算行駛曲率。

★ 估計方法（驗證見 tests/curvature/test_curvature.py）
  1. 每個 run 以弦長累積為弧長 s，線性內插重取樣到 ds≈0.05 m。
  2. 對 x(s)、y(s) 做高斯平滑（σ=0.10 m，視窗邊緣用歸一化權重）壓掉教導點座標抖動。
  3. 在平滑曲線上取 s-h、s、s+h（h=0.25 m）三點的外接圓：κ = 2·cross(b-a, c-a)/(|ab||bc||ca|)（有號）。
  4. 只輸出 s∈[m, L-m]，m=max(h,3σ)=0.30 m（平滑核完整支撐；邊緣歸一化會讓 κ 端點少估約 6%）；0.2 m ≤ L < 2m+0.05 的短 run 改用「首／中／末」三點外接圓
     給一個 run 級別的 κ（標 short=True，只有 3 點所以精度較差）；L < 0.2 m 的 run 不估計，弧長另外計入 skipped。
  偏差（實測於合成圓）：σ=0.10 m 的高斯平滑使半徑 R 的圓弧 κ 少估約 σ²/(2R²) 量級（R=0.75 時 <1%）；
  弦截斷使 κ 略多估（h=0.25、R=0.75 時 <1%）；點抖動 σ_noise=5 mm 時 κ 標準差約 0.02～0.03 /m。
"""
import csv
import json
import math
from pathlib import Path

import numpy as np

DS = 0.05
SIGMA = 0.10
H = 0.25
MIN_RUN = 0.20          # 比這短的 run 不估計
MIN_SEG_REPORT = 0.10   # 不可行路段清單：連續超限至少這麼長才列

# 曲率上限（1/m）。左／右分開；對稱的兩邊相同。來源：任務書 T6 表格＋T3 報告
LIMITS = {
    "firmware_min_R0.75":      (1 / 0.75, 1 / 0.75),
    "planner_R0.95":           (1 / 0.95, 1 / 0.95),
    "controller_clamp_L1.05_R1.20": (1 / 1.05, 1 / 1.20),
    "measured_0829_L1.030_R1.183":  (1 / 1.030, 1 / 1.183),
    # 補充（不在任務書前 4 列，但報告引用）
    "feasibility_R0.76":       (1 / 0.76, 1 / 0.76),
    "measured_0817_L0.944_R0.751":  (1 / 0.944, 1 / 0.751),
}
MEASURED = "measured_0829_L1.030_R1.183"


# ───────────────────────── 基本運算 ─────────────────────────
def gaussian_smooth(v, sigma_samples):
    """邊緣歸一化的高斯平滑（不外插）。"""
    if sigma_samples <= 0:
        return np.asarray(v, float)
    r = int(math.ceil(3 * sigma_samples))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma_samples) ** 2)
    v = np.asarray(v, float)
    num = np.convolve(v, k, mode="same")
    den = np.convolve(np.ones_like(v), k, mode="same")
    return num / den


def menger(a, b, c):
    """三點外接圓的有號曲率（逆時針為正）。共線回 0。"""
    ab, bc, ca = b - a, c - b, a - c
    la, lb, lc = np.hypot(*ab), np.hypot(*bc), np.hypot(*ca)
    den = la * lb * lc
    if den < 1e-12:
        return 0.0
    cross = ab[0] * (c - a)[1] - ab[1] * (c - a)[0]
    return float(2.0 * cross / den)


def split_runs(points):
    """切成連續段 [(d, [point,...]), ...]。切點有兩種：
    (1) d 旗標改變（錄製／規劃端標的折返）；
    (2) 幾何折返：相鄰兩個位移向量（各 >1 cm）夾角 >90°（內積<0），即使 d 沒變。
        教導路徑的 d 旗標在人工微退時會漏標（實例 path_4714e587f6f5 s≈10.5 m：x 先 -2.64 再退回 -2.58，d 仍是 +1），
        不切的話外接圓會把這個尖點當成 R≈4 cm 的彎（κ≈20）。
    """
    runs, cur, prev_disp = [], [], None
    for p in points:
        if cur:
            dx, dy = p["x"] - cur[-1]["x"], p["y"] - cur[-1]["y"]
            moved = math.hypot(dx, dy) > 0.01
            cusp = p["d"] != cur[-1]["d"]
            if not cusp and moved and prev_disp is not None and (dx * prev_disp[0] + dy * prev_disp[1]) < 0:
                cusp = True
            if cusp:
                runs.append((cur[-1]["d"], cur))
                cur, prev_disp = [], None
            elif moved:
                prev_disp = (dx, dy)
        cur.append(p)
    if cur:
        runs.append((cur[-1]["d"], cur))
    return runs


def arclen(xy):
    seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    return np.concatenate([[0.0], np.cumsum(seg)])


def run_profile(xy, ds=DS, sigma=SIGMA, h=H):
    """單一 run 的曲率剖面。xy: (n,2)。回傳 dict(s, kappa, xy, ds, short, length) 或 None（太短）。"""
    xy = np.asarray(xy, float)
    s = arclen(xy)
    L = float(s[-1])
    if len(xy) < 2 or L < MIN_RUN:
        return {"length": L, "s": np.array([]), "kappa": np.array([]), "xy": np.zeros((0, 2)),
                "ds": ds, "short": False, "skipped": True}
    # 去掉重複點（弧長不遞增）才能內插
    keep = np.concatenate([[True], np.diff(s) > 1e-9])
    xy, s = xy[keep], s[keep]
    margin = max(h, 3 * sigma)       # 平滑核完整支撐才可信；邊緣歸一化會讓 κ 少估（R=0.9 圓弧端點實測少 6%）
    if L < 2 * margin + ds:
        a, b, c = xy[0], np.array([np.interp(L / 2, s, xy[:, 0]), np.interp(L / 2, s, xy[:, 1])]), xy[-1]
        return {"length": L, "s": np.array([L / 2]), "kappa": np.array([menger(a, b, c)]),
                "xy": b[None, :], "ds": L, "short": True, "skipped": False}
    n = int(round(L / ds)) + 1
    g = np.linspace(0.0, L, n)
    ds_eff = g[1] - g[0]
    x = np.interp(g, s, xy[:, 0])
    y = np.interp(g, s, xy[:, 1])
    sm = sigma / ds_eff
    x, y = gaussian_smooth(x, sm), gaussian_smooth(y, sm)
    k = int(round(h / ds_eff))
    km = int(math.ceil(margin / ds_eff - 1e-9))
    idx = np.arange(km, n - km)
    P = np.stack([x, y], 1)
    kap = np.array([menger(P[i - k], P[i], P[i + k]) for i in idx])
    return {"length": L, "s": g[idx], "kappa": kap, "xy": P[idx], "ds": ds_eff, "short": False, "skipped": False}


def path_profile(points, **kw):
    """整條路徑 -> 取樣列表。每列：(path_s, kappa_p, d, side, x, y, run_id, ds_w, short)。
    path_s＝跨 run 累加的弧長（折返點不計距離）。另回傳 (computed_len, skipped_len, total_len)。"""
    rows, off, computed, skipped = [], 0.0, 0.0, 0.0
    for rid, (d, pts) in enumerate(split_runs(points)):
        xy = np.array([[p["x"], p["y"]] for p in pts], float)
        pr = run_profile(xy, **kw)
        L = pr["length"]
        if pr["skipped"]:
            skipped += L
        else:
            computed += L if not pr["short"] else L
            for s, k, p in zip(pr["s"], pr["kappa"], pr["xy"]):
                side = 1 if k * d > 0 else (-1 if k * d < 0 else 0)
                w = pr["length"] if pr["short"] else pr["ds"]
                rows.append((off + float(s), float(k), d, side, float(p[0]), float(p[1]), rid, float(w), pr["short"]))
        off += L
    return rows, computed, skipped, off


SUSPECT_PEAK = 2.0      # |κ|>2.0 /m ＝ R<0.5 m：比韌體硬限 0.75 m 還緊，車子不可能走出來 -> 視為定位抖動／離散化假象


def clean_rows(rows, peak=SUSPECT_PEAK, base=1 / 0.75):
    """去掉「疑似雜訊」：同一 run 內連續 |κ|>base（超過韌體硬限）的一段，若峰值 >peak，整段丟掉。
    回傳 (rows_clean, dropped_len_m, n_stretches)。這是保守的資料品質規則，不是物理模型；原始版另外照樣統計。"""
    drop = set()
    i, n = 0, len(rows)
    while i < n:
        if abs(rows[i][1]) > base:
            j = i
            while j + 1 < n and rows[j + 1][6] == rows[i][6] and abs(rows[j + 1][1]) > base:
                j += 1
            if max(abs(r[1]) for r in rows[i:j + 1]) > peak:
                drop.update(range(i, j + 1))
                nstr = 1
            i = j + 1
        else:
            i += 1
    kept = [r for k, r in enumerate(rows) if k not in drop]
    dl = sum(rows[k][7] for k in drop)
    # 連續區段數
    nstr, prev = 0, None
    for k in sorted(drop):
        if prev is None or k != prev + 1 or rows[k][6] != rows[prev][6]:
            nstr += 1
        prev = k
    return kept, dl, nstr


# ───────────────────────── 超限統計 ─────────────────────────
def limit_for(side, lim):
    """side>0 用左上限、side<0 用右上限；side==0（κ≈0）回 inf。"""
    return lim[0] if side > 0 else (lim[1] if side < 0 else float("inf"))


def over_stats(rows, lim):
    """回傳 dict：總弧長、超限弧長、占比、最長連續超限（m），左／右分開。
    每個取樣的代表弧長 = ds_w（短 run 用整段長度）。連續＝同一 run 內相鄰取樣且皆超限。"""
    out = {"n": len(rows)}
    for name, sd in (("L", 1), ("R", -1), ("all", None)):
        tot = over = 0.0
        best = cur = 0.0
        prev_run = None
        prev_over = False
        for r in rows:
            side = r[3]
            if sd is not None and side != sd:
                prev_over = False
                continue
            w = r[7]
            tot += w
            is_over = abs(r[1]) > limit_for(side, lim)
            if is_over:
                over += w
                cur = cur + w if (prev_over and prev_run == r[6]) else w
                best = max(best, cur)
            else:
                cur = 0.0
            prev_over, prev_run = is_over, r[6]
        out[f"len_{name}"] = tot
        out[f"over_{name}"] = over
        out[f"frac_{name}"] = (over / tot) if tot > 0 else 0.0
        out[f"longest_{name}"] = best
    return out


def over_segments(rows, lim, min_len=MIN_SEG_REPORT):
    """連續超限路段（同一 run、同一 side）。回傳 [dict(s0,s1,length,kmax,side,d,x0,y0,x1,y1)]"""
    segs, cur = [], None

    def close():
        nonlocal cur
        if cur is not None:
            cur["length"] = cur["s1"] - cur["s0"] + cur["w_last"]
            segs.append(cur)
        cur = None

    for r in rows:
        s, k, d, side, x, y, rid, w, short = r
        if side != 0 and abs(k) > limit_for(side, lim):
            if cur is not None and (cur["rid"] != rid or cur["side"] != side):
                close()
            if cur is None:
                cur = {"s0": s, "s1": s, "kmax": abs(k), "side": side, "d": d, "x0": x, "y0": y, "x1": x, "y1": y,
                       "rid": rid, "w_last": w, "short": short, "dpsi": 0.0}
            else:
                cur["dpsi"] += k * (s - cur["s1"])
                cur["s1"], cur["x1"], cur["y1"], cur["w_last"] = s, x, y, w
                cur["kmax"] = max(cur["kmax"], abs(k))
        else:
            close()
    close()
    segs = [g for g in segs if g["length"] >= min_len or g["short"]]
    return segs


# ───────────────────────── 資料載入 ─────────────────────────
def load_paths(path):
    return json.load(open(path, encoding="utf-8"))["paths"]


def load_waypoints(path):
    w = json.load(open(path, encoding="utf-8"))["waypoints_db"]
    out = []
    for k, v in w.items():
        p = v["pose"]["position"]
        out.append({"id": k, "name": v["name"], "map_id": v["map_id"], "x": p["x"], "y": p["y"]})
    return out


def nearest_waypoint(wps, map_id, x, y):
    best = None
    for w in wps:
        if w["map_id"] != map_id:
            continue
        dd = math.hypot(w["x"] - x, w["y"] - y)
        if best is None or dd < best[1]:
            best = (w["name"], dd)
    return best  # None = 該地圖沒有 waypoint


def per_path_table(paths, wps):
    """逐條路徑：回傳 (summary_rows, profiles{path_id: rows(原始)}, segments[實測能力超限，含 suspect 旗標])。
    摘要的 frac_/longest_ 欄用「去雜訊」後的取樣（clean_rows）；rawfrac_ 欄是原始取樣。"""
    summ, profs, segs = [], {}, []
    for pid, v in paths.items():
        pts = v["points"]
        rows, comp, skip, tot = path_profile(pts)
        crow, dropped, nstr = clean_rows(rows)
        profs[pid] = rows
        m = v["meta"]
        rec = {"path_id": pid, "name": m.get("name"), "source": m.get("source"), "map_id": m.get("map_id"),
               "n_points": len(pts), "length_m": round(tot, 3), "computed_m": round(comp, 3), "skipped_m": round(skip, 3),
               "noise_dropped_m": round(dropped, 3), "num_cusps": m.get("num_cusps"),
               "kappa_p95_abs": round(float(np.percentile([abs(r[1]) for r in crow], 95)), 3) if crow else None,
               "kappa_max_abs": round(max((abs(r[1]) for r in crow), default=0.0), 3)}
        for ln, lim in LIMITS.items():
            st = over_stats(crow, lim)
            rec[f"frac_{ln}"] = round(st["frac_all"], 4)
            rec[f"longest_m_{ln}"] = round(st["longest_all"], 3)
        rec[f"rawfrac_{MEASURED}"] = round(over_stats(rows, LIMITS[MEASURED])["frac_all"], 4)
        rec["feasible_measured"] = rec[f"frac_{MEASURED}"] == 0.0
        summ.append(rec)
        for g in over_segments(rows, LIMITS[MEASURED]):
            nw = nearest_waypoint(wps, m.get("map_id"), g["x0"], g["y0"])
            g.update({"path_id": pid, "name": m.get("name"), "map_id": m.get("map_id"), "source": m.get("source"),
                      "suspect": g["kmax"] > SUSPECT_PEAK,
                      "wp_name": nw[0] if nw else "(該地圖無 waypoint)", "wp_dist": round(nw[1], 2) if nw else None})
            segs.append(g)
    return summ, profs, segs


# ───────────────────────── 重播對照 ─────────────────────────
def kappa_at_points(points, rows, ahead=0.5):
    """對路徑每個點，回傳 (|κ| 於該處, |κ| 往前 ahead m 的最大值, side)。該處無估計（邊緣／被跳過）回 nan。
    對應方式：以點在其 run 內的弧長，找同 run 最近的取樣。"""
    out = []
    off = 0.0
    byrun = {}
    for r in rows:
        byrun.setdefault(r[6], []).append(r)
    for rid, (d, pts) in enumerate(split_runs(points)):
        xy = np.array([[p["x"], p["y"]] for p in pts], float)
        s = arclen(xy)
        rr = byrun.get(rid, [])
        for sl in s:
            if not rr:
                out.append((float("nan"), float("nan"), 0))
                continue
            sg = off + sl
            j = min(range(len(rr)), key=lambda i: abs(rr[i][0] - sg))
            here = rr[j]
            if abs(here[0] - sg) > 0.15 and not here[8]:
                # 離最近的取樣太遠（落在邊緣被捨棄的區間）：用最近值但仍回傳（邊緣外推）
                pass
            fwd = [abs(q[1]) for q in rr if sg <= q[0] <= sg + ahead]
            out.append((abs(here[1]), max(fwd) if fwd else abs(here[1]), here[3]))
        off += float(s[-1])
    return out


BINS = [(0.0, 0.2), (0.2, 0.5), (0.5, 0.8), (0.8, 0.95), (0.95, float("inf"))]


def bin_label(lo, hi):
    return f"{lo:g}–{hi:g}" if math.isfinite(hi) else f">{lo:g}"


def rank(v):
    v = np.asarray(v, float)
    order = np.argsort(v, kind="mergesort")
    r = np.empty(len(v))
    i = 0
    while i < len(v):
        j = i
        while j + 1 < len(v) and v[order[j + 1]] == v[order[i]]:
            j += 1
        r[order[i:j + 1]] = (i + j) / 2.0 + 1
        i = j + 1
    return r


def spearman(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) < 3 or np.ptp(x) == 0 or np.ptp(y) == 0:
        return float("nan")
    rx, ry = rank(x), rank(y)
    return float(np.corrcoef(rx, ry)[0, 1])


def spearman_perm_p(x, y, n_perm=5000, seed=0):
    rho = spearman(x, y)
    if rho != rho:
        return rho, float("nan")
    rng = np.random.default_rng(seed)
    y = np.asarray(y, float)
    cnt = sum(1 for _ in range(n_perm) if abs(spearman(x, rng.permutation(y))) >= abs(rho) - 1e-12)
    return rho, (cnt + 1) / (n_perm + 1)


def load_replay(path):
    rows = []
    with open(path, newline="", encoding="utf-8-sig") as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append({"t": float(r["t"]), "index": int(r["index"]), "total": int(r["total"]),
                             "ct": float(r["cross_track_m"]), "state": r["state"], "msg": r.get("message") or ""})
            except (ValueError, KeyError):
                continue
    return rows


def usable(r):
    """排除 state=escaping（cross_track 欄位混用，恆為 0）與 nan。"""
    return r["state"] != "escaping" and r["ct"] == r["ct"]


def bin_table(samples):
    """samples: [(kappa_abs, cross_track)] -> 每個 bin 的 n／median／p95。"""
    out = []
    for lo, hi in BINS:
        v = [c for k, c in samples if k == k and lo <= k < hi]
        out.append({"bin": bin_label(lo, hi), "n": len(v),
                    "median": float(np.median(v)) if v else None, "p95": float(np.percentile(v, 95)) if v else None})
    return out


def events(rows):
    """state 變成 blocked／escaping 的「進入事件」（連續同狀態只算一次）。回傳 [(state, index)]"""
    ev, prev = [], None
    for r in rows:
        if r["state"] in ("blocked", "escaping") and r["state"] != prev:
            ev.append((r["state"], r["index"]))
        prev = r["state"]
    return ev
