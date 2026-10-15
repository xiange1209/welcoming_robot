#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T6 主程式：讀路徑庫與重播 CSV，輸出 per-path 摘要、不可行路段、全庫統計、重播分箱（JSON＋CSV）。

用法（專案根目錄）：
    python3 tools/curvature/run_t6.py [--out docs/cloud/reports]
輸出：T6_per_path_summary.csv、T6_infeasible_segments.csv、T6_results.json（報告引用的所有數字）。
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import curvature_lib as C  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "maprun"))


def corner_width(R):
    """maprun/corner_check_cc.py 的最小走廊淨寬（90° 直角彎、零誤差）。"""
    import corner_check_cc as cc
    return cc.geometry(R)[0]


def library_stats(summ, profs, paths):
    out = {}
    uniq = {}
    for rec in summ:
        uniq.setdefault(rec["name"], []).append(rec)
    for ln, lim in C.LIMITS.items():
        fr = [r[f"frac_{ln}"] for r in summ]
        ls = [r[f"longest_m_{ln}"] for r in summ]
        n_over = sum(1 for f in fr if f > 0)
        # 全庫合併：所有路徑取樣的弧長加權
        tot = {k: 0.0 for k in ("len_L", "over_L", "len_R", "over_R", "len_all", "over_all")}
        longest = {"L": 0.0, "R": 0.0, "all": 0.0}
        raw_tot = {"len": 0.0, "over": 0.0}
        n_raw = 0
        for pid, rows in profs.items():
            rs = C.over_stats(rows, lim)
            raw_tot["len"] += rs["len_all"]; raw_tot["over"] += rs["over_all"]
            n_raw += 1 if rs["over_all"] > 0 else 0
            rows, _, _ = C.clean_rows(rows)
            st = C.over_stats(rows, lim)
            for k in tot:
                tot[k] += st[k]
            for s in longest:
                longest[s] = max(longest[s], st[f"longest_{s}"])
        # 大於 0.3 m 連續超限的路徑數
        n_long = sum(1 for l in ls if l >= 0.30)
        out[ln] = {"raw_n_paths_with_overlimit": n_raw, "raw_pool_frac_all": raw_tot["over"] / raw_tot["len"],
                   "n_paths": len(summ), "n_paths_with_overlimit": n_over, "n_fully_feasible": len(summ) - n_over,
                   "n_paths_longest_ge_0.30m": n_long,
                   "pool_frac_all": tot["over_all"] / tot["len_all"] if tot["len_all"] else 0,
                   "pool_frac_L": tot["over_L"] / tot["len_L"] if tot["len_L"] else 0,
                   "pool_frac_R": tot["over_R"] / tot["len_R"] if tot["len_R"] else 0,
                   "pool_len_L": tot["len_L"], "pool_len_R": tot["len_R"],
                   "longest_L": longest["L"], "longest_R": longest["R"], "longest_all": longest["all"],
                   "median_frac_all": float(np.median(fr)), "p90_frac_all": float(np.percentile(fr, 90))}
        # 去重（同名路徑只算一次，取該名稱最後一條）：_auto_ 路徑同名會重複規劃
        un = [v[-1] for v in uniq.values()]
        out[ln]["n_unique_names"] = len(un)
        out[ln]["n_unique_with_overlimit"] = sum(1 for r in un if r[f"frac_{ln}"] > 0)
    return out


def replay_analysis(paths, profs):
    res = {"trips": [], "pooled": {}}
    pooled_rows = []
    for f in sorted(glob.glob(str(ROOT / "docs/cloud/evidence/replay/*.csv"))):
        m = re.search(r"path_([0-9a-f]+)", os.path.basename(f))
        pid = "path_" + m.group(1)
        rows = C.load_replay(f)
        pts = paths[pid]["points"]
        kap = C.kappa_at_points(pts, profs[pid])
        trip = {"file": os.path.basename(f), "path_id": pid, "n_rows": len(rows)}
        use = [r for r in rows if C.usable(r)]
        trip["n_usable"] = len(use)
        trip["n_escaping"] = sum(1 for r in rows if r["state"] == "escaping")
        trip["n_nan_or_other_excluded"] = len(rows) - len(use) - trip["n_escaping"]
        trip["index_range"] = [min(r["index"] for r in rows), max(r["index"] for r in rows)]
        samples, samples_ahead = [], []
        for r in use:
            i = min(max(r["index"], 0), len(kap) - 1)
            samples.append((kap[i][0], r["ct"]))
            samples_ahead.append((kap[i][1], r["ct"]))
            pooled_rows.append({"trip": trip["file"], "index": r["index"], "ct": r["ct"], "k": kap[i][0], "ka": kap[i][1],
                                "state": r["state"], "msg": r["msg"]})
        trip["bins_here"] = C.bin_table(samples)
        trip["bins_ahead"] = C.bin_table(samples_ahead)
        ev = C.events(rows)
        trip["events"] = [{"state": s, "index": i, "kappa": kap[min(i, len(kap) - 1)][0],
                           "kappa_ahead": kap[min(i, len(kap) - 1)][1], "side": kap[min(i, len(kap) - 1)][2]} for s, i in ev]
        trip["by_index_k"] = None
        res["trips"].append(trip)
    # 合併
    pb = [(r["k"], r["ct"]) for r in pooled_rows]
    res["pooled"]["bins_here"] = C.bin_table(pb)
    res["pooled"]["bins_ahead"] = C.bin_table([(r["ka"], r["ct"]) for r in pooled_rows])
    k = [r["k"] for r in pooled_rows]; c = [r["ct"] for r in pooled_rows]
    res["pooled"]["n"] = len(pooled_rows)
    res["pooled"]["spearman_rows"] = C.spearman(k, c)
    res["pooled"]["spearman_rows_ahead"] = C.spearman([r["ka"] for r in pooled_rows], c)
    # 以「路徑索引」聚合（每個 index 取各趟 cross_track 的中位數）以減少 20 Hz 重複取樣的假大樣本
    by = {}
    for r in pooled_rows:
        by.setdefault((r["trip"], r["index"]), []).append(r)
    ik, ic = [], []
    for (t, i), v in by.items():
        ik.append(v[0]["k"]); ic.append(float(np.median([x["ct"] for x in v])))
    rho, p = C.spearman_perm_p(ik, ic)
    res["pooled"]["n_trip_index_cells"] = len(ik)
    res["pooled"]["spearman_cells"] = rho
    res["pooled"]["spearman_cells_perm_p"] = p
    res["pooled"]["cells_bins"] = C.bin_table(list(zip(ik, ic)))
    res["pooled"]["distinct_indices_covered"] = len({i for (_, i) in by})
    # 各趟單獨 Spearman（cells）
    for trip in res["trips"]:
        cells = {}
        for r in pooled_rows:
            if r["trip"] == trip["file"]:
                cells.setdefault(r["index"], []).append(r)
        tk = [v[0]["k"] for v in cells.values()]
        tc = [float(np.median([x["ct"] for x in v])) for v in cells.values()]
        trip["n_cells"] = len(tk)
        trip["spearman_cells"], trip["spearman_cells_p"] = C.spearman_perm_p(tk, tc)
    # 排除第 1 趟（只在 index 34–42 原地打轉，沒有沿路徑前進；起點離路徑 >1 m，cross_track 反映的是起點偏差）
    t1 = res["trips"][0]["file"]
    ex = [r for r in pooled_rows if r["trip"] != t1]
    res["pooled_excl_trip1"] = {"n": len(ex), "bins_here": C.bin_table([(r["k"], r["ct"]) for r in ex]),
                                "bins_ahead": C.bin_table([(r["ka"], r["ct"]) for r in ex]),
                                "spearman_rows": C.spearman([r["k"] for r in ex], [r["ct"] for r in ex])}
    byx = {}
    for r in ex:
        byx.setdefault((r["trip"], r["index"]), []).append(r)
    xk = [v[0]["k"] for v in byx.values()]; xc = [float(np.median([x["ct"] for x in v])) for v in byx.values()]
    res["pooled_excl_trip1"]["n_cells"] = len(xk)
    res["pooled_excl_trip1"]["spearman_cells"], res["pooled_excl_trip1"]["spearman_cells_p"] = C.spearman_perm_p(xk, xc)
    res["pooled_excl_trip1"]["cells_bins"] = C.bin_table(list(zip(xk, xc)))
    # blocked／escaping 進入事件落在哪個 κ 區間（以「趟＋index＋狀態」去重，避免 blocked<->escaping 來回重複計）
    evs = {}
    for trip in res["trips"]:
        for e in trip["events"]:
            evs.setdefault((trip["file"], e["state"], e["index"]), e)
    cnt = {}
    for (f, st, i), e in evs.items():
        for lo, hi in C.BINS:
            if lo <= e["kappa"] < hi:
                cnt.setdefault(C.bin_label(lo, hi), {"blocked": 0, "escaping": 0})[st] += 1
    res["events_unique_by_bin"] = cnt
    res["events_unique_total"] = len(evs)
    # 訊息裡的「★飽和」旗標：節點自己判定曲率飽和（|ω|≥98% 上限）
    sat = [r for r in pooled_rows if "★飽和" in r["msg"]]
    nsat = [r for r in pooled_rows if r["msg"] and "★飽和" not in r["msg"] and "曲率" in r["msg"]]
    res["pooled"]["sat_rows"] = {"n_sat": len(sat), "ct_median_sat": float(np.median([r["ct"] for r in sat])) if sat else None,
                                 "n_not_sat": len(nsat), "ct_median_not_sat": float(np.median([r["ct"] for r in nsat])) if nsat else None,
                                 "sat_indices": sorted({r["index"] for r in sat}),
                                 "my_k_at_sat_rows_median": float(np.nanmedian([r["k"] for r in sat])) if sat else None,
                                 "my_k_ahead_at_sat_rows_median": float(np.nanmedian([r["ka"] for r in sat])) if sat else None,
                                 "frac_sat_rows_with_k_ahead_ge_0.8": float(np.mean([r["ka"] >= 0.8 for r in sat if r["ka"] == r["ka"]])) if sat else None,
                                 "frac_notsat_rows_with_k_ahead_ge_0.8": float(np.mean([r["ka"] >= 0.8 for r in nsat if r["ka"] == r["ka"]])) if nsat else None}
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--paths", default=str(ROOT / ".smartnav/path_database/paths.json"))
    ap.add_argument("--waypoints", default=str(ROOT / ".smartnav/waypoint_database/waypoints.json"))
    ap.add_argument("--out", default=str(ROOT / "docs/cloud/reports"))
    a = ap.parse_args()
    paths = C.load_paths(a.paths)
    wps = C.load_waypoints(a.waypoints)
    summ, profs, segs = C.per_path_table(paths, wps)
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)

    cols = list(summ[0].keys())
    with open(out / "T6_per_path_summary.csv", "w", newline="", encoding="utf-8-sig") as fh:
        w = csv.DictWriter(fh, fieldnames=cols); w.writeheader(); w.writerows(summ)

    # 不可行路段：依最大 |κ| 由大到小
    for g in segs:
        R = 1 / g["kmax"]
        g["R_needed"] = R
        g["turn_deg"] = math.degrees(abs(g["dpsi"]))
        g["corner_min_width_at_Rneed"] = corner_width(max(R, 0.3))
    segs.sort(key=lambda g: (g["suspect"], -g["kmax"], -g["length"]))   # 非疑似雜訊的排前面，再依最大 |κ|
    scols = ["path_id", "name", "map_id", "s0", "s1", "length", "x0", "y0", "side", "d", "kmax", "R_needed", "turn_deg",
             "corner_min_width_at_Rneed", "wp_name", "wp_dist", "short", "suspect", "source"]
    with open(out / "T6_infeasible_segments.csv", "w", newline="", encoding="utf-8-sig") as fh:
        w = csv.writer(fh); w.writerow(["rank"] + scols)
        for i, g in enumerate(segs, 1):
            row = []
            for c in scols:
                v = g.get(c)
                if c == "side":
                    v = "左輪" if v > 0 else "右輪"
                if c == "d":
                    v = "前進" if v > 0 else "倒車"
                if isinstance(v, float):
                    v = round(v, 3)
                row.append(v)
            w.writerow([i] + row)

    res = {"library": library_stats(summ, profs, paths), "n_segments_measured": len(segs),
           "n_segments_suspect": sum(1 for g in segs if g["suspect"]),
           "n_segments_clean": sum(1 for g in segs if not g["suspect"]),
           "n_segments_clean_by_len": {t: sum(1 for g in segs if not g["suspect"] and g["length"] >= t) for t in (0.1, 0.3, 0.5)},
           "n_paths_clean_with_segment": len({g["path_id"] for g in segs if not g["suspect"]}),
           "replay": replay_analysis(paths, profs)}
    # 資料品質：yaw 與行進方向是否吻合、短 run 比例
    mism = tot = 0
    for v in paths.values():
        P = v["points"]
        for p, q in zip(P, P[1:]):
            dx, dy = q["x"] - p["x"], q["y"] - p["y"]
            if math.hypot(dx, dy) < 0.03:
                continue
            tot += 1
            exp = 0.0 if p["d"] > 0 else math.pi
            diff = math.atan2(dy, dx) - p["yaw"] - exp
            if abs(math.atan2(math.sin(diff), math.cos(diff))) > 1.0:
                mism += 1
    res["data_quality"] = {"yaw_vs_motion_mismatch": mism, "pairs": tot,
                           "total_len_m": sum(r["length_m"] for r in summ),
                           "computed_len_m": sum(r["computed_m"] for r in summ),
                           "skipped_len_m": sum(r["skipped_m"] for r in summ),
                           "n_sources": {s: sum(1 for r in summ if r["source"] == s) for s in {r["source"] for r in summ}}}
    # 依來源分群
    for src in ("plan", "teach"):
        sub = [r for r in summ if r["source"] == src]
        key = f"frac_{C.MEASURED}"
        res["data_quality"][f"{src}_n"] = len(sub)
        res["data_quality"][f"{src}_with_overlimit_measured"] = sum(1 for r in sub if r[key] > 0)
    res["segments_top20"] = [{k: (v if not isinstance(v, float) else round(v, 3)) for k, v in g.items()
                              if k in ("path_id", "name", "s0", "s1", "length", "x0", "y0", "side", "d", "kmax", "R_needed",
                                       "turn_deg", "corner_min_width_at_Rneed", "wp_name", "wp_dist", "short", "source")} for g in segs[:20]]
    # 規劃器路徑（source=plan）依建立日期分期：8/17 起規劃器半徑 0.80->0.95（planner-symmetric-turning-radius 筆記）
    per = {}
    for pid, v in paths.items():
        if v["meta"].get("source") != "plan":
            continue
        d = v["meta"]["created_at"][:10]
        wk = "<=08-16" if d <= "2026-08-16" else ("08-17..08-28" if d <= "2026-08-28" else ">=08-29")
        crow, _, _ = C.clean_rows(profs[pid])
        st = C.over_stats(crow, C.LIMITS[C.MEASURED])
        e = per.setdefault(wk, {"n": 0, "n_over": 0, "kmax": []})
        e["n"] += 1; e["n_over"] += 1 if st["over_all"] > 0 else 0
        e["kmax"].append(max((abs(x[1]) for x in crow), default=0.0))
    res["plan_by_period"] = {k: {"n": v["n"], "n_overlimit_measured": v["n_over"], "median_kmax": round(float(np.median(v["kmax"])), 3)} for k, v in per.items()}
    clean = [g for g in segs if not g["suspect"]]
    byw = {}
    for g in clean:
        near = g["wp_name"] if (g["wp_dist"] is not None and g["wp_dist"] <= 1.0) else "(>1 m 無 waypoint)"
        byw.setdefault(near, {"n": 0, "n_ge_0.3m": 0, "paths": set()})
        byw[near]["n"] += 1; byw[near]["n_ge_0.3m"] += 1 if g["length"] >= 0.3 else 0; byw[near]["paths"].add(g["path_id"])
    res["segments_by_waypoint"] = {k: {"n": v["n"], "n_ge_0.3m": v["n_ge_0.3m"], "n_paths": len(v["paths"])} for k, v in byw.items()}
    # corner_check：各半徑的最小走廊淨寬；0.967 m 門口（含 p95 循跡誤差 0.175）可通過的最大半徑
    res["corner_check"] = {f"R={R}": round(corner_width(R), 3) for R in (0.75, 0.836, 0.944, 1.030, 1.05, 1.183, 1.20)}
    lo, hi = 0.5, 1.5
    for _ in range(40):
        mid = (lo + hi) / 2
        if corner_width(mid) + 0.175 <= 0.967:
            lo = mid
        else:
            hi = mid
    res["corner_check"]["door_0.967_max_R_with_track_err_0.175"] = round(lo, 3)
    res["corner_check"]["door_0.967_max_R_geometry_only"] = None
    lo, hi = 0.5, 1.5
    for _ in range(40):
        mid = (lo + hi) / 2
        if corner_width(mid) <= 0.967:
            lo = mid
        else:
            hi = mid
    res["corner_check"]["door_0.967_max_R_geometry_only"] = round(lo, 3)
    json.dump(res, open(out / "T6_results.json", "w", encoding="utf-8"), ensure_ascii=False, indent=1, default=float)
    print(json.dumps({k: res[k] for k in ("n_segments_measured", "data_quality")}, ensure_ascii=False, indent=1, default=float))


if __name__ == "__main__":
    main()
