#!/usr/bin/env python3
"""循跡模擬評估（★ 2026-10-23 車上 agent）：規劃半徑 × 控制器 × 干擾情境，全部用 plan_radius_bench 存下來的真實規劃路徑。
用法：python3 eval_tracking.py <plan_radius 輸出目錄> [--sets 0.95,1.20] [--modes base,interp,pi] [--seeds 2] [--jobs 3]
輸出：<目錄>/track_eval.json、track_eval.txt
"""
import argparse
import json
import os
import statistics
import sys
import time
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import track_sim as ts  # noqa: E402

SCENARIOS = {
    "標準": dict(veh=dict(bias_res=0.03), loc=dict()),
    "trim補不足+0.08": dict(veh=dict(bias_res=0.08), loc=dict()),
    "trim補過頭-0.08": dict(veh=dict(bias_res=-0.08), loc=dict()),
    "舵機欠轉g0.85": dict(veh=dict(bias_res=0.03, g=0.85), loc=dict()),
    "舵機慢τ0.4+2拍": dict(veh=dict(bias_res=0.03, tau_s=0.4, delay_ticks=2), loc=dict()),
    "定位差4cm/2°": dict(veh=dict(bias_res=0.03), loc=dict(amcl_xy=0.04, amcl_yaw_deg=2.0)),
}


def one(job):
    key, pts, mode, scen, seed = job
    sc = SCENARIOS[scen]
    r = ts.run(pts, ts.Controller(mode), veh_kw=sc["veh"], loc_kw=sc["loc"], seed=seed)
    e = r["errs"]
    return dict(key=key, mode=mode, scen=scen, seed=seed, ok=r["ok"], why=r["why"], t=round(r["t"], 1),
                e_med=statistics.median(e) if e else 0.0, e_p95=sorted(e)[int(0.95 * (len(e) - 1))] if e else 0.0,
                e_max=max(e) if e else 0.0, node_med=statistics.median(r["node_xte"]) if r["node_xte"] else 0.0,
                node_max=max(r["node_xte"]) if r["node_xte"] else 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--sets", default="0.95,1.20")
    ap.add_argument("--modes", default="base,interp,pi")
    ap.add_argument("--scen", default=",".join(SCENARIOS))
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--jobs", type=int, default=3)
    a = ap.parse_args()
    paths = json.load(open(os.path.join(a.dir, "paths.json")))
    sets = a.sets.split(",")
    jobs = [(k, pts, m, s, seed) for k, pts in paths.items() if k.split("|")[0] in sets
            for m in a.modes.split(",") for s in a.scen.split(",") for seed in range(a.seeds)]
    t0 = time.time()
    with Pool(a.jobs) as pool:
        res = pool.map(one, jobs, chunksize=4)
    print(f"{len(res)} 次模擬，{time.time() - t0:.0f} 秒")
    lines = ["規劃半徑 | 控制器 | 情境 | 成功 | 真實偏離 中位／p95／最大（cm）| 節點顯示 xte 中位／最大（cm）| 平均時間 s"]
    table = []
    for rs in sets:
        for m in a.modes.split(","):
            for s in a.scen.split(","):
                g = [r for r in res if r["key"].startswith(rs + "|") and r["mode"] == m and r["scen"] == s]
                if not g:
                    continue
                okn = sum(r["ok"] for r in g)
                row = dict(radius=rs, mode=m, scen=s, n=len(g), ok=okn,
                           e_med=statistics.median(r["e_med"] for r in g) * 100,
                           e_p95=statistics.median(r["e_p95"] for r in g) * 100,
                           e_max=max(r["e_max"] for r in g) * 100,
                           e_max_med=statistics.median(r["e_max"] for r in g) * 100,
                           node_med=statistics.median(r["node_med"] for r in g) * 100,
                           node_max=statistics.median(r["node_max"] for r in g) * 100,
                           t=statistics.mean(r["t"] for r in g if r["ok"]) if okn else 0.0,
                           fails=sorted({f"{r['key'].split('|', 1)[1]}（{r['why']}）" for r in g if not r["ok"]}))
                table.append(row)
                lines.append(f"{rs} | {m:<6} | {s:<14} | {okn}/{len(g)} | {row['e_med']:.1f}／{row['e_p95']:.1f}／"
                             f"{row['e_max']:.1f}（每條最大的中位 {row['e_max_med']:.1f}）| {row['node_med']:.1f}／"
                             f"{row['node_max']:.1f} | {row['t']:.0f}")
    txt = "\n".join(lines)
    print(txt)
    open(os.path.join(a.dir, "track_eval.txt"), "w", encoding="utf-8").write(txt + "\n")
    json.dump(dict(table=table, runs=res), open(os.path.join(a.dir, "track_eval.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
