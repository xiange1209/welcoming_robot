#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 T6_results.json 排成報告用的 Markdown 表格（全庫統計、前 20 條不可行路段、重播分箱）。
用法：python3 tools/curvature/format_tables.py [docs/cloud/reports/T6_results.json]"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NAMES = {
    "firmware_min_R0.75": "韌體設計最小半徑 R=0.75（κ 1.333，左右同）",
    "planner_R0.95": "規劃器 minimum_turning_radius R=0.95（κ 1.053，左右同）",
    "controller_clamp_L1.05_R1.20": "path_teach 箝制 左 1.05／右 1.20（κ 0.952／0.833）",
    "measured_0829_L1.030_R1.183": "8/29 實測滿舵 左 1.030／右 1.183（κ 0.971／0.845）",
    "feasibility_R0.76": "〔補充〕可行性門檻 R=0.76（κ 1.316）",
    "measured_0817_L0.944_R0.751": "〔補充〕8/17 實測 左 0.944／右 0.751（會漂）",
}


def f(x, nd=3):
    return "—" if x is None else (f"{x:.{nd}f}" if isinstance(x, float) else str(x))


def main():
    p = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs/cloud/reports/T6_results.json"
    r = json.load(open(p, encoding="utf-8"))
    print("### 全庫超限統計（122 條路徑；去雜訊後；括號內為原始未去雜訊）\n")
    print("| 上限 | 完全可行路徑 | 有超限路徑（原始）| 最長連續超限 ≥0.3 m 的路徑 | 全庫超限弧長占比 合併／左／右 | 最長連續超限 左／右 (m) | 以名稱去重後有超限 |")
    print("|---|---:|---:|---:|---:|---:|---:|")
    for k, v in r["library"].items():
        print(f"| {NAMES[k]} | {v['n_fully_feasible']} | {v['n_paths_with_overlimit']}（{v['raw_n_paths_with_overlimit']}）| {v['n_paths_longest_ge_0.30m']} | "
              f"{v['pool_frac_all'] * 100:.1f}%／{v['pool_frac_L'] * 100:.1f}%／{v['pool_frac_R'] * 100:.1f}% | {v['longest_L']:.2f}／{v['longest_R']:.2f} | {v['n_unique_with_overlimit']}／{v['n_unique_names']} |")
    print("\n### 不可行路段（對實測能力 左 0.971／右 0.845 超限）前 20 條\n")
    print("| # | path_id | 路徑名稱 | 來源 | 起訖弧長 (m) | 長度 (m) | 起點 (x, y) | 輪子 | 方向 | 最大\\|κ\\| | 需要 R (m) | 轉角 (°) | 最近 waypoint（距離 m）| corner_check 對應最小淨寬 (m) |")
    print("|---:|---|---|---|---|---:|---|---|---|---:|---:|---:|---|---:|")
    for i, g in enumerate(r["segments_top20"], 1):
        turn = "—" if g["short"] else f"{g['turn_deg']:.0f}"
        print(f"| {i} | {g['path_id']} | {g['name']} | {g['source']}{'/短' if g['short'] else ''} | {g['s0']:.2f}–{g['s1']:.2f} | {g['length']:.2f} | ({g['x0']:.2f}, {g['y0']:.2f}) | "
              f"{'左' if g['side'] > 0 else '右'} | {'前進' if g['d'] > 0 else '倒車'} | {g['kmax']:.2f} | {g['R_needed']:.2f} | {turn} | {g['wp_name']}（{f(g['wp_dist'], 2)}）| {g['corner_min_width_at_Rneed']:.2f} |")
    rp = r["replay"]
    for key, title in (("pooled", "三趟全部（含第 1 趟）"), ("pooled_excl_trip1", "排除第 1 趟")):
        q = rp[key]
        print(f"\n### 重播 cross-track × 需要的 |κ|（{title}；列＝state≠escaping 且非 nan，n={q['n']}）\n")
        print("| 需要的 \\|κ\\| (1/m) | 列數 | cross-track 中位 (m) | p95 (m) | 〔以路徑點×趟聚合〕格數 | 中位 | p95 |")
        print("|---|---:|---:|---:|---:|---:|---:|")
        for a, b in zip(q["bins_here"], q["cells_bins"]):
            print(f"| {a['bin']} | {a['n']} | {f(a['median'])} | {f(a['p95'])} | {b['n']} | {f(b['median'])} | {f(b['p95'])} |")
    print()
    for t in rp["trips"]:
        print(f"- {t['file']}：列 {t['n_rows']}、可用 {t['n_usable']}、escaping 排除 {t['n_escaping']}、index {t['index_range'][0]}–{t['index_range'][1]}；"
              f"Spearman(格) ρ={t['spearman_cells']:.2f}（n={t['n_cells']}，排列 p={t['spearman_cells_p']:.4f}）")


if __name__ == "__main__":
    main()
