#!/usr/bin/env python3
"""自動建圖（frontier 探索）這一趟卡在哪裡——讀導航 log，一次印出來（唯讀）。

★ 2026-10-28 新寫。使用者：「自動建圖感覺目前不太行」「大部分都是我手動遙控的」。
  10/27 那趟（nav_20261027_175525.log，探索 13 分鐘）：23 個目標只有 4 個到達，
  10 個在 1~6 秒內 ABORTED（error_code=102＝控制器 TF_ERROR，map->odom 落後 44~115 ms），
  「no valid path found」38 次（規劃半徑 1.40 太大；10/24 以前是 0.95）。
  10/28 的修正：slam transform_timeout 0.2→1.0、探索期間規劃半徑暫時 0.95（map_service_cc 的 explore_planner_radius_m）。

用法：
    python3 ~/maprun/tools_1020/explore_summary.py                 # 最新一個有探索的導航 log
    python3 ~/maprun/tools_1020/explore_summary.py <導航 log 路徑>   # 指定（例如拿 10/27 那份當對照）
"""
import datetime
import glob
import os
import re
import sys

LOGS = os.path.expanduser("~/.smartnav/logs/system_control/nav_*.log")
PAT = re.compile(r"\[(\d{10}\.\d+)\] \[([A-Za-z_]+)\]: (.*)")
LAG = re.compile(r"Requested time ([0-9.]+) but the latest data is at time ([0-9.]+)")
CODE = re.compile(r"frontier finished with status (\w+) \((\d+)\), error_code=(\d+)")
# FollowPath 的錯誤碼（nav2_msgs/action/FollowPath）
FOLLOW_CODES = {100: "UNKNOWN", 101: "INVALID_CONTROLLER", 102: "TF_ERROR", 103: "INVALID_PATH",
                104: "PATIENCE_EXCEEDED", 105: "FAILED_TO_MAKE_PROGRESS", 106: "NO_VALID_CONTROL"}


def pick_log(argv):
    if len(argv) > 1:
        return argv[1]
    for f in sorted(glob.glob(LOGS), key=os.path.getmtime, reverse=True):
        try:
            with open(f, encoding="utf-8", errors="replace") as fh:
                if "Frontier goal accepted" in fh.read():
                    return f
        except OSError:
            pass
    return None


def main():
    path = pick_log(sys.argv)
    if not path:
        print("✗ 找不到有自動探索的導航 log（~/.smartnav/logs/system_control/nav_*.log）")
        return 1
    t_start = t_end = None
    sent = reached = suppressed = blocked = noplan = timeout_cost = 0
    aborted = {}
    lags = []
    radius = []
    targets = {}
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = PAT.search(line)
            if not m:
                continue
            t, node, msg = float(m.group(1)), m.group(2), m.group(3)
            if "自動探索已啟動" in msg or "已手動啟動自動探索" in msg:
                t_start = t_start or t
            elif "已停止自動探索" in msg:
                t_end = t
            elif "Sending frontier goal" in msg:
                sent += 1
                ref = re.search(r"frontier reference=\(([^)]*)\)", msg)
                if ref:
                    targets[ref.group(1)] = targets.get(ref.group(1), 0) + 1
            elif "reached by distance" in msg or "STATUS_SUCCEEDED" in msg:
                reached += 1
            elif "frontier finished with status" in msg:
                c = CODE.search(msg)
                if c and c.group(1) != "STATUS_SUCCEEDED":
                    k = f"{c.group(1)}/{FOLLOW_CODES.get(int(c.group(3)), c.group(3))}"
                    aborted[k] = aborted.get(k, 0) + 1
            elif "temporarily suppressed" in msg:
                suppressed += 1
            elif "blocked in global costmap" in msg:
                blocked += 1
            elif "no valid path found" in msg:
                noplan += 1
            elif "Costmap timed out" in msg:
                timeout_cost += 1
            elif "規劃半徑" in msg and node == "map_service_cc_node":
                radius.append(msg)
            lg = LAG.search(msg)
            if lg:
                lags.append(float(lg.group(1)) - float(lg.group(2)))

    fmt = lambda x: datetime.datetime.fromtimestamp(x).strftime("%H:%M:%S") if x else "?"
    dur = (t_end - t_start) / 60 if t_start and t_end else None
    print(f"══ 自動探索摘要：{os.path.basename(path)} ══")
    print(f"  探索期間：{fmt(t_start)} → {fmt(t_end)}" + (f"（{dur:.1f} 分）" if dur else ""))
    print(f"  探索時的規劃半徑：{'；'.join(radius) if radius else '（log 沒有改半徑的紀錄＝舊版，或 explore_planner_radius_m=0）'}")
    print(f"  送出目標 {sent} 個；到達 {reached}；中止 {sum(aborted.values())}"
          + (f"（{'、'.join(f'{k} {v}' for k, v in sorted(aborted.items()))}）" if aborted else ""))
    if lags:
        lags.sort()
        print(f"  ├ TF 落後（控制器查到未來）：{len(lags)} 次，落後 {lags[0] * 1000:.0f}~{lags[-1] * 1000:.0f} ms"
              "（＝slam 的 map->odom 太舊；10/28 起 transform_timeout 1.0：落後 <0.8 秒的這類應該歸零，秒級的是 slam 卡住）")
    print(f"  ├ 規劃不出路徑（no valid path found）：{noplan} 次；成本地圖逾時 {timeout_cost} 次")
    print(f"  ├ 目標被擋（blocked in global costmap）：{blocked} 次；全部前緣暫時被壓下：{suppressed} 次")
    rep = sorted(((v, k) for k, v in targets.items() if v >= 3), reverse=True)
    if rep:
        print("  └ 同一個前緣一直重送（≥3 次）：" + "、".join(f"({k}) ×{v}" for v, k in rep))
    print("  判讀：中止多且是 TF_ERROR → 看 slam 的 transform_timeout；no valid path 多 → 規劃半徑／目標在牆邊；")
    print("        到達少但沒什麼錯誤 → 前緣都被壓下（地圖其實已經沒有可去的地方，或前緣在車子進不去的窄處）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
