#!/usr/bin/env python3
"""每一趟帶位整理成一行（唯讀；只讀 log 與逐拍 CSV）——day_test.sh 的導航 A/B 用，也可以單獨跑。

一趟＝導航層 log 的「開始導航到 X」→「✓ 導航成功（終點誤差 …）」或「導航失敗: …」，中間的「重新規劃第 i/3 次」算同一趟。
偏離量取同一段時間的 ~/maprun/logs/replay_*_live_*.csv（xte_m 欄）。

用法：
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/nav_result_line.py                 # 最近一趟
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/nav_result_line.py --since 1793000000   # 這個時間（epoch 秒）之後的每一趟
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/nav_result_line.py --log <nav_*.log> --all   # 指定某一份 log 的全部
    ... --tsv 檔案 --tag 新   # 另外把每一趟附加成一列 TSV（day_test.sh 用）

★ 2026-10-26 車上新寫；用 10/20 實車 log 驗過解析。
"""
import argparse
import csv
import glob
import os
import re
import statistics
import time

LOG_DIR = os.path.expanduser("~/.smartnav/logs/system_control")
CSV_GLOB = os.path.expanduser("~/maprun/logs/replay_*_live_*.csv")
LINE = re.compile(r"\[(\d{10}\.\d+)\] \[(navigation_action_cc_node|path_teach_cc_node)\]: (.*)$")


def parse(paths):
    ev = []
    for p in paths:
        with open(p, encoding="utf-8", errors="replace") as fh:
            for ln in fh:
                m = LINE.search(ln.rstrip("\n"))
                if m:
                    ev.append((float(m.group(1)), m.group(2), m.group(3)))
    ev.sort()
    runs, cur, folds = [], None, None
    for t, node, msg in ev:
        if node == "navigation_action_cc_node":
            m = re.search(r"規劃完成：.*?(\d+) 個折返", msg)
            if m and cur is None:
                folds = int(m.group(1))
            m = re.match(r"開始導航到 (.+?)(（|$)", msg)
            if m and cur is None:
                cur = {"start": t, "target": m.group(1), "replans": 0, "folds": folds, "cand": 0, "degrade": 0}
                folds = None
                continue
            if cur is None:
                continue
            if msg.startswith("重新規劃第"):
                cur["replans"] += 1
            elif "導航成功" in msg:
                m = re.search(r"終點誤差 ([\d.]+) m、朝向差 ([\d.]+) 度", msg)
                cur.update(end=t, ok=True, err=float(m.group(1)) if m else None, yaw=float(m.group(2)) if m else None)
                runs.append(cur)
                cur = None
            elif msg.startswith("導航失敗"):
                cur.update(end=t, ok=False, reason=re.sub(r"^導航失敗[:：]\s*(導航失敗[:：])?\s*", "", msg)[:60])
                runs.append(cur)
                cur = None
        elif cur is not None:
            if "多候選規劃" in msg:
                cur["cand"] += 1
            elif "暫時降到" in msg:
                cur["degrade"] += 1
    if cur is not None:
        cur.update(end=None, ok=None)
        runs.append(cur)
    return runs


def xte(start, end):
    vals = []
    for p in glob.glob(CSV_GLOB):
        if os.path.getmtime(p) < start - 5:
            continue
        try:
            with open(p, encoding="utf-8-sig") as fh:
                for r in csv.DictReader(fh):
                    try:
                        t = float(r.get("epoch") or "nan")
                        if start <= t <= (end or time.time()) and r.get("xte_m"):
                            vals.append(abs(float(r["xte_m"])))
                    except ValueError:
                        continue
        except OSError:
            continue
    if not vals:
        return None
    vals.sort()
    return statistics.median(vals), vals[int(0.95 * (len(vals) - 1))], vals[-1]


def describe(r):
    hhmm = time.strftime("%H:%M:%S", time.localtime(r["start"]))
    extra = [f"重新規劃 {r['replans']} 次"]
    if r.get("folds") is not None:
        extra.append(f"折返 {r['folds']}")
    if r["cand"] or r["degrade"]:
        extra.append(f"多候選 {r['cand']}、降級 {r['degrade']}")
    x = xte(r["start"], r.get("end"))
    xs = f"偏離 中位 {100 * x[0]:.1f}／p95 {100 * x[1]:.1f}／最大 {100 * x[2]:.1f} cm" if x else "（沒有逐拍偏離資料）"
    if r["ok"] is None:
        head = f"{hhmm} → {r['target']}：進行中"
    elif r["ok"]:
        err = f"終點誤差 {r['err']:.2f} m、朝向差 {r['yaw']:.1f}°" if r.get("err") is not None else ""
        head = f"{hhmm} → {r['target']}：✓ 成功 {r['end'] - r['start']:.1f} s（{err}）"
    else:
        head = f"{hhmm} → {r['target']}：✗ 失敗 {r['end'] - r['start']:.1f} s（{r['reason']}）"
    return f"{head}｜{xs}｜{'、'.join(extra)}", x


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--since", type=float, default=None, help="只看這個時間（epoch 秒）之後開始的趟次")
    ap.add_argument("--all", action="store_true", help="印全部（預設只印最近一趟）")
    ap.add_argument("--log", default="", help="指定一份 nav_*.log（預設：最新的，或 --since 之後有寫入的）")
    ap.add_argument("--tsv", default="", help="附加寫入的 TSV 檔")
    ap.add_argument("--tag", default="", help="TSV 第一欄的標籤（例：新、舊）")
    a = ap.parse_args()
    logs = sorted(glob.glob(os.path.join(LOG_DIR, "nav_*.log")), key=os.path.getmtime)
    if a.log:
        logs = [a.log]
    elif a.since is not None:
        logs = [p for p in logs if os.path.getmtime(p) >= a.since] or logs[-1:]
    else:
        logs = logs[-1:]
    runs = parse(logs)
    if a.since is not None:
        runs = [r for r in runs if r["start"] >= a.since]
    if not (a.all or a.since is not None):
        runs = runs[-1:]
    if not runs:
        print("（這段時間沒有帶位紀錄）")
        return
    for r in runs:
        line, x = describe(r)
        print(line)
        if a.tsv and r["ok"] is not None:
            new = not os.path.exists(a.tsv)
            with open(a.tsv, "a", encoding="utf-8") as fh:
                if new:
                    fh.write("標籤\t時間\t目標\t結果\t秒\t終點誤差m\t朝向差度\t偏離中位cm\t偏離p95cm\t偏離最大cm\t重新規劃\t折返\t原因\n")
                fh.write("\t".join(str(v) for v in (
                    a.tag, time.strftime("%H:%M:%S", time.localtime(r["start"])), r["target"],
                    "成功" if r["ok"] else "失敗", f"{r['end'] - r['start']:.1f}",
                    r.get("err", "") if r.get("err") is not None else "", r.get("yaw", "") if r.get("yaw") is not None else "",
                    f"{100 * x[0]:.1f}" if x else "", f"{100 * x[1]:.1f}" if x else "", f"{100 * x[2]:.1f}" if x else "",
                    r["replans"], r.get("folds") if r.get("folds") is not None else "", r.get("reason", ""))) + "\n")


if __name__ == "__main__":
    main()
