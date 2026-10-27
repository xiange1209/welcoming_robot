#!/usr/bin/env python3
"""system_control 單元 log 的問題統計（★ 2026-10-24 車上 agent，只讀）。
把 WARN/ERROR/FATAL、Traceback、行程死掉的行抓出來，數字正規化後依「單元 × 節點 × 訊息樣板」計數。
用法：python3 log_issues.py [--days 20261020,20261021] [--top 8] [--dir ~/.smartnav/logs/system_control]"""
import argparse, collections, glob, os, re

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default=os.path.expanduser("~/.smartnav/logs/system_control"))
ap.add_argument("--days", default="")
ap.add_argument("--top", type=int, default=8)
a = ap.parse_args()
days = [d for d in a.days.split(",") if d]
LINE = re.compile(r"^(?:\[[^\]]+-\d+\]\s*)?\[(WARN|ERROR|FATAL)\]\s*\[[\d.]+\]\s*\[([^\]]+)\]:\s*(.*)$")
DIED = re.compile(r"process has died|exit code|Segmentation fault|core dumped|Killed", re.I)
stats = collections.defaultdict(lambda: collections.Counter())
example, files = {}, collections.defaultdict(set)
sev_total = collections.Counter()
tb = collections.Counter()
for f in sorted(glob.glob(os.path.join(a.dir, "*.log"))):
    base = os.path.basename(f)
    if days and not any(f"_{d}_" in base for d in days):
        continue
    unit = re.sub(r"_\d{8}_\d{6}.*$", "", base)
    try:
        lines = open(f, encoding="utf-8", errors="replace").read().splitlines()
    except OSError:
        continue
    for i, ln in enumerate(lines):
        m = LINE.match(ln.strip())
        if m:
            sev, node, msg = m.groups()
            tmpl = re.sub(r"\d+(\.\d+)?", "#", msg)[:110]
            key = (sev, node, tmpl)
            stats[unit][key] += 1
            example.setdefault((unit,) + key, msg[:200])
            files[(unit,) + key].add(base[len(unit) + 1:len(unit) + 16])
            sev_total[(unit, sev)] += 1
        elif "Traceback (most recent call last)" in ln:
            # 取 traceback 最後一行（例外訊息）
            j = i + 1
            while j < len(lines) and (lines[j].startswith(" ") or lines[j].startswith("\t") or lines[j].startswith("[")) and "Error" not in lines[j]:
                j += 1
            last = lines[j].strip() if j < len(lines) else "?"
            tb[(unit, re.sub(r"\d+", "#", last)[:120])] += 1
        elif DIED.search(ln) and ("[ERROR]" in ln or "died" in ln):
            k = ("PROC", "launch", re.sub(r"\d+", "#", ln.strip())[:110])
            stats[unit][k] += 1
            example.setdefault((unit,) + k, ln.strip()[:200])
            files[(unit,) + k].add(base[len(unit) + 1:len(unit) + 16])
for unit in sorted(stats):
    tot = {s: sev_total[(unit, s)] for s in ("ERROR", "WARN", "FATAL") if sev_total[(unit, s)]}
    print(f"\n=== {unit}：{tot}")
    for (sev, node, tmpl), n in stats[unit].most_common(a.top):
        fs = sorted(files[(unit, sev, node, tmpl)])
        print(f"  {n:5d}× {sev:<5} [{node}] {example[(unit, sev, node, tmpl)][:150]}")
        print(f"         檔：{', '.join(fs[:4])}{' …' if len(fs) > 4 else ''}（{len(fs)} 份）")
if tb:
    print("\n=== Traceback（例外）")
    for (unit, last), n in tb.most_common(15):
        print(f"  {n:4d}× {unit}: {last}")
