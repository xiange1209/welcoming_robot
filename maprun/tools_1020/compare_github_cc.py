#!/usr/bin/env python3
"""比對 GitHub repo（本機 clone）與車上實際檔案：src/ 對 ~/welcoming_robot_ws/src、maprun/ 對 ~/maprun。

只讀不改。比內容（不比時間戳）；忽略 __pycache__、.bak_*、dist 建置產物、node_modules。
用法：python3 compare_github_cc.py <clone 目錄> [--out 報告.md]
"""
import argparse
import hashlib
import os
import subprocess
import sys

ap = argparse.ArgumentParser()
ap.add_argument("clone")
ap.add_argument("--out", default="")
a = ap.parse_args()

HOME = os.path.expanduser("~")
PAIRS = [("src", os.path.join(HOME, "welcoming_robot_ws", "src")), ("maprun", os.path.join(HOME, "maprun"))]
SKIP_DIRS = {"__pycache__", "node_modules", ".git", "dist", "build", "install", "log"}


def skip(rel):
    parts = rel.split(os.sep)
    return (any(p in SKIP_DIRS for p in parts) or ".bak" in parts[-1] or parts[-1].endswith((".pyc", ".log"))
            or "wheeltec_ws" in parts)


def digest(path):
    with open(path, "rb") as fh:
        return hashlib.sha1(fh.read()).hexdigest()


tracked = subprocess.run(["git", "-C", a.clone, "ls-files"], capture_output=True, text=True).stdout.split("\n")
lines, totals = [], {}
for top, car_root in PAIRS:
    repo_files = sorted(f[len(top) + 1:] for f in tracked if f.startswith(top + "/") and not skip(f[len(top) + 1:]))
    same, diff, missing_on_car = [], [], []
    for rel in repo_files:
        car = os.path.join(car_root, rel)
        repo_path = os.path.join(a.clone, top, rel)
        if os.path.isdir(car) or os.path.isdir(repo_path) or os.path.islink(repo_path):
            continue  # 子模組（gitlink）或連結：不比內容
        if not os.path.exists(car):
            missing_on_car.append(rel)
        elif digest(car) != digest(os.path.join(a.clone, top, rel)):
            diff.append(rel)
        else:
            same.append(rel)
    # 車上有、repo 沒有的（只看 repo 有追蹤的套件／目錄底下，避免把整個家目錄列出來）
    repo_set = set(repo_files)
    tops = {r.split(os.sep)[0] for r in repo_files}
    only_car = []
    for t in sorted(tops):
        base = os.path.join(car_root, t)
        if not os.path.isdir(base):
            continue
        for dp, dns, fns in os.walk(base):
            dns[:] = [d for d in dns if d not in SKIP_DIRS]
            for fn in fns:
                rel = os.path.relpath(os.path.join(dp, fn), car_root)
                if not skip(rel) and rel not in repo_set:
                    only_car.append(rel)
    totals[top] = (len(same), len(diff), len(missing_on_car), len(only_car))
    lines.append(f"\n## {top}/（repo）↔ {car_root}（車上）")
    lines.append(f"相同 {len(same)}、內容不同 {len(diff)}、repo 有車上沒有 {len(missing_on_car)}、車上有 repo 沒有 {len(only_car)}")
    for label, items in (("內容不同", diff), ("repo 有、車上沒有", missing_on_car), ("車上有、repo 沒有", only_car)):
        if items:
            lines.append(f"\n### {label}（{len(items)}）")
            lines += [f"- {x}" for x in items]

head = subprocess.run(["git", "-C", a.clone, "log", "--oneline", "-1"], capture_output=True, text=True).stdout.strip()
report = f"# GitHub master（{head}）vs 車上\n" + "\n".join(lines) + "\n"
if a.out:
    open(a.out, "w", encoding="utf-8").write(report)
print(report if len(report) < 6000 else report[:6000] + "\n…（完整內容見輸出檔）")
for top, t in totals.items():
    print(f"{top}: 相同 {t[0]}／不同 {t[1]}／repo 有車上沒有 {t[2]}／車上有 repo 沒有 {t[3]}", file=sys.stderr)
