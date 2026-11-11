#!/usr/bin/env python3
"""每趟帶位結束後的快速報告（11/04 寫、11/10 沿用；車上 agent）。

用法：python3 -I trip_report.py <開始 epoch> [結束 epoch]
讀當天的 ~/.smartnav/logs/system_control/nav_*.log 與 ~/maprun/logs/replay_path_*_live_*.csv。
"""
import csv
import datetime
import glob
import math
import os
import re
import sys

L = datetime.datetime
t0 = float(sys.argv[1])
t1 = float(sys.argv[2]) if len(sys.argv) > 2 else 9e9
day = L.fromtimestamp(t0).strftime('%Y%m%d')
pat = re.compile(r'\[(\d{10}\.\d+)\] \[([A-Za-z_.0-9]+)\]: (.*)')
show = re.compile(r'收到導航請求|教導路徑規劃完成|開始導航到|重新規劃第|次失敗|\[新障礙\]|請讓路|✓ 導航成功|導航失敗|已被系統或使用者取消'
                  r'|控制迴圈只跑到|偏離路徑|推不動|判定卡住|脫困完成|緊急停止|放行導航|拒絕')
count = re.compile(r'防撞正在煞車|VelocityStop|車身側面只剩|繞過')

ev = set()
for p in glob.glob(f'/home/user/.smartnav/logs/system_control/nav_{day}_*.log'):
    if os.path.getmtime(p) < t0 - 5:
        continue
    for line in open(p, encoding='utf-8', errors='replace'):
        m = pat.search(line)
        if m and t0 - 2 <= float(m.group(1)) <= t1 + 5:
            ev.add((round(float(m.group(1)), 1), m.group(2), m.group(3)))
ev = sorted(ev)
counts = {}
print('== 導航 log 重點')
for t, node, msg in ev:
    c = count.search(msg)
    if c:
        counts[c.group(0)] = counts.get(c.group(0), 0) + 1
    if show.search(msg):
        print(f'  {L.fromtimestamp(t):%H:%M:%S} {msg[:130]}')
if counts:
    print('  次數：' + '、'.join(f'{k} {v}' for k, v in counts.items()))

truthy = lambda v: str(v).strip().lower() in ('1', 'true', 'yes')
print('== 逐拍紀錄')
for p in sorted(glob.glob(f'/home/user/maprun/logs/replay_path_*_live_{day}_*.csv')):
    ts = os.path.basename(p).split('_live_')[1][:15]
    st = L.strptime(ts, '%Y%m%d_%H%M%S').timestamp()
    if not (t0 - 5 <= st <= t1 + 5):
        continue
    rows = list(csv.DictReader(open(p, encoding='utf-8', errors='replace')))
    if not rows:
        continue
    vs, per, jumps, prev = [], {}, [], None
    for r in rows:
        try:
            t, x, y, yaw = float(r['epoch']), float(r['x']), float(r['y']), float(r['yaw_deg'])
        except (ValueError, KeyError):
            continue
        try:
            vs.append(abs(float(r.get('cmd_v_last') or 0)))
        except ValueError:
            pass
        d = r.get('dir', '?')
        s = per.setdefault(d, [0, 0, 0, 9.0])
        s[0] += 1
        s[1] += truthy(r.get('cm_braking'))
        s[2] += truthy(r.get('hard_stop'))
        try:
            s[3] = min(s[3], float(r['any_margin_m']))
        except (ValueError, KeyError):
            pass
        if prev:
            dt = t - prev[0]
            dist = math.hypot(x - prev[1], y - prev[2])
            dyaw = abs((yaw - prev[3] + 180) % 360 - 180)
            if 0 < dt < 3 and (dist / dt > 0.6 or dyaw / dt > 40) and (dist > 0.15 or dyaw > 10):
                jumps.append((L.fromtimestamp(t).strftime('%H:%M:%S'), round(dist, 2), round(dyaw, 1)))
        prev = (t, x, y, yaw)
    dur = float(rows[-1]['epoch']) - float(rows[0]['epoch'])
    name = {'1': '前進', '-1': '倒車'}
    parts = [f"{name.get(d, d)} {n} 拍、被防撞煞住 {100 * b / n:.0f}%、硬停 {100 * h / n:.0f}%、最小淨空 {m:.2f} m"
             for d, (n, b, h, m) in sorted(per.items(), key=lambda kv: kv[0], reverse=True)]
    print(f'  {ts[9:11]}:{ts[11:13]}:{ts[13:15]} 開始，{dur:.0f} 秒，最高指令 {max(vs) if vs else 0:.2f} m/s')
    for x in parts:
        print('    ' + x)
    print('    定位跳動：' + ('沒有' if not jumps else f'{len(jumps)} 次 {jumps[:3]}'))
