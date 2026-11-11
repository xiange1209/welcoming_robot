#!/usr/bin/env python3
"""快速量 CPU（11/10 車上 agent）：量 N 秒，印整體閒置與主要程式（單核百分比）。用法：cpu_snap.py <秒> <標籤> [輸出.csv]"""
import os, sys, time, csv
SEC, TAG = float(sys.argv[1]), sys.argv[2]
OUT = sys.argv[3] if len(sys.argv) > 3 else None
HZ = os.sysconf('SC_CLK_TCK')
def cpu():
    v = [int(x) for x in open('/proc/stat').readline().split()[1:]]; return sum(v), v[3] + v[4]
def procs():
    o = {}
    for d in os.listdir('/proc'):
        if d.isdigit():
            try:
                st = open(f'/proc/{d}/stat').read().rsplit(')', 1)[1].split()
                cmd = open(f'/proc/{d}/cmdline', 'rb').read().replace(b'\0', b' ').decode('utf-8', 'replace')
                o[int(d)] = (int(st[11]) + int(st[12]), cmd)
            except (OSError, IndexError, ValueError): pass
    return o
def label(c):
    if '__node:=' in c: return c.split('__node:=')[1].split()[0]
    for k, v in (('.vscode-server', 'VS Code'), ('claude', 'Claude Code'), ('astra_camera', 'astra_camera'), ('republish', 'republish（平板影像轉檔）'),
                 ('hmi_server', 'hmi_server'), ('latency_logger', 'latency_logger（記錄工具）'), ('battery_log', 'battery_log（記錄工具）')):
        if k in c: return v
    return os.path.basename(c.split(' ')[0])[:24] or '?'
p0, (t0, i0) = procs(), cpu(); s0 = time.time()
loads = []
while time.time() - s0 < SEC:
    time.sleep(2); loads.append(float(open('/proc/loadavg').read().split()[0]))
p1, (t1, i1) = procs(), cpu(); dur = time.time() - s0
per = {}
for pid, (tk, c) in p1.items():
    if pid in p0: per[label(c)] = per.get(label(c), 0) + 100.0 * (tk - p0[pid][0]) / HZ / dur
idle = 100.0 * (i1 - i0) / max(1, t1 - t0)
print(f'[{TAG}] {SEC:.0f} 秒：CPU 閒置 {idle:.1f}%、load 平均 {sum(loads) / len(loads):.1f}、溫度 {int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000:.1f}°C')
top = sorted(per.items(), key=lambda kv: -kv[1])[:10]
for k, v in top: print(f'   {v:6.1f}%  {k}')
if OUT:
    new = not os.path.exists(OUT)
    with open(OUT, 'a', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        if new: w.writerow(['標籤', '時間', '秒', 'CPU閒置%', 'load平均', '程式', '單核%'])
        for k, v in top: w.writerow([TAG, time.strftime('%H:%M:%S'), int(SEC), f'{idle:.1f}', f'{sum(loads) / len(loads):.1f}', k, f'{v:.1f}'])
