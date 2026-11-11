#!/usr/bin/env python3
"""連線稽核：每 0.5 秒用 `ss -tunap` 記下所有非 loopback 的連線與擁有它的程式（11/10 車上 agent）。

搭配同時間的 tcpdump（只抓離開區網的封包與 DNS）：tcpdump 抓得到每一個封包但不知道是哪個程式，
這支知道程式但太短的連線可能漏掉；分析時兩邊對起來。
用法：sudo python3 audit_ss.py <輸出.csv> [最長秒數]   （要 root 才看得到每個程式的名字）
"""
import csv
import os
import re
import subprocess
import sys
import time

OUT = sys.argv[1]
LIMIT = float(sys.argv[2]) if len(sys.argv) > 2 else 7200.0
proc_re = re.compile(r'\("([^"]+)",pid=(\d+)')


def cmdline(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return f.read().replace(b'\0', b' ').decode('utf-8', 'replace').strip()[:200]
    except OSError:
        return ''


seen_cmd = {}
new = not os.path.exists(OUT)
f = open(OUT, 'a', newline='', encoding='utf-8')
w = csv.writer(f)
if new:
    w.writerow(['epoch', 'proto', 'state', 'local', 'remote', 'process', 'pid', 'cmdline'])
t0 = time.time()
try:
    while time.time() - t0 < LIMIT:
        now = time.time()
        out = subprocess.run(['ss', '-tunapH'], capture_output=True, text=True).stdout
        for line in out.splitlines():
            parts = line.split()
            if len(parts) < 6:
                continue
            proto, state, local, remote = parts[0], parts[1], parts[4], parts[5]
            if remote.startswith(('127.', '[::1]', '[::ffff:127.', '*', '0.0.0.0', '[::]')):
                continue
            m = proc_re.search(line)
            name, pid = (m.group(1), m.group(2)) if m else ('?', '')
            if pid and pid not in seen_cmd:
                seen_cmd[pid] = cmdline(pid)
            w.writerow([f'{now:.2f}', proto, state, local, remote, name, pid, seen_cmd.get(pid, '')])
        f.flush()
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
