#!/usr/bin/env python3
"""連線稽核報告（11/10 車上 agent）：把 audit_ss.py 的 ss 取樣與 tcpdump 的封包對起來。

用法：python3 audit_report.py <ss.csv> <external.pcap> [開始 HH:MM:SS] [結束 HH:MM:SS] > 報告.md
  ss.csv：每 0.5 秒的連線＋程式（知道是誰，但太短的連線可能沒取樣到）
  pcap  ：只抓「有一端在區網外」的封包＋DNS（每個封包都有，但不知道是哪個程式）
輸出：①對外連線表（程式｜類別｜對方｜網域｜埠｜次數）②區網連線表 ③pcap 有、ss 沒對到的對外流量 ④DNS 查過的網域。
"""
import collections
import csv
import datetime
import ipaddress
import re
import subprocess
import sys

SS, PCAP = sys.argv[1], sys.argv[2]
day = datetime.date.today()


def hms(s):
    return datetime.datetime.combine(day, datetime.time.fromisoformat(s)).timestamp() if s else None


T0 = hms(sys.argv[3]) if len(sys.argv) > 3 else None
T1 = hms(sys.argv[4]) if len(sys.argv) > 4 else None
LAN = [ipaddress.ip_network(n) for n in ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16', '169.254.0.0/16',
                                          '224.0.0.0/4', '127.0.0.0/8', 'fe80::/10', 'ff00::/8', '::1/128')]


def split_addr(a):
    a = a.replace('%wlan0', '').replace('%eth0', '')
    if a.startswith('['):
        host, _, port = a[1:].partition(']:')
    else:
        host, _, port = a.rpartition(':')
    return host.replace('::ffff:', ''), port


def is_lan(host):
    try:
        ip = ipaddress.ip_address(host)
    except ValueError:
        return True
    return any(ip in n for n in LAN)


def category(proc, cmd):
    c = f'{proc} {cmd}'
    if '.vscode-server' in c or proc in ('code', 'node', 'MainThread') or proc.startswith(('git', 'npm')) or 'claude' in c:
        return '我（Claude Code）與 VS Code'
    if 'welcoming_robot_ws' in c or '/opt/ros/' in c or 'smartnav' in c or proc in ('hmi_server', 'llm_service', 'bank_reception'):
        return '機器人程式'
    if proc in ('sshd', 'sshd-session') or 'sshd' in c:
        return 'SSH（使用者筆電登入）'
    if proc in ('systemd-timesyn', 'systemd-resolve', 'NetworkManager', 'snapd', 'http', 'https', 'apt', 'apt-get',
                'unattended-upgr', 'avahi-daemon', 'packagekitd', 'fwupd', 'cloud-init') or 'apt' in c:
        return '系統服務'
    if proc in ('tcpdump', 'python3') and 'audit_ss.py' in c:
        return '稽核工具本身'
    if proc == 'ssh':
        return '我（Claude Code）與 VS Code'      # git over ssh（VS Code 的 git 自動同步）
    if proc == '?':
        return '沒有程式資訊（連線已關閉）'
    return '其他'


def short(cmd):
    m = re.search(r'install/([a-z_]+)/lib/[a-z_]+/([a-z_]+)', cmd)
    if m:
        return m.group(2)
    m = re.search(r'ros2 (run|launch) (\S+ \S+)', cmd)
    if m:
        return m.group(2)
    return cmd.split(' ')[0].rsplit('/', 1)[-1][:30]


# ── DNS：查過哪些網域、回了哪些 IP ──
dns_q, ip2name = collections.Counter(), {}
pc = subprocess.run(['tcpdump', '-nn', '-tt', '-r', PCAP], capture_output=True, text=True).stdout.splitlines()
flows = collections.Counter()
for line in pc:
    try:
        t = float(line.split()[0])
    except (ValueError, IndexError):
        continue
    if (T0 and t < T0) or (T1 and t > T1):
        continue
    for name in re.findall(r' A{1,4}\? ([\w.-]+)\.', line):
        dns_q[name] += 1
    for name_ip in re.findall(r' A ([\d.]+)', line):
        pass
    m = re.search(r'\? ([\w.-]+)\. .*', line)
    for ip in re.findall(r' A ([\d.]+)', line):
        q = re.search(r'q: A\? ([\w.-]+)\.', line)
        if q:
            ip2name[ip] = q.group(1)
    m = re.search(r'IP6? ([\w.:]+)\.(\d+) > ([\w.:]+)\.(\d+):', line)
    if not m:
        continue
    s, sp, d, dp = m.groups()
    if dp == '53' or sp == '53':
        continue
    ext, port = (d, dp) if not is_lan(d) else (s, sp)
    if is_lan(ext):
        continue
    flows[(ext, port)] += 1

# ── ss：誰連到哪裡 ──
seen = collections.defaultdict(lambda: [0, None, None, set()])
for r in csv.DictReader(open(SS, encoding='utf-8')):
    t = float(r['epoch'])
    if (T0 and t < T0) or (T1 and t > T1):
        continue
    host, port = split_addr(r['remote'])
    lhost, lport = split_addr(r['local'])
    if lport in ('22', '8080', '8443') and r['state'] != 'SYN-SENT':
        port = f'{lport}（對方連進來）'
    cat = category(r['process'], r['cmdline'])
    key = (cat, short(r['cmdline']) or r['process'], host, port, '區網' if is_lan(host) else '外網')
    s = seen[key]
    s[0] += 1
    s[1] = t if s[1] is None else min(s[1], t)
    s[2] = t if s[2] is None else max(s[2], t)

f = lambda t: datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S')
print(f'# 連線稽核（{f(T0) if T0 else "開始"}～{f(T1) if T1 else "結束"}）\n')
for where in ('外網', '區網'):
    print(f'## {where}連線（ss 每 0.5 秒取樣；次數＝被取樣到的次數）\n')
    print('| 類別 | 程式 | 對方位址 | 網域（DNS） | 埠 | 次數 | 第一次～最後 |')
    print('|---|---|---|---|---|--:|---|')
    for (cat, prog, host, port, w), (n, a, b, _) in sorted(seen.items(), key=lambda kv: (kv[0][0], -kv[1][0])):
        if w == where:
            print(f'| {cat} | {prog} | {host} | {ip2name.get(host, "")} | {port} | {n} | {f(a)}～{f(b)} |')
    print()
ss_ext = {(h, p.split('（')[0]) for (_, _, h, p, w) in seen if w == '外網'}
print('## pcap 有、ss 沒對到程式的對外流量（連線太短沒被取樣到）\n')
print('| 對方位址 | 網域（DNS） | 埠 | 封包數 |')
print('|---|---|---|--:|')
for (ip, port), n in flows.most_common():
    if (ip, port) not in ss_ext:
        print(f'| {ip} | {ip2name.get(ip, "")} | {port} | {n} |')
print('\n## DNS 查過的網域\n')
for name, n in dns_q.most_common():
    print(f'- {name}（{n} 次）')
