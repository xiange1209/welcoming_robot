#!/usr/bin/env python3
"""CPU 4 種情況（11/10 車上 agent；HANDOFF_car_agent_20261110 第二段第 6 項）。

★ 請先關掉 VS Code（它的 server 會吃 CPU），用 cmd.exe 的 ssh 登入車上再跑：
    ssh user@wheeltec-r680.local
    python3 ~/maprun/verify_out/day_1110/tools/cpu_scenarios.py
車子不會動（只開單元、不送導航目標）。全程約 15 分鐘；Ctrl-C 會停掉這支開的東西並保留已量的結果。

情況（依序累加／減少，用 HMI 的 API 開關單元，跟平板「系統開關」同一條路）：
    A 只開導航            sensors＋nav（定位模式）
    B 導航＋人臉          A＋camera（人臉模式）＋face＋user_auth
    C 全部開啟            B＋bank_reception＋asr_chain＋LLM（qwen2.5:3b，SSH 手動起，同今天的測試）
    D 全開但關人臉與相機  C－face－user_auth－camera
每種：單元起來後等 SETTLE 秒，再量 MEASURE 秒（/proc/stat 每 2 秒一筆；每個程式用 /proc/<pid>/stat 算區間平均）。
輸出：~/maprun/verify_out/day_1110/cpu/cpu_<HHMM>_summary.csv、_procs.csv、_samples.csv
用法：cpu_scenarios.py [--settle 60] [--measure 120] [--only A,B,C,D]
"""
import argparse
import csv
import glob
import json
import os
import signal
import subprocess
import sys
import time
import urllib.request

ap = argparse.ArgumentParser()
ap.add_argument('--settle', type=float, default=60.0)
ap.add_argument('--measure', type=float, default=120.0)
ap.add_argument('--only', default='A,B,C,D')
a = ap.parse_args()

OUT = os.path.expanduser('~/maprun/verify_out/day_1110/cpu')
os.makedirs(OUT, exist_ok=True)
TAG = time.strftime('%H%M')
BASE = 'http://127.0.0.1:8080'
HZ = os.sysconf('SC_CLK_TCK')
NCPU = os.cpu_count()
ENV = ('source /opt/ros/jazzy/setup.bash && source ~/welcoming_robot_ws/install/setup.bash && '
       'export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST CYCLONEDDS_URI=file://$HOME/cyclonedds.xml '
       'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp')


def call(method, path, body=None, tok=None):
    h = {'Content-Type': 'application/json'}
    if tok:
        h['Authorization'] = 'Bearer ' + tok
    req = urllib.request.Request(BASE + path, method=method, headers=h,
                                 data=json.dumps(body).encode() if body is not None else None)
    try:
        with urllib.request.urlopen(req, timeout=20) as r:
            return r.status, json.loads(r.read() or b'{}')
    except urllib.error.HTTPError as e:
        return e.code, {}


def login():
    env = subprocess.run(['systemctl', 'show', 'smartnav-hmi', '-p', 'Environment', '--value'],
                         capture_output=True, text=True).stdout.split()
    kv = dict(x.split('=', 1) for x in env if '=' in x)
    st, js = call('POST', '/api/login', {'username': 'admin', 'password': kv.get('SMARTNAV_ADMIN_PASSWORD', '')})
    return js.get('token') if st == 200 else None


def unit(tok, name, action):
    st, js = call('POST', f'/api/system/{name}/{action}', tok=tok)
    print(f'    {name} {action}：HTTP {st} {js.get("message", "")}', flush=True)
    time.sleep(2)


def wait_nav(timeout=150):
    t0 = time.time()
    while time.time() - t0 < timeout:
        logs = sorted(glob.glob(os.path.expanduser('~/.smartnav/logs/system_control/nav_*.log')), key=os.path.getmtime)
        if logs and os.path.getmtime(logs[-1]) > T_START and 'Managed nodes are active' in open(logs[-1], errors='replace').read():
            return True
        time.sleep(3)
    return False


llm = {'p': None}


def llm_start():
    if llm['p'] is None:
        log = open(os.path.join(OUT, f'cpu_{TAG}_llm.log'), 'w')
        llm['p'] = subprocess.Popen(['bash', '-c', ENV + ' && exec ros2 run smartnav_llm llm_service --ros-args '
                                     '-r __node:=llm_node -p model_name:=qwen2.5:3b'],
                                    stdout=log, stderr=log, start_new_session=True)
        print('    LLM（qwen2.5:3b）已在背景啟動', flush=True)


def llm_stop():
    if llm['p'] is not None:
        try:
            os.killpg(llm['p'].pid, signal.SIGINT)
            llm['p'].wait(timeout=20)
        except Exception:
            pass
        llm['p'] = None
        print('    LLM 已停', flush=True)


def cpu_times():
    with open('/proc/stat') as f:
        v = [int(x) for x in f.readline().split()[1:]]
    return sum(v), v[3] + v[4]


def procs():
    out = {}
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            with open(f'/proc/{d}/stat') as f:
                st = f.read().rsplit(')', 1)[1].split()
            with open(f'/proc/{d}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode('utf-8', 'replace')
            out[int(d)] = (int(st[11]) + int(st[12]), cmd)
        except (OSError, IndexError, ValueError):
            pass
    return out


def label(cmd):
    if '__node:=' in cmd:
        return cmd.split('__node:=')[1].split()[0]
    if '.vscode-server' in cmd:
        return 'VS Code server'
    if 'claude' in cmd:
        return 'Claude Code'
    for key in ('hmi_server', 'load_monitor', 'battery_log', 'cpu_scenarios', 'ros2 launch', 'ros2 run'):
        if key in cmd:
            return key
    return os.path.basename(cmd.split(' ')[0])[:30] or '?'


def measure(name):
    print(f'  量測 {a.measure:.0f} 秒…', flush=True)
    p0, (t0, i0) = procs(), cpu_times()
    s_rows, temps, loads = [], [], []
    tt0 = time.time()
    prev = (t0, i0)
    while time.time() - tt0 < a.measure:
        time.sleep(2)
        t, i = cpu_times()
        idle = 100.0 * (i - prev[1]) / max(1, t - prev[0])
        prev = (t, i)
        temp = int(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1000
        load1 = float(open('/proc/loadavg').read().split()[0])
        s_rows.append([name, time.strftime('%H:%M:%S'), f'{idle:.1f}', f'{load1:.2f}', f'{temp:.1f}'])
        temps.append(temp)
        loads.append(load1)
    p1, (t1, i1) = procs(), cpu_times()
    dur = time.time() - tt0
    idle_all = 100.0 * (i1 - i0) / max(1, t1 - t0)
    per = {}
    for pid, (ticks, cmd) in p1.items():
        if pid in p0:
            per.setdefault(label(cmd), 0.0)
            per[label(cmd)] += 100.0 * (ticks - p0[pid][0]) / HZ / dur
    try:
        thr = open('/sys/devices/platform/soc/soc:firmware/get_throttled').read().strip()
    except OSError:
        thr = '讀不到'
    idles = [float(r[2]) for r in s_rows]
    summ = [name, f'{idle_all:.1f}', f'{min(idles):.1f}', f'{sum(loads) / len(loads):.2f}', f'{max(temps):.1f}', thr,
            f'{per.get("VS Code server", 0) + per.get("Claude Code", 0):.1f}']
    top = sorted(per.items(), key=lambda kv: -kv[1])[:14]
    print(f'  ── {name}：CPU 閒置平均 {summ[1]}%（最低 {summ[2]}%）、load {summ[3]}、溫度最高 {summ[4]}°C、降頻旗標 {thr}、'
          f'VS Code＋Claude {summ[6]}%（單核百分比）', flush=True)
    for k, v in top[:8]:
        print(f'       {v:6.1f}%  {k}', flush=True)
    return summ, [[name, k, f'{v:.1f}'] for k, v in top], s_rows


SCEN = {
    'A': ('A 只開導航', [('sensors', 'start'), ('nav', 'start')], []),
    'B': ('B 導航＋人臉', [('camera', 'start:face'), ('face', 'start:normal'), ('user_auth', 'start')], []),
    'C': ('C 全部開啟', [('bank_reception', 'start'), ('asr_chain', 'start'), ('LLM', 'start')], []),
    'D': ('D 全開但關人臉與相機', [], [('face', 'stop'), ('user_auth', 'stop'), ('camera', 'stop')]),
}
T_START = time.time()
tok = login()
if not tok:
    sys.exit('✗ 登入 HMI 失敗')
pause = subprocess.run(['bash', '-c', ENV + ' && timeout 8 ros2 param get /hmi_server_node pause_camera_while_navigating'],
                       capture_output=True, text=True).stdout.strip()
print(f'導航中暫停相機（pause_camera_while_navigating）：{pause or "讀不到"}；車子全程不動', flush=True)
summ_rows, proc_rows, sample_rows = [], [], []
try:
    for key in a.only.split(','):
        name, starts, stops = SCEN[key]
        print(f'\n═══ {name} ═══', flush=True)
        for u, act in starts:
            llm_start() if u == 'LLM' else unit(tok, u, act)
        for u, act in stops:
            unit(tok, u, act)
        if key == 'A':
            print('  等導航就緒…', 'OK' if wait_nav() else '（150 秒內沒看到就緒，照樣量）', flush=True)
        print(f'  等 {a.settle:.0f} 秒穩定…', flush=True)
        time.sleep(a.settle)
        s, p, r = measure(name)
        summ_rows.append(s)
        proc_rows += p
        sample_rows += r
except KeyboardInterrupt:
    print('\n⏹ 中斷，保留已量的結果', flush=True)
finally:
    with open(os.path.join(OUT, f'cpu_{TAG}_summary.csv'), 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(['情況', 'CPU閒置平均%', 'CPU閒置最低%', 'load1平均', '溫度最高C', '降頻旗標', 'VSCode與Claude單核%'])
        w.writerows(summ_rows)
    with open(os.path.join(OUT, f'cpu_{TAG}_procs.csv'), 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(['情況', '程式', '單核百分比'])
        w.writerows(proc_rows)
    with open(os.path.join(OUT, f'cpu_{TAG}_samples.csv'), 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(['情況', '時間', 'CPU閒置%', 'load1', '溫度C'])
        w.writerows(sample_rows)
    llm_stop()
    print(f'\n結果在 {OUT}/cpu_{TAG}_*.csv。單元保持開著（要關請用平板「全部關閉」）。', flush=True)
