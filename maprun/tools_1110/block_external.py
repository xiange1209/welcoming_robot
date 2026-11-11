#!/usr/bin/env python3
"""封鎖外網測試（11/10 車上 agent；HANDOFF_car_agent_20261110 第二段第 8 項）：證明沒有外網時迎賓、問答照常運作。

★ 封鎖期間車上的 Claude Code 也連不到外網，所以這支要你自己用純 SSH 跑完：
    ssh user@wheeltec-r680.local
    python3 ~/maprun/verify_out/day_1110/tools/block_external.py          # 預設封鎖 10 分鐘
做了什麼：
  1. 確認迎賓需要的單元有開（相機人臉模式、人臉、認證、迎賓劇本、語音輸入、LLM），沒開就幫你開
  2. 封鎖前先測：外網（Telegram、查匯率的網站）連得到、區網（筆電 Ollama、HMI）連得到
  3. nftables 擋掉所有「目的地不在區網」的對外連線；區網（SSH、平板、Ollama）照常
     ★ 一定會自動解除：systemd 計時器到時間就刪掉規則（SSH 斷線也一樣）；Ctrl-C 或跑完也會立刻解除
  4. 再測一次：外網應該連不到、區網應該照常
  5. 一步一步提示你測功能（每步按 Enter）；假黑名單只在「確認外網已封鎖」後才發，所以 Telegram 不會真的送出
  6. 解除封鎖、確認外網恢復，從 log 整理結果 → ~/maprun/verify_out/day_1110/block/block_<HHMM>.txt
自測：block_external.py --selftest 15   （只封鎖 15 秒、檢查連線、不開單元、不互動）
"""
import argparse
import glob
import json
import os
import re
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request

ap = argparse.ArgumentParser()
ap.add_argument('--minutes', type=float, default=10.0)
ap.add_argument('--selftest', type=float, default=0.0)
a = ap.parse_args()

TABLE = 'smartnav_block'
RULES = f'''table inet {TABLE} {{
  chain out {{
    type filter hook output priority 0; policy accept;
    oif "lo" accept
    ip daddr {{ 127.0.0.0/8, 10.0.0.0/8, 172.16.0.0/12, 192.168.0.0/16, 169.254.0.0/16, 224.0.0.0/4, 255.255.255.255 }} accept
    ip6 daddr {{ ::1, fe80::/10, ff00::/8, fc00::/7 }} accept
    counter drop
  }}
}}
'''
OUT = os.path.expanduser('~/maprun/verify_out/day_1110/block')
os.makedirs(OUT, exist_ok=True)
TAG = time.strftime('%H%M')
LOGDIR = os.path.expanduser('~/.smartnav/logs/system_control')
ENV = ('source /opt/ros/jazzy/setup.bash && source ~/welcoming_robot_ws/install/setup.bash && '
       'export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST CYCLONEDDS_URI=file://$HOME/cyclonedds.xml '
       'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp')
report = []


def say(s=''):
    print(s, flush=True)
    report.append(s)


def sh(cmd, inp=None):
    return subprocess.run(cmd, input=inp, capture_output=True, text=True)


def reachable(url, t=5.0):
    try:
        urllib.request.urlopen(urllib.request.Request(url, method='GET'), timeout=t)
        return True
    except urllib.error.HTTPError:
        return True                    # 有回 HTTP 狀態＝連得到
    except Exception:
        return False


def ollama_url():
    logs = sorted(glob.glob(f'{LOGDIR}/llm_*.log'), key=os.path.getmtime)
    for p in reversed(logs[-3:]):
        m = re.search(r'Ollama：自動找到 (http://[^\s（]+)', open(p, errors='replace').read())
        if m:
            return m.group(1).rstrip('/')
    return 'http://MSI.local:11434'


EXT = [('Telegram（黑名單通報）', 'https://api.telegram.org'), ('查匯率 tw.rter.info', 'https://tw.rter.info/capi.php')]


def check(stage):
    lan = [('筆電 Ollama', ollama_url() + '/api/tags'), ('HMI（平板）', 'http://127.0.0.1:8080/')]
    res = {}
    for name, url in EXT + lan:
        res[name] = reachable(url)
    say(f'  [{stage}] ' + '｜'.join(f'{k} {"連得到" if v else "連不到"}' for k, v in res.items()))
    return res


def block(sec):
    sh(['sudo', '-n', 'nft', 'delete', 'table', 'inet', TABLE])
    r = sh(['sudo', '-n', 'nft', '-f', '-'], RULES)
    if r.returncode:
        sys.exit('✗ 封鎖失敗：' + r.stderr.strip())
    unit = f'smartnav-unblock-{int(time.time())}'
    r = sh(['sudo', '-n', 'systemd-run', f'--on-active={int(sec)}s', '--unit', unit,
            '/usr/sbin/nft', 'delete', 'table', 'inet', TABLE])
    say(f'  ⛔ 已封鎖外網；{int(sec)} 秒後自動解除（systemd 計時器 {unit}{"" if r.returncode == 0 else "（★ 計時器沒設成功，請記得 Ctrl-C 解除）"}）')
    return unit


def unblock(unit=None):
    r = sh(['sudo', '-n', 'nft', 'delete', 'table', 'inet', TABLE])
    if unit:
        sh(['sudo', '-n', 'systemctl', 'stop', f'{unit}.timer'])
    say('  ✓ 已解除封鎖' if r.returncode == 0 else '  （規則已經不在，可能計時器已解除）')


def counter():
    r = sh(['sudo', '-n', 'nft', 'list', 'table', 'inet', TABLE])
    m = re.search(r'counter packets (\d+) bytes (\d+) drop', r.stdout)
    return int(m.group(1)) if m else None


if a.selftest:
    say(f'== 自測：封鎖 {a.selftest:.0f} 秒（不開單元、不互動）')
    check('封鎖前')
    unit = block(max(60, a.selftest + 45))
    try:
        time.sleep(2)
        res = check('封鎖中')
        say(f'  擋下的封包：{counter()}')
        ok = all(not res[n] for n, _ in EXT) and res['筆電 Ollama'] and res['HMI（平板）']
        time.sleep(max(0, a.selftest - 12))
    finally:
        unblock(unit)
    time.sleep(2)
    after = check('解除後')
    say('自測結果：' + ('PASS（外網被擋、區網照常、解除後恢復）' if ok and all(after[n] for n, _ in EXT) else 'FAIL，請看上面'))
    sys.exit(0)

# ── 正式測試 ──────────────────────────────────────────────
env = sh(['systemctl', 'show', 'smartnav-hmi', '-p', 'Environment', '--value']).stdout.split()
kv = dict(x.split('=', 1) for x in env if '=' in x)


def api(method, path, body=None, tok=None):
    h = {'Content-Type': 'application/json'}
    if tok:
        h['Authorization'] = 'Bearer ' + tok
    req = urllib.request.Request('http://127.0.0.1:8080' + path, method=method, headers=h,
                                 data=json.dumps(body).encode() if body is not None else None)
    try:
        with urllib.request.urlopen(req, timeout=20) as r:
            return r.status, json.loads(r.read() or b'{}')
    except urllib.error.HTTPError as e:
        return e.code, {}


st, js = api('POST', '/api/login', {'username': 'admin', 'password': kv.get('SMARTNAV_ADMIN_PASSWORD', '')})
tok = js.get('token')
if not tok:
    sys.exit('✗ 登入 HMI 失敗')
say(f'== 封鎖外網測試 {time.strftime("%Y-%m-%d %H:%M")}（封鎖 {a.minutes:.0f} 分鐘）')
running = sh(['bash', '-c', 'ps -eo args']).stdout
for u, act, node in (('camera', 'start:face', 'camera'), ('face', 'start:normal', 'face_node'), ('user_auth', 'start', 'user_auth_node'),
                     ('bank_reception', 'start', 'bank_reception_node'), ('asr_chain', 'start', 'voice_trigger_node')):
    if f'__node:={node}' not in running and not (u == 'camera' and 'camera_face' in running):
        s, j = api('POST', f'/api/system/{u}/{act}', tok=tok)
        say(f'  開 {u}：HTTP {s}')
        time.sleep(2)
llm_p = None
if '__node:=llm_node' not in running:
    llm_p = subprocess.Popen(['bash', '-c', ENV + ' && exec ros2 run smartnav_llm llm_service --ros-args -r __node:=llm_node '
                              '-p model_name:=qwen2.5:3b'], stdout=open(f'{OUT}/block_{TAG}_llm.log', 'w'),
                             stderr=subprocess.STDOUT, start_new_session=True)
    say('  開 LLM（qwen2.5:3b）')
say('  等 40 秒讓單元就緒…')
time.sleep(40)
check('封鎖前')
t_block = time.time()
unit = block(a.minutes * 60)


def stop(*_):
    unblock(unit)
    raise SystemExit(130)


signal.signal(signal.SIGINT, stop)
steps = []
try:
    time.sleep(2)
    res = check('封鎖中')
    if any(res[n] for n, _ in EXT):
        say('✗ 外網沒有被擋住，不往下做（避免真的發 Telegram）')
        raise SystemExit(1)
    say(f'  已擋下 {counter()} 個對外封包')
    prompts = [
        ('認人問候', '請站到鏡頭前 1 公尺，等平板叫你的名字（貴賓問候）。聽到後按 Enter'),
        ('問營業時間', '對平板說「請問營業時間？」，等它回答完按 Enter'),
        ('查匯率（會連外）', '對平板說「美金匯率是多少？」，等它回答完按 Enter（應該查不到，但要正常回話、不當掉）'),
        ('帶我去櫃台', '對平板說「帶我去櫃台」：車會動，請跟在旁邊、急停拿在手上；到了按 Enter（不想開車就輸入 s 再按 Enter）'),
        ('緊急停止', '按平板的「緊急停止」，按完按 Enter'),
    ]
    for name, text in prompts:
        t0 = time.time()
        ans = input(f'\n▶ {name}：{text}\n> ').strip().lower()
        steps.append((name, t0, time.time(), 'skip' if ans == 's' else 'done'))
    say('\n▶ 黑名單（假身分）：外網已確認封鎖，發 15 秒假的黑名單身分「測試黑名單」，看 Telegram 送不出去時迎賓劇本怎麼反應…')
    t0 = time.time()
    pub = ('import time, rclpy\nfrom smartnav_msgs.msg import UserIdentity, UserType\nrclpy.init()\n'
           'n = rclpy.create_node("block_fake_blacklist")\np = n.create_publisher(UserIdentity, "user_identity", 10)\n'
           'm = UserIdentity(); m.user_name = "測試黑名單"; m.user_type = UserType(type=3); m.recognized = True; m.similarity = 0.86\n'
           'm.bbox = [250.0, 140.0, 330.0, 240.0]\nt = time.time()\nwhile time.time() - t < 15:\n'
           '    m.header.stamp = n.get_clock().now().to_msg(); p.publish(m); time.sleep(0.5)\n')
    subprocess.run(['bash', '-c', ENV + ' && python3 -'], input=pub, text=True)
    time.sleep(10)
    steps.append(('黑名單通報（假身分）', t0, time.time(), 'done'))
finally:
    unblock(unit)
    t_unblock = time.time()
    time.sleep(2)
    check('解除後')
    if llm_p is not None:
        try:
            os.killpg(llm_p.pid, signal.SIGINT)
        except Exception:
            pass

# ── 從 log 整理 ──────────────────────────────────────────
pat = re.compile(r'\[(\d{10}\.\d+)\] \[([A-Za-z_.0-9]+)\]: (.*)')
keep = re.compile(r'迎賓語音|貴賓您好|收到使用者輸入|最終決策回應|Telegram|通報|匯率|exchange|錯誤|失敗|Error|Traceback|開始導航到|✓ 導航成功|導航失敗|已被系統或使用者取消|緊急停止')
lines = []
for p in glob.glob(f'{LOGDIR}/*.log') + glob.glob(f'{OUT}/block_{TAG}_llm.log'):
    if os.path.getmtime(p) < t_block - 5:
        continue
    for line in open(p, errors='replace'):
        m = pat.search(line)
        if m and t_block <= float(m.group(1)) <= t_unblock + 2 and keep.search(m.group(3)):
            lines.append((float(m.group(1)), m.group(2), m.group(3)[:150]))
say('\n== 封鎖期間各步驟（時間）')
for name, t0, t1, st in steps:
    say(f'  {name}：{time.strftime("%H:%M:%S", time.localtime(t0))}～{time.strftime("%H:%M:%S", time.localtime(t1))} {"（跳過）" if st == "skip" else ""}')
say('\n== 封鎖期間的相關 log')
for t, node, msg in sorted(set(lines)):
    say(f'  {time.strftime("%H:%M:%S", time.localtime(t))} {node[:20]} | {msg}')
path = f'{OUT}/block_{TAG}.txt'
open(path, 'w', encoding='utf-8').write('\n'.join(report) + '\n')
print(f'\n結果存在 {path}。回 VS Code 跟車上 agent 說「封鎖外網測完了」，它會幫你分析。', flush=True)
