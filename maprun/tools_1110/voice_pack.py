#!/usr/bin/env python3
"""收音測試整理（11/10 車上 agent）：把 asr_cer.py 的三輪結果＋asr_chain log＋latency_logger 合成一張逐句表。

用法：python3 -I voice_pack.py <輸出資料夾>
輸出：voice_rounds_1110.csv（逐句）、voice_summary_1110.md（每輪一行）、asr_cer_<HHMM>_遮蔽.txt（多出來的段落內容拿掉）
★ 不輸出：旁人／雜音段落的內容、機器人講話的內容（迎賓詞含已註冊者的名字）、latency_logger 的對話文字。
"""
import csv
import datetime as dt
import os
import re
import statistics as st
import sys

OUT = sys.argv[1]
A = os.path.expanduser('~/maprun/verify_out/day_1110/asr/')
LOGS = os.path.expanduser('~/.smartnav/logs/system_control/')
TURNS = os.path.expanduser('~/maprun/verify_out/day_1110/story/latency/turns.csv')
ROUNDS = [
    ('第1輪', '1745', '平板', '初始等待 3000 ms（原設定）；辨識優先權 0（-10 沒權限）；人臉／相機 nice +10；迎賓劇本開著（測試中問候 3 次）',
     'asr_chain_20261110_171355.log', '17:45:19', '17:48:04'),
    ('第2輪重測', '1804', '平板', '初始等待 1000 ms；辨識 nice -10；人臉／相機 nice +10；迎賓劇本關',
     'asr_chain_20261110_175151.log', '18:04:41', '18:06:18'),
    ('第3輪', '1808', '相機麥克風（Astra S，增益 60，left 單聲道）', '初始等待 1000 ms；辨識與收音 nice -10；人臉／相機 nice +10；迎賓劇本關',
     'asr_chain_20261110_175151.log', '18:08:03', '18:09:22'),
]
ROW = re.compile(r'^\s*(\d+)\s+(★)?\s*([\d.]+)%\s+([\d.]+)%\s+(\d+)/\s*(\d+)/\s*(\d+)\s+(\S+)\s+(\S+)\s+(\S+)\s*$')


def hms(t):
    return dt.datetime.fromtimestamp(t).strftime('%H:%M:%S')


def parse_table(path):
    rows, cur = [], None
    for line in open(path, encoding='utf-8'):
        m = ROW.match(line)
        if m:
            cur = dict(zip(['句', '熱詞', '嚴格CER%', '寬鬆CER%', '取', '刪', '插', 'overflow', '佇列滿', '對齊'], m.groups()))
            cur['熱詞'] = '是' if cur['熱詞'] else ''
            rows.append(cur)
            continue
        if cur is not None and '參考：' in line:
            cur['參考'] = re.sub(r'（熱詞：.*）', '', line.split('參考：', 1)[1].strip())
        elif cur is not None and '辨識：' in line:
            s = line.split('辨識：', 1)[1].strip()
            m2 = re.search(r'〔(\d\d:\d\d:\d\d)〕', s)
            cur['辨識'] = re.sub(r'\s*〔.*〕', '', s).strip()
            cur['最終時間'] = m2.group(1) if m2 else ''
    return rows


def log_events(path, t0, t1):
    ev = []
    for line in open(os.path.join(LOGS, path), encoding='utf-8', errors='replace'):
        m = re.search(r'\[(\d{10}\.\d+)\]', line)
        if not m:
            continue
        t = float(m.group(1))
        if not (t0 <= hms(t) <= t1):
            continue
        if 'IDLE -> COMMAND' in line:
            ev.append(('cmd', t, ''))
        elif 'COMMAND -> IDLE' in line:
            ev.append(('idle', t, ''))
        elif '最終結果' in line:
            m2 = re.search(r": '(.*)'", line)
            ev.append(('final', t, m2.group(1) if m2 else ''))
    return ev


def partial_gaps(csvp):
    out, lc, lt = {}, None, None
    for r in csv.DictReader(open(csvp, encoding='utf-8')):
        if r['kind'] == 'partial':
            if r['text'] != lt:
                lc, lt = float(r['epoch']), r['text']
        elif r['kind'] == 'final':
            if lc:
                out[r['clock']] = float(r['epoch']) - lc
            lc, lt = None, None
    return out


turns = [r for r in csv.DictReader(open(TURNS, encoding='utf-8')) if r['來源'] == '語音']
os.makedirs(OUT, exist_ok=True)
allrows, summ = [], []
for name, tag, mic, cond, logf, t0, t1 in ROUNDS:
    table = parse_table(os.path.join(A, f'asr_cer_{tag}.txt'))
    ev = log_events(logf, t0, t1)
    gaps = partial_gaps(os.path.join(A, f'asr_cer_{tag}.csv'))
    # 每個 final 對應前一個 cmd／idle
    finals, last_cmd, last_idle = {}, None, None
    for k, t, txt in ev:
        if k == 'cmd':
            last_cmd = t
        elif k == 'idle':
            last_idle = t
        elif k == 'final':
            finals[hms(t)] = (t, last_cmd, last_idle)
            last_cmd = last_idle = None
    n_trig = sum(1 for k, _, _ in ev if k == 'cmd')
    lat = {'段長': [], '收尾': [], '出字': [], '說完': []}
    for r in table:
        r.update({'輪次': name, '收音': mic, '條件': cond})
        f = finals.get(r.get('最終時間', ''))
        seg = vad_end = None
        if f and f[1] and f[2]:
            seg, vad_end = f[2] - f[1], f[0] - f[2]
        gap = gaps.get(r.get('最終時間', ''))
        said = ''
        if r.get('最終時間'):
            tt = dt.datetime.strptime(r['最終時間'], '%H:%M:%S')
            for tr in turns:
                if abs((dt.datetime.strptime(tr['時間'], '%H:%M:%S') - tt).total_seconds()) <= 1 and tr['說完話→文字']:
                    said = float(tr['說完話→文字'])
                    break
        r['VAD段長s'] = f'{seg:.2f}' if seg is not None else ''
        r['VAD收尾到最終s'] = f'{vad_end:.2f}' if vad_end is not None else ''
        r['最後出字到最終s'] = f'{gap:.2f}' if gap is not None else ''
        r['說完話到文字s'] = f'{said:.2f}' if said != '' else ''
        for k, v in (('段長', seg), ('收尾', vad_end), ('出字', gap), ('說完', said if said != '' else None)):
            if v is not None:
                lat[k].append(v)
        allrows.append(r)
    txt = open(os.path.join(A, f'asr_cer_{tag}.txt'), encoding='utf-8').read()
    cer = re.search(r'整體 CER（嚴格）：(\d+)/(\d+) = ([\d.]+)%', txt).groups()
    cer2 = re.search(r'整體 CER（寬鬆.*?：(\d+)/(\d+) = ([\d.]+)%', txt).groups()
    miss = re.search(r'漏掉.*?：(\d+) 句', txt).group(1)
    extra = re.search(r'多出來的.*?：(\d+) 段', txt).group(1)
    q = re.search(r'音訊處理佇列已滿：(\d+) 行', txt).group(1)
    ov = re.search(r'input overflow：(\d+) 行', txt).group(1)
    head_err = sum(1 for r in table if r.get('辨識') and not r['辨識'].startswith('（') and r['參考'][:1] != r['辨識'][:1])
    med = lambda k: f'{st.median(lat[k]):.2f}' if lat[k] else ''
    summ.append([name, mic, cond, f'{cer[2]}%（{cer[0]}/{cer[1]}）', f'{cer2[2]}%', miss, head_err, n_trig, extra, ov, q,
                 med('說完'), med('出字'), med('收尾'), med('段長')])
    # 遮蔽版 txt：多出來的段落只留時間與字數
    red = re.sub(r'^(\s+\d\d:\d\d:\d\d)  「(.*?)」', lambda m: f'{m.group(1)}  （內容不打包，{len(m.group(2))} 字）', txt, flags=re.M)
    open(os.path.join(OUT, f'asr_cer_{tag}_遮蔽.txt'), 'w', encoding='utf-8').write(red)

cols = ['輪次', '收音', '句', '熱詞', '參考', '辨識', '嚴格CER%', '寬鬆CER%', '取', '刪', '插', 'overflow', '佇列滿', '對齊', '最終時間',
        '說完話到文字s', '最後出字到最終s', 'VAD收尾到最終s', 'VAD段長s', '條件']
with open(os.path.join(OUT, 'voice_rounds_1110.csv'), 'w', newline='', encoding='utf-8-sig') as f:
    w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
    w.writeheader()
    w.writerows(allrows)
hd = ['輪次', '收音', '條件', '嚴格CER', '寬鬆CER', '漏掉句', '句首錯句', 'VAD觸發次數', '多出來的段', 'input overflow 行', '佇列滿行',
      '說完話→文字 中位 s', '最後出字→最終 中位 s', 'VAD收尾→最終 中位 s', 'VAD段長 中位 s']
with open(os.path.join(OUT, 'voice_summary_1110.md'), 'w', encoding='utf-8') as f:
    f.write('| ' + ' | '.join(hd) + ' |\n|' + '---|' * len(hd) + '\n')
    for s in summ:
        f.write('| ' + ' | '.join(str(x) for x in s) + ' |\n')
print(open(os.path.join(OUT, 'voice_summary_1110.md'), encoding='utf-8').read())
