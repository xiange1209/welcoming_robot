#!/usr/bin/env python3
"""ASR 實際辨識準確度（交接文件第 9 項・語音部分）：使用者照稿唸 10 句，逐字比對 /user_text，算 CER。

用法（車上；語音輸入單元要開著。★ LLM 單元要關，理由見下面「⚠ 安全」）：
    source ~/maprun/env.sh
    python3 ~/maprun/tools_1020/asr_cer.py --print-script          # 印出要唸的稿子（★＝含熱詞）
    python3 ~/maprun/tools_1020/asr_cer.py --print-script --paced  # 稿子一句一句出現（每句唸完留 3 秒）
    python3 ~/maprun/tools_1020/asr_cer.py                         # 錄 180 秒（--secs 改），結束自動比對
    python3 ~/maprun/tools_1020/asr_cer.py --start-after 15:42:10  # 這個時刻之前收到的不算（使用者還沒開始唸）
    python3 ~/maprun/tools_1020/asr_cer.py --from-csv ~/maprun/verify_out/asr_cer_1542.csv [--skip 8]
    python3 ~/maprun/tools_1020/asr_cer.py --from-log --start-after 15:42:10 --until 15:45:00
        # 補救：這支沒開著的時候，直接拿 asr_chain log 的「✓ 最終結果」行算（★ 那是後處理「之前」的原始結果）
錄音中 Ctrl-C（或 kill -INT）會提早結束，照樣比對、寫檔。

量什麼：
    訂 /user_text（最終結果＝後處理＋回音過濾之後、真正送進 LLM 的字串）與 /partial_text；
    另外旁聽 /speech_text（機器人要講的話）與 /playback_status（平板真的在唸），只為了標出
    「測試期間機器人有沒有出聲」。全部唯讀：只建訂閱，不建任何發布者（rclpy 內建的 /parameter_events 除外）。
    每一則都即時寫進 ~/maprun/verify_out/asr_cer_<HHMM>.csv（kind＝final/partial/speech_text/playback），
    結束後寫 asr_cer_<HHMM>.txt（就是螢幕上那張表）。

怎麼比對：
    正規化：NFKC，只留文字與數字（標點、空白、符號全去掉）。
      嚴格 CER：其餘照原樣比——「台／臺／檯」、簡／繁不同都算錯（跟 tools_0819 的 norm() 同一套）。
      寬鬆 CER：逐字轉簡體（OpenCC t2s）＋小異體字表（臺檯颱→台、裡裏→里、妳→你…）＋阿拉伯數字逐字轉國字，
                視為相同。★ 模型詞表是簡體、經 s2twp 才變繁體，同一個「台」寫成「臺」還是「檯」是轉換器
                依上下文決定的，不是聽錯——兩個數字都報，差距＝字形問題的份量。
    對齊（動態規劃，照唸的順序）：一句可以對到 1～3 段（被斷開）、相鄰兩句可以併成 1 段、
      一句可以沒有對到任何段（漏掉，CER 100%）、一段可以不屬於任何一句（「多出來的（旁人／雜音）」，
      ★ 不計入 CER，但逐條列出時間與內容，自己判斷是不是真的旁人）。
      ⚠ 爛到比「什麼都沒有」還差的結果（編輯距離 > 參考字數）也會被判成「多出來的」，該句以 100% 計。
    CER = (取代＋刪除＋插入) / 參考字數；整體 CER = 各句錯字加總 ÷ 參考字數加總（不是各句 CER 平均）。

每句的 overflow／佇列滿（分清楚「模型認錯」與「聲音根本沒送到模型」）：
    該句「第一則 partial 往前 2 秒」到「最後一則 final」之間，asr_chain log 裡
      input overflow     voice_trigger：PortAudio 來不及讀，音訊被丟掉
      音訊處理佇列已滿    speech_recognizer：辨識跟不上，音訊塊被丟掉
    各有幾行（用行內的 [epoch] 比時間）。往前 2 秒是因為 partial 比聲音晚出來（佇列最多積 1.6 秒）；
    不會早於上一則 final。沒有 partial 時（--from-log）改用 log 的 VAD 起點「IDLE -> COMMAND」。
    ★ 這兩欄不是 0 的句子，錯字可能是掉音訊造成的；總表另外分組報「有掉／沒掉」兩組的 CER。

⚠ 安全：稿子裡有「帶我去貴賓室」「櫃檯在哪裡」——LLM 與導航都開著時車會真的開過去，而使用者正站在
  相機前 30～50 cm。所以偵測到 LLM 節點訂著 /user_text 時這支**拒絕開始**，除非加 --llm-ok
  （例如確定導航關著、想看整條鏈）。LLM 開著也會讓平板把回答唸回麥克風，量到的就不只是使用者的聲音。

★ 這次的 CER 跟 8/19 的 25.6% 不能直接比：那次音源是 TTS、每句取「最好的一段」；這次是真人、全部段落照順序對齊。
★ 2026-10-20 車上新寫，只用假資料（隔離的 ROS_DOMAIN_ID=78＋假 log）驗證過，沒有在實車上量過。
"""
import argparse
import csv
import datetime as dt
import functools
import glob
import os
import re
import subprocess
import sys
import time
import unicodedata

# （句子, 類型）。★ 數字寫國字：模型詞表沒有阿拉伯數字（tokens.txt 只有一個 "2"），一定吐國字。
# ★ 每句都確認過 s2twp(t2s(句子)) == 句子：模型完全聽對時，嚴格 CER 就是 0。
SENTENCES = [
    ("我要開戶", "短指令"),
    ("請問櫃檯在哪裡", "問句（今天聽成「櫃櫃裡來臺在哪裡」那句）"),
    ("帶我去貴賓室", "短指令（⚠ 導航指令）"),
    ("你們幾點關門", "問句"),
    ("我的信用卡不見了怎麼辦", "問句"),
    ("我想領兩千塊", "短指令・數字"),
    ("我的號碼牌是一百二十八號", "數字"),
    ("機器人你叫什麼名字", "問句"),
    ("今天外面會不會下雨", "問句"),
    ("我在等我朋友", "短句"),
]
HOTWORD_FILES = [
    os.path.expanduser("~/welcoming_robot_ws/install/smartnav_audio/share/smartnav_audio/config/hotwords.txt"),
    os.path.expanduser("~/welcoming_robot_ws/src/smartnav_audio/config/hotwords.txt"),
]
OUT_DIR = os.path.expanduser("~/maprun/verify_out")
LOG_DIR = os.path.expanduser("~/.smartnav/logs/system_control")
PAUSE_S = 3.0  # 稿子：每句之間停幾秒（VAD 靜音 0.5 s＋sherpa 端點 1.2 s，留足）
PRE_MARGIN_S = 2.0  # overflow 視窗：第一則 partial 往前多少秒
BURST_GAP_S = 3.0  # 同一句的 partial 之間最多隔多久（超過就是前一段話留下的）
MAX_SPLIT = 3  # 一句最多對到幾段
EXTRA_PEN = 0.5  # 判成「多出來的」的懲罰（字數單位）：跟「什麼都沒有」一樣爛的段落仍算給那一句
EPS = 0.01  # 拆句／併句的結構懲罰，平手時選最簡單的解釋

try:
    from opencc import OpenCC

    _T2S = OpenCC("t2s")
except Exception:  # noqa: BLE001
    _T2S = None

DIGIT_ZH = dict(zip("0123456789", "零一二三四五六七八九"))
VARIANTS = {
    "臺": "台", "檯": "台", "颱": "台", "裡": "里", "裏": "里", "妳": "你", "祢": "你", "〇": "零",
    "账": "帐", "賬": "帐", "帳": "帐", "著": "着", "週": "周", "於": "于",
}


# ── 正規化與編輯距離 ─────────────────────────────────────────
def norm_strict(s):
    """NFKC 後只留文字（L*）與數字（N*）：標點、空白、符號全去掉。"""
    s = unicodedata.normalize("NFKC", s or "")
    return "".join(ch for ch in s if unicodedata.category(ch)[0] in "LN")


@functools.lru_cache(maxsize=None)
def _lenient_char(ch):
    c = DIGIT_ZH.get(ch) or ch.casefold()
    if len(c) != 1:
        c = ch
    if _T2S is not None:
        try:
            s = _T2S.convert(c)
            if len(s) == 1:
                c = s
        except Exception:  # noqa: BLE001
            pass
    return VARIANTS.get(c, c)


def norm_lenient(strict):
    """★ 逐字對應（長度不變），所以嚴格與寬鬆可以共用同一個切點。"""
    return "".join(_lenient_char(ch) for ch in strict)


@functools.lru_cache(maxsize=None)
def ed_ops(ref, hyp):
    """字元級編輯距離，回傳 (距離, 取代, 刪除, 插入)。刪除＝參考有、辨識沒有。"""
    prev = [(j, 0, 0, j) for j in range(len(hyp) + 1)]
    for i in range(1, len(ref) + 1):
        cur = [(i, 0, i, 0)]
        for j in range(1, len(hyp) + 1):
            p = prev[j - 1]
            sub = p if ref[i - 1] == hyp[j - 1] else (p[0] + 1, p[1] + 1, p[2], p[3])
            d = prev[j]
            ins = cur[j - 1]
            cur.append(min(sub, (d[0] + 1, d[1], d[2] + 1, d[3]), (ins[0] + 1, ins[1], ins[2], ins[3] + 1),
                           key=lambda t: t[0]))
        prev = cur
    return prev[-1]


def ed(ref, hyp):
    return ed_ops(ref, hyp)[0]


def align(refs, hyps):
    """照順序對齊（refs、hyps 都是寬鬆正規化後的字串）。

    回傳 [(op, [參考索引], [段落索引])]，op ∈ match / merge / miss / extra。
    ★ 鬆弛順序＝平手時的偏好：先 1 對 1、再拆句、併句，最後才是「漏掉＋多出來」。
    """
    n, m = len(refs), len(hyps)
    inf = float("inf")
    cost = [[inf] * (m + 1) for _ in range(n + 1)]
    back = [[None] * (m + 1) for _ in range(n + 1)]
    cost[0][0] = 0.0

    def relax(i, j, c, bp):
        if c < cost[i][j] - 1e-9:
            cost[i][j] = c
            back[i][j] = bp

    for i in range(n + 1):
        for j in range(m + 1):
            c = cost[i][j]
            if c == inf:
                continue
            if i < n:
                for k in range(1, MAX_SPLIT + 1):  # 一句 ↔ k 段
                    if j + k > m:
                        break
                    relax(i + 1, j + k, c + ed(refs[i], "".join(hyps[j:j + k])) + EPS * (k - 1), ("match", i, j, 1, k))
                if i + 1 < n and j < m:  # 相鄰兩句 ↔ 一段
                    relax(i + 2, j + 1, c + ed(refs[i] + refs[i + 1], hyps[j]) + EPS, ("merge", i, j, 2, 1))
                relax(i + 1, j, c + len(refs[i]), ("miss", i, j, 1, 0))  # 這句沒對到任何段
            if j < m:
                relax(i, j + 1, c + EXTRA_PEN, ("extra", i, j, 0, 1))  # 這段不屬於任何一句
    steps, i, j = [], n, m
    while (i, j) != (0, 0):
        op, i0, j0, di, dj = back[i][j]
        steps.append((op, list(range(i0, i0 + di)), list(range(j0, j0 + dj))))
        i, j = i0, j0
    return steps[::-1]


# ── 熱詞 ─────────────────────────────────────────────────────
def load_hotwords():
    for p in HOTWORD_FILES:
        try:
            with open(p, encoding="utf-8") as fh:
                ws = [ln.strip().split(":")[0].replace(" ", "") for ln in fh]
            ws = [w for w in ws if w and not w.startswith("#")]
            if ws:
                return ws, p
        except OSError:
            continue
    return [], "（找不到 hotwords.txt）"


def hotwords_in(ref, hotwords):
    """回傳這句含的熱詞（用句子自己的字顯示）。比對在寬鬆空間做（熱詞檔是簡體）。"""
    rs = norm_strict(ref)
    rl = norm_lenient(rs)
    found = []
    for w in hotwords:
        wl = norm_lenient(norm_strict(w))
        k = rl.find(wl) if wl else -1
        if k >= 0 and rs[k:k + len(wl)] not in found:
            found.append(rs[k:k + len(wl)])
    return found


# ── 時間 ─────────────────────────────────────────────────────
def clock(epoch):
    return dt.datetime.fromtimestamp(epoch).strftime("%H:%M:%S") if epoch else "--:--:--"


def clock_to_epoch(hms, ref_epoch):
    """HH:MM[:SS] → epoch，日期取 ref_epoch 那一天（本機時區）。"""
    parts = [int(x) for x in hms.split(":")]
    while len(parts) < 3:
        parts.append(0)
    day = dt.datetime.fromtimestamp(ref_epoch)
    return day.replace(hour=parts[0], minute=parts[1], second=parts[2], microsecond=0).timestamp()


def clock_synced():
    try:
        r = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
                           capture_output=True, text=True, timeout=3)
        return r.stdout.strip() or "?"
    except Exception:  # noqa: BLE001
        return "?"


# ── asr_chain log ────────────────────────────────────────────
LOG_RE = re.compile(r"^\[([^\]]+)\] \[(\w+)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)$")
FINAL_RE = re.compile(r"最終結果（(.+?) 斷句）: '(.*)'$")


def find_logs(explicit, t_from):
    if explicit:
        return [explicit]
    fs = sorted(glob.glob(os.path.join(LOG_DIR, "asr_chain_*.log")), key=os.path.getmtime)
    if not fs:
        return []
    # 視窗開始之後還有寫入的都要掃（中途重啟 ASR 會換檔）；行內有 epoch，多掃不會重算
    hit = [f for f in fs if t_from is None or os.path.getmtime(f) >= t_from - 120]
    return hit or fs[-1:]


def parse_logs(paths):
    ev = {k: [] for k in ("overflow", "qfull", "onset", "echo", "drop", "tts", "split", "forced", "final")}
    status = {}
    for p in paths:
        try:
            fh = open(p, encoding="utf-8", errors="replace")
        except OSError:
            continue
        with fh:
            for raw in fh:
                line = raw.rstrip("\n")
                m = LOG_RE.match(line)
                if not m:
                    if "ASR 使用 Astra S" in line:
                        status["card"] = line.split("]:", 1)[-1].strip()
                    continue
                t, msg = float(m.group(3)), m.group(5)
                if "input overflow" in msg:
                    ev["overflow"].append(t)
                elif "音訊處理佇列已滿" in msg:
                    ev["qfull"].append(t)
                elif "IDLE -> COMMAND" in msg:
                    ev["onset"].append(t)
                elif "🔁 丟棄回音" in msg:
                    ev["echo"].append(t)
                elif msg.startswith("⊘ 丟棄"):
                    ev["drop"].append(t)
                elif "平板回報開始朗讀" in msg:
                    ev["tts"].append(t)
                elif "🔀 TTS" in msg:
                    ev["split"].append(t)
                elif "⏱ 單句已達" in msg:
                    ev["forced"].append(t)
                elif "最終結果" in msg:
                    fm = FINAL_RE.search(msg)
                    if fm:
                        ev["final"].append((t, fm.group(2), fm.group(1)))
                if "熱詞已啟用" in msg or "未提供熱詞檔" in msg or "熱詞載入失敗" in msg:
                    status["hotword"] = msg.strip()
                if "雙麥克風模式" in msg:
                    status["mic_mode"] = msg.strip()
    return ev, status


def count_in(ts, a, b):
    return sum(1 for t in ts if a <= t <= b)


def window_for(hidx, hyps, partial_ts, all_final_ts, onsets):
    """該句的時間窗：第一則 partial 往前 PRE_MARGIN_S，到最後一段 final；不早於上一則 final。"""
    t_first, t_end = hyps[hidx[0]]["epoch"], hyps[hidx[-1]]["epoch"]
    if t_first is None or t_end is None:
        return None
    prev = max((t for t in all_final_ts if t < t_first - 1e-6), default=None)
    start, cur = None, t_first
    for t in sorted((t for t in partial_ts if t <= t_first and (prev is None or t > prev)), reverse=True):
        if cur - t > BURST_GAP_S:
            break
        start = cur = t
    if start is None:  # 沒有 partial：用 VAD 起點
        cands = [t for t in onsets if t <= t_first and (prev is None or t > prev)]
        start = max(cands) if cands else t_first - 5.0
    start -= PRE_MARGIN_S
    if prev is not None:
        start = max(start, prev)
    return start, t_end


# ── 比對與報告 ───────────────────────────────────────────────
def pct(e, n):
    return f"{100.0 * e / n:5.1f}%" if n else "  -  "


def analyze(finals, partial_ts, all_final_ts, skip, ev, status, extra_info):
    """finals：[{text, epoch}]（已依 --start-after/--until 篩過）。回傳報告的每一行。"""
    hotwords, hw_src = load_hotwords()
    ref_ids = [i for i in range(len(SENTENCES)) if (i + 1) not in skip]
    refs = [SENTENCES[i][0] for i in ref_ids]
    rs = [norm_strict(r) for r in refs]
    rl = [norm_lenient(x) for x in rs]
    hs = [norm_strict(h["text"]) for h in finals]
    hl = [norm_lenient(x) for x in hs]
    per, extras = {}, []
    for op, ri, hj in align(rl, hl):
        if op == "extra":
            extras.append(hj[0])
        elif op == "miss":
            per[ri[0]] = dict(op="漏掉", hyps=[], parts=[""], s=ed_ops(rs[ri[0]], ""), l=ed_ops(rl[ri[0]], ""))
        elif op == "match":
            i = ri[0]
            per[i] = dict(op="1段" if len(hj) == 1 else f"拆成{len(hj)}段", hyps=hj, parts=[hs[j] for j in hj],
                          s=ed_ops(rs[i], "".join(hs[j] for j in hj)), l=ed_ops(rl[i], "".join(hl[j] for j in hj)))
        else:  # merge：在寬鬆字串上找最好的切點，嚴格用同一個切點
            i, j = ri[0], hj[0]
            k = min(range(len(hl[j]) + 1), key=lambda x: ed(rl[i], hl[j][:x]) + ed(rl[i + 1], hl[j][x:]))
            per[i] = dict(op="與下句併1段", hyps=[j], parts=[hs[j][:k]], s=ed_ops(rs[i], hs[j][:k]),
                          l=ed_ops(rl[i], hl[j][:k]))
            per[i + 1] = dict(op="與上句併1段", hyps=[j], parts=[hs[j][k:]], s=ed_ops(rs[i + 1], hs[j][k:]),
                              l=ed_ops(rl[i + 1], hl[j][k:]))

    out = list(extra_info)
    out.append(f"參考句：{len(refs)} 句" + (f"（跳過 #{','.join(str(x) for x in sorted(skip))}）" if skip else "")
               + f"；寬鬆比對用 OpenCC：{'有' if _T2S else '沒有（只用內建異體字表）'}")
    out.append("")
    out.append(" #  熱詞  嚴格CER  寬鬆CER  取/刪/插  overflow 佇列滿  對齊")
    groups = {"hot": [0, 0, 0], "cold": [0, 0, 0], "drop": [0, 0, 0], "clean": [0, 0, 0]}
    tot_s = tot_l = tot_n = 0
    missed = []
    for i, sid in enumerate(ref_ids):
        r = per[i]
        n = len(rs[i])
        hw = hotwords_in(refs[i], hotwords)
        win = window_for(r["hyps"], finals, partial_ts, all_final_ts, ev["onset"]) if r["hyps"] else None
        ovf = count_in(ev["overflow"], *win) if win else None
        qf = count_in(ev["qfull"], *win) if win else None
        s, l_ = r["s"], r["l"]
        tot_s, tot_l, tot_n = tot_s + s[0], tot_l + l_[0], tot_n + n
        for g in (("hot" if hw else "cold"),) + ((("drop" if (ovf or qf) else "clean"),) if win else ()):
            groups[g][0] += s[0]
            groups[g][1] += l_[0]
            groups[g][2] += n
        if r["op"] == "漏掉":
            missed.append(sid + 1)
        out.append(f"{sid + 1:2d}   {'★' if hw else '　'}   {pct(s[0], n)}   {pct(l_[0], n)}   "
                   f"{s[1]:2d}/{s[2]:2d}/{s[3]:2d}   {('-' if ovf is None else ovf):>6}  {('-' if qf is None else qf):>5}"
                   f"   {r['op']}")
        out.append(f"       參考：{refs[i]}" + (f"（熱詞：{'、'.join(hw)}）" if hw else ""))
        if r["hyps"]:
            when = "／".join(clock(finals[j]["epoch"]) for j in r["hyps"])
            out.append(f"       辨識：{' | '.join(r['parts'])}　〔{when}〕")
        else:
            out.append("       辨識：（沒有對到任何辨識結果，以 100% 計）")
    out.append("-" * 64)
    out.append(f"整體 CER（嚴格）：{tot_s}/{tot_n} = {pct(tot_s, tot_n).strip()}")
    out.append(f"整體 CER（寬鬆，異體字／簡繁／數字寫法視為相同）：{tot_l}/{tot_n} = {pct(tot_l, tot_n).strip()}")

    def g(name):
        e_s, e_l, n_ = groups[name]
        return f"嚴格 {pct(e_s, n_).strip()}／寬鬆 {pct(e_l, n_).strip()}（{n_} 字）"

    out.append(f"  含熱詞的句子：{g('hot')}　不含熱詞：{g('cold')}")
    out.append(f"  該句有掉音訊（overflow 或佇列滿 > 0）：{g('drop')}　沒掉：{g('clean')}")
    out.append(f"漏掉（沒有對到任何辨識結果，以 100% 計入）：{len(missed)} 句" + (f"（#{', #'.join(map(str, missed))}）" if missed else ""))
    out.append(f"多出來的（旁人／雜音）：{len(extras)} 段，不計入 CER" + ("：" if extras else ""))
    for j in extras:
        out.append(f"    {clock(finals[j]['epoch'])}  「{finals[j]['text']}」")
    out.append(f"（熱詞依 {hw_src} 判定）")
    return out


def validity_lines(ev, status, t_lo, t_hi, speech_n, playback_n, extra=()):
    out = ["-" * 64, "有效性檢查（asr_chain log，時間窗 " + f"{clock(t_lo)}～{clock(t_hi)}）："]
    hw = status.get("hotword", "（log 裡沒看到熱詞那一行）")
    out.append(f"  熱詞：{hw}" + ("　← ★ 熱詞沒生效，「含熱詞」那組沒有加分" if "已啟用" not in hw else ""))
    out.append(f"  麥克風：{status.get('mic_mode', '?')}　{status.get('card', '')}")
    if t_lo and t_hi and t_hi > t_lo:
        mins = (t_hi - t_lo) / 60.0
        o, q = count_in(ev["overflow"], t_lo, t_hi), count_in(ev["qfull"], t_lo, t_hi)
        out.append(f"  input overflow：{o} 行（{o / mins:.1f} 行/分）；音訊處理佇列已滿：{q} 行（{q / mins:.1f} 行/分）")
        out.append(f"  回音過濾丟棄：{count_in(ev['echo'], t_lo, t_hi)} 次；後處理丟棄（太短／整句同字）："
                   f"{count_in(ev['drop'], t_lo, t_hi)} 次；20 秒強制收尾：{count_in(ev['forced'], t_lo, t_hi)} 次")
        out.append(f"  平板回報朗讀（log）：{count_in(ev['tts'], t_lo, t_hi)} 次；TTS 強制切句：{count_in(ev['split'], t_lo, t_hi)} 次")
    if speech_n is not None:
        out.append(f"  測試期間 /speech_text（機器人要講話）：{speech_n} 則；/playback_status：{playback_n} 則"
                   + ("　← ⚠ 不是 0：TTS 可能混進麥克風" if (speech_n or playback_n) else ""))
    out.extend(extra)
    return out


# ── 三種資料來源 ─────────────────────────────────────────────
def unique_stem(out_dir, base):
    stem, k = os.path.join(out_dir, base), 2
    while os.path.exists(stem + ".csv") or os.path.exists(stem + ".txt"):
        stem, k = os.path.join(out_dir, f"{base}_{k}"), k + 1
    return stem


def filt(rows, lo, hi):
    return [r for r in rows if r["epoch"] is None or ((lo is None or r["epoch"] >= lo) and (hi is None or r["epoch"] <= hi))]


def record(args, stem):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import Bool, String

    rclpy.init()
    node = Node("asr_cer_probe", enable_rosout=False, start_parameter_services=False)
    rows, m0 = [], time.monotonic()
    out = {}  # CSV 等圖檢查通過才開；之前收到的先留在 rows，開檔時補寫

    def on(kind, text):
        now = time.time()
        row = dict(seq=len(rows) + 1, kind=kind, epoch=f"{now:.3f}", clock=clock(now),
                   rel_s=f"{time.monotonic() - m0:.2f}", text=text)
        rows.append(row)
        if out:
            out["w"].writerow(row)
            out["fh"].flush()
        if kind == "final":
            print(f"  [{row['rel_s']:>7}s {row['clock']}] 最終：「{text}」", flush=True)
        elif kind == "speech_text":
            print(f"  [{row['rel_s']:>7}s {row['clock']}] ⚠ 機器人要講話（/speech_text）：「{text}」", flush=True)
        elif kind == "playback":
            print(f"  [{row['rel_s']:>7}s {row['clock']}] ⚠ 平板{'開始' if text == 'True' else '結束'}朗讀", flush=True)

    node.create_subscription(String, "/user_text", lambda m: on("final", m.data), 10)
    node.create_subscription(String, "/partial_text", lambda m: on("partial", m.data), 10)
    node.create_subscription(String, "/speech_text", lambda m: on("speech_text", m.data), 10)
    node.create_subscription(Bool, "/playback_status", lambda m: on("playback", str(bool(m.data))), 10)

    # 圖檢查：誰在發 /user_text、誰在訂（LLM 在訂就拒絕）
    me, pubs, t0 = node.get_name(), [], time.monotonic()
    while time.monotonic() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        pubs = [x.node_name for x in node.get_publishers_info_by_topic("/user_text")]
        if "speech_recognizer_node" in pubs:
            break
    t1 = time.monotonic()
    while time.monotonic() - t1 < 0.5:
        rclpy.spin_once(node, timeout_sec=0.1)
    subs = sorted({x.node_name for x in node.get_subscriptions_info_by_topic("/user_text")} - {me})
    echo_ok = "speech_recognizer_node" in {x.node_name for x in node.get_subscriptions_info_by_topic("/speech_text")}
    graph = [f"/user_text 發布端：{', '.join(pubs) or '（沒有）'}；訂閱端：{', '.join(subs) or '（只有這支）'}；"
             f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '?')}"]
    print(graph[0], flush=True)
    if "speech_recognizer_node" not in pubs:
        print("  ⚠ 沒看到 speech_recognizer_node 在發 /user_text —— 語音輸入單元有開嗎？（照樣錄，可能 0 段）", flush=True)
    if not echo_ok:
        print("  ⚠ speech_recognizer_node 沒訂 /speech_text：回音過濾沒有東西可比", flush=True)
    llm = [s for s in subs if "llm" in s.lower()]
    if llm and not args.llm_ok:
        print(f"✗ 偵測到 LLM 節點訂著 /user_text（{', '.join(llm)}）——拒絕開始。\n"
              "  稿子有「帶我去貴賓室」「櫃檯在哪裡」：LLM＋導航開著時車會真的開過去，使用者就站在相機前。\n"
              "  先關 LLM 單元（語音輸入照開）；確定導航關著又想看整條鏈才加 --llm-ok。", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(2)
    if llm:
        graph.append(f"  ⚠ LLM 開著（--llm-ok）：{', '.join(llm)}——回答可能被平板唸回麥克風")
    fh = open(stem + ".csv", "w", newline="", encoding="utf-8")
    out["w"] = csv.DictWriter(fh, fieldnames=["seq", "kind", "epoch", "clock", "rel_s", "text"])
    out["w"].writeheader()
    for r in rows:
        out["w"].writerow(r)
    out["fh"] = fh
    fh.flush()

    t_start = time.time()
    print(f"▶ 開始錄 {args.secs:.0f} 秒（{clock(t_start)} 起；Ctrl-C 提早結束）。請使用者照稿唸，每句之間停 {PAUSE_S:.0f} 秒。", flush=True)
    nxt = 30.0
    try:
        while time.monotonic() - m0 < args.secs:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - m0 >= nxt:
                print(f"  …剩 {args.secs - nxt:.0f} 秒", flush=True)
                nxt += 30.0
    except (KeyboardInterrupt, ExternalShutdownException):
        print("（提早結束）", flush=True)
    finally:
        fh.close()
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            rclpy.try_shutdown()
        except Exception:  # noqa: BLE001
            pass
    for r in rows:
        r["epoch"] = float(r["epoch"])
    return rows, t_start, time.time(), graph


def read_csv(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            ep = r.get("epoch")
            rows.append(dict(kind=r.get("kind") or "final", text=r.get("text", ""),
                             epoch=float(ep) if ep not in (None, "") else None))
    return rows


def main():
    ap = argparse.ArgumentParser(description="ASR 照稿唸 10 句、逐字比對 CER（說明見檔頭）")
    ap.add_argument("--print-script", action="store_true", help="印出要唸的稿子後結束")
    ap.add_argument("--paced", action="store_true", help="稿子一句一句出現（隱含 --print-script）")
    ap.add_argument("--secs", type=float, default=180.0, help="錄多久（秒，預設 180）")
    ap.add_argument("--start-after", metavar="HH:MM:SS", help="這個時刻之前的辨識結果不算")
    ap.add_argument("--until", metavar="HH:MM:SS", help="這個時刻之後的辨識結果不算")
    ap.add_argument("--from-csv", metavar="CSV", help="離線重算（這支寫的 CSV，或至少有 text 欄的 CSV）")
    ap.add_argument("--from-log", action="store_true", help="從 asr_chain log 的「✓ 最終結果」行算（後處理前）")
    ap.add_argument("--asr-log", metavar="LOG", help="指定 asr_chain log（預設：自動找時間窗內有寫入的）")
    ap.add_argument("--skip", default="", help="不計分的句號，例如 8 或 3,8（使用者確定沒唸）")
    ap.add_argument("--llm-ok", action="store_true", help="LLM 在訂 /user_text 也照樣開始（⚠ 見檔頭「安全」）")
    ap.add_argument("--out-dir", default=OUT_DIR, help=f"輸出目錄（預設 {OUT_DIR}）")
    args = ap.parse_args()
    skip = {int(x) for x in re.split(r"[,\s]+", args.skip.strip()) if x}

    if args.print_script or args.paced:
        hotwords, hw_src = load_hotwords()
        print(f"=== ASR 準確度測試稿（共 {len(SENTENCES)} 句）===")
        print("對著車上相機（麥克風在相機裡）30～50 cm，正常音量、正常速度；一句中間不要停頓。")
        print(f"每句唸完停 {PAUSE_S:.0f} 秒再唸下一句。唸錯不用重唸，直接接下一句。旁邊的人請先不要講話。")
        print("數字照平常的說法唸（一百二十八、兩千）。\n")
        for k, (s, kind) in enumerate(SENTENCES, 1):
            hw = hotwords_in(s, hotwords)
            mark = f"★熱詞（{'、'.join(hw)}）" if hw else "（無熱詞）"
            print(f"{k:2d}. {s}　　{mark}　{kind}", flush=True)
            if args.paced:
                time.sleep(PAUSE_S + 0.3 * len(s))
            elif k < len(SENTENCES):
                print(f"    （停 {PAUSE_S:.0f} 秒）")
        print(f"\n★＝含熱詞（依 {hw_src}）。⚠ 今天的 log 寫「未提供熱詞檔，跳過熱詞」——熱詞其實沒生效，"
              "詳見 ~/maprun/verify_out/voice_notes_20261020.md。")
        return

    os.makedirs(args.out_dir, exist_ok=True)
    speech_n = playback_n = None
    graph, extra_val = [], []
    if args.from_csv:
        rows = read_csv(args.from_csv)
        src = f"離線重算 {args.from_csv}"
        stem = os.path.splitext(args.from_csv)[0] + "_recalc"
        t_ref = next((r["epoch"] for r in rows if r["epoch"]), time.time())
    elif args.from_log:
        paths = [args.asr_log] if args.asr_log else find_logs(None, None)[-1:]
        ev0, _ = parse_logs(paths)
        rows = [dict(kind="final", text=tx, epoch=t) for t, tx, _src in ev0["final"]]
        src = f"asr_chain log「✓ 最終結果」行（★ 後處理之前的原始結果）：{', '.join(paths)}"
        stem = unique_stem(args.out_dir, "asr_cer_log_" + dt.datetime.now().strftime("%H%M"))
        t_ref = rows[0]["epoch"] if rows else time.time()
        with open(stem + ".csv", "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=["seq", "kind", "epoch", "clock", "rel_s", "text"])
            w.writeheader()
            for k, r in enumerate(rows, 1):
                w.writerow(dict(seq=k, kind="final", epoch=f"{r['epoch']:.3f}", clock=clock(r["epoch"]), rel_s="", text=r["text"]))
    else:
        stem = unique_stem(args.out_dir, "asr_cer_" + dt.datetime.now().strftime("%H%M"))
        rows, t_start, t_stop, graph = record(args, stem)
        src = f"即時錄製 {clock(t_start)}～{clock(t_stop)}（{t_stop - t_start:.0f} 秒）"
        t_ref = t_start
        extra_val.append(f"  系統時鐘同步（NTPSynchronized）：{clock_synced()}　← 不是 yes 的話，log 時間對不上")

    lo = clock_to_epoch(args.start_after, t_ref) if args.start_after else None
    hi = clock_to_epoch(args.until, t_ref) if args.until else None
    all_finals = [r for r in rows if r["kind"] == "final"]
    finals = filt(all_finals, lo, hi)
    partial_ts = [r["epoch"] for r in rows if r["kind"] == "partial" and r["epoch"]]
    all_final_ts = [r["epoch"] for r in all_finals if r["epoch"]]
    ts = [r["epoch"] for r in finals if r["epoch"]] + [t for t in partial_ts if (lo is None or t >= lo) and (hi is None or t <= hi)]
    if args.from_csv or args.from_log:
        t_lo = lo if lo is not None else (min(ts) - PRE_MARGIN_S if ts else None)
        t_hi = hi if hi is not None else (max(ts) if ts else None)
    else:
        t_lo, t_hi = max(t_start, lo or 0), t_stop if hi is None else min(t_stop, hi)
    in_win = [r for r in rows if r["epoch"] and t_lo and t_hi and t_lo <= r["epoch"] <= t_hi]
    if any(r["kind"] in ("speech_text", "playback") for r in rows) or not (args.from_csv or args.from_log):
        speech_n = sum(1 for r in in_win if r["kind"] == "speech_text")
        playback_n = sum(1 for r in in_win if r["kind"] == "playback")
    ev, status = parse_logs(find_logs(args.asr_log, t_lo))

    head = [f"==== ASR 準確度（{dt.datetime.now():%Y-%m-%d %H:%M}）====", f"資料：{src}"]
    cut = len(all_finals) - len(finals)
    head.append(f"最終結果：{len(finals)} 段參與比對" + (f"（--start-after／--until 之外 {cut} 段不算）" if cut else ""))
    head.extend(graph)
    lines = analyze(finals, partial_ts, all_final_ts, skip, ev, status, head)
    lines += validity_lines(ev, status, t_lo, t_hi, speech_n, playback_n, extra_val)
    text = "\n".join(lines)
    print(text)
    with open(stem + ".txt", "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    print(f"\n→ {stem}.txt" + ("" if args.from_csv else f"\n→ {stem}.csv"))


if __name__ == "__main__":
    main()
