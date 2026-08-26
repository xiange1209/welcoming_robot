#!/usr/bin/env python3
"""驗證 /speech_text 的斷句與清洗不會把數字切碎。

    python3 tts_split_check.py            # 新舊對照
    python3 tts_split_check.py --old      # 只看舊行為（重現 bug）

## 為什麼需要這支

`RosStreamHandler` 把 `.` `:` `,` 當成斷句符號，而銀行機器人講的話裡
這三個符號幾乎都出現在**數字中間**：

    「營業時間是上午9:00到下午3:30」 -> 唸成「上午9」「00到下午3」「30」

而 HMI 畫面顯示的是 `/llm_response`（完整正確），平板唸的是 `/speech_text`
（被切碎的）—— **畫面對、聲音錯**，現場很難察覺是同一個 bug。

## 這支不需要實機

純字串處理，在筆電上就能跑完。它複製 `RosStreamHandler` 的**逐 token**
行為（一次餵一個字元），因為串流是這個 bug 的關鍵：
lookahead `(?!\\d)` 需要「下一個字元」才能判斷 `9:` 的冒號是時刻還是標點，
而 token 是一個一個到的。一次餵整句測不出這個問題。

## 改壞了怎麼看出來

「陳先生您好，歡迎光臨。」必須切成兩句（全形逗號、句號仍要斷句）。
若連它都不斷句了，表示 split_pattern 改壞，TTS 會變成一大段沒有停頓。
"""
import argparse
import re
import sys

# ── 舊版（有 bug）─────────────────────────────────────
OLD_SPLIT = re.compile(r"([,.\!?;:，。！？；：\n])")
OLD_CLEAN = re.compile(r"[^\w一-龥\s]|[_-]")

# ── 新版 ───────────────────────────────────────────────
# 數字之間的 . : , 不是標點。全形標點不受影響（不會出現在數字中間）。
NEW_SPLIT = re.compile(r"((?<!\d)[,.:;](?!\d)|[!?，。！？；：\n])")
NEW_THOUSANDS = re.compile(r"(?<=\d),(?=\d\d\d)")     # 1,250,000 -> 1250000
NEW_CLEAN = re.compile(r"[^\w一-龥\s.:%]|_")  # . : % 要留

CASES = [
    # (輸入, 期望送進 TTS 的句子)
    ("營業時間是上午9:00到下午3:30。", ["營業時間是上午9:00到下午3:30"]),
    ("本行台幣定存年利率3.5%，美金匯率31.25。",
     ["本行台幣定存年利率3.5%", "美金匯率31.25"]),
    ("您的帳戶餘額是1,250,000元。", ["您的帳戶餘額是1250000元"]),
    ("請至3樓B區的VIP櫃檯。", ["請至3樓B區的VIP櫃檯"]),
    ("陳先生您好，歡迎光臨。", ["陳先生您好", "歡迎光臨"]),
    ("開戶約需30分鐘，請帶雙證件。", ["開戶約需30分鐘", "請帶雙證件"]),
]


def simulate(text, split_re, clean_re, thousands_re=None, defer=False):
    """複製 RosStreamHandler 的逐 token 行為。

    Args:
        defer: True = buffer 停在分隔符上時退回去等下一個 token
               （新版必要，否則 lookahead 看不到數字的後半段）
    """
    def clean(s):
        if thousands_re is not None:
            s = thousands_re.sub("", s)
        return " ".join(clean_re.sub("", s).split())

    buffer = ""
    current = ""
    out = []
    for tok in text:                       # 一次一個字元 = 逐 token 串流
        buffer += tok
        parts = split_re.split(buffer)
        buffer = parts.pop() if parts else ""
        if defer and parts and split_re.fullmatch(parts[-1]):
            buffer = parts.pop() + buffer
        for item in parts:
            if not item:
                continue
            if split_re.match(item):
                s = clean(current)
                if s:
                    out.append(s)
                current = ""
            else:
                current += item
    rest = clean(current + buffer)          # on_llm_end
    if rest:
        out.append(rest)
    return out


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument("--old", action="store_true", help="只跑舊版（重現 bug）")
    args = ap.parse_args()

    failed = 0
    for text, want in CASES:
        print(f"輸入   : {text}")
        old = simulate(text, OLD_SPLIT, OLD_CLEAN)
        print(f"  舊版 : {old}")
        if args.old:
            print()
            continue
        got = simulate(text, NEW_SPLIT, NEW_CLEAN, NEW_THOUSANDS, defer=True)
        ok = got == want
        print(f"  新版 : {got}   {'✓' if ok else '✗ 期望 ' + str(want)}")
        if not ok:
            failed += 1
        print()

    if args.old:
        return 0
    total = len(CASES)
    print(f"{'✓ 全部通過' if not failed else '✗ 失敗'} {total - failed}/{total}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
