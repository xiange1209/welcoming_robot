#!/usr/bin/env python3
"""ASR 錯誤拆解 —— 分清楚是「VAD 沒觸發」還是「辨識錯」

## 為什麼需要這支

8/19 量到 CER **25.6%**，但那個數字把兩件完全不同的事混在一起：

    念了 20 句 -> VAD 只觸發 12 次 -> 出來 12 句文字，其中 3 句有錯

  「8 句完全沒反應」與「3 句認錯字」的解法**完全相反**：
    沒觸發 -> 收音、增益、VAD 門檻、雙麥克風係數
    認錯字 -> 熱詞、模型、後處理

  只看一個 CER 數字，會在錯的地方加工。這正是本專案反覆踩到的坑。

## 用法

    # 1. 準備測試句（一行一句，預設用內建的 20 句銀行語句）
    python3 ~/maprun/tools_0819/asr_breakdown.py --list > sentences.txt

    # 2. 開始量測，照著螢幕提示一句一句念
    python3 ~/maprun/tools_0819/asr_breakdown.py --out cer.csv --wait 8

每句給 --wait 秒的時間；時間到還沒收到 /user_text 就記成**未觸發**。

## 輸出

  per-sentence: 目標句 / 辨識結果 / 是否觸發 / 編輯距離 / 該句 CER
  總結：觸發率、觸發者的 CER、整體 CER（含未觸發算全錯）

★ **三個數字都要寫進報告**：只寫「整體 CER」會讓讀者以為是模型不好，
  但真正的瓶頸可能是收音。
"""
import argparse
import csv
import sys
import time

import rclpy
import re
import unicodedata

from rclpy.node import Node
from std_msgs.msg import String

SENTENCES = [
    "你好", "請問櫃檯在哪裡", "帶我去貴賓室", "我要開戶", "我要提款",
    "我要貸款", "你們幾點關門", "營業時間是什麼時候", "請帶我去服務檯",
    "我要找真人服務", "我要辦網路銀行", "匯款手續費多少", "換匯要帶什麼證件",
    "這裡可以繳費嗎", "提款機在哪裡", "請幫我叫行員", "我要查餘額",
    "請問洗手間在哪", "謝謝", "再見",
]


# ★★ 2026-08-20：正規化必須與 tools_0814/asr_cer.py 完全一致 ★★
#
#   README.md 已經寫了「整體 CER 25.6%」，那個數字是 asr_cer.py 量的，
#   而它有 norm()（NFKC + 去標點空白）且取**編輯距離最小**的那一段。
#   本工具原本兩件都沒做（不正規化、取第一則），量出來的數字必然比較差，
#   兩個數字並列在同一份報告裡會直接自相矛盾而且無法解釋。
#
#   實算差距：目標「今天的匯率是多少」，ASR 吐 ["今天的匯", "今天的匯率是多少"]
#     取第一則 -> CER 50.0%     取最好 -> CER 0.0%
#
#   所以本工具**兩個都輸出**（cer_first / cer_best），
#   讓 8/19 的 best-of 數字可以從明天的 CSV 重算出來對得起來。
PUNCT = re.compile(r"[\s，。、？！；：「」『』（）,.\?!;:\"'()]+")


def norm(s: str) -> str:
    """NFKC 正規化 + 去標點空白。與 tools_0814/asr_cer.py 的 norm() 相同。"""
    s = unicodedata.normalize("NFKC", s)
    return PUNCT.sub("", s)


def edit_distance(a: str, b: str) -> int:
    """字元級編輯距離（Levenshtein）。中文 CER 的分母是字元數，所以逐字算。"""
    if not a:
        return len(b)
    if not b:
        return len(a)
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1]


class Collector(Node):
    """收集一句話期間的**所有**辨識段落。

    ★★ 2026-08-20 兩個修正 ★★

    (1) **收全段，不是只收第一則。** 原本 `if self.got is None` 只留第一則，
        而註解自己寫著「後面那則屬於下一次比對」—— 那句話本身就是 bug 的宣告：
        它明確承認殘段會被算到下一句頭上，程式卻沒有做任何事阻止。
        ASR 一次朗讀常吐 2 段（tools_0814/asr_cer.py 的註解就是為此而寫，
        8/19 因此量出 CER 144%）。

    (2) **訂閱 /partial_text。** 只看 /user_text 的話，下面三種丟棄
        與「麥克風根本沒收到聲音」在 CSV 裡長得一模一樣，全部記成未觸發：
          - _postprocess 判定太短／整句同字
          - _is_echo 判定是回音（預設比對窗 25 秒，量測時 LLM 開著就會中）
        有 partial 但沒有 final = VAD **有**觸發，是被文字層過濾掉的；
        partial 也沒有 = 真的沒觸發。
        少了這一欄，判讀規則「觸發率低 -> 瓶頸在收音」會把人導去調雙麥克風係數，
        而真因在回音過濾 —— 那正是這支工具想避免的錯。
    """

    def __init__(self):
        super().__init__("asr_breakdown")
        self.segs = []          # 這一句期間收到的所有 /user_text
        self.partials = 0       # 這一句期間收到幾則 /partial_text
        self.create_subscription(String, "/user_text", self._cb, 10)
        self.create_subscription(String, "/partial_text", self._partial_cb, 10)

    def _cb(self, msg: String):
        self.segs.append(msg.data)

    def _partial_cb(self, msg: String):
        if msg.data and msg.data.strip():
            self.partials += 1

    def reset(self):
        self.segs = []
        self.partials = 0

    def drain(self, secs=0.6):
        """把上一句留在 DDS 佇列裡的殘段排掉。

        ★ `self.segs = []` 只清 Python 變數，**佇列（depth 10）沒有清**。
          不排空的話下一句一開始就會被立刻餵進殘段，整份 CSV 從此錯位，
          而且表面上完全正常（每一列都有 got）。
          三句錯位一次的實算：正常 0/15 = 0.0% -> 錯位後 14/15 = 93.3%。
        """
        t = time.time()
        while time.time() - t < secs:
            rclpy.spin_once(self, timeout_sec=0.05)
        dropped = list(self.segs)
        self.reset()
        return dropped


def main():
    ap = argparse.ArgumentParser(description="ASR 錯誤拆解（未觸發 vs 認錯字）")
    # ★ 2026-08-20：預設檔名帶時間戳。原本固定 "asr_cer.csv" + open(..., "w")，
    #   三個人依序量測時只改 --speaker 不改 --out，第二個人會直接蓋掉第一個人
    #   —— 與 run_nav_cc.sh 記錄的「8/17 覆蓋等於銷毀證據」是同一個坑。
    ap.add_argument("--out", default="")
    ap.add_argument("--quiet", type=float, default=1.5,
                    help="收到第一則之後再等幾秒，把同一句被切成的其他段落一起收進來")
    ap.add_argument("--wait", type=float, default=8.0, help="每句等幾秒")
    ap.add_argument("--sentences", help="自訂測試句檔案（一行一句）")
    ap.add_argument("--list", action="store_true", help="印出內建測試句就結束")
    ap.add_argument("--speaker", default="", help="受試者標記，方便分人統計")
    args = ap.parse_args()

    if args.list:
        print("\n".join(SENTENCES))
        return 0

    if not args.out:
        _tag = (args.speaker or "x").replace("/", "_")
        args.out = "asr_cer_%s_%s.csv" % (_tag, time.strftime("%Y%m%d_%H%M%S"))

    sents = SENTENCES
    if args.sentences:
        with open(args.sentences, encoding="utf-8") as f:
            sents = [ln.strip() for ln in f if ln.strip()]

    rclpy.init()
    node = Collector()
    rows = []
    try:
        for i, target in enumerate(sents, 1):
            # ★ 先排空上一句的殘段（見 Collector.drain 的說明）
            dropped = node.drain(0.6)
            if dropped:
                print(f"    （排掉上一句的殘段 {len(dropped)} 則：{dropped}）", flush=True)

            print(f"[{i}/{len(sents)}] 請念：**{target}**", flush=True)
            ref = norm(target)
            deadline = time.time() + args.wait
            # 收到第一則之後再多等 quiet 秒，把同一句被切成的其他段落一起收進來
            first_at = None
            while time.time() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if node.segs and first_at is None:
                    first_at = time.time()
                if first_at and time.time() - first_at > args.quiet:
                    break

            segs = list(node.segs)
            partials = node.partials
            node.reset()
            best = ""

            if segs:
                # ★ 兩種都算：first 是保守值，best 與 8/19 的 asr_cer.py 同口徑
                d_first = edit_distance(ref, norm(segs[0]))
                scored = sorted((edit_distance(ref, norm(g)), g) for g in segs)
                d_best, best = scored[0]
                cer_first = d_first / max(1, len(ref))
                cer_best = d_best / max(1, len(ref))
                triggered, filtered = 1, 0
                print(f"    -> {len(segs)} 段 {segs}")
                print(f"       first CER {cer_first:.1%}（距離 {d_first}）"
                      f"　best「{best}」CER {cer_best:.1%}（距離 {d_best}）")
            else:
                d_first = d_best = len(ref)
                cer_first = cer_best = 1.0
                triggered = 0
                # ★ 有 partial 但沒有 final = VAD 有觸發，是被文字層過濾掉的
                filtered = 1 if partials > 0 else 0
                if filtered:
                    print(f"    -> ★ 有 partial（{partials} 則）但沒有最終文字"
                          f" —— VAD 有觸發，被後處理或回音過濾丟掉")
                else:
                    print("    -> ★ 完全沒有反應（VAD 未觸發，瓶頸在收音端）")

            rows.append({
                "idx": i, "speaker": args.speaker, "target": target,
                "target_norm": ref, "n_chars": len(ref),
                "n_seg": len(segs), "segments": " | ".join(segs),
                "got_first": segs[0] if segs else "",
                "got_best": best,
                "triggered": triggered, "partials": partials,
                "filtered_not_silent": filtered,
                "dist_first": d_first, "dist_best": d_best,
                "cer_first": round(cer_first, 4), "cer_best": round(cer_best, 4),
                "wall_time": f"{time.time():.6f}",
            })
    except KeyboardInterrupt:
        print("\n中斷")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not rows:
        return 1
    with open(args.out, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    trig = [r for r in rows if r["triggered"]]
    filt = [r for r in rows if r["filtered_not_silent"]]
    tot_chars = sum(r["n_chars"] for r in rows)
    trig_chars = sum(r["n_chars"] for r in trig)

    def _pct(rows_, key):
        c = sum(r["n_chars"] for r in rows_)
        return (sum(r[key] for r in rows_) / c) if c else 0.0

    print()
    print("=== 結果（%d 句 / %d 字） -> %s ===" % (len(rows), tot_chars, args.out))
    print("  ① 觸發率        %d/%d = %.1f%%   <- 低的話問題在**收音／VAD**，不是模型"
          % (len(trig), len(rows), 100.0 * len(trig) / len(rows)))
    if filt:
        print("     其中 %d 句**有 partial 但沒有最終文字** —— VAD 有觸發，" % len(filt))
        print("     是被後處理或回音過濾丟掉的。★ 這幾句不該算成『收音不好』，")
        print("     要回頭看 speech_recognizer 的 _postprocess / _is_echo。")
    if trig_chars:
        print("  ② 觸發者的 CER  first %.1f%%　best %.1f%%   <- **辨識模型**的成績"
              % (100 * _pct(trig, "dist_first"), 100 * _pct(trig, "dist_best")))
    print("  ③ 整體 CER      first %.1f%%　best %.1f%%   <- 未觸發算全錯，使用者實際感受"
          % (100 * _pct(rows, "dist_first"), 100 * _pct(rows, "dist_best")))
    print()
    print("★ best 欄與 8/19 的 tools_0814/asr_cer.py **同口徑**（正規化 + 取最佳段），")
    print("  README 已寫的 25.6% 要跟 best 比，不要跟 first 比。")
    print("★ 三個數字都要寫進報告。只寫 ③ 會讓讀者以為是模型不好，")
    print("  但如果 ① 很低，真正的瓶頸是收音。")
    print()
    print("=== 可直接貼進報告的表格 ===")
    print()
    print("| 受試者 | 句數 | 字數 | 觸發率 | 觸發者 CER | 整體 CER |")
    print("|---|---|---|---|---|---|")
    print("| %s | %d | %d | %.1f%% | %.1f%% | %.1f%% |" % (
        args.speaker or "—", len(rows), tot_chars,
        100.0 * len(trig) / len(rows),
        100 * _pct(trig, "dist_best"), 100 * _pct(rows, "dist_best")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
