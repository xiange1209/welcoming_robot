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
from rclpy.node import Node
from std_msgs.msg import String

SENTENCES = [
    "你好", "請問櫃檯在哪裡", "帶我去貴賓室", "我要開戶", "我要提款",
    "我要貸款", "你們幾點關門", "營業時間是什麼時候", "請帶我去服務檯",
    "我要找真人服務", "我要辦網路銀行", "匯款手續費多少", "換匯要帶什麼證件",
    "這裡可以繳費嗎", "提款機在哪裡", "請幫我叫行員", "我要查餘額",
    "請問洗手間在哪", "謝謝", "再見",
]


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
    def __init__(self):
        super().__init__("asr_breakdown")
        self.got = None
        self.create_subscription(String, "/user_text", self._cb, 10)

    def _cb(self, msg: String):
        # 只收第一則：一句話可能被切成兩則，後面那則屬於下一次比對
        if self.got is None:
            self.got = msg.data


def main():
    ap = argparse.ArgumentParser(description="ASR 錯誤拆解（未觸發 vs 認錯字）")
    ap.add_argument("--out", default="asr_cer.csv")
    ap.add_argument("--wait", type=float, default=8.0, help="每句等幾秒")
    ap.add_argument("--sentences", help="自訂測試句檔案（一行一句）")
    ap.add_argument("--list", action="store_true", help="印出內建測試句就結束")
    ap.add_argument("--speaker", default="", help="受試者標記，方便分人統計")
    args = ap.parse_args()

    if args.list:
        print("\n".join(SENTENCES))
        return 0

    sents = SENTENCES
    if args.sentences:
        with open(args.sentences, encoding="utf-8") as f:
            sents = [ln.strip() for ln in f if ln.strip()]

    rclpy.init()
    node = Collector()
    rows = []
    try:
        for i, target in enumerate(sents, 1):
            node.got = None
            print(f"\n[{i}/{len(sents)}] 請念：**{target}**", flush=True)
            deadline = time.time() + args.wait
            while time.time() < deadline and node.got is None:
                rclpy.spin_once(node, timeout_sec=0.1)
            got = node.got
            triggered = got is not None
            if triggered:
                dist = edit_distance(target, got)
                cer = dist / max(1, len(target))
                print(f"    -> 「{got}」  編輯距離 {dist}  CER {cer:.1%}")
            else:
                dist, cer = len(target), 1.0
                print("    -> ★ 沒有收到任何文字（VAD 未觸發）")
            rows.append({"idx": i, "speaker": args.speaker, "target": target,
                         "got": got or "", "triggered": int(triggered),
                         "edit_distance": dist, "cer": round(cer, 4)})
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
    tot_chars = sum(len(r["target"]) for r in rows)
    tot_dist = sum(r["edit_distance"] for r in rows)
    trig_chars = sum(len(r["target"]) for r in trig)
    trig_dist = sum(r["edit_distance"] for r in trig)
    print()
    print(f"=== 結果（{len(rows)} 句） -> {args.out} ===")
    print(f"  ① 觸發率        {len(trig)}/{len(rows)} = {len(trig) / len(rows):.1%}"
          "   <- 低的話問題在**收音／VAD**，不是模型")
    if trig_chars:
        print(f"  ② 觸發者的 CER  {trig_dist / trig_chars:.1%}"
              "   <- 這才是**辨識模型**的成績")
    print(f"  ③ 整體 CER      {tot_dist / tot_chars:.1%}"
          "   <- 未觸發算全錯，是使用者實際感受")
    print()
    print("★ 三個數字都要寫進報告。只寫 ③ 會讓讀者以為是模型不好，")
    print("  但如果 ① 很低，真正的瓶頸是收音。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
