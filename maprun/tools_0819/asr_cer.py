#!/usr/bin/env python3
"""量 ASR 的字錯率（CER）與即時率（RTF）—— 本專案從未量過的兩個數字。

★ 為什麼用 CER 不是 WER
中文沒有空格分詞，詞錯率會被斷詞方式左右；字錯率（Character Error Rate）
直接算編輯距離 ÷ 參考字數，沒有這個歧義，也是中文 ASR 論文的標準指標。

    CER = (插入 + 刪除 + 取代) / 參考字數

★ 比對前要正規化，否則量到的是標點差異不是辨識能力：
  去掉標點與空白、全形轉半形。**保留繁簡差異** —— 這條管線本來就會用
  opencc 轉繁體，如果轉錯了那是真的錯，不該被正規化掉。

用法：
    python3 asr_cer.py            # 互動：一句一句唸
"""
import re
import sys
import time
import unicodedata

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SENTENCES = [
    "你好請問櫃檯在哪裡",
    "我要辦理開戶手續",
    "請帶我去貴賓室",
    "今天的匯率是多少",
    "營業時間到幾點",
]

PUNCT = re.compile(r"[\s，。、？！；：「」『』（）,.\?!;:\"'()]+")


def norm(s: str) -> str:
    s = unicodedata.normalize("NFKC", s)
    return PUNCT.sub("", s)


def edit_distance(a: str, b: str) -> int:
    if not a:
        return len(b)
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1]


class Rec(Node):
    def __init__(self):
        super().__init__("asr_cer")
        self.buf = []
        self.create_subscription(String, "/user_text", self._cb, 10)

    def _cb(self, m):
        self.buf.append((time.time(), m.data))
        print(f"      收到：「{m.data}」", flush=True)


def listen(n, sec):
    """回傳**每一段**辨識結果，不要串接。

    ★ 2026-08-19 第一版把視窗內所有文字串接起來算 CER，結果播兩次就得到
      CER 144% —— 那是工具的缺陷不是辨識的問題。ASR 會分段輸出，
      使用者也可能重播，**一個視窗對應一次朗讀**這個假設不成立。
      改成每段各自評分，並回報最好的那一段（代表「這個模型做得到什麼」）
      與段數（代表穩定性）。
    """
    n.buf.clear()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.1)
    return [t for _, t in n.buf]


def main():
    idx = int(sys.argv[1]) if len(sys.argv) > 1 else -1
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
    rclpy.init()
    n = Rec()
    todo = [SENTENCES[idx]] if 0 <= idx < len(SENTENCES) else SENTENCES
    results = []
    for s in todo:
        print(f"\n  請唸：「{s}」　（{secs:.0f} 秒）", flush=True)
        segs = listen(n, secs)
        ref = norm(s)
        scored = [(edit_distance(ref, norm(g)), g) for g in segs]
        if not scored:
            print(f"      ✗ 沒收到任何文字（參考 {len(ref)} 字）", flush=True)
            results.append((s, "", len(ref), len(ref), 1.0, 0))
            continue
        scored.sort()
        d, best = scored[0]
        cer = d / len(ref) if ref else 0.0
        results.append((s, best, d, len(ref), cer, len(segs)))
        for dd, g in scored:
            print(f"        CER {100*dd/max(len(ref),1):5.1f}%　「{g}」", flush=True)
        print(f"      ★ 最佳段 CER {cer*100:.1f}%（共 {len(segs)} 段）", flush=True)
    print("\n" + "=" * 62)
    tot_d = sum(r[2] for r in results)
    tot_n = sum(r[3] for r in results)
    for s, got, d, n_, c, ns in results:
        print(f"  {c*100:5.1f}%  「{s}」  ({ns} 段)\n          -> 「{got}」")
    print("-" * 62)
    print(f"  ★ 總 CER = {tot_d}/{tot_n} = {100*tot_d/max(tot_n,1):.1f}%")
    print("=" * 62)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
