#!/usr/bin/env python3
"""第 4 步自動化：故事線跑的時候自動偵測每一輪（每一趟帶位），結束一趟就印出來、自動登記。全部做完按一次 Enter。

★ 2026-10-27 新寫（使用者：「這些都沒有自動化的方式?因為這樣很難測跟計算」）。
  以前每一輪要回 SSH 按 Enter；而且按早按晚會把兩輪算成一輪（10/27 第 1 輪就被多按的 Enter 記成空的）。
  現在照導航 log 的「開始導航到 X」→「導航成功／失敗」切輪，跟人什麼時候按鍵無關。

用法：python3 story_watch.py --since <epoch 秒> --tag demo_eco_1030
"""
import argparse
import glob
import os
import select
import subprocess
import sys
import time

T = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, T)
import nav_result_line as N  # noqa: E402

LOG_DIR = os.environ.get("STORY_WATCH_LOG_DIR") or N.LOG_DIR


def runs_since(since):
    logs = [p for p in glob.glob(os.path.join(LOG_DIR, "nav_*.log")) if os.path.getmtime(p) >= since - 5]
    return [r for r in N.parse(sorted(logs)) if r["start"] >= since] if logs else []


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--since", type=float, default=time.time())
    ap.add_argument("--tag", default="demo")
    ap.add_argument("--poll", type=float, default=3.0)
    a = ap.parse_args()
    done, n_ok = set(), 0
    print("  （每一輪結束會自動出現在這裡；全部做完按 Enter 結束，Ctrl-C 也可以）", flush=True)
    try:
        while True:
            for r in runs_since(a.since):
                if r["ok"] is None or r["start"] in done:
                    continue
                done.add(r["start"])
                n_ok += bool(r["ok"])
                line, _ = N.describe(r)
                print(f"  第 {len(done)} 輪：{line}", flush=True)
                subprocess.run([os.path.join(T, "result.sh"), "3-1", "PASS" if r["ok"] else "WARN",
                                f"故事線第 {len(done)} 輪（{a.tag}）：{line}"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
            ready, _, _ = select.select([sys.stdin], [], [], a.poll)   # 等 Enter，順便當輪詢間隔
            if ready:
                sys.stdin.readline()
                break
    except KeyboardInterrupt:
        print()
    pending = [r for r in runs_since(a.since) if r["ok"] is None]
    for r in pending:
        print(f"  （還在進行中、沒算進去：{N.describe(r)[0]}）")
    print(f"══ 故事線（{a.tag}）：{len(done)} 輪，帶位成功 {n_ok} 輪 ══", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
