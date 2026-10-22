# -*- coding: utf-8 -*-
"""2026-10-22：朗讀前的數字整理（HMI 前端 lib/speechText.ts 的 forSpeech）的行為測試。

背景：瀏覽器 speechSynthesis 把阿拉伯數字當「數量」唸，「165」會唸成「一百六十五」。
10/21 做了「關鍵字＋3~10 位數字」；10/22 車上實測連字號號碼（0800-123456、02-2345-6789）、
關鍵字和數字中間隔字（電話是0912…）、全形數字都沒處理到，所以改寫成 speechText.ts。

測什麼（直接用 Node 執行真的 TypeScript 原始檔，不是照抄邏輯）
  ✓ 該逐字唸的：165 反詐騙專線（各種位置）、0800-123456、02-2345-6789、(02) 2345-6789、
    電話是0912345678、全形１６５、沒有關鍵字但長得像電話的 09 開頭 10 碼／0 開頭加連字號
  ✓ 不該動的：下午3點半、號碼牌128號、撥 3 次、專線 2 條、09:00-17:00、日期 2026-10-22
  ✓ 已經是中文數字的句子再處理一次不變（冪等）
不測：iPad 實際唸出來的聲音（要在裝置上聽）。

用法：python -X utf8 maprun/tools_1021/test_tts_forspeech.py   （需要 Node 22.6 以上；沒有就跳過）
"""
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

CAR = Path(os.environ.get("SMARTNAV_REPO") or Path(__file__).resolve().parents[2])
SRC = CAR / "src/smartnav_hmi/frontend/src/lib/speechText.ts"
results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


node = shutil.which("node")
if not node:
    print("  （沒有 node，跳過）")
    sys.exit(0)
ver = subprocess.run([node, "-v"], capture_output=True, text=True).stdout.strip()
m = re.match(r"v(\d+)\.(\d+)", ver)
if not m or (int(m[1]), int(m[2])) < (22, 6):
    print(f"  （node {ver} 不支援 --experimental-strip-types，需要 22.6 以上，跳過）")
    sys.exit(0)
check(SRC.is_file(), f"原始檔存在：{SRC.name}")

CHANGE = [
    ("請撥打165反詐騙專線", "請撥打一六五反詐騙專線"),
    ("遇到可疑情況請找櫃檯行員或撥打 165 反詐騙專線。", "遇到可疑情況請找櫃檯行員或撥打 一六五 反詐騙專線。"),
    ("反詐騙專線165", "反詐騙專線一六五"),
    ("請撥打165", "請撥打一六五"),
    ("您可以撥打客服熱線 0800-123456 聯絡我們", "您可以撥打客服熱線 零八零零，一二三四五六 聯絡我們"),
    ("請撥 02-2345-6789", "請撥 零二，二三四五，六七八九"),
    ("電話是0912345678", "電話是零九一二三四五六七八"),
    ("請撥打１６５反詐騙專線", "請撥打一六五反詐騙專線"),
    ("客服專線 (02) 2345-6789", "客服專線 零二，二三四五，六七八九"),
    ("請撥打165，或洽詢02-2345-6789", "請撥打一六五，或洽詢零二，二三四五，六七八九"),
    ("我的手機是0912345678", "我的手機是零九一二三四五六七八"),
    ("165 反詐騙專線", "一六五 反詐騙專線"),
]
KEEP = [
    "營業時間到下午3點半",
    "我的號碼牌是128號",
    "請撥 3 次",
    "專線 2 條",
    "專線服務時間 24 小時",
    "營業時間 09:00-17:00",
    "更新日期 2026-10-22",
    "週一到週五上午九點到下午三點半營業。",
    "",
]
inputs = [a for a, _ in CHANGE] + KEEP + [b for _, b in CHANGE]

JS = """
import { forSpeech } from %s;
let s = ""; for await (const c of process.stdin) s += c;
console.log(JSON.stringify(JSON.parse(s).map((x) => forSpeech(x))));
""" % json.dumps(SRC.resolve().as_uri())
r = subprocess.run(
    [node, "--experimental-strip-types", "--no-warnings", "--input-type=module", "-e", JS],
    input=json.dumps(inputs), capture_output=True, text=True, encoding="utf-8",
)
if r.returncode != 0:
    check(False, f"node 執行失敗：{r.stderr.strip()[:300]}")
    print(f"\n{sum(results)}/{len(results)} 通過")
    sys.exit(1)
out = json.loads(r.stdout)

print("── 1. 該逐字唸的 ──")
for (src, want), got in zip(CHANGE, out):
    check(got == want, f"{src!r} → {got!r}" + ("" if got == want else f"（應為 {want!r}）"))

print("── 2. 不該動的 ──")
for src, got in zip(KEEP, out[len(CHANGE):]):
    check(got == src, f"{src!r} 不變" + ("" if got == src else f"（變成 {got!r}）"))

print("── 3. 冪等：處理過的句子再處理一次不變 ──")
again = out[len(CHANGE) + len(KEEP):]
for (_, want), got in zip(CHANGE, again):
    check(got == want, f"{want!r} 再處理不變" + ("" if got == want else f"（變成 {got!r}）"))

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
