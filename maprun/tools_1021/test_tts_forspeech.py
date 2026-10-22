# -*- coding: utf-8 -*-
"""2026-10-22：朗讀前的數字整理（HMI 前端 lib/speechText.ts 的 forSpeech）的行為測試。

背景：瀏覽器 speechSynthesis 把阿拉伯數字當「數量」唸，「165」會唸成「一百六十五」。
10/21 做了「關鍵字＋3~10 位數字」；10/22 車上實測連字號號碼（0800-123456、02-2345-6789）、
關鍵字和數字中間隔字（電話是0912…）、全形數字都沒處理到，所以改寫成 speechText.ts。

測什麼（直接用 Node 執行真的 TypeScript 原始檔，不是照抄邏輯）
  ✓ 該逐字唸的：165 反詐騙專線（各種位置）、0800-123456、02-2345-6789、(02) 2345-6789、
    電話是0912345678、全形１６５、1-800-555-1234、沒有關鍵字但長得像電話的 09 開頭 10 碼／0 開頭加連字號
  ✓ 該改成「到」的（iOS 會把「-」唸成「負」，車上 agent 10/22 實測）：時間範圍 09:00-17:00、
    10-20 分鐘、3-5 個工作天、第 1-20 號；日期 2026-10-22 → 2026年10月22日
  ✓ 銀行用語：金額範圍 1,000-12,000 元／1000-2000 元（千分位與小數、百分號都認得）→ 唸「到」；帳號 012-345-678901 逐字唸
  ✓ 不該動的：下午3點半、號碼牌128號、撥 3 次、專線 2 條、氣溫 -5 度（負號）、1,000 元
  ✓ 已經處理過的句子再處理一次不變（冪等）
  （車上 agent 的 12 個案例全部收進來；原本的 34 個中，時間範圍與日期兩項依新規則改了期望值）
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
    # ── 車上 agent 10/22 的案例（hmi_ipad_20261022/tests/forspeech_test.mjs）──
    ("客服專線 0800-123456", "客服專線 零八零零，一二三四五六"),
    ("電話 02-2345-6789", "電話 零二，二三四五，六七八九"),
    ("營業時間 09:00-17:00", "營業時間 09:00到17:00"),
    ("營業時間 9:00～15:30", "營業時間 9:00到15:30"),
    ("今天是 2026-10-22", "今天是 2026年10月22日"),
    ("大約 10-20 分鐘", "大約 10到20 分鐘"),
    ("3-5 個工作天", "3到5 個工作天"),
    ("請撥打 1-800-555-1234", "請撥打 一，八零零，五五五，一二三四"),
    ("第 1-20 號", "第 1到20 號"),
    # ── 筆電端補的 ──
    ("更新日期 2026-10-22", "更新日期 2026年10月22日"),
    ("2026/10/22 開會", "2026年10月22日 開會"),
    ("週一到週五 9-17 點", "週一到週五 9到17 點"),
    ("0800-123456", "零八零零，一二三四五六"),
    # ── 銀行常見用語（審查員與筆電端 10/22 探測發現：車上 agent 原本的「≥7 位數或 0 開頭就是電話」會把金額範圍唸壞）──
    ("單筆限額 1,000-2,000 元", "單筆限額 1,000到2,000 元"),
    ("單筆限額 1,000-12,000 元", "單筆限額 1,000到12,000 元"),
    ("手續費 10,000-20,000 元", "手續費 10,000到20,000 元"),
    ("額度 100,000-200,000 元", "額度 100,000到200,000 元"),
    ("額度 1000-2000 元", "額度 1000到2000 元"),
    ("年費 1200-3000 元", "年費 1200到3000 元"),
    ("額度 500-1000 元", "額度 500到1000 元"),
    ("年利率 0.5%-1.5%", "年利率 0.5%到1.5%"),
    ("NT$10,000-50,000", "NT$10,000到50,000"),
    ("帳號 012-345-678901", "帳號 零一二，三四五，六七八九零一"),
    ("分行代號 004-1234", "分行代號 零零四，一二三四"),
    ("客服專線 0800-123-456", "客服專線 零八零零，一二三，四五六"),
]
KEEP = [
    "營業時間到下午3點半",
    "我的號碼牌是128號",
    "請撥 3 次",
    "專線 2 條",
    "專線服務時間 24 小時",
    "氣溫 -5 度",
    "額度 1,000 元",
    "上限 5,000,000 元",
    "約 12.5 公里",
    "滿 18 歲以上",
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
