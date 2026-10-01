#!/usr/bin/env python3
"""scan_self_echo.py 的分析邏輯測試（不需要 ROS）：python test_scan_self_echo.py

合成情境照 9/29 的假設：兩側鋁擠條 0.165 m 在八成的幀出現、正前方 0.30 m 偶發（兩成的幀）。
預期：兩側被判成自身回波扇區；正前方比例太低不會進遮罩建議，但**一定要**出現在
「會讓 path_teach 印 0 cm／硬停」的清單裡 —— 9/29 前方那種讀數就是偶發的。
"""
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scan_self_echo as s  # noqa: E402

N_BEAMS = 390                       # 與車上 N10、Gazebo 模擬相同
AM = -math.pi
INC = 2 * math.pi / N_BEAMS
fails = 0


def check(cond, msg):
    global fails
    print(("  ✓ " if cond else "  ✗ ") + msg)
    if not cond:
        fails += 1


def frame(bar_on, front_on, bg=3.0):
    rs = []
    for i in range(N_BEAMS):
        d = math.degrees(AM + i * INC) % 360.0
        r = bg
        if bar_on and (86 <= d < 116 or 251 <= d < 273):
            r = 0.165
        if front_on and (d >= 358 or d < 2):
            r = 0.30
        rs.append(r)
    return (AM, INC, 0.15, 12.0, rs)


print("幾何（與 path_teach._body_margin 同一套）")
check(s.body_distance(0.165, math.radians(90)) == 0.0, "側面 0.165 m（鋁擠條）落在車身內 → 0 cm")
check(s.body_distance(0.165, math.radians(84)) == 0.0, "84 度 0.165 m 也在車身內（9/29 −29 cm 那筆的方向）")
check(s.body_distance(0.30, 0.0) == 0.0, "正前方 0.30 m 在車身內（前保險桿在雷達前 0.311 m）")
check(abs(s.body_distance(1.0, 0.0) - (1.0 - 0.311)) < 1e-9, "正前方 1.0 m → 離車身 0.689 m")
check(abs(s.body_distance(0.40, math.radians(90)) - (0.40 - 0.185)) < 1e-9, "側面 0.40 m → 離車身 0.215 m")

print("扇區偵測（含跨 0 度）")
v = [0.0] * 360
for d in list(range(84, 118)) + list(range(250, 275)) + [355, 356, 357, 358, 359, 0, 1, 2, 3, 4]:
    v[d] = 0.9
runs = s.runs_above(v, 0.5)
check(runs == [(84, 117), (250, 274), (355, 4)] or sorted(runs) == [(84, 117), (250, 274), (355, 4)],
      f"找到三段且跨 0 那段沒被切開：{runs}")
check(s.runs_above([0.0] * 360, 0.5) == [], "全部沒命中 → 空")
check(s.runs_above([1.0] * 360, 0.5) == [(0, 359)], "全部命中 → 一整圈")

print("遮罩參數（scan_filter 不支援跨 0 → 要拆段）")
check(s.to_mask_param([(84, 117)], 3) == [81.0, 121.0], "84~117 外擴 3 → [81, 121]")
check(s.to_mask_param([(355, 4)], 3) == [352.0, 360.0, 0.0, 8.0], "355~4 外擴 3 → 拆成 [352,360] ＋ [0,8]")
check(s.to_mask_param([(1, 2)], 3) == [358.0, 360.0, 0.0, 6.0], "1~2 外擴後跨 0 也要拆")
check(s.covered(100, s.CURRENT_MASK) and not s.covered(0, s.CURRENT_MASK), "現行預設蓋住 100 度、沒蓋住 0 度")

print("端到端：100 幀合成掃描")
frames = [frame(bar_on=(k % 10 < 8), front_on=(k % 10 < 2)) for k in range(100)]
st = s.EchoStats(near_m=0.5)
for fr in frames:
    st.add(*fr)
text, runs, mask = s.summarize(st, 0.5, 3.0, "/scan", "合成")
nr, br = st.ratio(st.near), st.ratio(st.body)
check(st.frames == 100, "收 100 幀")
check(abs(nr[100] - 0.8) < 1e-9 and abs(nr[0] - 0.2) < 1e-9, f"near 比例：側面 {nr[100]:.2f}、正前 {nr[0]:.2f}")
check(any(a <= 100 <= b for a, b in runs) and any(a <= 260 <= b for a, b in runs), f"兩側判成自身回波：{runs}")
check(not s.covered(0, mask), "正前方只佔兩成的幀 → 不進遮罩建議")
check(br[0] > 0 and br[359] > 0, "正前方仍列入「會讓 path_teach 硬停」")
check("會讓 path_teach 印「0 cm」" in text and "0~1°" in text.replace("359~360°＋0~1°", "0~1°"), "摘要有列出正前方")
check("沒蓋到的自身回波角度" not in text, "兩側都在現行預設 [84,118,249,275] 內")

print("原始幀存檔 → 離線重算要得到同樣結果")
with tempfile.TemporaryDirectory() as td:
    p = os.path.join(td, "raw.csv")
    s.write_raw(p, frames)
    st2 = s.EchoStats(near_m=0.5)
    for fr in s.read_raw(p):
        st2.add(*fr)
    check(st2.near == st.near and st2.body == st.body, "CSV 來回一致")
    rc = s.main(["--from-csv", p, "--out", os.path.join(td, "out")])
    check(rc == 0 and os.path.exists(os.path.join(td, "out", "summary.txt"))
          and os.path.exists(os.path.join(td, "out", "scan_self_echo.tsv")), "--from-csv 產出 summary.txt 與 tsv")

print("量程外與無效值")
st3 = s.EchoStats()
st3.add(AM, INC, 0.15, 12.0, [float("inf")] * 10 + [float("nan")] * 10 + [0.10] * 10 + [3.0] * (N_BEAMS - 30))
check(sum(st3.near) == 0 and sum(st3.body) == 0, "inf／nan／低於 range_min（0.10 < 0.15）都不算回波")

print(f"\n{'全部通過' if not fails else f'{fails} 項失敗'}")
sys.exit(1 if fails else 0)
