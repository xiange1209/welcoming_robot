#!/usr/bin/env python3
"""直角彎可通過性計算 —— 去實驗室選展示路線時用這支。

    python3 corner_check_cc.py              # 印這台車的規格表
    python3 corner_check_cc.py 1.35         # 問「淨寬 1.35 m 的彎過得去嗎」
    python3 corner_check_cc.py 1.35 0.20    # 同上，但循跡誤差用 0.20 m

## 為什麼需要這支

2026-08-27 之前一直以為「門口 0.967 m 太窄所以過不去」。
算出來才發現**幾何上過得去**：最壞情況（左轉 0.944 m）只需要 0.843 m，
還有 12.4 cm 餘裕。真正吃掉餘裕的是**循跡誤差** —— 家裡走廊實測貼牆偏 32.8 cm，
是可用餘裕的 2.6 倍。

所以選路線的判準是 **幾何最小寬度 + 實測循跡誤差**，不是只看幾何。

## 幾何模型

L 形直角彎、兩臂同寬 W、車走各臂中線、單一圓弧、零誤差。
車體相對後軸（base_footprint）：前懸 FRONT、後懸 REAR、半寬 HALFW。

轉彎中心 C = (W/2 + R, W/2 + R)，兩道約束：

  1. **外牆**：外前角的掃掠半徑要塞得進外側
         R_out = hypot(FRONT, R + HALFW) <= W/2 + R
  2. **內角**：內角 (W, W) 必須落在掃掠內圓裡（沒被掃到）
         sqrt(2) * (R - W/2) <= R_in = R - HALFW

★ 實測四種半徑下**都是約束 2 勝出** —— 卡住的是內角不是外牆。
★ 掃掠帶寬（R_out - R_in）幾乎不隨半徑變（0.484~0.498 m），
  因為它主要由車體尺寸決定；半徑影響的是「中心要擺多遠」。
  所以校準舵機把左轉 0.944 拉回 0.75，省下的是 **11.3 cm 的走廊需求**。
"""
import math
import sys

# nav2_senior_akm_cc.yaml: footprint [[-0.09,-0.185],[-0.09,0.185],[0.40,0.185],[0.40,-0.185]]
FRONT, REAR, HALFW = 0.40, 0.09, 0.185      # 自後軸；車長 0.49、車寬 0.37
PAD = 0.02                                   # footprint_padding

# 迴轉半徑（後軸中心）。★ 韌體設計是對稱 0.750，實車量到的**會漂**：
#   8/17  左 0.944 / 右 0.751（右比左緊 27%）
#   8/29  左 1.030 / 右 1.183（方向反過來，差 13%）—— 電壓/舵機磨耗
# 引用前先重量：python3 ~/maprun/steer_asym_check_cc.py（兩分鐘）
# ★ 2026-08-30 修正（CC-01/CC-11）：舊表把「最壞」標在 1.183 上，那是錯的。
#   w_inner = 2R(1-1/sqrt2) + sqrt2*hw 對 R **單調遞增** —— 半徑愈大、需要的
#   走廊愈寬。所以最壞情況是控制器箝制的最大值 1.20，不是實測的 1.183。
#   （直覺：掃掠帶寬幾乎不變，但轉彎中心得擺得更遠，內角就切得更進來。）
RADII = [
    ("左轉（8/29 實測 1.030）", 1.030),
    ("左轉箝制（path_teach 1.05）", 1.05),
    ("右轉（8/29 實測 1.183）", 1.183),
    ("右轉箝制（path_teach 1.20，最壞）", 1.20),
]
# 8/29 三趟合計、良好定位、脫困前 2467 筆（走廊段）的 95 百分位。
# 舊值 0.328 是單趟貼牆事件的最大偏移，高估四倍 —— 已被當天資料取代。
DEFAULT_TRACK_ERR = 0.175


def geometry(R, pad=PAD):
    """回傳 (最小走廊淨寬, 掃掠內徑, 掃掠外徑, 哪道約束勝出)。"""
    hw, fr = HALFW + pad, FRONT + pad
    r_in = R - hw
    r_out = math.hypot(fr, R + hw)
    w_outer = 2.0 * (r_out - R)                       # 約束 1
    w_inner = 2.0 * (R - r_in / math.sqrt(2.0))       # 約束 2
    if w_inner >= w_outer:
        return w_inner, r_in, r_out, "內角"
    return w_outer, r_in, r_out, "外牆"


def spec_table():
    print("★ 這台車轉 90° 直角彎所需的走廊淨寬")
    print("  （兩臂同寬、走中線、單一圓弧、**零誤差**的理想值）\n")
    print(f"  {'情況':<24}{'掃掠內徑':>9}{'掃掠外徑':>9}{'帶寬':>8}{'最小淨寬':>10}  卡在")
    for name, R in RADII:
        w, ri, ro, who = geometry(R)
        print(f"  {name:<24}{ri:>9.3f}{ro:>9.3f}{ro - ri:>8.3f}{w:>10.3f}  {who}")
    print("\n  ★ 四種半徑都是**內角**約束勝出，外牆從來不是瓶頸。")


def check(width, track_err=DEFAULT_TRACK_ERR):
    print(f"★ 檢查：走廊淨寬 {width:.3f} m，循跡誤差取 {track_err:.3f} m\n")
    worst = max(geometry(R)[0] for _, R in RADII)
    print(f"  幾何最小需求（最壞情況）  {worst:.3f} m")
    print(f"  ★ 選路門檻：≥ {worst + track_err:.3f} m 可過（95%）、"
          f"≥ {worst + 2 * track_err:.3f} m 保證過")
    print(f"  加上循跡誤差              {worst + track_err:.3f} m")
    print(f"  實際淨寬                  {width:.3f} m")
    margin_geo = width - worst
    margin_all = width - worst - track_err
    print()
    if margin_all >= 0:
        print(f"  ✅ 可通過，扣掉誤差後仍餘 {margin_all:+.3f} m")
    elif margin_geo >= 0:
        print(f"  ⚠ **幾何過得去（餘 {margin_geo:+.3f} m）但循跡誤差吃光餘裕**")
        print(f"     缺 {-margin_all:.3f} m。")
        # ★ 2026-08-30（CC-11）：舊版這裡寫死「家裡門口 0.967 就是這一型、只差
        #   5.1 cm」——那是用 8/17 舊半徑（左 0.944 -> 幾何 0.843）算的。
        #   換成 8/29 半徑之後 0.967 落在下面的 ❌ 分支（幾何就過不去），
        #   這行敘述已經不適用，留著會讓人以為只差一點點。
        print(f"     → 對策（8/29 對照實測後**反轉**）：優先讓**規劃器現場算**——")
        print(f"       它天生受最小迴轉半徑約束；教導路徑錄製端不檢查可行性，")
        print(f"       「人開過」不代表車開得出來（實測有一段要求 R=0.236 m，是用推的）。")
    else:
        print(f"  ❌ 幾何上就過不去，缺 {-margin_geo:.3f} m。換路線。")
    print()
    print("  ★ 提醒：這個模型假設**標準 90° 彎、兩臂同寬且走中線**，是保守側估計。")
    print("    實際彎道兩臂不同寬時，用**較窄那一臂**的值代入會偏保守（可接受）。")
    print("    有門框、消防栓、垃圾桶之類的局部束縮，要量**最窄處**。")
    print("  ★ 8/29 驗證：模型判家裡門口 0.967 過不去 —— 與實機一致（出門方向")
    print("    當天零次自主通過；白天兩趟『成功』其實是人把車抱過門口段）。")
    print("    真因是轉向能力絕對值不足：要過需 R≤0.836，實車只有 1.03。")
    print("    → 先做舵機中位/行程校準（目標回到韌體的 0.75），再來談選路。")
    print("    輸入的淨寬務必捲尺實測。")


if __name__ == "__main__":
    if len(sys.argv) == 1:
        spec_table()
        print()
        check(0.967)      # 家裡門口
    else:
        w = float(sys.argv[1])
        e = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_TRACK_ERR
        spec_table()
        print()
        check(w, e)
