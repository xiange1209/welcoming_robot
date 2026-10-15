#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轉向／位姿鏈路離線分析（不依賴 rclpy，只用標準函式庫；可在筆電或雲端跑）。

用途：分辨「方向舵沒打到底／HMI 位置跑掉」到底是哪一層造成的。
輸入是 record_steering_chain.py 錄下的 CSV（長格式，一列一則訊息）：

    t,stamp,src,label,v,w,x,y,yaw
    t      接收時間（秒，節點時鐘）          stamp  訊息 header 時間（秒；沒有就空白）
    src    cmd_smoothed／cmd_trimmed／cmd_prefloor／cmd_final（= /cmd_vel，送給驅動的最後一站）
           odom_raw（/odom，驅動由「上行 Vz 積分」得到）／odom_ekf（/odom_combined）
           imu_raw／imu_unbiased（w = 陀螺儀 z）／amcl（/amcl_pose）／tf_map_base／marker
    label  只有 marker 用：實驗階段名稱（straight_fwd、left_lock、right_lock、slow_left、reverse_left…），
           遇到下一個 marker 就結束上一段；名稱是 end 的 marker 只用來結束上一段。
    v,w    線速度 m/s、角速度 rad/s        x,y,yaw  位姿（m、rad）

子命令
    analyze  <csv> [--json out.json]     分段分析＋假設判定（見 FLAGS）
    model    [--vx 0.1 0.15 0.25]        印出「ROS 指令 -> 韌體 -> 舵機 PWM」逐站換算表（含量化、箝制）

★ 本檔的韌體換算（firmware_chain）逐行對應 docs/cloud/evidence/steering/ 的原始碼，行號寫在函式註解裡；
  讀不到的常數（SERVO_INIT）標註為假設，不當事實。
★ 判定門檻是「可操作的經驗值」，不是統計檢定；每個旗標都會同時印出它依據的數字，請看數字不要只看旗標。
"""
import argparse
import csv
import json
import math
import sys
from collections import defaultdict

# ───────────────────────── 韌體鏈路模型 ─────────────────────────
# 全部常數都來自 docs/cloud/evidence/steering/ 內的原始碼（行號為該副本的行號）
AXLE = 0.322            # robot_select_init.h  SENIOR_AKM_axlespacing
WHEEL = 0.322           # robot_select_init.h  SENIOR_AKM_wheelspacing
MIN_TURN_R = 0.750      # robot_select_init.h:21  SENIOR_AKM_MIN_TURN_RADIUS
ANGLE_MIN, ANGLE_MAX = -0.50, 0.34   # balance.c:65  target_limit_float(AngleR,-0.50f,0.34f)
SERVO_RATIO = 636.56    # balance.c:56  Ratio
SERVO_MIN, SERVO_MAX = 1020, 1890    # balance.c:25  預設值（可被 APP 參數 0x35/0x36 改，usartx.c:499-500）
SERVO_INIT_ASSUMED = 1500            # ★ 假設：SERVO_INIT 的定義不在證據包；車上記憶寫 1500，未經原始碼驗證


def c_trunc_short(x: float) -> int:
    """C++ `short t = double*1000;` 的行為：向 0 截斷（不是四捨五入），超出 int16 範圍在 C++ 是未定義行為，這裡飽和處理。
    wheeltec_robot.cpp:250/257/264"""
    t = int(x)  # Python int() 向 0 截斷，與 C 相同
    return max(-32768, min(32767, t))


def servo_pwm_from_angle(angle_r: float) -> float:
    """balance.c:80  Angle_Servo = -0.2137*a^2 + 1.439*a + 0.009599；PWM 位移 = Angle_Servo * Ratio(636.56)（不含中位與偏置）"""
    return (-0.2137 * angle_r ** 2 + 1.439 * angle_r + 0.009599) * SERVO_RATIO


def firmware_chain(vx: float, wz: float, servo_bias: int = 0, servo_init: int = SERVO_INIT_ASSUMED) -> dict:
    """ROS /cmd_vel (vx, wz) -> 11 bytes -> 韌體 -> 舵機 PWM，逐站回傳。

    站 1  wheeltec_robot.cpp:250,264        short = trunc(x*1000)
    站 2  usartx.c:389-391                   還原成 m/s、rad/s（假設 /1000；XYZ_Target_Speed_transition 本體不在證據包的片段內）
    站 3  usartx.c:594-640 Vz_to_Akm_Angle    |Vx/Vz| <= 0.75 時把 Vz 壓到 |Vx|/0.75；Vx 或 Vz 為 0 -> AngleR = 0（舵機回正）
    站 4  balance.c:61,65                    R 由「箝制前」的 AngleR 算；然後 AngleR 箝到 [-0.50, +0.34]
    站 5  balance.c:70-71                    後輪目標速度（用箝制前的 R）；上行 Vz = (B-A)/輪距（usartx.c:52）= Vx/R
    站 6  balance.c:80-86                    角度 -> PWM（二次多項式），再箝到 [Servo_min, Servo_max]
    """
    tx_x, tx_z = c_trunc_short(vx * 1000.0), c_trunc_short(wz * 1000.0)
    Vx, Vz = tx_x / 1000.0, tx_z / 1000.0
    min_r_hit = False
    if Vz != 0.0 and Vx != 0.0:
        if abs(Vx / Vz) <= MIN_TURN_R:                       # usartx.c:622
            Vz = math.copysign(abs(Vx) / MIN_TURN_R, Vz)      # usartx.c:626-629
            min_r_hit = True
        R = Vx / Vz                                          # usartx.c:631
        angle_r = math.atan(AXLE / (R + 0.5 * WHEEL))        # usartx.c:633
    else:
        R, angle_r = float("inf"), 0.0                       # usartx.c:637
    angle_c = max(ANGLE_MIN, min(ANGLE_MAX, angle_r))        # balance.c:65
    angle_clamped = angle_c != angle_r
    if angle_r != 0.0:
        R2 = AXLE / math.tan(angle_r) - 0.5 * WHEEL          # balance.c:61（箝制前）
        a_t = Vx * (R2 - 0.5 * WHEEL) / R2                   # balance.c:70
        b_t = Vx * (R2 + 0.5 * WHEEL) / R2                   # balance.c:71
    else:
        a_t = b_t = Vx
    uplink_wz = (b_t - a_t) / WHEEL                          # usartx.c:52（假設輪速 PI 追得上目標）
    pwm_off = servo_pwm_from_angle(angle_c)
    pwm_raw = servo_init + servo_bias + pwm_off
    pwm = max(SERVO_MIN, min(SERVO_MAX, pwm_raw))            # balance.c:86
    return {
        "tx_vx": tx_x, "tx_wz": tx_z, "vx_q": Vx, "wz_q": tx_z / 1000.0,
        "kappa_cmd": (wz / vx) if vx else 0.0, "kappa_after_quant": (tx_z / tx_x) if tx_x else 0.0,
        "min_r_hit": min_r_hit, "angle_r": angle_r, "angle_clamped": angle_clamped, "angle_used": angle_c,
        "uplink_wz": uplink_wz, "pwm_offset": pwm_off, "pwm_raw": pwm_raw, "pwm": pwm,
        "pwm_clipped": round(pwm_raw - pwm, 1),
    }


# ───────────────────────── 讀檔與取樣 ─────────────────────────
def _f(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def load_csv(path):
    """回傳 {src: [(t, stamp, v, w, x, y, yaw, label), ...]}（依 t 排序）。壞列略過並計數。"""
    by, bad = defaultdict(list), 0
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            t, src = _f(row.get("t")), (row.get("src") or "").strip()
            if t is None or not src:
                bad += 1
                continue
            by[src].append((t, _f(row.get("stamp")), _f(row.get("v")), _f(row.get("w")),
                            _f(row.get("x")), _f(row.get("y")), _f(row.get("yaw")), (row.get("label") or "").strip()))
    for k in by:
        by[k].sort(key=lambda r: r[0])
    return by, bad


def hold_at(series, field, t):
    """取樣保持：回傳 t 之前最後一筆的欄位值（沒有或值為 None 回 None）。series 已依時間排序。"""
    lo, hi, best = 0, len(series), None
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            best, lo = mid, mid + 1
        else:
            hi = mid
    return None if best is None else series[best][field]


# 欄位索引
T, STAMP, V, W, X, Y, YAW, LABEL = range(8)


def unwrap(angles):
    out, off, prev = [], 0.0, None
    for a in angles:
        if prev is not None:
            d = a - prev
            if d > math.pi:
                off -= 2 * math.pi
            elif d < -math.pi:
                off += 2 * math.pi
        out.append(a + off)
        prev = a
    return out


def slope_through_origin(xs, ys):
    sxx = sum(x * x for x in xs)
    return (sum(x * y for x, y in zip(xs, ys)) / sxx) if sxx > 1e-12 else None


def median(xs):
    xs = sorted(xs)
    n = len(xs)
    if not n:
        return None
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def corr(xs, ys):
    n = len(xs)
    if n < 3:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx < 1e-12 or syy < 1e-12:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / math.sqrt(sxx * syy)


# ───────────────────────── 分段 ─────────────────────────
def segments(by):
    """依 marker 切段 -> [(label, t0, t1)]。沒有 marker 就整份一段 'all'。"""
    marks = by.get("marker", [])
    all_t = [r[0] for s in by.values() for r in s]
    if not all_t:
        return []
    if not marks:
        return [("all", min(all_t), max(all_t))]
    out = []
    for i, m in enumerate(marks):
        t1 = marks[i + 1][0] if i + 1 < len(marks) else max(all_t)
        if m[LABEL] and m[LABEL] != "end" and t1 > m[0]:
            out.append((m[LABEL], m[0], t1))
    return out


GRID_HZ = 20.0
SETTLE_S = 1.5   # 每段開頭不計入比率的暫態秒數


def grid(t0, t1):
    n = int((t1 - t0) * GRID_HZ)
    return [t0 + i / GRID_HZ for i in range(max(0, n))]


def analyze_segment(by, label, t0, t1, min_speed=0.03):
    """單一階段的所有指標。取樣保持到 20 Hz；只用 |v|>min_speed 的樣本算比率。"""
    g = grid(t0, t1)
    res = {"label": label, "duration": round(t1 - t0, 2), "n_grid": len(g)}
    imu = by.get("imu_unbiased") or by.get("imu_raw") or []
    res["imu_src"] = "imu_unbiased" if by.get("imu_unbiased") else ("imu_raw" if by.get("imu_raw") else None)
    rows = []
    for t in g:
        rows.append({
            "t": t,
            "v_odom": hold_at(by.get("odom_raw", []), V, t),
            "w_enc": hold_at(by.get("odom_raw", []), W, t),
            "w_ekf": hold_at(by.get("odom_ekf", []), W, t),
            "w_gyro": hold_at(imu, W, t),
            "v_cmd": hold_at(by.get("cmd_final", []), V, t),
            "w_cmd": hold_at(by.get("cmd_final", []), W, t),
        })
    # 比率類指標只用「穩態」樣本：扣掉每段開頭的 settle 秒（舵機行程＋延遲的暫態，另用 rise_time／lag 量）
    settle = min(SETTLE_S, 0.25 * (t1 - t0))
    res["settle_s"] = round(settle, 2)
    mov = [r for r in rows if r["t"] >= t0 + settle and None not in (r["v_odom"], r["w_enc"], r["w_gyro"])
           and abs(r["v_odom"]) > min_speed]
    res["n_moving"] = len(mov)
    if len(mov) < 10:
        res["note"] = "移動樣本不足 10 筆（車沒動或缺 odom／imu 資料）"
        return res
    res["v_mean_abs"] = round(sum(abs(r["v_odom"]) for r in mov) / len(mov), 3)
    # 曲率（有號）：κ = w / v。倒車時 v<0，κ 的正負 = 輪子實際打哪邊（與 path_teach._clamp_curv 同一慣例）
    k_enc = [r["w_enc"] / r["v_odom"] for r in mov]
    k_gyro = [r["w_gyro"] / r["v_odom"] for r in mov]
    res["kappa_enc_med"], res["kappa_gyro_med"] = round(median(k_enc), 3), round(median(k_gyro), 3)
    res["R_enc_m"] = round(1 / abs(median(k_enc)), 2) if abs(median(k_enc)) > 1e-3 else None
    res["R_gyro_m"] = round(1 / abs(median(k_gyro)), 2) if abs(median(k_gyro)) > 1e-3 else None
    # 增益：陀螺（物理）對編碼器（上行 Vz）；過原點回歸 + 逐點比率中位數
    we, wg = [r["w_enc"] for r in mov], [r["w_gyro"] for r in mov]
    res["g_gyro_over_enc"] = _r(slope_through_origin(we, wg))
    res["corr_gyro_enc"] = _r(corr(we, wg))
    cm = [r for r in mov if r["w_cmd"] is not None and abs(r["w_cmd"]) > 0.01]
    if len(cm) >= 10:
        wc = [r["w_cmd"] for r in cm]
        res["g_enc_over_cmd"] = _r(slope_through_origin(wc, [r["w_enc"] for r in cm]))
        res["g_gyro_over_cmd"] = _r(slope_through_origin(wc, [r["w_gyro"] for r in cm]))
        res["corr_enc_cmd"] = _r(corr(wc, [r["w_enc"] for r in cm]))
        # 延遲：cmd_final.w -> imu.w，掃 0~1.5 秒
        res["lag_cmd_to_gyro_s"], res["lag_corr"] = best_lag(rows)
        res["rise_time_s"] = rise_time(rows)
    ek = [r for r in mov if r["w_ekf"] is not None]
    if len(ek) >= 10:
        we2 = [r["w_enc"] for r in ek]
        wg2 = [r["w_gyro"] for r in ek]
        wk = [r["w_ekf"] for r in ek]
        res["g_ekf_over_enc"] = _r(slope_through_origin(we2, wk))
        res["g_ekf_over_gyro"] = _r(slope_through_origin(wg2, wk))
    # 指令鏈各站的曲率（κ = w/v 的中位數）：看哪一站改了曲率。預期：trimmed−smoothed = steering_trim 的 trim（-0.235），
    # prefloor−trimmed ≈ 0、final−prefloor ≈ 0（防撞與死區地板都應保持曲率）。
    stage = {}
    for name in ("cmd_smoothed", "cmd_trimmed", "cmd_prefloor", "cmd_final"):
        ser = by.get(name, [])
        ks = []
        for r in rows:
            if r["t"] < t0 + settle:
                continue
            vv, ww = hold_at(ser, V, r["t"]), hold_at(ser, W, r["t"])
            if vv is not None and ww is not None and abs(vv) > min_speed:
                ks.append(ww / vv)
        if len(ks) >= 10:
            stage[name] = round(median(ks), 3)
    res["stage_kappa"] = stage
    # 積分：陀螺 vs AMCL / TF 的偏航變化
    dt = 1.0 / GRID_HZ
    res["dyaw_gyro_deg"] = round(math.degrees(sum(r["w_gyro"] * dt for r in rows if r["w_gyro"] is not None)), 1)
    res["dyaw_enc_deg"] = round(math.degrees(sum(r["w_enc"] * dt for r in rows if r["w_enc"] is not None)), 1)
    dist = sum(abs(r["v_odom"]) * dt for r in rows if r["v_odom"] is not None)
    res["distance_m"] = round(dist, 2)
    if dist > 0.2:
        res["gyro_deg_per_m"] = round(res["dyaw_gyro_deg"] / dist, 2)
    for key in ("amcl", "tf_map_base"):
        s = [r for r in by.get(key, []) if t0 <= r[0] <= t1 and r[YAW] is not None]
        if len(s) >= 2:
            yaws = unwrap([r[YAW] for r in s])
            res[f"dyaw_{key}_deg"] = round(math.degrees(yaws[-1] - yaws[0]), 1)
            res[f"n_{key}"] = len(s)
    return res


def _r(x, nd=3):
    return None if x is None else round(x, nd)


def best_lag(rows, max_lag_s=1.5):
    """cmd_final.w 領先 imu.w 幾秒（正規化互相關最大處）。"""
    pairs = [(r["w_cmd"], r["w_gyro"]) for r in rows if r["w_cmd"] is not None and r["w_gyro"] is not None]
    n = len(pairs)
    if n < 40:
        return None, None
    c = [p[0] for p in pairs]
    gy = [p[1] for p in pairs]
    best, best_k = None, None
    for k in range(0, int(max_lag_s * GRID_HZ) + 1):
        if n - k < 20:
            break
        r = corr(c[: n - k], gy[k:])
        if r is not None and (best is None or r > best):
            best, best_k = r, k
    if best is None:
        return None, None
    return round(best_k / GRID_HZ, 2), round(best, 3)


def rise_time(rows):
    """階躍響應：從 cmd_final 的 |w| 第一次超過 0.01 起，到陀螺 |w| 達到『後半段中位數』的 63% 要多久（秒）。
    量的是 舵機行程時間＋底盤動態（steering_trim 的低通已經在 cmd_final 裡了）。韌體在 Vx=0 時會把舵機回正
    （usartx.c:618,637），所以每次從停止起步都會重新從中位打出去 —— 這個數字就是那段時間。"""
    pts = [(r["t"], r["w_cmd"], r["w_gyro"]) for r in rows if r["w_cmd"] is not None and r["w_gyro"] is not None]
    if len(pts) < 40:
        return None
    t_on = next((t for t, wc, _ in pts if abs(wc) > 0.01), None)
    if t_on is None:
        return None
    half = sorted(abs(g) for t, _, g in pts[len(pts) // 2:])
    target = 0.63 * half[len(half) // 2]
    if target < 0.01:
        return None
    t_hit = next((t for t, _, g in pts if t >= t_on and abs(g) >= target), None)
    return None if t_hit is None else round(t_hit - t_on, 2)


# ───────────────────────── 假設判定 ─────────────────────────
FLAGS = {
    "STAGE_KAPPA_SHIFT": "防撞（trimmed→prefloor）或死區地板（prefloor→final）改變了曲率 > 0.05 /m → 這兩站本應保持曲率不變",
    "SLOW_RISE": "從停止起步到陀螺達 63% 超過 0.6 s → 起步後舵機要先從中位打出去，這段時間轉向不足（折返／三點轉向最明顯）",
    "STEER_SHORT": "實際轉向 < 韌體以為的（陀螺/編碼器 < 0.85，且在大曲率階段）→ 舵機行程/校準/中位/PWM 箝制，非格式問題",
    "UPLINK_FOLLOWS_COMMAND": "編碼器 Vz ≈ 指令 ω（0.9~1.1）卻和陀螺差很多 → 上行 Vz 是『指令推算』，對舵機沒轉到的部分是瞎的",
    "EKF_FOLLOWS_ENCODER": "EKF 的 ω 跟編碼器一致、不跟陀螺 → odom_combined 也高估轉向",
    "EKF_FOLLOWS_GYRO": "EKF 的 ω 跟陀螺一致 → odom_combined（AMCL 的里程計來源）看到的是真實轉向",
    "GYRO_SCALE": "陀螺積分的偏航變化與 AMCL/TF 差 >10%（且變化 >30°）→ 陀螺比例（GYROSCOPE_RATIO）或 AMCL 有問題",
    "LATENCY": "指令到陀螺反應延遲 > 0.5 s → 舵機/低通/鏈路延遲，純追蹤相位落後",
    "AMCL_LAG": "AMCL 訊息接收時間比 header 時間晚 > 0.3 s → HMI 用接收時間當基準會造成 v×延遲 的落後",
    "AMCL_JUMP": "AMCL 相鄰兩則跳 > 0.5 m 或 > 30° 而 EKF 里程計沒這麼大 → 定位被拉走（走廊縱向歧義/脫困轉丟）",
    "ODOM_RAW_EKF_DIVERGE": "整份資料 /odom 與 /odom_combined 的偏航累積差 > 5° → HMI 外推用 /odom 會和 AMCL 吃的里程計不一致",
}


def decide(segs, by):
    flags = {}

    def add(name, evidence):
        flags.setdefault(name, []).append(evidence)

    for s in segs:
        if "g_gyro_over_enc" not in s:
            continue
        big = abs(s.get("kappa_enc_med") or 0) > 0.3
        if big and (s["g_gyro_over_enc"] or 1) < 0.85:
            add("STEER_SHORT", f"{s['label']}: 陀螺/編碼器={s['g_gyro_over_enc']}（κ_enc={s['kappa_enc_med']}，R_enc={s.get('R_enc_m')}→R_gyro={s.get('R_gyro_m')} m）")
        # 鎖定階段指令幾乎是常數，相關係數沒有意義（變異數≈0），所以只看比率；有相關係數時一併印出
        if big and (s["g_gyro_over_enc"] or 1) < 0.85 and s.get("g_enc_over_cmd") and 0.9 <= s["g_enc_over_cmd"] <= 1.1:
            add("UPLINK_FOLLOWS_COMMAND", f"{s['label']}: 編碼器/指令={s['g_enc_over_cmd']}（相關 {s.get('corr_enc_cmd')}），陀螺/編碼器={s['g_gyro_over_enc']}")
        if big and s.get("g_ekf_over_enc") and s.get("g_ekf_over_gyro"):
            de, dg = abs(s["g_ekf_over_enc"] - 1), abs(s["g_ekf_over_gyro"] - 1)
            if (s["g_gyro_over_enc"] or 1) < 0.85:
                if de < 0.1 and dg > 0.15:
                    add("EKF_FOLLOWS_ENCODER", f"{s['label']}: EKF/編碼器={s['g_ekf_over_enc']}，EKF/陀螺={s['g_ekf_over_gyro']}")
                elif dg < 0.1 and de > 0.15:
                    add("EKF_FOLLOWS_GYRO", f"{s['label']}: EKF/陀螺={s['g_ekf_over_gyro']}，EKF/編碼器={s['g_ekf_over_enc']}")
        if s.get("lag_cmd_to_gyro_s") is not None and s["lag_cmd_to_gyro_s"] > 0.5 and (s.get("lag_corr") or 0) > 0.5:
            add("LATENCY", f"{s['label']}: 領先 {s['lag_cmd_to_gyro_s']} s（相關 {s['lag_corr']}）")
        sk = s.get("stage_kappa") or {}
        for lo, hi in (("cmd_trimmed", "cmd_prefloor"), ("cmd_prefloor", "cmd_final")):
            if lo in sk and hi in sk and abs(sk[hi] - sk[lo]) > 0.05:
                add("STAGE_KAPPA_SHIFT", f"{s['label']}: κ({lo})={sk[lo]} → κ({hi})={sk[hi]}")
        if s.get("rise_time_s") is not None and s["rise_time_s"] > 0.6:
            add("SLOW_RISE", f"{s['label']}: 起步到 63% 轉向 {s['rise_time_s']} s")
        for key in ("amcl", "tf_map_base"):
            d = s.get(f"dyaw_{key}_deg")
            if d is not None and abs(d) > 30 and abs(s["dyaw_gyro_deg"]) > 1e-6:
                ratio = s["dyaw_gyro_deg"] / d
                if abs(ratio - 1) > 0.10:
                    add("GYRO_SCALE", f"{s['label']}: 陀螺積分 {s['dyaw_gyro_deg']}° vs {key} {d}°（比 {ratio:.2f}）")
    am = by.get("amcl", [])
    lags = [r[0] - r[STAMP] for r in am if r[STAMP] is not None]
    if len(lags) >= 5 and median(lags) > 0.3:
        add("AMCL_LAG", f"接收時間 − header 時間 中位數 {median(lags):.2f} s（{len(lags)} 則）")
    ek = by.get("odom_ekf", [])
    for a, b in zip(am, am[1:]):
        if None in (a[X], a[Y], a[YAW], b[X], b[Y], b[YAW]):
            continue
        jump = math.hypot(b[X] - a[X], b[Y] - a[Y])
        djaw = abs(math.degrees(math.atan2(math.sin(b[YAW] - a[YAW]), math.cos(b[YAW] - a[YAW]))))
        if jump > 0.5 or djaw > 30:
            ea, eb = hold_at(ek, X, a[0]), hold_at(ek, X, b[0])
            ey_a, ey_b = hold_at(ek, Y, a[0]), hold_at(ek, Y, b[0])
            moved = math.hypot(eb - ea, ey_b - ey_a) if None not in (ea, eb, ey_a, ey_b) else None
            if moved is None or jump > moved + 0.4 or djaw > 30:
                add("AMCL_JUMP", f"t={b[0]:.1f}: 跳 {jump:.2f} m／{djaw:.0f}°（EKF 里程計同期 {('%.2f m' % moved) if moved is not None else '未知'}）")
    ro, ek2 = by.get("odom_raw", []), by.get("odom_ekf", [])
    if len(ro) > 10 and len(ek2) > 10 and ro[0][YAW] is not None and ek2[0][YAW] is not None:
        y1 = math.degrees(unwrap([r[YAW] for r in ro if r[YAW] is not None])[-1] - [r[YAW] for r in ro if r[YAW] is not None][0])
        y2 = math.degrees(unwrap([r[YAW] for r in ek2 if r[YAW] is not None])[-1] - [r[YAW] for r in ek2 if r[YAW] is not None][0])
        if abs(y1 - y2) > 5:
            add("ODOM_RAW_EKF_DIVERGE", f"整份資料偏航累積：/odom {y1:.1f}° vs /odom_combined {y2:.1f}°（差 {y1 - y2:.1f}°）")
    return flags


def analyze(path):
    by, bad = load_csv(path)
    segs = [analyze_segment(by, *seg) for seg in segments(by)]
    flags = decide(segs, by)
    return {"file": path, "bad_rows": bad, "sources": {k: len(v) for k, v in by.items()},
            "segments": segs, "flags": flags}


def print_report(rep, out=sys.stdout):
    p = lambda *a: print(*a, file=out)
    p(f"檔案：{rep['file']}　壞列 {rep['bad_rows']}　來源：{rep['sources']}")
    missing = [k for k in ("cmd_final", "odom_raw", "imu_unbiased", "amcl") if k not in rep["sources"] and not (k == "imu_unbiased" and "imu_raw" in rep["sources"])]
    if missing:
        p(f"⚠ 缺少來源：{missing}（對應的指標會是空的，不是 0）")
    p("")
    p("階段".ljust(14) + "秒".rjust(6) + "|v|".rjust(7) + "κ_編".rjust(8) + "κ_陀".rjust(8) + "R編".rjust(7) + "R陀".rjust(7)
      + "陀/編".rjust(8) + "編/令".rjust(8) + "陀/令".rjust(8) + "EKF/陀".rjust(8) + "延遲s".rjust(7) + "起步s".rjust(7) + "偏航陀°".rjust(9) + "AMCL°".rjust(8))
    f = lambda x, w=8: (str(x) if x is not None else "-").rjust(w)
    for s in rep["segments"]:
        if "g_gyro_over_enc" not in s:
            p(s["label"].ljust(14) + f"（{s.get('note', '無資料')}）")
            continue
        p(s["label"].ljust(14) + f(s["duration"], 6) + f(s["v_mean_abs"], 7) + f(s["kappa_enc_med"]) + f(s["kappa_gyro_med"])
          + f(s.get("R_enc_m"), 7) + f(s.get("R_gyro_m"), 7) + f(s["g_gyro_over_enc"]) + f(s.get("g_enc_over_cmd")) + f(s.get("g_gyro_over_cmd"))
          + f(s.get("g_ekf_over_gyro")) + f(s.get("lag_cmd_to_gyro_s"), 7) + f(s.get("rise_time_s"), 7) + f(s["dyaw_gyro_deg"], 9) + f(s.get("dyaw_amcl_deg"), 8))
    for s in rep["segments"]:
        sk = s.get("stage_kappa")
        if sk:
            p(f"  指令鏈曲率 {s['label']}: " + " → ".join(f"{k.replace('cmd_', '')}={v}" for k, v in sk.items()))
    p("")
    if not rep["flags"]:
        p("旗標：無（沒有任何假設被數字支持；若現象仍在，檢查是否真的涵蓋了左右滿舵與低速階段）")
    for name, ev in rep["flags"].items():
        p(f"旗標 {name}：{FLAGS[name]}")
        for e in ev[:6]:
            p(f"    - {e}")
    p("")
    p("判讀備忘：陀/編 < 1 ＝ 實際轉得比上行 Vz 少；若同時 編/令 ≈ 1 ＝ 上行 Vz 只是指令的回聲。"
      "R陀 對照 8/29 實測（左 1.03／右 1.18 m）與韌體 0.75 m。")


# ───────────────────────── model 子命令 ─────────────────────────
def print_model(vxs, out=sys.stdout):
    p = lambda *a: print(*a, file=out)
    p("ROS 指令 -> 韌體 -> 舵機（SERVO_INIT 假設 1500、SERVO_BIAS=0；實際值請在車上讀 OLED/APP）")
    p("vx(m/s)  κ(1/m)  R(m)   tx_vx tx_wz  κ量化後  AngleR(rad/deg)  箝制?  上行Vz  PWM位移  PWM")
    for vx in vxs:
        for k in (-1.6, -1.333, -1.07, -0.83, -0.5, -0.2, -0.05, -0.01, 0.01, 0.05, 0.2, 0.5, 0.72, 0.95, 1.333, 1.6):
            c = firmware_chain(vx, k * vx)
            mark = ("最小半徑" if c["min_r_hit"] else "") + ("/角度箝" if c["angle_clamped"] else "") + ("/PWM削" if c["pwm_clipped"] else "")
            p(f"{vx:6.3f} {k:8.2f} {(1 / abs(k)):6.2f} {c['tx_vx']:6d} {c['tx_wz']:6d} {c['kappa_after_quant']:8.3f}"
              f"  {c['angle_used']:+.3f}/{math.degrees(c['angle_used']):+6.1f}  {mark or '-':>10} {c['uplink_wz']:+.3f} {c['pwm_offset']:+8.1f} {c['pwm']:7.1f}")
        p("")
    p("量化：Vz 以 1 mrad/s 為單位且向 0 截斷（wheeltec_robot.cpp:264）。")
    p("vx(m/s)  1 mrad/s 的曲率誤差(1/m)  對應前輪角誤差(deg)  PWM 誤差(counts，κ≈0 附近)")
    for vx in vxs:
        dk = 0.001 / vx
        dang = math.degrees(math.atan(AXLE * dk))
        dpwm = 1.439 * AXLE * dk * SERVO_RATIO
        p(f"{vx:6.3f}   {dk:10.4f}              {dang:10.3f}            {dpwm:8.2f}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("analyze")
    a.add_argument("csv")
    a.add_argument("--json")
    m = sub.add_parser("model")
    m.add_argument("--vx", type=float, nargs="*", default=[0.10, 0.15, 0.25])
    args = ap.parse_args(argv)
    if args.cmd == "model":
        print_model(args.vx)
        return 0
    rep = analyze(args.csv)
    print_report(rep)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(rep, fh, ensure_ascii=False, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
