# -*- coding: utf-8 -*-
"""tools/steer_diag/analyze_steering_logs.py 的測試（合成資料；不需要 ROS 2、不需要 pytest）。

跑法：python3 tests/steer_diag/test_analyze.py        （與 tests/nav_stub/ 同風格：印 ✓／✗，最後印 N/M 通過）

測什麼
  A. 韌體換算（firmware_chain）：截斷、最小半徑箝制、Vx=0 回正、PWM 箝制與 SERVO_BIAS、path_teach 加 trim 後實際到達的 PWM。
     期望值全部是從 docs/cloud/evidence/steering/ 的原始碼手算的，不是從程式輸出抄的。
  B. 分析器（合成車）：造一台「已知增益／延遲／陀螺比例」的車，寫成 CSV，確認分析器
     (1) 估得出那個增益／延遲 (2) 該亮的旗標有亮 (3) 健康的車不亮 (4) 缺資料時不當機。
不測什麼
  - 真車的數字。合成車的增益（0.73／0.63）取自 8/29 實測的 R（1.03／1.183 對韌體 0.75），
    那是『假設情境』，不是『已驗證的車』。
"""
import io
import math
import os
import random
import sys
import tempfile
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

REPO = Path(os.environ.get("SMARTNAV_REPO") or Path(__file__).resolve().parents[2])
sys.path.insert(0, str(REPO / "tools" / "steer_diag"))
import analyze_steering_logs as A  # noqa: E402

results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


def near(a, b, tol):
    return a is not None and abs(a - b) <= tol


# ═══ A. 韌體換算 ═════════════════════════════════════════════════
print("── A. 韌體換算 ──")
check(A.c_trunc_short(0.0999 * 1000) == 99, "short = 0.0999*1000 截斷成 99（不是四捨五入成 100）")
check(A.c_trunc_short(-0.0555 * 1000) == -55, "負數向 0 截斷：-55.5 -> -55（不是 -56）")
check(A.c_trunc_short(0.095 * 1000) == 95 and A.c_trunc_short(0.15 * 1000) == 150,
      "常用速度 0.095／0.15 乘 1000 不會因浮點誤差少 1（實際掃過 1..399 mm/s 全部正確）")
check(all(A.c_trunc_short(k / 1000 * 1000) == k for k in range(1, 400)), "k/1000*1000 對 1..399 全部還原成 k")

c = A.firmware_chain(0.15, 0.30)       # κ=2.0 -> R=0.5 < 0.75
check(c["min_r_hit"] and near(c["uplink_wz"], 0.15 / 0.75, 1e-9), "κ=2.0（R=0.5）：韌體壓到 R=0.75，上行 Vz = Vx/0.75 = 0.200（靜默壓制）")
check(c["tx_wz"] == 300 and near(c["wz_q"], 0.300, 1e-9), "壓制發生在韌體內：ROS 端仍然送 300（沒有任何回報）")

c = A.firmware_chain(0.0, 0.10)
check(c["angle_used"] == 0.0 and near(c["pwm_offset"], 0.009599 * 636.56, 1e-6),
      "Vx=0 且 Vz≠0：AngleR=0 -> 舵機回到（偏 +6.1 的）中位（usartx.c:618,637）")
c = A.firmware_chain(0.15, 0.0)
check(c["angle_used"] == 0.0, "Vz=0：AngleR=0")

left = A.firmware_chain(0.15, (1 / 0.75) * 0.15)
right = A.firmware_chain(0.15, -(1 / 0.75) * 0.15)
check(near(left["angle_used"], 0.3394, 2e-3) and near(left["pwm_offset"], 301, 3), "左滿舵（R=0.75）：AngleR≈+0.339，PWM 位移≈+301")
check(near(right["angle_used"], -0.4999, 4e-3) and near(right["pwm_offset"], -484, 3),
      "右滿舵（R=0.75）：AngleR≈-0.50，PWM 位移≈-484（ω=-0.2 乘 1000 得 -199.99999999999997，截斷成 -199：浮點乘積也會少 1 mrad/s）")
check(right["tx_wz"] == -199, "實例：-(1/0.75)*0.15 -> short 是 -199 不是 -200（截斷而非四捨五入）")
check(right["pwm"] == 1020 and near(right["pwm_clipped"], -3.4, 0.6), "右滿舵在 SERVO_BIAS=0 時已被 Servo_min=1020 削掉約 3 counts")
check(A.firmware_chain(0.15, (1 / 0.75) * 0.15, servo_bias=100)["pwm"] == 1890 and
      near(A.firmware_chain(0.15, (1 / 0.75) * 0.15, servo_bias=100)["pwm_clipped"], 11, 1),
      "SERVO_BIAS=+100：左滿舵被 Servo_max=1890 削掉約 11 counts")
check(near(A.firmware_chain(0.15, -(1 / 0.75) * 0.15, servo_bias=-100)["pwm_clipped"], -104, 2),
      "SERVO_BIAS=-100：右滿舵被削掉約 104 counts（偏置電位器直接決定哪一邊先到底）")

# path_teach 的曲率上限（左 1/1.05、右 1/1.20）再加 steering_trim 的 -0.235 rad/m 之後，韌體實際收到的
kl = A.firmware_chain(0.15, (1 / 1.05 - 0.235) * 0.15)
kr = A.firmware_chain(0.15, (-1 / 1.20 - 0.235) * 0.15)
check(near(kl["pwm_offset"], 187, 3) and near(kl["pwm_offset"] / left["pwm_offset"], 0.62, 0.02),
      "path_teach 左上限 + trim：只到韌體左行程的 62%（PWM +187／+301）")
check(near(kr["pwm_offset"], -375, 4) and near(kr["pwm_offset"] / right["pwm_offset"], 0.78, 0.02),
      "path_teach 右上限 + trim：只到韌體右行程的 78%（PWM -375／-484）")
check(not kl["min_r_hit"] and not kr["min_r_hit"], "正常導航從不觸發韌體最小半徑箝制（箝制在 path_teach 先發生）")

# 量化：1 mrad/s 在 0.10 m/s = 0.01 /m
c1 = A.firmware_chain(0.10, 0.0009)
check(c1["tx_wz"] == 0 and c1["angle_used"] == 0.0, "ω=0.9 mrad/s 被截成 0 -> 舵機回正（低速小曲率被吃掉）")

# ═══ B. 合成車 ═══════════════════════════════════════════════════
print("── B. 合成車與分析器 ──")


def simulate(path, phases, g_left=1.0, g_right=1.0, delay=0.25, tau=0.2, ekf="gyro", gyro_scale=1.0,
             amcl_lag=0.1, amcl_jump_at=None, noise=0.001, seed=7, drop=(), bad_rows=0, no_markers=False):
    """phases: [(label, v, kappa, seconds)]，kappa = w/v（有號；>0 = 輪子打左，倒車時 <0 = 打右），cmd_final 的 w = kappa*v。
        kappa 也可以是函式 f(秒數)->kappa（蛇行用）。
    車：編碼器上行 Vz = 韌體換算結果（指令的回聲）；真實偏航率 = g(左右) × 上行 Vz，經 delay 與一階 tau；
        陀螺 = 真實偏航率 × gyro_scale；EKF = 陀螺 或 編碼器；AMCL = 真實偏航，每 0.5 s 一則。"""
    rnd = random.Random(seed)
    dt, t0 = 0.01, 1000.0
    rows, t, yaw_true, yaw_enc, yaw_ekf = [], t0, 0.0, 0.0, 0.0
    phys_w, enc_hist = 0.0, []
    x_true = 0.0
    amcl_t = 0.0
    def add(src, tt, stamp=None, label="", v=None, w=None, x=None, y=None, yaw=None):
        if src in drop:
            return
        f = lambda a: "" if a is None else f"{a:.6f}"
        rows.append(f"{tt:.4f},{f(stamp)},{src},{label},{f(v)},{f(w)},{f(x)},{f(y)},{f(yaw)}")
    for label, v, kappa, secs in phases:
        if not no_markers:
            add("marker", t, label=label)
        n = int(secs / dt)
        for i in range(n):
            k_now = kappa((i * dt)) if callable(kappa) else kappa
            w_cmd = k_now * v
            ch = A.firmware_chain(v, w_cmd) if v else A.firmware_chain(0.0, 0.0)
            enc_w = ch["uplink_wz"] if v else 0.0
            enc_hist.append(enc_w)
            lag_i = int(delay / dt)
            delayed = enc_hist[-1 - lag_i] if len(enc_hist) > lag_i else 0.0
            side_left = k_now >= 0
            g = g_left if side_left else g_right
            target = g * delayed
            phys_w += (target - phys_w) * dt / max(tau, dt)
            yaw_true += phys_w * dt
            yaw_enc += enc_w * dt
            yaw_ekf += (phys_w * gyro_scale if ekf == "gyro" else enc_w) * dt
            x_true += v * dt
            if i % 5 == 0:     # 20 Hz
                add("cmd_final", t, v=v, w=w_cmd)
                add("odom_raw", t, stamp=t, v=v, w=enc_w, x=x_true, y=0.0, yaw=yaw_enc)
            if i % 2 == 0:     # 50 Hz
                add("imu_unbiased", t, stamp=t, w=phys_w * gyro_scale + rnd.gauss(0, noise))
            if i % 10 == 0:    # 10 Hz
                add("odom_ekf", t, stamp=t, v=v, w=(phys_w * gyro_scale if ekf == "gyro" else enc_w), x=x_true, y=0.0, yaw=yaw_ekf)
            if t - amcl_t >= 0.5:
                amcl_t = t
                jump = 3.0 if (amcl_jump_at is not None and t - t0 > amcl_jump_at) else 0.0
                add("amcl", t, stamp=t - amcl_lag, x=x_true + jump, y=0.0, yaw=yaw_true)
            t += dt
    if not no_markers:
        add("marker", t, label="end")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("t,stamp,src,label,v,w,x,y,yaw\n")
        fh.write("\n".join(rows) + "\n")
        for _ in range(bad_rows):
            fh.write("garbage,,,,\n")
    return path


tmp = tempfile.mkdtemp()
PH = [("straight_fwd", 0.15, 0.0, 6), ("left_lock", 0.10, 1 / 0.75, 8), ("right_lock", 0.10, -1 / 0.75, 8)]


def run(name, **kw):
    p = simulate(os.path.join(tmp, name + ".csv"), kw.pop("phases", PH), **kw)
    return A.analyze(p)


def seg(rep, label):
    return next(s for s in rep["segments"] if s["label"] == label)


# B1 舵機沒轉到：增益 0.73／0.63，上行 Vz 是指令回聲，EKF 跟陀螺
r = run("short", g_left=0.73, g_right=0.63)
L, R = seg(r, "left_lock"), seg(r, "right_lock")
check(near(L["g_gyro_over_enc"], 0.73, 0.04) and near(R["g_gyro_over_enc"], 0.63, 0.04),
      f"估得出左 0.73／右 0.63 的增益（得 {L['g_gyro_over_enc']}／{R['g_gyro_over_enc']}）")
check(near(L["R_enc_m"], 0.75, 0.03) and near(L["R_gyro_m"], 1.03, 0.1) and near(R["R_gyro_m"], 1 / (0.63 / 0.75), 0.12),
      f"左滿舵：韌體以為 R={L['R_enc_m']} m，實際 R={L['R_gyro_m']} m（對應 8/29 的 1.03）；右 {R['R_enc_m']}→{R['R_gyro_m']} m（對應 1.183）")
check(near(L["g_enc_over_cmd"], 1.0, 0.05), f"編碼器 Vz ≈ 指令 ω（回聲）：編碼器/指令≈1（得 {L['g_enc_over_cmd']}）")
for flag in ("STEER_SHORT", "UPLINK_FOLLOWS_COMMAND", "EKF_FOLLOWS_GYRO"):
    check(flag in r["flags"], f"旗標 {flag} 亮起")
check("EKF_FOLLOWS_ENCODER" not in r["flags"], "EKF_FOLLOWS_ENCODER 不亮（EKF 跟的是陀螺）")
st = seg(r, "straight_fwd")
check(abs(st["dyaw_gyro_deg"]) < 2, f"直行階段偏航變化≈0（得 {st['dyaw_gyro_deg']}°）")
check(near(L["dyaw_amcl_deg"], L["dyaw_gyro_deg"], 8), f"陀螺積分與 AMCL 一致（{L['dyaw_gyro_deg']}° vs {L['dyaw_amcl_deg']}°）→ 沒有陀螺比例問題")
check("GYRO_SCALE" not in r["flags"], "GYRO_SCALE 不亮")

# B2 健康的車
r = run("healthy")
L = seg(r, "left_lock")
check(near(L["g_gyro_over_enc"], 1.0, 0.04), f"健康車：陀螺/編碼器≈1（得 {L['g_gyro_over_enc']}）")
check("STEER_SHORT" not in r["flags"] and "UPLINK_FOLLOWS_COMMAND" not in r["flags"], "健康車：不亮 STEER_SHORT／UPLINK_FOLLOWS_COMMAND")
check(not {"GYRO_SCALE", "LATENCY", "AMCL_LAG", "AMCL_JUMP", "ODOM_RAW_EKF_DIVERGE", "SLOW_RISE"} & set(r["flags"]),
      f"健康車：其他旗標都不亮（實際亮 {sorted(r['flags'])}）")

# B3 EKF 跟編碼器
r = run("ekf_enc", g_left=0.73, g_right=0.63, ekf="enc")
check("EKF_FOLLOWS_ENCODER" in r["flags"] and "EKF_FOLLOWS_GYRO" not in r["flags"], "EKF 跟編碼器 -> EKF_FOLLOWS_ENCODER 亮")
check("ODOM_RAW_EKF_DIVERGE" not in r["flags"], "（此情境 /odom 與 /odom_combined 同步，不該亮 ODOM_RAW_EKF_DIVERGE）")

# B4 延遲
SLALOM = lambda s: 0.6 * (1.0 if int(s / 2.0) % 2 == 0 else -1.0)     # 每 2 秒左右交替
r = run("lag", delay=0.8, tau=0.05, phases=[("slalom", 0.12, SLALOM, 16)])
L = seg(r, "slalom")
check(near(L["lag_cmd_to_gyro_s"], 0.8, 0.12) and "LATENCY" in r["flags"], f"延遲 0.8 s：估得 {L['lag_cmd_to_gyro_s']} s，LATENCY 亮")
r = run("lag_small", delay=0.2, tau=0.05, phases=[("slalom", 0.12, SLALOM, 16)])
check("LATENCY" not in r["flags"], "延遲 0.2 s：LATENCY 不亮")

# B5 陀螺比例錯 25%（AMCL 為真）
r = run("scale", gyro_scale=1.25, g_left=1.0, g_right=1.0)
check("GYRO_SCALE" in r["flags"], "陀螺比例 ×1.25 -> GYRO_SCALE 亮（陀螺積分對 AMCL 偏航不一致）")

# B6 AMCL 跳位與延遲
r = run("jump", amcl_jump_at=10.0)
check("AMCL_JUMP" in r["flags"], "AMCL 在 EKF 沒動的情況下跳 3 m -> AMCL_JUMP 亮")
r = run("amcl_lag", amcl_lag=0.6)
check("AMCL_LAG" in r["flags"], "AMCL header 時間比接收時間早 0.6 s -> AMCL_LAG 亮")
r = run("amcl_ok", amcl_lag=0.1)
check("AMCL_LAG" not in r["flags"] and "AMCL_JUMP" not in r["flags"], "AMCL 延遲 0.1 s、無跳位 -> 兩個旗標都不亮")

# B7 起步上升時間（舵機慢）
r = run("slow", tau=1.2, delay=0.05, phases=[("left_lock", 0.10, 1 / 0.75, 10)])
L = seg(r, "left_lock")
check(L["rise_time_s"] is not None and L["rise_time_s"] > 0.6 and "SLOW_RISE" in r["flags"], f"舵機慢（τ=1.2 s）：上升時間 {L['rise_time_s']} s，SLOW_RISE 亮")
r = run("fast", tau=0.1, delay=0.05, phases=[("left_lock", 0.10, 1 / 0.75, 10)])
check("SLOW_RISE" not in r["flags"], "舵機快（τ=0.1 s）：SLOW_RISE 不亮")

# B8 倒車的符號慣例：倒車 + 右打 -> 車頭逆時針（ω>0）；κ=w/v 為負 = 輪子打右
r = run("rev", phases=[("reverse_right", -0.10, -1 / 1.2, 8)], g_right=0.9)   # w = κ*v = (-0.83)*(-0.10) = +0.083（逆時針）
S = seg(r, "reverse_right")
check(S["kappa_enc_med"] < 0 and S["kappa_gyro_med"] < 0, f"倒車輪子打右：κ=w/v 為負（κ_enc={S['kappa_enc_med']}），與 path_teach._clamp_curv 同一慣例")

# B9 缺資料／空檔／壞列／沒有 marker
r = run("nomark", no_markers=True, g_left=0.8, g_right=0.8)
check(len(r["segments"]) == 1 and r["segments"][0]["label"] == "all", "沒有 marker：整份當一段 'all'")
r = run("nogyro", drop=("imu_unbiased", "imu_raw"))
check(all("g_gyro_over_enc" not in s for s in r["segments"]), "缺陀螺資料：不當機，相關指標留空")
buf = io.StringIO()
A.print_report(r, out=buf)
check("缺少來源" in buf.getvalue(), "報表明講缺了哪些來源（不是印 0）")
empty = os.path.join(tmp, "empty.csv")
Path(empty).write_text("t,stamp,src,label,v,w,x,y,yaw\n", encoding="utf-8")
r = A.analyze(empty)
check(r["segments"] == [] and r["flags"] == {}, "空檔：不當機、沒有假旗標")
r = run("bad", bad_rows=5)
check(r["bad_rows"] == 5, "壞列被計數（5 列）而不是讓整份失敗")
buf = io.StringIO()
A.print_report(run("short2", g_left=0.73, g_right=0.63), out=buf)
check("STEER_SHORT" in buf.getvalue() and "R陀" in buf.getvalue(), "文字報表含旗標說明與 R 欄")

# B10 指令鏈各站曲率：trim 的位移是預期，floor 偷改曲率才是問題
def stage_csv(name, floor_scale):
    """smoothed κ=+0.95 -> trimmed κ=+0.715（trim -0.235）-> prefloor 同 -> final：floor 把 v 抬到 0.1 並把 w 乘 floor_scale*(0.1/v)"""
    path = os.path.join(tmp, name + ".csv")
    rows = ["t,stamp,src,label,v,w,x,y,yaw", "1000.0,,marker,lock,,,,,"]
    for i in range(300):
        t = 1000.0 + i * 0.05
        v = 0.06
        rows.append(f"{t:.3f},,cmd_smoothed,,{v},{0.95 * v:.5f},,,")
        rows.append(f"{t:.3f},,cmd_trimmed,,{v},{0.715 * v:.5f},,,")
        rows.append(f"{t:.3f},,cmd_prefloor,,{v},{0.715 * v:.5f},,,")
        rows.append(f"{t:.3f},,cmd_final,,0.1,{0.715 * 0.1 * floor_scale:.5f},,,")
        rows.append(f"{t:.3f},,odom_raw,,0.1,{0.715 * 0.1:.5f},,,")
        rows.append(f"{t:.3f},,imu_unbiased,,,{0.715 * 0.1:.5f},,,")
    rows.append("1015.0,,marker,end,,,,,")
    Path(path).write_text("\n".join(rows) + "\n", encoding="utf-8")
    return path


r = A.analyze(stage_csv("stage_ok", 1.0))
sk = r["segments"][0]["stage_kappa"]
check(near(sk["cmd_trimmed"] - sk["cmd_smoothed"], -0.235, 0.01), f"各站曲率：trim 使 κ 位移 -0.235（{sk['cmd_smoothed']} → {sk['cmd_trimmed']}）")
check("STAGE_KAPPA_SHIFT" not in r["flags"], "死區地板按比例縮放 ω（κ 不變）-> STAGE_KAPPA_SHIFT 不亮")
r = A.analyze(stage_csv("stage_bad", 0.7))
check("STAGE_KAPPA_SHIFT" in r["flags"], "若地板抬速度卻沒同比縮放 ω（κ 變 0.5）-> STAGE_KAPPA_SHIFT 亮")

# C. model 子命令可跑
buf = io.StringIO()
A.print_model([0.10, 0.25], out=buf)
check("1 mrad/s" in buf.getvalue() and "PWM" in buf.getvalue(), "model 子命令輸出逐站換算表")

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
