#!/usr/bin/env python3
"""循跡模擬（★ 2026-10-23 車上 agent）：path_teach_cc 的純追蹤（現行）vs 改良版，在「像這台車」的模型上跑同一批規劃路徑。

車子模型（數字的出處都在記憶筆記／節點註解）：
  - 參考點＝後軸（base_footprint），軸距 0.322 m（廠商 Axle_spacing）
  - 底盤轉向偏移 +0.235 rad/m（8/29 實測偏左 13.46 度/m）；steering_trim 補 trim（殘差 bias_res 可設）
  - 舵機：實際角 = g × 指令角 + 偏移角，一階延遲 tau_s；實際極限 左 R 1.03／右 R 1.183 m（8/29 實測）；韌體指令硬限 R 0.75
  - 指令延遲 1 拍、速度一階延遲 0.3 s、死區 0.085 m/s
  - 定位：控制器看到的位姿 = 真值 + AMCL 修正（每走 10 cm 換一次，σ amcl_xy／amcl_yaw）+ 每拍白雜訊
控制器（照 path_teach_cc 現行預設寫）：
  - 15.4 Hz（8/24 實測）、_closest_index 不跨折返、折返停 0.6 s、終點 0.20 m、偏離 0.60 m 中止
  - 前視 ld = clamp((1.2|v| + 0.30) × (倒車 ×1.8), 0.30, 0.80)、後軸參考、曲率箝制 左 1/1.05／右 1/1.20
  - mode base   ：目標＝第一個距離 ≥ ld 的路徑點（現行）
  - mode interp ：目標＝沿路徑內插到剛好 ld 的位置
  - mode pi     ：interp ＋ 橫向誤差積分（以「走過的距離」積分、前進／倒車各一個積分器、抗飽和）
評分用真值位姿到「目前這一段（同方向）折線」的距離，不是到最近頂點。
"""
import math
import random

L_AXLE = 0.322
BIAS0 = 0.235                     # 底盤原始偏移（rad/m，正＝偏左）
R_PHYS_L, R_PHYS_R = 1.03, 1.183  # 實際做得到的最小半徑（8/29）
R_FW = 0.75                       # 韌體指令硬限
R_CMD_L, R_CMD_R = 1.05, 1.20     # path_teach 的箝制
DEADBAND = 0.085


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


# ───────────────────────── 路徑工具 ─────────────────────────

def segments(pts):
    """同方向的連續段 [(a, b)]：頂點 a..b-1 的 d 相同；這一段的折線一直畫到 pts[b]（折返點或終點）。"""
    out, s = [], 0
    for i in range(1, len(pts)):
        if pts[i][3] != pts[i - 1][3]:
            out.append((s, i))
            s = i
    out.append((s, len(pts)))
    return out


def seg_of(segs, idx):
    for a, b in segs:
        if a <= idx < b:
            return a, b
    return segs[-1]


def project(pts, a, b, x, y, lo, hi):
    """(x, y) 投影到這一段折線 pts[i]→pts[i+1]（i ∈ [max(a,lo), min(b,hi))）。
    回傳 (帶號橫向誤差：行進方向的左邊為正, 線段起點索引)"""
    best_d2, best_e, best_i = float("inf"), 0.0, max(a, lo)
    for i in range(max(a, lo), max(max(a, lo) + 1, min(b, hi))):
        j = i + 1
        if j >= len(pts):
            break
        x0, y0, x1, y1 = pts[i][0], pts[i][1], pts[j][0], pts[j][1]
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        if L2 < 1e-12:
            continue
        t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / L2))
        px, py = x0 + t * dx, y0 + t * dy
        d2 = (x - px) ** 2 + (y - py) ** 2
        if d2 < best_d2:
            best_d2, best_i = d2, i
            best_e = (dx * (y - py) - dy * (x - px)) / math.sqrt(L2)
    return best_e, best_i


class Controller:
    def __init__(self, mode="base", ki_dist=1.5, ki_max=0.15, i_gate=0.25, rate=15.4,
                 la_k=1.2, la_min=0.30, la_max=0.80, rev_scale=1.8, v_fwd=0.15, v_rev=0.10):
        self.mode, self.ki_dist, self.ki_max, self.i_gate = mode, ki_dist, ki_max, i_gate
        self.dt = 1.0 / rate
        self.la_k, self.la_min, self.la_max, self.rev_scale = la_k, la_min, la_max, rev_scale
        self.v_fwd, self.v_rev = v_fwd, v_rev
        self.integ = {1: 0.0, -1: 0.0}
        self.prev_xy = None
        self.sat_ticks = 0

    def lookahead(self, v, direction):
        s = self.rev_scale if direction < 0 else 1.0
        return max(self.la_min, min(self.la_max, (self.la_k * abs(v) + self.la_min) * s))

    @staticmethod
    def closest_index(pts, x, y, hint):     # = path_teach._closest_index
        limit = min(len(pts), hint + 120)
        d0 = pts[hint][3]
        for i in range(hint, limit):
            if pts[i][3] != d0:
                limit = i + 1
                break
        bi, bd = hint, float("inf")
        for i in range(hint, limit):
            d = (pts[i][0] - x) ** 2 + (pts[i][1] - y) ** 2
            if d < bd:
                bd, bi = d, i
        return bi

    @staticmethod
    def la_vertex(pts, start, x, y, ld):    # = path_teach._lookahead_point
        d0 = pts[start][3]
        i = start
        while i < len(pts) - 1:
            if pts[i][3] != d0:
                return pts[i]
            if math.hypot(pts[i][0] - x, pts[i][1] - y) >= ld:
                return pts[i]
            i += 1
        return pts[-1]

    @staticmethod
    def la_interp(pts, seg_i, x, y, ld):
        """從投影所在線段往前，找這一段折線上第一個與 (x, y) 距離剛好 ld 的點；
        走到折返點（下一個方向的第一點）或終點還找不到就回傳那一點——與 la_vertex 停在折返點同一個規則。"""
        d0 = pts[seg_i][3]
        i = seg_i
        while i < len(pts) - 1 and pts[i][3] == d0:
            x0, y0, x1, y1 = pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1]
            if math.hypot(x1 - x, y1 - y) >= ld:
                dx, dy = x1 - x0, y1 - y0
                fx, fy = x0 - x, y0 - y
                A = dx * dx + dy * dy
                B = 2 * (fx * dx + fy * dy)
                C = fx * fx + fy * fy - ld * ld
                disc = B * B - 4 * A * C
                if A > 1e-12 and disc >= 0:
                    t = max(0.0, min(1.0, (-B + math.sqrt(disc)) / (2 * A)))
                    return (x0 + t * dx, y0 + t * dy, pts[i][2], d0)
                return pts[i + 1]
            i += 1
        return pts[i]

    @staticmethod
    def clamp_curv(k, direction):           # = path_teach._clamp_curv
        left = (k >= 0.0) if direction >= 0 else (k <= 0.0)
        cap = 1.0 / (R_CMD_L if left else R_CMD_R)
        return max(-cap, min(cap, k)), abs(k) >= cap

    def command(self, pts, idx, segs, pose, direction):
        """回傳 (v, w)。曲率都在「行進座標」裡算（倒車時鏡射），與 path_teach._pure_pursuit 相同。"""
        x, y, yaw = pose
        v = self.v_fwd if direction > 0 else self.v_rev
        ld = self.lookahead(v, direction)
        a, b = seg_of(segs, idx)
        if self.mode == "base":
            tgt = self.la_vertex(pts, idx, x, y, ld)
        else:
            e, si = project(pts, a, b, x, y, idx - 2, idx + 3)
            tgt = self.la_interp(pts, si, x, y, ld)
        dx, dy = tgt[0] - x, tgt[1] - y
        c, s = math.cos(-yaw), math.sin(-yaw)
        bx, by = dx * c - dy * s, dx * s + dy * c
        ldd = math.hypot(bx, by)
        if ldd < 1e-3:
            return 0.0, 0.0
        if direction < 0:
            bx, by = -bx, -by
        k = 2.0 * by / (ldd * ldd)
        if self.mode == "pi":
            ds = 0.0 if self.prev_xy is None else math.hypot(x - self.prev_xy[0], y - self.prev_xy[1])
            self.prev_xy = (x, y)
            ki = (2.0 / (ld * ld)) / max(0.2, self.ki_dist)   # 純追蹤等效位置增益 ÷ 積分距離
            _kc, sat = self.clamp_curv(k - ki * self.integ[direction], direction)
            if abs(e) < self.i_gate and not sat and ds < 0.1:  # 抗飽和：誤差大交給純追蹤、飽和不積
                self.integ[direction] += e * ds
            lim = self.ki_max / ki
            self.integ[direction] = max(-lim, min(lim, self.integ[direction]))
            k -= ki * self.integ[direction]
        k, sat = self.clamp_curv(k, direction)
        self.sat_ticks += sat
        vv = v if direction > 0 else -v
        return vv, abs(vv) * k


class Vehicle:
    def __init__(self, x, y, yaw, g=1.0, bias_res=0.0, tau_s=0.20, tau_v=0.30, delay_ticks=1):
        self.x, self.y, self.yaw = x, y, yaw
        self.v, self.delta = 0.0, 0.0
        self.g, self.tau_s, self.tau_v = g, tau_s, tau_v
        self.trim = -(BIAS0 - bias_res)                   # steering_trim 的值；bias_res＝補不到的殘差（正＝還會偏左）
        self.dbias = math.atan(BIAS0 * L_AXLE)
        self.dmax_l = math.atan(L_AXLE / R_PHYS_L)
        self.dmax_r = math.atan(L_AXLE / R_PHYS_R)
        self.queue = [(0.0, 0.0)] * delay_ticks
        self.applied = (0.0, 0.0)

    def set_cmd(self, v_cmd, w_cmd):
        self.queue.append((v_cmd, w_cmd))
        self.applied = self.queue.pop(0)

    def integrate(self, dt):
        v_cmd, w_cmd = self.applied
        w_cmd = w_cmd + self.trim * v_cmd                 # steering_trim_cc：ω += trim × v
        if abs(v_cmd) < DEADBAND:
            v_target, d_target = 0.0, self.delta          # 停著：舵角不動
        else:
            v_target = v_cmd
            kb = max(-1.0 / R_FW, min(1.0 / R_FW, w_cmd / v_cmd))   # 韌體：R = Vx / Vz，硬限 0.75 m
            d_target = max(-self.dmax_r, min(self.dmax_l, self.g * math.atan(L_AXLE * kb) + self.dbias))
        self.delta += (d_target - self.delta) * min(1.0, dt / self.tau_s)
        self.v += (v_target - self.v) * min(1.0, dt / self.tau_v)
        kb = math.tan(self.delta) / L_AXLE
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = norm(self.yaw + self.v * kb * dt)


class Localizer:
    """控制器看到的位姿：AMCL 每走 10 cm 換一次修正量（一階自迴歸，穩態 σ＝設定值），再加每拍白雜訊。"""
    def __init__(self, rng, amcl_xy=0.02, amcl_yaw_deg=1.0, white_xy=0.003, white_yaw_deg=0.2):
        self.rng, self.sxy, self.syaw = rng, amcl_xy, math.radians(amcl_yaw_deg)
        self.wxy, self.wyaw = white_xy, math.radians(white_yaw_deg)
        self.off = [0.0, 0.0, 0.0]
        self.travel, self.last = 0.0, None

    def see(self, veh):
        if self.last is not None:
            self.travel += math.hypot(veh.x - self.last[0], veh.y - self.last[1])
        self.last = (veh.x, veh.y)
        if self.travel >= 0.10:
            self.travel = 0.0
            a = 0.7
            q = math.sqrt(1 - a * a)
            self.off = [a * self.off[0] + self.rng.gauss(0, self.sxy * q),
                        a * self.off[1] + self.rng.gauss(0, self.sxy * q),
                        a * self.off[2] + self.rng.gauss(0, self.syaw * q)]
        return (veh.x + self.off[0] + self.rng.gauss(0, self.wxy),
                veh.y + self.off[1] + self.rng.gauss(0, self.wxy),
                norm(veh.yaw + self.off[2] + self.rng.gauss(0, self.wyaw)))


def run(pts, ctrl, veh_kw=None, loc_kw=None, seed=0, start_err=(0.0, 0.0, 0.0), t_max=None):
    """跑一條路徑。回傳 dict：ok、why、t、errs（每拍 |e|，真值到同方向段折線）、node_xte（節點現行算法：到最近頂點）"""
    rng = random.Random(seed)
    veh = Vehicle(pts[0][0] + start_err[0], pts[0][1] + start_err[1], norm(pts[0][2] + start_err[2]),
                  **(veh_kw or {}))
    loc = Localizer(rng, **(loc_kw or {}))
    segs = segments(pts)
    idx, cur_dir = 0, pts[0][3]
    dt = ctrl.dt
    length = sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(pts, pts[1:]))
    t_max = t_max or (length / 0.08 + 3.0 * len(segs) + 20.0)
    t, errs, node_xte, pause = 0.0, [], [], 0.0
    while t < t_max:
        pose = loc.see(veh)
        idx = ctrl.closest_index(pts, pose[0], pose[1], idx)
        a, b = seg_of(segs, idx)
        e_true, _i = project(pts, a, b, veh.x, veh.y, idx - 3, idx + 4)
        errs.append(abs(e_true))
        lo, hi = max(0, idx - 60), min(len(pts), idx + 61)
        node_xte.append(math.sqrt(min((p[0] - pose[0]) ** 2 + (p[1] - pose[1]) ** 2 for p in pts[lo:hi])))
        if node_xte[-1] > 0.60:
            return dict(ok=False, why="偏離 0.60 m 中止", t=t, errs=errs, node_xte=node_xte)
        if idx >= len(pts) - 3 and math.hypot(pts[-1][0] - pose[0], pts[-1][1] - pose[1]) <= 0.20:
            return dict(ok=True, why="到達", t=t, errs=errs, node_xte=node_xte)
        if pause > 0.0:
            v, w = 0.0, 0.0
            pause -= dt
        elif pts[idx][3] != cur_dir:
            cur_dir = pts[idx][3]
            pause = 0.6 + 5 * dt                          # _stop(5) + cusp_pause
            ctrl.prev_xy = None
            v, w = 0.0, 0.0
        else:
            v, w = ctrl.command(pts, idx, segs, pose, cur_dir)
        veh.set_cmd(v, w)
        for _ in range(4):                                # 每拍細分 4 步積分
            veh.integrate(dt / 4)
        t += dt
    return dict(ok=False, why="逾時", t=t, errs=errs, node_xte=node_xte)
