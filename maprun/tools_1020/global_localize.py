#!/usr/bin/env python3
"""全圖掃描比對定位：車子停在哪都可以，拿一筆雷射去比整張地圖，找出最吻合的位置與朝向（車不會動）。

★ 2026-10-27 新寫（使用者：「不能自己判斷嗎?」——不想每次把車停回貼膠帶的起點）。
  map_service 的 /align_pose 只在「猜的位置」附近小範圍搜尋；這裡把同一套評分（雷射點落在地圖牆上 ±1 格的比例）
  做到整張圖：所有可站的空格 × 每 5 度，先粗掃、再對前幾名細掃。
  平板的「全域定位」是 AMCL 撒粒子，在走廊會收斂到錯的那一段（8/06 錯 3 m），而且要車子動起來；
  這裡不用動，而且會看「第二名」：兩個差很遠的位置分數差不多（對稱的房間）就**說不確定**，不硬選。

給 localize_here.py --auto 用；也可以單獨離線測：
    python3 global_localize.py --selftest ~/.smartnav/map_database/data/<map_id>.yaml
"""
import math
import os
import sys
import time

import numpy as np

# 雷射相對 base_footprint 的位移（跟 map_service_cc_node._align_pose_with_scan 一樣，robot_model.yaml 的 base_to_laser）
LASER_OFF = (0.08874, 0.00067)
MAX_RANGE = 8.0      # 跟 map_service 一樣只用 8 m 內的點
MIN_RANGE = 0.30     # ★ 車上的鋁擠條、車身離雷射 0.1~0.3 m，地圖上不會有 → 丟掉
ROBOT_CLEAR = 0.15   # 候選位置離牆至少這麼遠（車身內切半徑 0.11 + 餘裕）


# ── 地圖 ──────────────────────────────────────────────────────────────
def _read_pgm(path):
    with open(path, "rb") as fh:
        data = fh.read()
    # P5 標頭：魔術字、寬、高、最大值（中間可能有 # 註解）
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    i += 1
    if tokens[0] != b"P5":
        raise ValueError(f"{path} 不是 P5 PGM")
    w, h, maxv = int(tokens[1]), int(tokens[2]), int(tokens[3])
    dtype = np.uint8 if maxv < 256 else np.dtype(">u2")
    img = np.frombuffer(data, dtype=dtype, count=w * h, offset=i).reshape(h, w)
    return img.astype(np.float64) / maxv


def load_map_yaml(path):
    """nav2 地圖 yaml＋pgm → (grid, res, ox, oy)。grid[gy, gx]：-1 未知、0 空、100 佔據；gy=0 在原點那一邊（跟 OccupancyGrid 一樣）"""
    meta = {}
    for ln in open(path, encoding="utf-8"):
        if ":" in ln and not ln.lstrip().startswith("#"):
            k, v = ln.split(":", 1)
            meta[k.strip()] = v.strip()
    img_path = meta["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(path), img_path)
    img = _read_pgm(img_path)
    negate = int(meta.get("negate", "0"))
    occ_th, free_th = float(meta.get("occupied_thresh", 0.65)), float(meta.get("free_thresh", 0.196))
    p = img if negate else 1.0 - img
    grid = np.full(img.shape, -1, dtype=np.int8)
    grid[p > occ_th] = 100
    grid[p < free_th] = 0
    grid = grid[::-1, :]                       # 影像第 0 列是最上面（y 最大）
    origin = [float(v) for v in meta["origin"].strip("[] ").split(",")[:2]]
    return grid, float(meta["resolution"]), origin[0], origin[1]


def grid_from_msg(msg):
    """nav_msgs/OccupancyGrid → (grid, res, ox, oy)"""
    w, h = msg.info.width, msg.info.height
    grid = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
    grid = np.where(grid < 0, -1, np.where(grid >= 65, 100, np.where(grid <= 25, 0, -1))).astype(np.int8)
    return grid, float(msg.info.resolution), msg.info.origin.position.x, msg.info.origin.position.y


def _shift_or(mask, dx, dy):
    out = np.zeros_like(mask)
    h, w = mask.shape
    ys, yd = (slice(0, h - dy), slice(dy, h)) if dy >= 0 else (slice(-dy, h), slice(0, h + dy))
    xs, xd = (slice(0, w - dx), slice(dx, w)) if dx >= 0 else (slice(-dx, w), slice(0, w + dx))
    out[yd, xd] = mask[ys, xs]
    return out


class MapIndex:
    def __init__(self, grid, res, ox, oy):
        self.res, self.ox, self.oy = res, ox, oy
        self.h, self.w = grid.shape
        occ = grid >= 65
        # 「打中」＝落在佔據格或它上下左右一格（跟 map_service 的 score 一樣）
        hit = occ.copy()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            hit |= _shift_or(occ, dx, dy)
        self.hit = hit
        # 可站的位置：已知空格、離佔據格與未知格至少 ROBOT_CLEAR
        bad = (grid != 0)
        n = max(1, int(math.ceil(ROBOT_CLEAR / res)))
        grow = bad.copy()
        for _ in range(n):
            g2 = grow.copy()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                g2 |= _shift_or(grow, dx, dy)
            grow = g2
        self.free = ~grow

    def candidates(self, step_m):
        k = max(1, int(round(step_m / self.res)))
        gy, gx = np.nonzero(self.free[::k, ::k])
        gy, gx = gy * k, gx * k
        return np.stack([self.ox + (gx + 0.5) * self.res, self.oy + (gy + 0.5) * self.res], axis=1)

    def score(self, xy, yaw, pts):
        """xy：(M,2) 候選位置；yaw：一個角度；pts：(N,2) 雷射座標系的點 → (M,) 打中比例"""
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.array([[c, -s], [s, c]])
        p = (pts + np.asarray(LASER_OFF)) @ rot.T          # 雷射點換到車身座標再轉 yaw
        wx = xy[:, None, 0] + p[None, :, 0]
        wy = xy[:, None, 1] + p[None, :, 1]
        gx = np.floor((wx - self.ox) / self.res).astype(np.int32)
        gy = np.floor((wy - self.oy) / self.res).astype(np.int32)
        ok = (gx >= 0) & (gx < self.w) & (gy >= 0) & (gy < self.h)
        hits = np.zeros(gx.shape, dtype=bool)
        hits[ok] = self.hit[gy[ok], gx[ok]]
        return hits.mean(axis=1)


# ── 雷射 ──────────────────────────────────────────────────────────────
def scan_to_points(ranges, angle_min, angle_inc):
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + np.arange(len(r)) * angle_inc
    ok = np.isfinite(r) & (r >= MIN_RANGE) & (r <= MAX_RANGE)
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)


def _subsample(pts, n):
    if len(pts) <= n:
        return pts
    idx = np.linspace(0, len(pts) - 1, n).round().astype(int)
    return pts[idx]


# ── 搜尋 ──────────────────────────────────────────────────────────────
def global_search(mi, pts, coarse_step=0.15, yaw_step_deg=5.0, n_coarse=120, top_k=24):
    """回傳 (候選清單, 秒數)。候選：[(score, x, y, yaw)]，已細掃、由高到低、彼此相距 > 0.5 m 或 > 25°"""
    t0 = time.monotonic()
    if len(pts) < 30:
        return [], 0.0
    cand = mi.candidates(coarse_step)
    if len(cand) == 0:
        return [], 0.0
    pc = _subsample(pts, n_coarse)
    yaws = np.radians(np.arange(0.0, 360.0, yaw_step_deg))
    sc = np.empty((len(cand), len(yaws)))
    for k, yaw in enumerate(yaws):
        sc[:, k] = mi.score(cand, yaw, pc)
    # 粗掃前幾名（在 位置×朝向 上做非極大值抑制）
    order = np.argsort(sc, axis=None)[::-1]
    picks = []
    for flat in order:
        i, k = np.unravel_index(flat, sc.shape)
        x, y, yaw = cand[i, 0], cand[i, 1], yaws[k]
        if all(math.hypot(x - px, y - py) > 0.3 or _dyaw(yaw, pyaw) > math.radians(20) for _, px, py, pyaw in picks):
            picks.append((sc[i, k], x, y, yaw))
        if len(picks) >= top_k or sc[i, k] < 0.5 * sc.flat[order[0]]:
            break
    # 細掃：每個候選附近 ±0.12 m（3 cm）× ±6°（1°），用比較多的點
    pf = _subsample(pts, 360)
    d = np.arange(-0.12, 0.1201, 0.03)
    gx, gy = np.meshgrid(d, d)
    off = np.stack([gx.ravel(), gy.ravel()], axis=1)
    refined = []
    for _, x, y, yaw in picks:
        best = (-1.0, x, y, yaw)
        xy = np.array([x, y]) + off
        for dyaw in np.radians(np.arange(-6.0, 6.01, 1.0)):
            s = mi.score(xy, yaw + dyaw, pf)
            j = int(np.argmax(s))
            if s[j] > best[0]:
                best = (float(s[j]), float(xy[j, 0]), float(xy[j, 1]), float(_wrap(yaw + dyaw)))
        refined.append(best)
    refined.sort(key=lambda r: -r[0])
    out = []
    for r in refined:
        if all(math.hypot(r[1] - o[1], r[2] - o[2]) > 0.5 or _dyaw(r[3], o[3]) > math.radians(25) for o in out):
            out.append(r)
    return out, time.monotonic() - t0


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _dyaw(a, b):
    return abs(_wrap(a - b))


def verdict(cands, min_score=0.55, min_margin=0.10):
    """(ok, 說明)。ok＝最佳分數夠高、而且明顯贏過第二名（差很遠的另一個位置）"""
    if not cands:
        return False, "沒有可用的雷射點或地圖上沒有可站的位置"
    best = cands[0]
    second = cands[1] if len(cands) > 1 else None
    if best[0] < min_score:
        return False, f"最吻合的位置也只有 {best[0] * 100:.0f}%（要 ≥ {min_score * 100:.0f}%）——地圖可能過期、或周圍被人／東西擋住太多"
    if second is not None and best[0] - second[0] < min_margin:
        far = math.hypot(best[1] - second[1], best[2] - second[2])
        return False, (f"有兩個位置都很像（{best[0] * 100:.0f}% vs {second[0] * 100:.0f}%，相距 {far:.1f} m、"
                       f"朝向差 {math.degrees(_dyaw(best[3], second[3])):.0f}°）——不確定，請把車移到特徵明顯的地方再試，或用地點定位")
    return True, f"吻合 {best[0] * 100:.0f}%（第二名 {second[0] * 100:.0f}%）" if second else f"吻合 {best[0] * 100:.0f}%"


# ── 離線自我測試：用地圖本身模擬雷射 ─────────────────────────────────
def simulate_scan(mi, grid, x, y, yaw, n_beams=450, noise=0.02, rng=None, person=True, occlude=True):
    rng = rng or np.random.default_rng(0)
    occ = grid >= 65
    lx = x + LASER_OFF[0] * math.cos(yaw) - LASER_OFF[1] * math.sin(yaw)
    ly = y + LASER_OFF[0] * math.sin(yaw) + LASER_OFF[1] * math.cos(yaw)
    ang = np.linspace(-math.pi, math.pi, n_beams, endpoint=False)
    steps = np.arange(0.05, 10.0, mi.res * 0.5)
    wa = yaw + ang
    px = lx + np.outer(np.cos(wa), steps)
    py = ly + np.outer(np.sin(wa), steps)
    gx = np.floor((px - mi.ox) / mi.res).astype(int)
    gy = np.floor((py - mi.oy) / mi.res).astype(int)
    ok = (gx >= 0) & (gx < mi.w) & (gy >= 0) & (gy < mi.h)
    hit = np.zeros(gx.shape, dtype=bool)
    hit[ok] = occ[gy[ok], gx[ok]]
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    r = np.where(first >= 0, steps[np.clip(first, 0, None)], np.inf)
    r = r + rng.normal(0, noise, size=r.shape)
    if person:   # 一個人站在 0.6~1.5 m（約 0.45 m 寬）——地圖上沒有
        d = rng.uniform(0.6, 1.5)
        c = rng.uniform(-math.pi, math.pi)
        half = math.atan2(0.225, d)
        m = np.abs(np.angle(np.exp(1j * (ang - c)))) < half
        r[m] = np.minimum(r[m], d)
    if occlude:  # 兩根鋁擠條擋住的扇區（各約 12°）
        for c in (math.radians(115), math.radians(-115)):
            m = np.abs(np.angle(np.exp(1j * (ang - c)))) < math.radians(6)
            r[m] = 0.2
    return r, -math.pi, 2 * math.pi / n_beams


def selftest(map_yaml, n=25, seed=1):
    grid, res, ox, oy = load_map_yaml(map_yaml)
    mi = MapIndex(grid, res, ox, oy)
    rng = np.random.default_rng(seed)
    free = mi.candidates(0.05)
    print(f"地圖 {os.path.basename(map_yaml)}：{grid.shape[1]}×{grid.shape[0]} 格、{res} m/格、可站位置 {len(free)} 個（5 cm）")
    ok_n = right_n = refused = wrong = 0
    errs, secs = [], []
    for t in range(n):
        x, y = free[rng.integers(len(free))]
        yaw = rng.uniform(-math.pi, math.pi)
        r, amin, ainc = simulate_scan(mi, grid, x, y, yaw, rng=rng)
        pts = scan_to_points(r, amin, ainc)
        cands, dt = global_search(mi, pts)
        secs.append(dt)
        ok, why = verdict(cands)
        if not cands:
            refused += 1
            continue
        b = cands[0]
        e_xy, e_yaw = math.hypot(b[1] - x, b[2] - y), math.degrees(_dyaw(b[3], yaw))
        right = e_xy < 0.15 and e_yaw < 5
        if ok:
            ok_n += 1
            right_n += right
            wrong += not right
            errs.append((e_xy, e_yaw))
        else:
            refused += 1
        tag = "✓" if ok and right else ("✗錯" if ok else "？拒")
        print(f"  {t + 1:2d} 真 ({x:5.2f},{y:5.2f},{math.degrees(yaw):5.0f}°) 找到 ({b[1]:5.2f},{b[2]:5.2f},{math.degrees(b[3]):5.0f}°) "
              f"誤差 {e_xy * 100:4.1f} cm/{e_yaw:4.1f}° {tag} {why}  {dt:.1f}s")
    print(f"結果：{n} 次｜判定成功 {ok_n}（其中正確 {right_n}、錯 {wrong}）｜拒絕（不確定）{refused}｜"
          f"每次 {np.median(secs):.1f} s（最多 {max(secs):.1f} s）")
    if errs:
        e = np.array(errs)
        print(f"正確時誤差：位置中位 {np.median(e[:, 0]) * 100:.1f} cm、朝向中位 {np.median(e[:, 1]):.1f}°")
    return wrong == 0


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "--selftest":
        sys.exit(0 if selftest(os.path.expanduser(sys.argv[2]), n=int(sys.argv[3]) if len(sys.argv) > 3 else 25) else 1)
    print(__doc__)
