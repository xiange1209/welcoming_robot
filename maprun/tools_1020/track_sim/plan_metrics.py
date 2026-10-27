"""規劃路徑的品質指標（★ 2026-10-23 車上 agent）。點格式 [x, y, yaw, d]（d：+1 前進／-1 倒車，path_teach 的標法）。
path_teach_cc 的 plan_select（多候選擇優）用的是同一套算法——兩邊改了要一起改。"""
import math

R_LEFT, R_RIGHT, MARGIN, WIN = 1.05, 1.20, 0.95, 0.15   # path_teach 的預設值

# 分數權重（越小越好，單位「相當於多走幾公尺」）——依據見 path_teach_cc 的 plan_select 說明
W_INFEASIBLE_M = 10.0     # 每公尺「車子轉不過去」的弧長
W_KINK = 3.0              # 每個朝向折角 > 15° 的折返點
W_MICRO = 2.0             # 每段 < 0.30 m 的微折返（原地鋸齒轉向）
W_CUSP = 0.8              # 每個折返點（停車換向）
W_REVERSE_M = 1.0         # 倒車每公尺額外再算一公尺


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def annotate(pts):
    """path_teach._annotate_directions"""
    for i in range(len(pts) - 1):
        fwd = (pts[i + 1][0] - pts[i][0]) * math.cos(pts[i][2]) + (pts[i + 1][1] - pts[i][1]) * math.sin(pts[i][2])
        if abs(fwd) > 1e-3:
            pts[i][3] = 1 if fwd > 0 else -1
        elif i > 0:
            pts[i][3] = pts[i - 1][3]
    if len(pts) >= 2:
        pts[-1][3] = pts[-2][3]
    return pts


def geometric_infeasible(pts):
    """xy 三點外接圓（點距 ≥ 0.10 m、不跨折返）：回傳 (做不到的弧長 m, 最緊半徑)"""
    segs, s = [], 0
    for i in range(1, len(pts)):
        if pts[i][3] != pts[i - 1][3]:
            segs.append((s, i))
            s = i
    segs.append((s, len(pts)))
    bad_len, worst = 0.0, float("inf")
    for a, b in segs:
        sp = [pts[a]]
        for p in pts[a + 1:min(b + 1, len(pts))]:
            if math.hypot(p[0] - sp[-1][0], p[1] - sp[-1][1]) >= 0.10:
                sp.append(p)
        for p0, p1, p2 in zip(sp, sp[1:], sp[2:]):
            ax, ay, bx, by = p1[0] - p0[0], p1[1] - p0[1], p2[0] - p1[0], p2[1] - p1[1]
            cross = ax * by - ay * bx
            A, B, C = math.hypot(ax, ay), math.hypot(bx, by), math.hypot(p2[0] - p0[0], p2[1] - p0[1])
            if abs(cross) < 1e-9 or C < 0.5 * (A + B):
                continue
            r = A * B * C / (2 * abs(cross))
            worst = min(worst, r)
            left = (cross > 0) if p0[3] > 0 else (cross < 0)
            if r < (R_LEFT if left else R_RIGHT) * MARGIN:
                bad_len += 0.5 * (A + B)
    return bad_len, worst


def cusp_quality(pts):
    """(折返點數, 幾何朝向跳變 > 15° 的折返點數, 微折返段數：兩個折返點之間 < 0.30 m)"""
    cusps, kinked = [], 0
    for c in range(1, len(pts) - 1):
        if pts[c][3] == pts[c - 1][3]:
            continue
        cusps.append(c)
        din = math.atan2(pts[c][1] - pts[c - 1][1], pts[c][0] - pts[c - 1][0])
        dout = math.atan2(pts[c + 1][1] - pts[c][1], pts[c + 1][0] - pts[c][0])
        hin = din if pts[c - 1][3] > 0 else din + math.pi
        hout = dout if pts[c][3] > 0 else dout + math.pi
        kinked += abs(math.degrees(norm(hout - hin))) > 15.0
    micro = 0
    for c0, c1 in zip(cusps, cusps[1:]):
        if sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(c0, c1)) < 0.30:
            micro += 1
    return len(cusps), kinked, micro


def lengths(pts):
    length = rev = 0.0
    for p, q in zip(pts, pts[1:]):
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        length += d
        if p[3] < 0:
            rev += d
    return length, rev


def score(pts):
    """回傳 (分數, 明細 dict)"""
    length, rev = lengths(pts)
    bad_len, worst = geometric_infeasible(pts)
    nc, kinked, micro = cusp_quality(pts)
    s = (length + W_REVERSE_M * rev + W_INFEASIBLE_M * bad_len + W_KINK * kinked + W_MICRO * micro + W_CUSP * nc)
    return s, dict(長=round(length, 2), 倒車=round(rev, 2), 做不到m=round(bad_len, 2), 折返=nc, 折角=kinked,
                   微折返=micro, 最緊半徑=round(worst, 2) if worst < 1e9 else None)
