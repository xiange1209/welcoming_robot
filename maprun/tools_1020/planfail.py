#!/usr/bin/env python3
"""唯讀：規劃路徑 × 地圖 × 現在的雷射。用法: planfail.py <path_id> <out.png>"""
import json, math, pathlib, struct, sys, time, zlib
import numpy as np

H = pathlib.Path.home() / ".smartnav"
pid, out_png = sys.argv[1], sys.argv[2]
paths = json.loads((H / "path_database/paths.json").read_text())["paths"]
pts = paths[pid]["points"]
nav = json.loads((H / "nav_state.json").read_text())["last_active_map"]
yml = (H / f"map_database/data/{nav}.yaml").read_text()
origin = json.loads(next(l for l in yml.splitlines() if l.startswith("origin")).split(":", 1)[1].replace("0]", "0]"))
RES = 0.05; OX, OY = origin[0], origin[1]
raw = (H / f"map_database/data/{nav}.pgm").read_bytes()
toks, i = [], 0
while len(toks) < 4:
    while raw[i:i + 1].isspace(): i += 1
    if raw[i:i + 1] == b"#":
        while raw[i:i + 1] != b"\n": i += 1
        continue
    j = i
    while not raw[j:j + 1].isspace(): j += 1
    toks.append(raw[i:j]); i = j
i += 1
W_, H_ = int(toks[1]), int(toks[2])
img = np.frombuffer(raw[i:i + W_ * H_], dtype=np.uint8).reshape(H_, W_)
occ = img <= 10
free = img >= 250
oy_idx, ox_idx = np.where(occ)
OCX = OX + (ox_idx + 0.5) * RES                 # 佔據格中心（地圖座標）
OCY = OY + (H_ - 1 - oy_idx + 0.5) * RES
REAR, FRONT, HALFW = -0.09, 0.40, 0.185          # base_footprint 在後輪軸心


def body_margin(x, y, yaw, px=OCX, py=OCY):
    """點雲到車身矩形的距離（< 0 不會出現；0＝壓到）。與 path_teach._body_margin 同一套幾何。"""
    dx, dy = px - x, py - y
    c, s = math.cos(yaw), math.sin(yaw)
    cx = c * dx + s * dy
    cy = -s * dx + c * dy
    ox = np.maximum(np.maximum(REAR - cx, cx - FRONT), 0.0)
    oy = np.maximum(np.abs(cy) - HALFW, 0.0)
    d = np.hypot(ox, oy)
    k = int(np.argmin(d))
    return float(d[k]), (float(cx[k]), float(cy[k]))


print(f"路徑 {pid}：{len(pts)} 點，地圖 {nav}（{W_}x{H_} @{RES}，佔據格 {int(occ.sum())}）")
cusps = [k for k in range(1, len(pts)) if pts[k]["d"] != pts[k - 1]["d"]]
print(f"  折返（d 變號）在索引 {cusps}（共 {len(cusps)} 個）")
p0, pn = pts[0], pts[-1]
print(f"  起點 ({p0['x']:+.2f},{p0['y']:+.2f}) yaw {math.degrees(p0['yaw']):+.0f}° d={p0['d']}   終點 ({pn['x']:+.2f},{pn['y']:+.2f}) yaw {math.degrees(pn['yaw']):+.0f}°")

marg = []
for k, p in enumerate(pts):
    m, _ = body_margin(p["x"], p["y"], p["yaw"])
    marg.append(m)
marg = np.array(marg)
tight = [k for k, m in enumerate(marg) if m < 0.06]
near = [k for k, m in enumerate(marg) if 0.06 <= m < 0.15]
print("\n── 這條路徑『本身』離地圖上的牆多近（車身矩形到最近佔據格中心；格子 5 cm，誤差約 ±2.5 cm）──")
print(f"  餘裕 < 6 cm（＝重播時一定會觸發『外緣煞停』的位置）: 索引 {tight or '無'}")
print(f"  餘裕 6~15 cm: 索引 {near or '無'}")
print(f"  最小餘裕 {marg.min()*100:.1f} cm @索引 {int(marg.argmin())}；起點(索引0) {marg[0]*100:.1f} cm")
print("  逐點(索引:cm) " + " ".join(f"{k}:{m*100:.0f}" for k, m in enumerate(marg)))

print("\n── 我自己算的曲率（同一方向段內 R = 弧長/朝向差；門檻 左1.00／右1.14）──")
bad = []
for k in range(1, len(pts)):
    a, b = pts[k - 1], pts[k]
    if a["d"] != b["d"]:
        continue
    ds = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
    dy = (b["yaw"] - a["yaw"] + math.pi) % (2 * math.pi) - math.pi
    if abs(dy) < 1e-3 or ds < 1e-3:
        continue
    R = ds / abs(dy)
    side = "左" if dy > 0 else "右"
    gate = 1.00 if dy > 0 else 1.14
    if R < gate:
        bad.append((k - 1, k, side, ds, math.degrees(abs(dy)), R))
for a_, b_, side, ds, dd, R in bad:
    print(f"  索引 {a_}~{b_}：{ds:.3f} m 內往{side}轉 {dd:.1f}° → 需要半徑 {R:.3f} m")
print(f"  共 {len(bad)} 段低於門檻（log 說 5 處）")

# ── 現在的車與雷射 ──
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
rclpy.init()
n = Node("planfail_cc")
buf = Buffer(); TransformListener(buf, n)
scans = []
n.create_subscription(LaserScan, "/scan", lambda m: scans.append(m), qos_profile_sensor_data)
t0 = time.time()
tf = None
while time.time() - t0 < 8 and (not scans or tf is None):
    rclpy.spin_once(n, timeout_sec=0.1)
    try:
        tf = buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
    except Exception:
        pass
if tf is None or not scans:
    print("\n（現在查不到 TF 或 /scan，略過）"); sys.exit(0)
q = tf.transform.rotation
car_yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)
cx0, cy0 = tf.transform.translation.x, tf.transform.translation.y
sc = scans[-1]
a = sc.angle_min + np.arange(len(sc.ranges)) * sc.angle_increment
r = np.array(sc.ranges, dtype=float)
ok = np.isfinite(r) & (r > sc.range_min) & (r < min(sc.range_max, 8.0))
LX, LY = 0.08874, 0.00067
bx = LX + r[ok] * np.cos(a[ok]); by = LY + r[ok] * np.sin(a[ok])      # 車體座標
mx = cx0 + math.cos(car_yaw) * bx - math.sin(car_yaw) * by              # 地圖座標
my = cy0 + math.sin(car_yaw) * bx + math.cos(car_yaw) * by
gx = ((mx - OX) / RES).astype(int); gy = (H_ - 1 - ((my - OY) / RES)).astype(int)
inside = (gx >= 1) & (gx < W_ - 1) & (gy >= 1) & (gy < H_ - 1)
hit = np.zeros(len(mx), dtype=bool)
for dx_, dy_ in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)):
    hit |= inside & occ[np.clip(gy + dy_, 0, H_ - 1), np.clip(gx + dx_, 0, W_ - 1)]
print("\n── 車子現在（14:5x 失敗後停的位置）──")
print(f"  map→base_footprint ({cx0:+.2f},{cy0:+.2f}) yaw {math.degrees(car_yaw):+.0f}°")
dists = [math.hypot(cx0 - p["x"], cy0 - p["y"]) for p in pts]
k = int(np.argmin(dists))
print(f"  離規劃路徑最近的點：索引 {k}（相距 {dists[k]:.2f} m；該點朝向 {math.degrees(pts[k]['yaw']):+.0f}°，d={pts[k]['d']}）")
print(f"  掃描吻合度 {100*hit[inside].mean():.0f}%（{int(inside.sum())} 束；與地圖對不上的 {int((~hit & inside).sum())} 束）")
# 四向餘裕（車體座標，以車身矩形為準）
def sector_margin(lo, hi):
    ang = np.degrees(np.arctan2(by, bx)) % 360
    m = (ang >= lo) & (ang <= hi) if lo <= hi else ((ang >= lo) | (ang <= hi))
    if not m.any(): return None
    cxs, cys = bx[m], by[m]
    ox = np.maximum(np.maximum(REAR - cxs, cxs - FRONT), 0.0); oy = np.maximum(np.abs(cys) - HALFW, 0.0)
    return float(np.hypot(ox, oy).min())
for name, (lo, hi) in {"正前 ±30°": (330, 30), "左前 30~90°": (30, 90), "左後 90~150°": (90, 150), "正後 150~210°": (150, 210),
                       "右後 210~270°": (210, 270), "右前 270~330°": (270, 330)}.items():
    v = sector_margin(lo, hi)
    print(f"  {name:<12} 車身外緣最近 {'—' if v is None else f'{v*100:.0f} cm'}")
# 地圖上沒有、但雷射打到的近距離東西（車體座標）
unm = (~hit) & inside & (np.hypot(bx, by) < 1.2)
if unm.any():
    ang = np.degrees(np.arctan2(by[unm], bx[unm])) % 360
    print(f"  近 1.2 m 內『地圖上沒有』的回波 {int(unm.sum())} 束；方位(車體座標,0=正前,逆時針) {np.percentile(ang,10):.0f}°~{np.percentile(ang,90):.0f}°，距離 {np.hypot(bx[unm],by[unm]).min():.2f}~{np.hypot(bx[unm],by[unm]).max():.2f} m")
else:
    print("  近 1.2 m 內沒有『地圖上沒有』的回波 → 車旁的東西都在地圖裡（＝是牆／已建的障礙，不是新東西）")
# 車現在的位置離地圖障礙多近
m_now, (cxn, cyn) = body_margin(cx0, cy0, car_yaw)
print(f"  以地圖算，車身離最近佔據格 {m_now*100:.1f} cm（在車體座標 前{cxn:+.2f} 左{cyn:+.2f} m 處）")

# ── 圖 ──
S = 10
x0 = min(min(p["x"] for p in pts), cx0) - 1.0; x1 = max(max(p["x"] for p in pts), cx0) + 1.0
y0 = min(min(p["y"] for p in pts), cy0) - 1.0; y1 = max(max(p["y"] for p in pts), cy0) + 1.0
Wp, Hp = int((x1 - x0) * S / (RES * 1.0) * RES) + 1, int((y1 - y0) * S / (RES * 1.0) * RES) + 1
Wp, Hp = int((x1 - x0) / RES * S / 5) + 1, int((y1 - y0) / RES * S / 5) + 1
pxm = lambda x: int((x - x0) / RES * S / 5)
pym = lambda y: Hp - 1 - int((y - y0) / RES * S / 5)
canvas = np.full((Hp, Wp, 3), 205, np.uint8)
for yy in range(Hp):
    for xx in range(0, Wp, 1):
        pass
# 逐像素取樣地圖
ys_, xs_ = np.mgrid[0:Hp, 0:Wp]
mxw = x0 + xs_ * (RES * 5 / S); myw = y0 + (Hp - 1 - ys_) * (RES * 5 / S)
gxx = np.clip(((mxw - OX) / RES).astype(int), 0, W_ - 1); gyy = np.clip((H_ - 1 - (myw - OY) / RES).astype(int), 0, H_ - 1)
canvas[free[gyy, gxx]] = 255
canvas[occ[gyy, gxx]] = 0
def dot(x, y, col, r=3):
    cxp, cyp = pxm(x), pym(y)
    canvas[max(cyp - r, 0):cyp + r + 1, max(cxp - r, 0):cxp + r + 1] = col
def line(xa, ya, xb, yb, col):
    n_ = int(max(abs(pxm(xb) - pxm(xa)), abs(pym(yb) - pym(ya)))) + 1
    for t in np.linspace(0, 1, n_):
        xx, yy = pxm(xa + (xb - xa) * t), pym(ya + (yb - ya) * t)
        if 0 <= xx < Wp and 0 <= yy < Hp: canvas[yy, xx] = col
for k in range(1, len(pts)):
    line(pts[k - 1]["x"], pts[k - 1]["y"], pts[k]["x"], pts[k]["y"], (0, 90, 255) if pts[k]["d"] > 0 else (255, 140, 0))
for k, p in enumerate(pts):
    dot(p["x"], p["y"], (220, 0, 0) if marg[k] < 0.06 else ((255, 200, 0) if marg[k] < 0.15 else (0, 160, 0)), 2)
dot(p0["x"], p0["y"], (255, 0, 255), 5); dot(pn["x"], pn["y"], (0, 0, 255), 5)
dot(pts[22]["x"], pts[22]["y"], (0, 0, 0), 5) if len(pts) > 22 else None
for xx_, yy_ in zip(mx[::2], my[::2]):
    dot(xx_, yy_, (0, 200, 200), 1)
for L in range(0, 60):    # 車身矩形
    for edge in ((REAR + (FRONT - REAR) * L / 59, -HALFW), (REAR + (FRONT - REAR) * L / 59, HALFW)):
        ex, ey = edge
        dot(cx0 + math.cos(car_yaw) * ex - math.sin(car_yaw) * ey, cy0 + math.sin(car_yaw) * ex + math.cos(car_yaw) * ey, (0, 0, 255), 1)
for L in range(0, 20):
    for edge in ((REAR, -HALFW + 2 * HALFW * L / 19), (FRONT, -HALFW + 2 * HALFW * L / 19)):
        ex, ey = edge
        dot(cx0 + math.cos(car_yaw) * ex - math.sin(car_yaw) * ey, cy0 + math.sin(car_yaw) * ex + math.cos(car_yaw) * ey, (0, 0, 255), 1)
fx, fy = FRONT + 0.15, 0.0
line(cx0, cy0, cx0 + math.cos(car_yaw) * 0.6, cy0 + math.sin(car_yaw) * 0.6, (255, 0, 0))
def png(arr):
    hh, ww, _ = arr.shape
    rows = b"".join(b"\x00" + arr[y].tobytes() for y in range(hh))
    def ch(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    return b"\x89PNG\r\n\x1a\n" + ch(b"IHDR", struct.pack(">IIBBBBB", ww, hh, 8, 2, 0, 0, 0)) + ch(b"IDAT", zlib.compress(rows, 6)) + ch(b"IEND", b"")
pathlib.Path(out_png).parent.mkdir(parents=True, exist_ok=True)
pathlib.Path(out_png).write_bytes(png(canvas))
print(f"\n圖：{out_png}（{Wp}x{Hp}）。藍線＝前進段、橘線＝倒車段；點：綠＝餘裕>15cm、黃＝6~15cm、紅＝<6cm；"
      "洋紅＝路徑起點、藍＝終點、黑圈＝索引22；青點＝現在的雷射；藍框＝車身(現在)，紅線＝車頭方向")
n.destroy_node(); rclpy.shutdown()
