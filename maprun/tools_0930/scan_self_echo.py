#!/usr/bin/env python3
"""自身回波量測（2026-09-30）—— 改 self_mask_sectors_deg 之前先量，不要猜。

★ 背景：9/29 四條重播全在 5 秒內進脫困（nav_20260929_160257.log:675-681）。
  path_teach 用**原始 /scan** 量到落在車身矩形內的點 → 側面 0 cm、外緣 −1 cm 硬停
  → 車子沒動 5 秒 → 卡住偵測判「有指令沒動」→ 脫困又被負淨空中止。
  掃描沒有過期（stale 警告 0 次），所以那些點是真的量到的。
  嫌疑：架高相機的鋁擠條（兩側；遮罩只給了 /scan_slam，path_teach／局部成本地圖／
  collision_monitor 吃的是原始 /scan），以及前方另有「前 0.05 m」讀數（:808、:898）——
  9/23 只看了兩個側向扇區，**光遮兩側可能不夠**。

做法：車子停在**空曠處**（四周 1 m 內沒東西、人走開），靜止錄 N 幀原始 /scan，逐度統計：
  - near 比例：這一度有「有效回波且 r < --near」的幀數 / 總幀數
      → 比例 > --thresh 的連續角度 = 自身回波扇區 → 建議的 self_mask_sectors_deg
  - body 比例：這一度有點落在 path_teach 車身矩形內（_body_margin 同一套幾何）的幀數 / 總幀數
      → **只要 > 0 就列出來**：這些方向就是讓 path_teach 印「0 cm」或硬停的來源，偶發也算
★ 建議換個朝向（原地轉約 90 度）再量一次：自身回波跟著車子轉、角度不變；環境裡的東西會換角度。

輸出（collect_data.sh 會把 verify_out/ 整個帶回）：
  ~/maprun/verify_out/<時間>_self_echo/
    summary.txt             人看的結論（也會印在螢幕上）
    scan_self_echo.tsv      逐度統計
    scan_self_echo_raw.csv  原始幀（回筆電可用 --from-csv 換門檻重算，不用再上車）

用法：
  source ~/maprun/env.sh                      # 沒 source 會收不到 /scan（CycloneDDS、LOCALHOST）
  ~/maprun/tools_0930/scan_self_echo.py       # 預設 100 幀，N10 約 10 秒
  ~/maprun/tools_0930/scan_self_echo.py --frames 300 --near 0.5 --thresh 0.5
  python scan_self_echo.py --from-csv scan_self_echo_raw.csv --near 0.3   # 離線重算（不需要 ROS）

前置：底盤＋雷達要開（平板「系統開關 → 底盤 + 雷達 + IMU」）。只訂閱、不發任何指令，車子不會動。
"""
import argparse
import csv
import math
import os
import statistics
import sys
import time
from datetime import datetime

# ── 車身矩形：照抄 path_teach_cc_node._body_margin（:1776-1779，雷達座標系）─────────
#   base_footprint 在後輪軸心；前保險桿在它前方 0.40 m、後保險桿在後方 0.09 m、半寬 0.185 m
BODY_FRONT_BASE = 0.40
BODY_REAR_BASE = 0.09
BODY_HALF_W = 0.185
LASER_X = 0.089            # path_teach 的 laser_x_offset_m 預設（9/29 車上 TF 實測 0.08874）
CURRENT_MASK = [84.0, 118.0, 249.0, 275.0]   # scan_filter_cc_node 的 self_mask_sectors_deg 預設


def body_distance(r, a_rad, laser_x=LASER_X):
    """點到車身矩形的距離（公尺，0 = 已在車身內）。與 _body_margin 的 any_margin 同算法。"""
    front = BODY_FRONT_BASE - laser_x
    rear = -(BODY_REAR_BASE + laser_x)
    x, y = r * math.cos(a_rad), r * math.sin(a_rad)
    ox = max(rear - x, x - front, 0.0)
    oy = max(abs(y) - BODY_HALF_W, 0.0)
    return math.hypot(ox, oy)


class EchoStats:
    """逐度（0~359，0 = 正前方、逆時針為正，與 scan_filter 相同）累計每幀有沒有命中。"""

    def __init__(self, near_m=0.5, close_m=0.06, laser_x=LASER_X):
        self.near_m, self.close_m, self.laser_x = near_m, close_m, laser_x
        self.frames = 0
        self.valid = [0] * 360
        self.near = [0] * 360
        self.body = [0] * 360      # 點在車身矩形內（0 cm）
        self.close = [0] * 360     # 離車身 < close_m（path_teach 外緣硬停門檻 6 cm）
        self.near_r = [[] for _ in range(360)]   # 每幀這一度最近的 near 距離
        self.meta = None

    def add(self, angle_min, inc, rmin, rmax, ranges):
        self.frames += 1
        if self.meta is None:
            self.meta = (angle_min, inc, rmin, rmax, len(ranges))
        v, n, b, c = set(), {}, set(), set()
        for i, r in enumerate(ranges):
            if r != r or not (rmin <= r <= rmax):     # nan、inf、量程外 = 沒有回波
                continue
            a = angle_min + i * inc
            d = int(math.degrees(a) % 360.0) % 360
            v.add(d)
            if r < self.near_m:
                n[d] = min(r, n.get(d, r))
            bd = body_distance(r, a, self.laser_x)
            if bd <= 0.0:
                b.add(d)
            if bd < self.close_m:
                c.add(d)
        for d in v:
            self.valid[d] += 1
        for d, r in n.items():
            self.near[d] += 1
            self.near_r[d].append(r)
        for d in b:
            self.body[d] += 1
        for d in c:
            self.close[d] += 1

    def ratio(self, counts):
        return [x / self.frames if self.frames else 0.0 for x in counts]


def runs_above(values, thresh):
    """values 是 360 個比例；回傳 > thresh 的連續角度 [(起, 迄)]（含端點），處理 359→0 相接。"""
    on = [x > thresh for x in values]
    if not any(on):
        return []
    if all(on):
        return [(0, 359)]
    start = on.index(False)            # 從一個「沒命中」的角度起算，扇區就不會被切在起點
    runs, cur = [], None
    for k in range(1, 361):
        d = (start + k) % 360
        if on[d]:
            cur = [d, d] if cur is None else [cur[0], d]
        elif cur is not None:
            runs.append(tuple(cur))
            cur = None
    if cur is not None:
        runs.append(tuple(cur))
    return runs


def to_mask_param(runs, pad=3.0):
    """扇區 → self_mask_sectors_deg 的扁平清單，外擴 pad 度。

    ★ scan_filter 用 `lo <= a <= hi` 判斷、**不支援跨 0 度**，所以跨 0 的扇區拆成兩段。
    第 d 度代表 [d, d+1) 這一格。
    """
    flat = []
    for s, e in runs:
        length = ((e - s) % 360) + 1 + 2 * pad
        lo = (s - pad) % 360.0
        if length >= 360:
            flat += [0.0, 360.0]
        elif lo + length <= 360.0:
            flat += [round(lo, 1), round(lo + length, 1)]
        else:
            flat += [round(lo, 1), 360.0, 0.0, round(lo + length - 360.0, 1)]
    return flat


def covered(d, flat):
    return any(flat[i] <= d + 0.5 <= flat[i + 1] for i in range(0, len(flat) - 1, 2))


def fmt_run(s, e):
    return f"{s}~{e}°" if s <= e else f"{s}~360°＋0~{e}°"


def summarize(st, thresh, pad, topic, source):
    near_ratio, body_ratio = st.ratio(st.near), st.ratio(st.body)
    close_ratio, valid_ratio = st.ratio(st.close), st.ratio(st.valid)
    runs = runs_above(near_ratio, thresh)
    mask = to_mask_param(runs, pad)
    L = []
    L.append(f"自身回波量測   {datetime.now():%Y-%m-%d %H:%M:%S}   來源 {source}")
    if st.meta:
        am, inc, rmin, rmax, n = st.meta
        L.append(f"{topic}：{st.frames} 幀，每幀 {n} 點，angle_min {math.degrees(am):.1f}°、"
                 f"增量 {math.degrees(inc):.3f}°，量程 {rmin:.2f}~{rmax:.1f} m")
    L.append(f"判準：near = r < {st.near_m:.2f} m；near 比例 > {thresh:.0%} 算自身回波；"
             f"車身矩形 雷達前 {BODY_FRONT_BASE - st.laser_x:.3f}／後 {BODY_REAR_BASE + st.laser_x:.3f}／半寬 {BODY_HALF_W} m")
    L.append("")
    if runs:
        L.append("自身回波扇區（near 比例 > 門檻）：")
        for s, e in runs:
            degs = [d % 360 for d in range(s, s + ((e - s) % 360) + 1)]
            rs = [r for d in degs for r in st.near_r[d]]
            L.append(f"  {fmt_run(s, e):<16} {len(degs):>3} 度  near 比例中位 "
                     f"{statistics.median(near_ratio[d] for d in degs):.2f}  距離中位 "
                     f"{statistics.median(rs) if rs else float('nan'):.3f} m")
        L.append(f"建議 self_mask_sectors_deg（外擴 {pad:g} 度）：{mask}")
    else:
        L.append(f"沒有 near 比例 > {thresh:.0%} 的角度 —— 靜止時看不到穩定的自身回波。")
    miss = [d for d in range(360) if near_ratio[d] > thresh and not covered(d, CURRENT_MASK)]
    L.append(f"現行預設 {CURRENT_MASK}：" + (f"★ 沒蓋到的自身回波角度 {len(miss)} 個：{_compact(miss)}"
                                             if miss else "量到的自身回波都有蓋到"))
    L.append("")
    hits = [d for d in range(360) if body_ratio[d] > 0]
    if hits:
        L.append("★ 會讓 path_teach 印「0 cm」／硬停的角度（點落在車身矩形內，偶發也算）：")
        for s, e in runs_above(body_ratio, 0.0):
            degs = [d % 360 for d in range(s, s + ((e - s) % 360) + 1)]
            tag = "（已在現行遮罩內，但 path_teach 吃的是原始 /scan）" if all(covered(d, CURRENT_MASK) for d in degs) else ""
            L.append(f"  {fmt_run(s, e):<16} 最高 {max(body_ratio[d] for d in degs):.0%} 的幀{tag}")
    else:
        L.append("沒有任何點落在車身矩形內 —— 靜止時 path_teach 不會因自身回波硬停。")
    near_body = [d for d in range(360) if close_ratio[d] > 0 and body_ratio[d] == 0]
    if near_body:
        L.append(f"離車身 < {st.close_m * 100:.0f} cm（外緣硬停門檻）但不在車身內的角度：{_compact(near_body)}")
    dead = [d for d in range(360) if valid_ratio[d] == 0]
    if dead:
        L.append(f"整段沒有有效回波的角度（被擋住或量程外）：{_compact(dead)}")
    L.append("")
    L.append("★ 空曠處量的才算數。換個朝向再量一次：兩次都出現的角度才是自身回波。")
    return "\n".join(L), runs, mask


def _compact(degs):
    if not degs:
        return "無"
    out, s, p = [], degs[0], degs[0]
    for d in degs[1:] + [None]:
        if d is not None and d == p + 1:
            p = d
            continue
        out.append(f"{s}" if s == p else f"{s}~{p}")
        if d is not None:
            s = p = d
    return "、".join(out) + " 度"


# ── 資料來源 ────────────────────────────────────────────────────────────────
def collect_ros(topic, frames, timeout):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = Node("scan_self_echo")
    got = []

    def cb(msg):
        if len(got) < frames:
            got.append((msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max, list(msg.ranges)))

    node.create_subscription(LaserScan, topic, cb, qos_profile_sensor_data)
    t0 = time.monotonic()
    last = -1
    while rclpy.ok() and len(got) < frames and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(got) // 20 != last:
            last = len(got) // 20
            print(f"  已收 {len(got)}/{frames} 幀", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return got


def write_raw(path, frames):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["angle_min", "angle_increment", "range_min", "range_max", "ranges..."])
        for am, inc, rmin, rmax, ranges in frames:
            w.writerow([am, inc, rmin, rmax] + [("inf" if r == float("inf") else r) for r in ranges])


def read_raw(path):
    frames = []
    with open(path, newline="", encoding="utf-8") as f:
        rows = csv.reader(f)
        next(rows)
        for row in rows:
            am, inc, rmin, rmax = map(float, row[:4])
            frames.append((am, inc, rmin, rmax, [float(x) for x in row[4:]]))
    return frames


def write_tsv(path, st):
    nr, br, cr, vr = st.ratio(st.near), st.ratio(st.body), st.ratio(st.close), st.ratio(st.valid)
    with open(path, "w", encoding="utf-8") as f:
        f.write("deg\tvalid_ratio\tnear_ratio\tnear_median_m\tbody_ratio\tclose_ratio\n")
        for d in range(360):
            med = statistics.median(st.near_r[d]) if st.near_r[d] else ""
            f.write(f"{d}\t{vr[d]:.3f}\t{nr[d]:.3f}\t{med if med == '' else f'{med:.3f}'}\t{br[d]:.3f}\t{cr[d]:.3f}\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description="逐度量測光達的自身回波（決定 self_mask_sectors_deg 用）")
    ap.add_argument("--topic", default="/scan", help="要量原始掃描，不是 /scan_slam（預設 /scan）")
    ap.add_argument("--frames", type=int, default=100)
    ap.add_argument("--timeout", type=float, default=60.0, help="收不滿就在幾秒後停（預設 60）")
    ap.add_argument("--near", type=float, default=0.5, help="近距離門檻，公尺（預設 0.5）")
    ap.add_argument("--thresh", type=float, default=0.5, help="near 比例超過多少算自身回波（預設 0.5）")
    ap.add_argument("--pad", type=float, default=3.0, help="建議遮罩外擴幾度（預設 3，與 9/23 相同）")
    ap.add_argument("--laser-x", type=float, default=LASER_X, help="雷達在後輪軸心前方幾公尺（預設 0.089）")
    ap.add_argument("--from-csv", help="改讀先前存的 scan_self_echo_raw.csv（離線重算，不需要 ROS）")
    ap.add_argument("--out", help="輸出資料夾（預設 ~/maprun/verify_out/<時間>_self_echo）")
    a = ap.parse_args(argv)

    if a.from_csv:
        frames = read_raw(a.from_csv)
        source = a.from_csv
        out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.from_csv)),
                                    f"reanalysis_{datetime.now():%Y%m%d_%H%M%S}")
    else:
        print(f"訂閱 {a.topic}，收 {a.frames} 幀（車子保持靜止、人離開 1 m 外）……", flush=True)
        frames = collect_ros(a.topic, a.frames, a.timeout)
        source = f"即時 {a.topic}"
        out = a.out or os.path.expanduser(f"~/maprun/verify_out/{datetime.now():%Y%m%d_%H%M%S}_self_echo")
    if not frames:
        print(f"✗ 沒收到任何 {a.topic}。底盤＋雷達沒開？或這個 shell 沒 source ~/maprun/env.sh"
              "（CycloneDDS／ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST 不一致就收不到）")
        return 1

    st = EchoStats(near_m=a.near, laser_x=a.laser_x)
    for fr in frames:
        st.add(*fr)
    text, _, _ = summarize(st, a.thresh, a.pad, a.topic, source)
    print()
    print(text)

    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "summary.txt"), "w", encoding="utf-8") as f:
        f.write(text + "\n")
    write_tsv(os.path.join(out, "scan_self_echo.tsv"), st)
    if not a.from_csv:
        write_raw(os.path.join(out, "scan_self_echo_raw.csv"), frames)
    print(f"\n已存：{out}")
    if len(frames) < a.frames and not a.from_csv:
        print(f"⚠ 只收到 {len(frames)}/{a.frames} 幀（逾時 {a.timeout:.0f} 秒）—— 比例的解析度會比較粗")
    return 0


if __name__ == "__main__":
    sys.exit(main())
