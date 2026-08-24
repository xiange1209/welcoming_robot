#!/usr/bin/env python3
"""量車子四周八個扇區的淨空（雷達中心為原點）。

★ 用法上的關鍵（踩過的坑）
靜止時**人與異物在雷達上無法區分**。所以看的不是「最近值」而是
「最近值與中位數差多少」：差很多 = 有東西在動（人），差不多 = 固定障礙。
真的要確認，請人離開再量一次。
"""
import math, sys, time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
S = {"正前 (±20°)": (-20, 20), "左前": (20, 70), "左側": (70, 110), "左後": (110, 160),
     "正後": (160, 200), "右後": (-160, -110), "右側": (-110, -70), "右前": (-70, -20)}


class C(Node):
    def __init__(s):
        super().__init__("sectors")
        s.acc = {k: [] for k in S}
        s.n = 0
        s.create_subscription(LaserScan, "/scan", s.cb, 10)

    def cb(s, m):
        s.n += 1
        cur = {k: [] for k in S}
        for i, r in enumerate(m.ranges):
            if r <= 0 or math.isinf(r) or math.isnan(r) or r < m.range_min:
                continue
            a = math.degrees(m.angle_min + i * m.angle_increment)
            a = (a + 180) % 360 - 180
            for k, (lo, hi) in S.items():
                if k == "正後":
                    if a >= 160 or a <= -160:
                        cur[k].append(r)
                elif lo <= a <= hi:
                    cur[k].append(r)
        for k in S:
            if cur[k]:
                s.acc[k].append(min(cur[k]))


def main():
    rclpy.init()
    c = C()
    t = time.time()
    while rclpy.ok() and time.time() - t < SECS:
        rclpy.spin_once(c, timeout_sec=0.2)
    print(f"取樣 {c.n} 幀（雷達中心為原點；車頭比雷達再多 0.311 m、車尾 0.179 m）")
    for k in S:
        v = c.acc[k]
        if not v:
            print(f"  {k:12s} 無有效回波")
            continue
        lo, mid = min(v), sorted(v)[len(v) // 2]
        flag = "  ← 差很多，可能是會動的東西" if mid - lo > 0.15 else ""
        print(f"  {k:12s} 最近 {lo:.2f} m   中位 {mid:.2f} m{flag}")
    c.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
