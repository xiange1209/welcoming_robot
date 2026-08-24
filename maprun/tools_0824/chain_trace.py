#!/usr/bin/env python3
"""同時錄速度指令鏈的四站，找出「死區補償在哪一段消失」。

★ 為什麼要這支（2026-08-24）
`nav_trace.py` 只錄 `/cmd_vel`（鏈的最後一站），量到的結果是：
    前進 463 筆，平均指令 +0.080 m/s，平均實速 -0.001 m/s
而 `path_teach_cc` 的 `_lift_deadband()` 明明會把任何非零速度抬到
`min_move_speed`（預設 0.10），所以 **0.080 不該出現在鏈的尾端**。

指令要經過三站才到底盤：
    path_teach_cc --> cmd_vel_smoothed --> steering_trim_cc --> cmd_vel_trimmed
                  --> collision_monitor --> cmd_vel --> 底盤

只錄尾端分不出是誰改的。這支同時訂四條（含 /odom 實速），
每 0.5 秒印一次同一瞬間的四個值，並在結束時統計每一站
「落在死區以下的非零指令」有多少筆 —— 哪一站開始變多，就是哪一站在改。

用法：
    python3 chain_trace.py [秒數]      # 預設 90 秒
"""
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
DEADBAND = 0.085          # 實測底盤低速死區（23.7 V 時量的）
MIN_MOVE = 0.10           # path_teach_cc 的 min_move_speed 預設值

STATIONS = ["cmd_vel_smoothed", "cmd_vel_trimmed", "cmd_vel"]


class Chain(Node):
    def __init__(self):
        super().__init__("chain_trace")
        self.rows = []
        self.cur = {s: None for s in STATIONS}
        self.cur["odom"] = None
        self.hist = {s: [] for s in STATIONS}
        self.hist["odom"] = []
        for s in STATIONS:
            self.create_subscription(Twist, "/" + s, self._mk(s), 10)
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.t0 = time.monotonic()
        self.create_timer(0.1, self._tick)

    def _mk(self, name):
        def cb(msg: Twist):
            self.cur[name] = msg.linear.x
            self.hist[name].append(msg.linear.x)
        return cb

    def _odom(self, msg: Odometry):
        self.cur["odom"] = msg.twist.twist.linear.x
        self.hist["odom"].append(msg.twist.twist.linear.x)

    def _tick(self):
        t = time.monotonic() - self.t0
        self.rows.append((t, self.cur["cmd_vel_smoothed"], self.cur["cmd_vel_trimmed"],
                          self.cur["cmd_vel"], self.cur["odom"]))


def band(vals):
    """回傳 (非零筆數, 落在死區以下的筆數, 落在 min_move 以下的筆數)"""
    nz = [v for v in vals if v is not None and abs(v) > 1e-6]
    if not nz:
        return 0, 0, 0
    return (len(nz),
            sum(1 for v in nz if abs(v) < DEADBAND),
            sum(1 for v in nz if abs(v) < MIN_MOVE))


def main():
    rclpy.init()
    n = Chain()
    print(f"  錄 {DUR:.0f} 秒 —— 現在去下重播指令")
    print(f"  死區 {DEADBAND} m/s　min_move_speed {MIN_MOVE} m/s")
    try:
        while rclpy.ok() and time.monotonic() - n.t0 < DUR:
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass

    print("\n" + "=" * 74)
    print("  【每一站有多少非零指令落在死區以下】")
    print(f"  {'站':22s} {'非零筆數':>8s} {'<0.085 死區':>12s} {'<0.10 應被抬':>13s}")
    print("  " + "-" * 70)
    prev_bad = None
    culprit = None
    for s in STATIONS + ["odom"]:
        cnt, under_db, under_mm = band(n.hist[s])
        if cnt == 0:
            print(f"  {s:22s} {'（沒有訊息）':>8s}")
            continue
        pct = under_db / cnt * 100
        label = "實速" if s == "odom" else s
        print(f"  {label:22s} {cnt:8d} {under_db:7d} ({pct:4.1f}%) {under_mm:8d} ({under_mm/cnt*100:4.1f}%)")
        if s != "odom":
            if prev_bad is not None and under_mm > prev_bad * 1.5 + 5 and culprit is None:
                culprit = s
            prev_bad = under_mm
    print("  " + "-" * 70)
    if culprit:
        print(f"  ★ 死區以下的指令在 **{culprit}** 這一站明顯變多 —— 元凶在它前面那個節點")
    else:
        print("  ★ 各站比例接近 —— 補償要嘛全程有效，要嘛從來沒生效（看第一站的數字）")

    first = STATIONS[0]
    cnt, under_db, under_mm = band(n.hist[first])
    if cnt:
        if under_mm > cnt * 0.05:
            print(f"  ⚠ **第一站 {first} 就有 {under_mm} 筆 (<{MIN_MOVE})** —— "
                  f"`_lift_deadband()` 根本沒把它們抬起來")
        else:
            print(f"  ✓ 第一站幾乎沒有低於 {MIN_MOVE} 的指令 —— 補償在發布端是有效的")

    # 同一瞬間的對照樣本
    sample = [r for r in n.rows if r[1] is not None and abs(r[1]) > 1e-6]
    if sample:
        print("\n  【同一瞬間四站對照（取前 12 筆有指令的取樣）】")
        print(f"  {'t':>6s} {'smoothed':>9s} {'trimmed':>9s} {'cmd_vel':>9s} {'實速':>8s}")
        for t, a, b, c, o in sample[:12]:
            f = lambda v: f"{v:+.3f}" if v is not None else "   --"
            print(f"  {t:6.1f} {f(a):>9s} {f(b):>9s} {f(c):>9s} {f(o):>8s}")
    print("=" * 74)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
