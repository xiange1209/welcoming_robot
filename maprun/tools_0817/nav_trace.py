#!/usr/bin/env python3
"""導航全程逐筆記錄：指令 vs 實速，並且**專門抓 /cmd_vel 的發布空窗**。

★ 為什麼要抓空窗（2026-08-17 的假設）
底盤韌體有 **1.0 秒指令逾時**：超過 1 秒沒收到新指令就自己停馬達
（見 hmi_server_node._teleop_tick 的註解，那條 20 Hz 補送迴圈就是為此存在）。
而這台 Pi 4 現在 CPU 忙碌 90~100%、load 15。如果控制節點被排程餓到，
/cmd_vel 出現 >1 秒的空窗，底盤就停 —— 但 stuck_detector_cc 記的是
「最後一筆指令值」，所以它看到的仍是 0.130 m/s，於是報成
「指令 0.130 但實測 0.000」，看起來像車子卡住，實際是指令流斷掉。

這支同時記三件事，才分得出「沒下指令」「下了但底盤沒收到」「收到但沒動」：
    /cmd_vel        控制器實際發出什麼、間隔多久
    /odom           輪速（底盤真的動了沒）
    /odom_combined  EKF 融合速度（stuck_detector_cc 讀的就是這個）

輸出 CSV 到 ~/maprun/logs/nav_trace_<時間>.csv，逐筆保存（專案規定）。

用法：
    python3 nav_trace.py 90        # 記錄 90 秒
"""
import csv
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
GAP_WARN = 0.30      # 超過這個間隔就記一筆「空窗」
GAP_FATAL = 1.00     # 韌體指令逾時


class Trace(Node):
    def __init__(self):
        super().__init__("nav_trace")
        self.rows = []
        self.cmd_t = []
        self.last_cmd = None
        self.cmd = (0.0, 0.0)
        self.raw = 0.0
        self.ekf = 0.0
        self.create_subscription(Twist, "/cmd_vel", self._cmd, 20)
        self.create_subscription(Odometry, "/odom", self._raw, 20)
        self.create_subscription(Odometry, "/odom_combined", self._ekf, 20)
        self.t0 = time.time()

    def _cmd(self, m):
        t = time.time()
        self.cmd_t.append(t)
        self.cmd = (m.linear.x, m.angular.z)
        self.rows.append((t - self.t0, m.linear.x, m.angular.z, self.raw, self.ekf))

    def _raw(self, m):
        self.raw = m.twist.twist.linear.x

    def _ekf(self, m):
        self.ekf = m.twist.twist.linear.x


def main():
    rclpy.init()
    n = Trace()
    print(f"  記錄 {DUR:.0f} 秒　現在去平板上按導航……")
    try:
        while rclpy.ok() and time.time() - n.t0 < DUR:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    os.makedirs("/home/user/maprun/logs", exist_ok=True)
    path = f"/home/user/maprun/logs/nav_trace_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_sec", "cmd_vx", "cmd_wz", "odom_vx", "ekf_vx"])
        w.writerows([(f"{a:.3f}", f"{b:.4f}", f"{c:.4f}", f"{d:.4f}", f"{e:.4f}")
                     for a, b, c, d, e in n.rows])

    print("\n" + "=" * 66)
    print(f"  /cmd_vel {len(n.cmd_t)} 筆　CSV -> {path}")
    if len(n.cmd_t) < 2:
        print("  沒有指令，這趟量不到東西。")
        print("=" * 66)
        return 0

    gaps = [(n.cmd_t[i] - n.cmd_t[i - 1], n.cmd_t[i - 1] - n.t0)
            for i in range(1, len(n.cmd_t))]
    span = n.cmd_t[-1] - n.cmd_t[0]
    print(f"  發布期間 {span:.1f} 秒　平均 {len(n.cmd_t)/span:.1f} Hz")
    print("-" * 66)
    print("  【指令空窗】韌體逾時 1.0 秒")
    big = [g for g in gaps if g[0] >= GAP_WARN]
    fatal = [g for g in gaps if g[0] >= GAP_FATAL]
    gaps_sorted = sorted(gaps, reverse=True)[:8]
    print(f"    >= {GAP_WARN}s 的空窗 {len(big)} 次　"
          f">= {GAP_FATAL}s（會讓底盤停）{len(fatal)} 次")
    for g, at in gaps_sorted:
        mark = "  ★底盤會停" if g >= GAP_FATAL else ""
        print(f"      {g:6.3f}s　發生在第 {at:6.2f} 秒{mark}")
    print("-" * 66)
    print("  【指令 vs 實速】只看有下指令的筆數")
    moving_cmd = [r for r in n.rows if abs(r[1]) > 0.02]
    if moving_cmd:
        dead = [r for r in moving_cmd if abs(r[3]) < 0.02]
        print(f"    有指令 {len(moving_cmd)} 筆，其中輪速 <0.02 的有 "
              f"{len(dead)} 筆（{100*len(dead)/len(moving_cmd):.0f}%）")
        print(f"    指令 vx 平均 {sum(abs(r[1]) for r in moving_cmd)/len(moving_cmd):+.3f}　"
              f"輪速平均 {sum(abs(r[3]) for r in moving_cmd)/len(moving_cmd):+.3f}")
    else:
        print("    全程沒有非零指令")
    neg = [r for r in n.rows if r[1] < -0.01]
    print("-" * 66)
    print("  【MPPI／重播 有沒有倒車】")
    print(f"    linear.x 最小 {min((r[1] for r in n.rows), default=0):+.3f}　"
          f"負值 {len(neg)} 筆")
    print("=" * 66)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
