#!/usr/bin/env python3
"""緊急停止的實機碼錶：訂 /cmd_vel，量「最後一筆非零速度」到「車速歸零」的秒數。

★ 為什麼要這支而不是用眼睛看
8/14 修好的是 MPPI 那條路徑（0.70 秒停穩）。8/17 才發現**語音/LLM 發起的導航
走的是 action 而不是那條路徑**，紅鈕對它無效。修法是對 action 送「取消全部」
（全零 goal_id）。這次要驗的就是那個修法，而「車看起來停了」不夠 ——
要有數字，而且要能分辨「真的收到取消」與「剛好走完」。

★ 它同時記錄 linear.x 有沒有出現負值 —— 那是 MPPI 三點轉向（前後打舵）的
  直接證據，順手一起量，不用為它單獨跑一趟。

用法：
    python3 estop_watch.py            # 一直監看，Ctrl-C 結束並印報告
    python3 estop_watch.py 120        # 最多監看 120 秒
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LIMIT = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
ZERO = 1e-4


class Watch(Node):
    def __init__(self):
        super().__init__("estop_watch")
        self.create_subscription(Twist, "/cmd_vel", self._cb, 10)
        self.samples = []          # (t, vx, wz)
        self.last_move_t = None    # 最後一筆非零
        self.moving = False
        self.stops = []            # (最後非零時刻, 歸零時刻)
        self.min_vx = 0.0
        self.max_vx = 0.0
        self.neg_count = 0
        self.t0 = time.time()
        print("  監看 /cmd_vel 中……（現在去平板上操作，結束按 Ctrl-C）")

    def _cb(self, msg):
        t = time.time()
        vx, wz = msg.linear.x, msg.angular.z
        self.samples.append((t, vx, wz))
        self.min_vx = min(self.min_vx, vx)
        self.max_vx = max(self.max_vx, vx)
        if vx < -0.01:
            self.neg_count += 1
        nonzero = abs(vx) > ZERO or abs(wz) > ZERO
        if nonzero:
            if not self.moving:
                print(f"  [{t - self.t0:6.2f}s] ▶ 開始動  vx={vx:+.3f} wz={wz:+.3f}")
            self.moving = True
            self.last_move_t = t
        else:
            if self.moving and self.last_move_t is not None:
                dt = t - self.last_move_t
                self.stops.append((self.last_move_t, t))
                print(f"  [{t - self.t0:6.2f}s] ■ 停止   距最後一筆非零 {dt:.3f} 秒")
            self.moving = False


def report(n):
    print("\n" + "=" * 62)
    dur = time.time() - n.t0
    hz = len(n.samples) / dur if dur > 0 else 0
    print(f"  /cmd_vel 共 {len(n.samples)} 筆　{dur:.1f} 秒　平均 {hz:.1f} Hz")
    print("-" * 62)
    print("  【緊急停止】")
    if not n.stops:
        if n.moving:
            print("  ✗ 車子還在動，全程沒有歸零 —— 停止沒生效")
        else:
            print("  ─ 全程沒有動過，這趟量不到東西")
    for i, (a, b) in enumerate(n.stops, 1):
        print(f"    第 {i} 次歸零：最後非零 → 零 費時 {b - a:.3f} 秒")
    print("-" * 62)
    print("  【MPPI 前後打舵（三點轉向）】")
    print(f"    linear.x 範圍  {n.min_vx:+.3f} ~ {n.max_vx:+.3f} m/s")
    if n.neg_count:
        print(f"    ✓ 出現 {n.neg_count} 筆負速度 —— MPPI **有**倒車")
    else:
        print("    ✗ 全程沒有負速度 —— MPPI 從頭到尾只前進，沒有倒車")
    print("=" * 62)


def main():
    rclpy.init()
    n = Watch()
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.2)
            if LIMIT and time.time() - n.t0 > LIMIT:
                break
    except KeyboardInterrupt:
        pass
    report(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
