#!/usr/bin/env python3
"""急停反應時間＋滑行距離（11/10 車上 agent；補 estop_timing.py 沒算的距離）。

t0＝HMI 處理急停時發的 /unwedge_abort（跟 estop_timing.py 同一個基準：HMI 收到請求的時刻，
不含平板到 Pi 的 Wi-Fi 延遲）。
每次急停記：t0 當下車速、車速連續 0.5 s < 0.02 m/s 的起點（反應時間）、
t0 到停下之間 /odom 走過的路程（滑行距離，輪式里程計）。
用法：python3 estop_slide.py <輸出.csv> [最長秒數]；Ctrl-C 結束。
"""
import csv
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty

OUT = sys.argv[1]
LIMIT = float(sys.argv[2]) if len(sys.argv) > 2 else 3600.0
STOP_V, STOP_HOLD, WINDOW = 0.02, 0.5, 6.0


class Slide(Node):
    def __init__(self):
        super().__init__('estop_slide_tmp')
        self.odom = []          # (t, x, y, v)
        self.pending = []       # t0 還沒算完的急停
        self.t_start = time.time()
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Empty, '/unwedge_abort', self.on_estop, 10)
        self.create_timer(0.5, self.tick)
        new = not os.path.exists(OUT)
        self.f = open(OUT, 'a', newline='', encoding='utf-8')
        self.w = csv.writer(self.f)
        if new:
            self.w.writerow(['t0', '當下車速m_s', '反應時間s', '滑行距離cm', '說明'])
        print('等急停…（/unwedge_abort）', flush=True)

    def on_odom(self, m):
        t = time.time()
        v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        self.odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y, v))
        self.odom = [o for o in self.odom if t - o[0] < 30.0]

    def on_estop(self, _):
        self.pending.append(time.time())

    def tick(self):
        now = time.time()
        for t0 in list(self.pending):
            if now - t0 < WINDOW:
                continue
            self.pending.remove(t0)
            seg = [o for o in self.odom if t0 - 3.0 <= o[0] <= t0 + WINDOW]
            before = [o for o in seg if o[0] <= t0]
            after = [o for o in seg if o[0] > t0]
            v0 = before[-1][3] if before else float('nan')
            moving = any(o[3] > 0.05 for o in before)
            t_stop = None
            for i, o in enumerate(after):
                if o[3] < STOP_V and all(p[3] < STOP_V for p in after[i:] if p[0] - o[0] <= STOP_HOLD) \
                        and after[-1][0] - o[0] >= STOP_HOLD:
                    t_stop = o[0]
                    break
            dist = 0.0
            pts = ([before[-1]] if before else []) + [o for o in after if t_stop is None or o[0] <= t_stop]
            for a, b in zip(pts, pts[1:]):
                dist += math.hypot(b[1] - a[1], b[2] - a[2])
            note = '' if moving else '無效：按下前 3 秒車沒在動（>0.05 m/s）'
            react = (t_stop - t0) if t_stop else float('nan')
            row = [time.strftime('%H:%M:%S', time.localtime(t0)), f'{v0:.3f}', f'{react:.3f}', f'{dist * 100:.1f}', note]
            self.w.writerow(row)
            self.f.flush()
            print('急停：', dict(zip(['t0', '車速', '反應s', '滑行cm', '說明'], row)), flush=True)
        if now - self.t_start > LIMIT:
            raise SystemExit


rclpy.init()
n = Slide()
try:
    rclpy.spin(n)
except (SystemExit, KeyboardInterrupt):
    pass
