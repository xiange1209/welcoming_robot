#!/usr/bin/env python3
"""每 30 秒把 /PowerVoltage 記一列到 CSV（11/04 寫、11/10 沿用；車上 agent）。用法：python3 battery_log.py <輸出.csv> [最長秒數]"""
import csv, sys, time, datetime, os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
out = sys.argv[1]; limit = float(sys.argv[2]) if len(sys.argv) > 2 else 6 * 3600
class Log(Node):
    def __init__(self):
        super().__init__('battery_logger_tmp')
        self.v = None; self.t0 = time.time()
        self.create_subscription(Float32, '/PowerVoltage', lambda m: setattr(self, 'v', m.data), 10)
        self.create_timer(30.0, self.tick)
        new = not os.path.exists(out)
        self.f = open(out, 'a', newline='', encoding='utf-8'); self.w = csv.writer(self.f)
        if new: self.w.writerow(['時間', '電壓V'])
    def tick(self):
        if self.v is not None:
            self.w.writerow([datetime.datetime.now().strftime('%H:%M:%S'), f'{self.v:.2f}']); self.f.flush()
        if time.time() - self.t0 > limit: raise SystemExit
rclpy.init(); n = Log()
try: rclpy.spin(n)
except (SystemExit, KeyboardInterrupt): pass
