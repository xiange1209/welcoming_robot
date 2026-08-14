#!/usr/bin/env python3
"""錄下人工遙控脫困的全過程，之後拿來分析「人是怎麼做到的」。

錄：指令(v, ω)、對地位姿、兩側與前後淨空。10 Hz。
"""
import csv, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf2_ros

DUR = int(sys.argv[1]) if len(sys.argv) > 1 else 180
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/user/rescue_demo.csv"
HALF = 0.185


class Rec(Node):
    def __init__(self):
        super().__init__("record_rescue")
        self.v = self.w = 0.0
        self.left = self.right = self.front = self.rear = float("nan")
        self.create_subscription(Twist, "cmd_vel", self._cmd, 10)
        self.create_subscription(LaserScan, "scan", self._scan, qos_profile_sensor_data)
        self.buf = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.buf, self)

    def _cmd(self, m):
        self.v, self.w = m.linear.x, m.angular.z

    def _scan(self, m):
        L = R = F = B = None
        a = m.angle_min
        for r in m.ranges:
            ang = a; a += m.angle_increment
            if not (m.range_min < r < m.range_max) or r != r:
                continue
            d = math.degrees(ang); lat = abs(r * math.sin(ang))
            if 30 <= d <= 150:   L = lat if L is None else min(L, lat)
            if -150 <= d <= -30: R = lat if R is None else min(R, lat)
            if -20 <= d <= 20:   F = r if F is None else min(F, r)
            if abs(d) >= 160:    B = r if B is None else min(B, r)
        self.left  = (L - HALF) if L is not None else float("nan")
        self.right = (R - HALF) if R is not None else float("nan")
        self.front = F if F is not None else float("nan")
        self.rear  = B if B is not None else float("nan")

    def pose(self):
        try:
            t = self.buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception:
            return None


def main():
    rclpy.init(); n = Rec()
    rows = []
    t0 = time.time()
    print(f"  ── 開始記錄 {DUR} 秒，請開始遙控脫困 ──")
    last = 0
    while time.time() - t0 < DUR and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)
        t = time.time() - t0
        if t - last < 0.1:
            continue
        last = t
        p = n.pose()
        rows.append([f"{t:.2f}", f"{n.v:+.3f}", f"{n.w:+.3f}",
                     f"{p[0]:.3f}" if p else "", f"{p[1]:.3f}" if p else "",
                     f"{math.degrees(p[2]):.1f}" if p else "",
                     f"{n.left:+.3f}", f"{n.right:+.3f}",
                     f"{n.front:.2f}", f"{n.rear:.2f}"])
        if int(t) % 10 == 0 and abs(t - int(t)) < 0.11:
            print(f"   {t:5.0f}s  指令 v{n.v:+.2f} ω{n.w:+.2f}   "
                  f"左{n.left:+.3f} 右{n.right:+.3f}  前{n.front:.2f} 後{n.rear:.2f}")
    with open(OUT, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t","cmd_v","cmd_w","x","y","yaw_deg","left_clear","right_clear","front","rear"])
        w.writerows(rows)
    print(f"  ✓ 已存 {OUT}（{len(rows)} 筆）")
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
