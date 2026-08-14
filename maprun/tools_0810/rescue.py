#!/usr/bin/env python3
"""人工救車：把貼右牆的車子拉開。

★ 直接發 /cmd_vel 繞過 collision_monitor —— 車子已經貼牆時防撞會停止輸出，
  正常路徑送不出任何指令（2026-08-06 實測：發 89 筆、位移 0.00 m）。

安全設計（每一項都會讓它停）：
  · 速度硬上限 0.05 m/s
  · 位移上限 0.30 m
  · 時間上限 20 秒
  · 前方淨空 < 0.50 m 立刻停
  · 右側淨空 > 0.20 m 就達成目的、停
  · 離開時 try/finally 一定送零速
"""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

V = 0.05                # m/s
R_TURN = 0.85           # 左轉半徑（比最小 0.80 稍寬，留餘裕）
HALF_W = 0.185          # 車身半寬
MAX_DIST = 0.30
MAX_TIME = 20.0
FRONT_MIN = 0.50
RIGHT_GOAL = 0.20


class Rescue(Node):
    def __init__(self):
        super().__init__("rescue")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self._odom, qos_profile_sensor_data)
        self.front = None
        self.right = None
        self.x = self.y = None
        self.x0 = self.y0 = None

    def _scan(self, m):
        fr, ri = [], []
        a = m.angle_min
        for r in m.ranges:
            ang = a
            a += m.angle_increment
            if not (m.range_min < r < m.range_max) or r != r:
                continue
            d = math.degrees(ang)
            if -20 <= d <= 20:
                fr.append(r)
            # 右側：−30 ~ −120 度，取「側向距離」= r·|sin|，再扣掉車身半寬
            if -120 <= d <= -30:
                ri.append(abs(r * math.sin(ang)) - HALF_W)
        self.front = min(fr) if fr else None
        self.right = min(ri) if ri else None

    def _odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        if self.x0 is None:
            self.x0, self.y0 = self.x, self.y

    def moved(self):
        if self.x0 is None or self.x is None:
            return 0.0
        return math.hypot(self.x - self.x0, self.y - self.y0)

    def stop(self):
        t = Twist()
        for _ in range(5):
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        # 先收滿感測資料
        t0 = time.time()
        while (self.front is None or self.right is None or self.x is None) and time.time() - t0 < 6.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.front is None or self.right is None:
            print("  ✗ 收不到 /scan，不動作"); return False
        print(f"  起始：前方 {self.front:.2f} m   右側餘隙 {self.right:+.3f} m")
        if self.right > RIGHT_GOAL:
            print("  右側已經夠開，不需要救"); return True
        if self.front < FRONT_MIN:
            print(f"  ✗ 前方只有 {self.front:.2f} m（< {FRONT_MIN}），不敢往前，請手動處理")
            return False

        cmd = Twist()
        cmd.linear.x = V
        cmd.angular.z = V / R_TURN          # 正 = 左轉，離開右牆
        t0 = time.time()
        reason = "時間到"
        try:
            while time.time() - t0 < MAX_TIME:
                if self.front is not None and self.front < FRONT_MIN:
                    reason = f"前方只剩 {self.front:.2f} m"; break
                if self.right is not None and self.right > RIGHT_GOAL:
                    reason = f"右側已拉開到 {self.right:.3f} m"; break
                if self.moved() > MAX_DIST:
                    reason = f"已移動 {self.moved():.3f} m 達上限"; break
                self.pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.1)
                el = time.time() - t0
                if int(el * 10) % 10 == 0:
                    print(f"   {el:4.1f}s  前 {self.front:.2f}  右餘隙 {self.right:+.3f}  "
                          f"已移動 {self.moved():.3f} m")
        finally:
            self.stop()
        print(f"  ── 停止：{reason}")
        print(f"  結果：移動 {self.moved():.3f} m，右側餘隙 {self.right:+.3f} m，前方 {self.front:.2f} m")
        return self.right is not None and self.right > 0.03


def main():
    rclpy.init()
    n = Rescue()
    ok = False
    try:
        ok = n.run()
    finally:
        n.stop()
        n.destroy_node(); rclpy.shutdown()
    print("  ✓ 已送零速" if True else "")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
