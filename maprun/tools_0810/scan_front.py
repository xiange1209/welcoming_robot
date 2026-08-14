#!/usr/bin/env python3
"""看車頭前方的掃描剖面：判斷擋住的是「人」還是「牆」。

窄而近 = 人或箱子（地圖上不會有）；寬而平 = 牆（那就是定位錯了）。
★ /scan 是 BEST_EFFORT，一定要用 qos_profile_sensor_data 訂閱。
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Front(Node):
    def __init__(self):
        super().__init__("scan_front")
        self.got = False
        self.create_subscription(LaserScan, "scan", self._cb, qos_profile_sensor_data)

    def _cb(self, m: LaserScan):
        if self.got:
            return
        self.got = True
        print(f"  光束 {len(m.ranges)}，角度 {math.degrees(m.angle_min):.0f}° ~ "
              f"{math.degrees(m.angle_max):.0f}°，量程 {m.range_min:.2f}~{m.range_max:.1f} m")
        print()
        print("  角度      距離     圖示（每格 0.2 m）")
        # 車頭是 0 度。取 -60~+60 度，每 5 度取該扇區的最小值
        for deg in range(-60, 61, 5):
            a = math.radians(deg)
            lo, hi = a - math.radians(2.5), a + math.radians(2.5)
            vals = []
            for i, r in enumerate(m.ranges):
                ang = m.angle_min + i * m.angle_increment
                if lo <= ang <= hi and m.range_min < r < m.range_max and not math.isinf(r):
                    vals.append(r)
            if not vals:
                print(f"  {deg:+4d}°     無回波")
                continue
            d = min(vals)
            print(f"  {deg:+4d}°   {d:6.2f} m  {'#' * int(min(40, d / 0.2))}")


def main():
    rclpy.init()
    n = Front()
    for _ in range(80):
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.got:
            break
    if not n.got:
        print("  ✗ 收不到 /scan")
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
