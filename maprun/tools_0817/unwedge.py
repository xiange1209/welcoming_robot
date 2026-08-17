#!/usr/bin/env python3
"""把楔在牆邊、規劃器已經拒絕出手的車子推開。

★ 為什麼需要這支（2026-08-17 實際遇到的死結）
車子跑完一趟後停在右牆邊 0.15 m 處，成本地圖上車體壓在內切格（99）上，
於是 `planner_server` 回 **"Start occupied"**，**所有目標都規劃失敗**
（大廳/門口/起點三個都試過）。而車子內建的脫困只在導航進行中才會啟動 ——
導航根本起不了跑，脫困就永遠不會被呼叫。**車子把自己開進了一個出不來的狀態。**

這支繞過規劃器，直接發一小段「前進 + 往離牆的那側打舵」的指令。
阿克曼不能橫移，所以唯一能增加側向距離的方法就是邊走邊轉。

★ 安全
  - 只走 DIST（預設 0.40 m），到了就停
  - 每個週期看正前方扇區，低於 SAFE_M 立刻停
  - 不論怎麼結束都補送零速
  - 速度 0.12 m/s，高於 [[chassis-low-speed-deadband]] 量到的 0.085 死區

用法：
    python3 unwedge.py            # 自動判斷牆在哪一側，往反方向打
    python3 unwedge.py left       # 強制往左打
    python3 unwedge.py right 0.6  # 往右打、走 0.6 m
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

SIDE = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] in ("left", "right") else "auto"
_d = [a for a in sys.argv[1:] if a not in ("left", "right")]
DIST = float(_d[0]) if _d else 0.40

SPEED = 0.12
SAFE_M = 0.45
FAN = math.radians(20)
CURV = 1.0            # 1/R，R = 1.0 m；左轉極限是 1/0.944 = 1.059，留一點餘裕


class Unwedge(Node):
    def __init__(self):
        super().__init__("unwedge")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.front = None
        self.left = None
        self.right = None
        self.pos = None
        self.start = None

    def _scan(self, m):
        f = l = r = None
        for i, d in enumerate(m.ranges):
            if d <= 0.05 or math.isinf(d) or math.isnan(d):
                continue
            a = m.angle_min + i * m.angle_increment
            if -FAN <= a <= FAN:
                f = d if f is None else min(f, d)
            elif math.radians(60) <= a <= math.radians(120):
                l = d if l is None else min(l, d)
            elif math.radians(-120) <= a <= math.radians(-60):
                r = d if r is None else min(r, d)
        self.front, self.left, self.right = f, l, r

    def _odom(self, m):
        p = m.pose.pose.position
        self.pos = (p.x, p.y)
        if self.start is None:
            self.start = (p.x, p.y)

    def stop(self):
        z = Twist()
        for _ in range(10):
            self.pub.publish(z)
            time.sleep(0.02)


def main():
    rclpy.init()
    n = Unwedge()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 6.0:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.front is not None and n.pos is not None:
            break
    if n.front is None:
        print("✗ 收不到 /scan，不動車。"); n.stop(); return 1

    print(f"  淨空　前 {n.front:.2f} m　左 "
          f"{'—' if n.left is None else f'{n.left:.2f}'} m　右 "
          f"{'—' if n.right is None else f'{n.right:.2f}'} m")
    if n.front < SAFE_M + DIST:
        print(f"✗ 前方只有 {n.front:.2f} m，不夠走 {DIST:.2f} m + 裕度 {SAFE_M:.2f} m。不動車。")
        n.stop(); return 1

    side = SIDE
    if side == "auto":
        if n.left is None or n.right is None:
            print("✗ 量不到兩側，無法自動判斷。請指定 left 或 right。")
            n.stop(); return 1
        side = "left" if n.left > n.right else "right"
        print(f"  自動判斷：往**{'左' if side == 'left' else '右'}**打"
              f"（那一側比較空）")

    # 曲率為正 = 往左（前進時）
    curv = CURV if side == "left" else -CURV
    cmd = Twist()
    cmd.linear.x = SPEED
    cmd.angular.z = SPEED * curv

    n.start = n.pos
    t0 = time.time()
    reason = "走完預定距離"
    try:
        while rclpy.ok():
            if time.time() - t0 > DIST / SPEED + 4.0:
                reason = "逾時"; break
            if n.front is not None and n.front < SAFE_M:
                reason = f"前方 {n.front:.2f} m 低於安全距離"; break
            if n.start and n.pos and math.hypot(n.pos[0] - n.start[0],
                                                n.pos[1] - n.start[1]) >= DIST:
                break
            n.pub.publish(cmd)
            rclpy.spin_once(n, timeout_sec=0.05)
    finally:
        n.stop()

    moved = math.hypot(n.pos[0] - n.start[0], n.pos[1] - n.start[1]) if n.pos and n.start else 0
    print(f"\n  結束：{reason}　實際移動 {moved:.3f} m")
    print(f"  淨空　前 {n.front:.2f} m　左 "
          f"{'—' if n.left is None else f'{n.left:.2f}'} m　右 "
          f"{'—' if n.right is None else f'{n.right:.2f}'} m")
    print("  ★ 接下來跑 costmap_at_robot.py 確認車體已離開內切格，再試規劃。")
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
