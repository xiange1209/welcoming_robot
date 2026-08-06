#!/usr/bin/env python3
"""轉向不對稱：直接量三個實際值，不要從比例反推。

## 為什麼要重量（2026-08-06 的教訓）

那天用兩點打滿舵的資料反推中位：

    R_left  = 0.944 m （舵角 18.83 度）
    R_right = 0.751 m （舵角 23.21 度，正好是韌體硬限）
    用 δ_right = X + δ₀ 反推 -> δ₀ = −2.19 度（偏右）

但**直接下 ω = 0 走 1.398 m，量到的是 +17.75 度（偏左）** —— 方向相反。

★ **兩點打滿舵的資料不足以推出中位。** 打滿舵時舵角被韌體的
`Servo_min/max` 削掉（右邊正好落在硬限 0.751），削掉多少不知道，
反推出來的偏移就是錯的。

## 這支工具量什麼

三段各自獨立，互相不依賴：

    ① 直走（ω = 0）      -> 零位偏移 δ₀      這是「中位在哪」
    ② 左打滿             -> R_left           這是「往左能轉多急」
    ③ 右打滿             -> R_right          這是「往右能轉多急」

零位偏移只由 ① 決定 —— 不從 ②③ 反推，因為那兩個受硬限截斷污染。
②③ 之間的差異才是真正的左右增益不對稱。

## 用法

    # 空曠處（大廳），前後左右各留 1.5 m 以上
    ~/maprun/run_sensors_cc.sh false
    python3 ~/maprun/steer_asym_check_cc.py

    python3 ~/maprun/steer_asym_check_cc.py --dry   # 只印計畫不開車

★ 車子會自己動。開始前會檢查前方淨空，但**側向要你自己看**。
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

WHEELBASE = 0.322          # 軸距（廠商韌體值）
MIN_RADIUS = 0.80          # 控制器箝制值
SPEED = 0.08               # 低速：舵機動態的影響最小
SEG_SEC = 6.0              # 每段時間
NEED_FRONT = 1.2           # 開始前要求的前方淨空


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class SteerAsym(Node):
    def __init__(self):
        super().__init__("steer_asym_check_cc")
        self._lock = threading.Lock()
        self.odom = None
        self.scan = None
        # ★ 發到 cmd_vel_trimmed 而不是 cmd_vel：這樣會經過 steering_trim_cc，
        #   量到的是**補償後**的實際行為。要量硬體原始的不對稱，
        #   先把 auto_trim 關掉並把 trim_rad_per_m 設 0。
        self.pub = self.create_publisher(Twist, "cmd_vel_smoothed", 10)
        self.create_subscription(Odometry, "odom_combined", self._odom_cb, 10)
        self.create_subscription(LaserScan, "scan", self._scan_cb,
                                 qos_profile_sensor_data)

    def _odom_cb(self, msg):
        with self._lock:
            self.odom = msg

    def _scan_cb(self, msg):
        with self._lock:
            self.scan = msg

    def front_clearance(self) -> float:
        with self._lock:
            scan = self.scan
        if scan is None:
            return 0.0
        best = float("inf")
        ang = scan.angle_min
        for r in scan.ranges:
            a = ang
            ang += scan.angle_increment
            if not (scan.range_min <= r <= scan.range_max) or r != r:
                continue
            if abs(norm(a)) <= math.radians(15.0):
                best = min(best, r)
        return best if best < float("inf") else 0.0

    def pose(self):
        with self._lock:
            o = self.odom
        if o is None:
            return None
        return o.pose.pose.position.x, o.pose.pose.position.y, yaw_of(o)

    def stop(self, sec: float = 1.5) -> None:
        end = time.monotonic() + sec
        while time.monotonic() < end and rclpy.ok():
            self.pub.publish(Twist())
            time.sleep(0.05)

    def run_segment(self, label: str, curvature: float):
        """走一段，回傳 (弧長, 朝向變化)。curvature 為 0 就是直走。"""
        p0 = self.pose()
        if p0 is None:
            self.get_logger().error("拿不到 odom")
            return None
        x0, y0, yaw0 = p0
        self.get_logger().info(f"▶ {label}：v={SPEED:.2f} m/s，κ={curvature:+.3f} /m，{SEG_SEC:.0f} 秒")

        msg = Twist()
        msg.linear.x = SPEED
        msg.angular.z = SPEED * curvature       # 阿克曼 ω = v·κ
        end = time.monotonic() + SEG_SEC
        while time.monotonic() < end and rclpy.ok():
            self.pub.publish(msg)               # 韌體逾時 1.0 秒，必須持續送
            time.sleep(0.05)
        self.stop()

        p1 = self.pose()
        if p1 is None:
            return None
        arc = math.hypot(p1[0] - x0, p1[1] - y0)
        dyaw = norm(p1[2] - yaw0)
        return arc, dyaw


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true", help="只印計畫，不開車")
    args = ap.parse_args()

    kappa = 1.0 / MIN_RADIUS
    if args.dry:
        print("計畫（每段 %.0f 秒 @ %.2f m/s，約走 %.2f m）：" % (SEG_SEC, SPEED, SEG_SEC * SPEED))
        print("  ① 直走     κ = 0")
        print("  ② 左打滿   κ = %+.3f /m" % +kappa)
        print("  ③ 右打滿   κ = %+.3f /m" % -kappa)
        print("需要前後 1.5 m、左右各 1.0 m 以上的淨空")
        return

    rclpy.init()
    node = SteerAsym()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    for _ in range(100):
        if node.pose() is not None and node.scan is not None:
            break
        time.sleep(0.1)
    if node.pose() is None:
        print("等不到 odom_combined，先確認 run_sensors_cc.sh 起來了")
        sys.exit(1)

    front = node.front_clearance()
    if front < NEED_FRONT:
        print("前方淨空只有 %.2f m（需要 %.2f m），換個空曠的地方" % (front, NEED_FRONT))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    print("前方淨空 %.2f m，開始。側向請自己確認有 1 m 以上。" % front)

    results = {}
    try:
        node.stop(2.0)
        for label, k in (("① 直走", 0.0), ("② 左打滿", +kappa), ("③ 右打滿", -kappa)):
            r = node.run_segment(label, k)
            if r is None:
                print("量測中斷")
                break
            results[label] = r
            time.sleep(1.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop(0.5)

    print()
    print("=" * 68)
    print("結果")
    print("=" * 68)
    for label, (arc, dyaw) in results.items():
        deg = math.degrees(dyaw)
        r_txt = "—（直走）"
        if abs(dyaw) > math.radians(1.0):
            r_txt = "%.3f m" % (arc / abs(dyaw))
        print("  %-10s 走 %.3f m，轉 %+.2f 度   實際半徑 %s" % (label, arc, deg, r_txt))

    print()
    if "① 直走" in results:
        arc, dyaw = results["① 直走"]
        if arc > 0.05:
            per_m = math.degrees(dyaw) / arc
            # δ₀ = atan(L / R)，其中 1/R = dyaw/arc
            delta0 = math.degrees(math.atan(WHEELBASE * dyaw / arc))
            print("零位偏移（只由①決定，不從②③反推）：")
            print("    %+.2f 度/m  ->  等效舵角偏移 %+.2f 度（%s）" % (
                per_m, delta0, "偏左" if dyaw > 0 else "偏右"))
            print("    ★ trim_rad_per_m 應該設 %+.4f 來抵消它" % (-dyaw / arc))

    if "② 左打滿" in results and "③ 右打滿" in results:
        al, dl = results["② 左打滿"]
        ar, dr = results["③ 右打滿"]
        if abs(dl) > math.radians(1.0) and abs(dr) > math.radians(1.0):
            rl, rr = al / abs(dl), ar / abs(dr)
            asym = abs(rl - rr) / max(rl, rr) * 100.0
            print()
            print("左右不對稱：R_left %.3f m vs R_right %.3f m  ->  %.1f%%" % (rl, rr, asym))
            if min(rl, rr) < 0.75:
                print("    ⚠ 其中一邊 < 0.75 m（韌體硬限）——那一邊的舵角被 Servo_min/max 削掉了，")
                print("      這個值代表『硬體極限』而不是『指令對應的角度』。")
            print("    ★ 不要用這兩個值去反推零位 —— 被硬限截斷的資料推不出中位（8/06 就錯在這）。")
    print("=" * 68)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
