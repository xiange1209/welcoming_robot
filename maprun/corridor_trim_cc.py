#!/usr/bin/env python3
"""用走廊本身當基準量轉向零位偏移，並在撞牆前自動中止。

## 為什麼不用 odom / IMU

`steering_trim_cc` 內建的量測是拿 `odom_combined` 的 yaw 差值算的，
那條路徑上有 IMU 零偏殘差。2026-08-03 實測殘差約 0.2 度/分，
在 0.15 m/s 下相當於 0.022 度/m 的污染——單看還好，但它會**隨時間漂**，
所以不同時間量到的值無法互相比較。

走廊的兩面牆是平行直線，而且**不會漂**。用它當角度基準，
量到的就是純粹的機械偏移。

## 量什麼

對左右牆各做一次最小平方直線擬合（車體座標系）：

    左牆點  y = a_L * x + b_L      斜率 a 就是車頭相對牆面的夾角 (tan)
    右牆點  y = a_R * x + b_R      截距 b 就是離該牆的距離

heading = atan((a_L + a_R) / 2)    兩牆平均，抵消單邊雜訊
lateral = (b_L + b_R) / 2          正 = 偏左，負 = 偏右

車子直行時 heading 隨距離的變化率就是要量的偏轉率（度/公尺）。

## 為什麼這樣就不會撞牆

每個控制週期都知道離兩邊各多遠。距離小於 `abort_clearance` 立刻停車，
不必等人反應。0.99 m 走廊、車寬 0.37 m（半寬 0.185），
留 0.25 m 表示車身邊緣離牆還有 6.5 公分才中止。

用法：
    corridor_trim_cc.py [trim值] [距離公尺]
    corridor_trim_cc.py -0.072 2.0
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan

SPEED = 0.15
ABORT_CLEARANCE = 0.25      # 離牆小於這個距離就中止（車身半寬 0.185）
WALL_MIN, WALL_MAX = 0.25, 1.2   # 牆面點的側向距離範圍
X_WINDOW = 1.5              # 只取車身前後這個範圍內的點做擬合


def fit_line(pts):
    """最小平方擬合 y = a*x + b，回傳 (a, b, 點數)"""
    n = len(pts)
    if n < 8:
        return None
    sx = sum(p[0] for p in pts)
    sy = sum(p[1] for p in pts)
    sxx = sum(p[0] * p[0] for p in pts)
    sxy = sum(p[0] * p[1] for p in pts)
    den = n * sxx - sx * sx
    if abs(den) < 1e-9:
        return None
    a = (n * sxy - sx * sy) / den
    b = (sy - a * sx) / n
    return a, b, n


def corridor_state(scan):
    """回傳 (heading_rad, lateral_m, dist_left, dist_right) 或 None"""
    left, right = [], []
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or not (scan.range_min < r < 6.0):
            continue
        ang = scan.angle_min + i * scan.angle_increment
        x, y = r * math.cos(ang), r * math.sin(ang)
        if abs(x) > X_WINDOW:
            continue
        if WALL_MIN < y < WALL_MAX:
            left.append((x, y))
        elif -WALL_MAX < y < -WALL_MIN:
            right.append((x, y))

    fl, fr = fit_line(left), fit_line(right)
    if fl is None or fr is None:
        return None
    # 兩牆的斜率平均：單邊有雜訊時互相抵消
    heading = math.atan((fl[0] + fr[0]) / 2.0)
    # 截距就是 x=0（車體原點）處到牆的側向距離
    return heading, (fl[1] + fr[1]) / 2.0, fl[1], abs(fr[1])


def main():
    trim = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    target_dist = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    rclpy.init()
    n = rclpy.create_node("corridor_trim_cc")
    st = {}
    n.create_subscription(LaserScan, "/scan", lambda m: st.__setitem__("scan", m),
                          qos_profile_sensor_data)
    n.create_subscription(Odometry, "/odom_combined", lambda m: st.__setitem__("odom", m), 10)
    pub = n.create_publisher(
        Twist, "/cmd_vel",
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST))

    end = time.time() + 8
    while time.time() < end and len(st) < 2:
        rclpy.spin_once(n, timeout_sec=0.3)
    if len(st) < 2:
        print("  收不到 /scan 或 /odom_combined")
        return

    s0 = corridor_state(st["scan"])
    if s0 is None:
        print("  擬合不出兩面牆——車子可能不在走廊裡")
        return
    h0, lat0, dl, dr = s0
    o = st["odom"].pose.pose.position
    x0, y0 = o.x, o.y

    print(f"  起點：離左牆 {dl:.3f} m、離右牆 {dr:.3f} m、"
          f"車頭相對走廊 {math.degrees(h0):+.2f} 度")
    print(f"  trim={trim:+.4f}  目標距離 {target_dist:.1f} m  "
          f"（離牆 < {ABORT_CLEARANCE} m 自動中止）")

    samples = []
    aborted = None
    t = Twist()
    t.linear.x = SPEED
    t.angular.z = trim * SPEED      # 這就是 steering_trim 做的事

    t_start = time.time()
    while time.time() - t_start < target_dist / SPEED + 3.0:
        pub.publish(t)
        rclpy.spin_once(n, timeout_sec=0.02)
        cur = corridor_state(st.get("scan"))
        o = st["odom"].pose.pose.position
        d = math.hypot(o.x - x0, o.y - y0)
        if cur:
            h, lat, dl_now, dr_now = cur
            samples.append((d, h))
            if min(dl_now, dr_now) < ABORT_CLEARANCE:
                aborted = f"離牆只剩 {min(dl_now, dr_now):.3f} m"
                break
        if d >= target_dist:
            break
        time.sleep(0.03)

    for _ in range(25):
        pub.publish(Twist())
        time.sleep(0.04)
        rclpy.spin_once(n, timeout_sec=0.01)

    time.sleep(1.0)
    end = time.time() + 2
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)

    s1 = corridor_state(st["scan"])
    o = st["odom"].pose.pose.position
    dist = math.hypot(o.x - x0, o.y - y0)

    if aborted:
        print(f"  ⚠ 已自動中止：{aborted}")
    if s1 and dist > 0.15:
        h1, lat1, dl1, dr1 = s1
        dh = math.degrees(h1 - h0)
        print(f"  終點：離左牆 {dl1:.3f} m、離右牆 {dr1:.3f} m")
        print(f"  走了 {dist:.3f} m，車頭相對走廊轉了 {dh:+.2f} 度")
        print(f"  ★ 偏轉率 {dh/dist:+.3f} 度/m   （橫向移動 {(lat1-lat0)*100:+.1f} cm）")
    else:
        print(f"  位移只有 {dist:.3f} m，無法計算")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
