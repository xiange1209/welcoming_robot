#!/usr/bin/env python3
"""拿「當下這一幀雷射」去跟地圖比對，回答「車子現在physically在哪個已知地點」。

★ 為什麼需要這支
AMCL 說的位姿不一定對（走廊長軸有歧義、IMU 零偏會讓靜止的車一直漂）。
pose_check_cc.py 只能告訴你「現在這個估計對不對」，不能告訴你「那正確答案是什麼」。
這支把每個已知 waypoint 當候選解，各算一次雷射落在地圖障礙格上的比例，
分數最高的那個就是車子真正所在 —— 不用問人，也不用移動車子。

★ 分數怎麼讀
  - 最高分 >= 0.90 而且領先第二名 >= 0.05  -> 可信，就是它
  - 兩個以上分數接近                        -> 走廊對稱歧義，這支分不出來，要問人
  - 全部都低 (< 0.80)                       -> 車子不在任何已知地點

計分方式與 map_service_cc_node._align_pose_with_scan 完全一致（含雷射相對
base_footprint 的位移 0.08874 / 0.00067 與 1 格容差），所以分數可以互相比較。

用法：
    python3 where_am_i.py           # 只評分，不改任何東西
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from smartnav_msgs.srv import ListWaypoints

# 雷射相對 base_footprint 的位移（robot_model.yaml: senior_akm base_to_laser）
LX_OFF, LY_OFF = 0.08874, 0.00067


class WhereAmI(Node):
    def __init__(self):
        super().__init__("where_am_i")
        self.grid = None
        self.scan = None

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        # 用過濾後的雷射：跟導航實際吃的同一條，否則比出來的分數對不上
        self.create_subscription(LaserScan, "/scan_filtered", self._on_scan, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)

    def _on_map(self, msg):
        if self.grid is None:
            self.grid = msg

    def _on_scan(self, msg):
        self.scan = msg


def build(grid, scan):
    info = grid.info
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    w, h = info.width, info.height

    occupied = set()
    for gy in range(h):
        row = gy * w
        for gx in range(w):
            if grid.data[row + gx] >= 65:
                occupied.add((gx, gy))

    pts = []
    for i in range(0, len(scan.ranges), 3):
        r = scan.ranges[i]
        if r <= 0.0 or math.isinf(r) or math.isnan(r) or r > 8.0:
            continue
        a = scan.angle_min + i * scan.angle_increment
        pts.append((r * math.cos(a), r * math.sin(a)))
    return occupied, pts, (res, ox, oy)


def score(occupied, pts, geo, bx, by, byaw):
    res, ox, oy = geo
    c, sn = math.cos(byaw), math.sin(byaw)
    lx = bx + LX_OFF * c - LY_OFF * sn
    ly = by + LX_OFF * sn + LY_OFF * c
    hit = 0
    for dx, dy in pts:
        wx = lx + dx * c - dy * sn
        wy = ly + dx * sn + dy * c
        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        if (gx, gy) in occupied or (gx + 1, gy) in occupied or (gx - 1, gy) in occupied \
                or (gx, gy + 1) in occupied or (gx, gy - 1) in occupied:
            hit += 1
    return hit / len(pts) if pts else 0.0


def local_best(occupied, pts, geo, x, y, yaw, span=0.30, yaw_span=15.0):
    """在候選點附近找最佳解 —— 車子停的位置不會跟 waypoint 完全重合。

    ★ 若回傳的最佳解落在視窗邊界上，代表真正的解在視窗外，要放大 span 再跑一次，
      否則你拿到的是「被截斷的答案」而不是最佳解。
    """
    n = max(1, int(round(span / 0.10)))
    m = max(1, int(round(yaw_span / 5.0)))
    best = (score(occupied, pts, geo, x, y, yaw), x, y, yaw)
    for iy in range(-m, m + 1):
        yy = yaw + math.radians(iy * 5.0)
        for ix in range(-n, n + 1):
            for jy in range(-n, n + 1):
                s = score(occupied, pts, geo, x + ix * 0.10, y + jy * 0.10, yy)
                if s > best[0]:
                    best = (s, x + ix * 0.10, y + jy * 0.10, yy)
    return best


SPAN = float(sys.argv[1]) if len(sys.argv) > 1 else 0.30
YAW_SPAN = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0


def main():
    rclpy.init()
    node = WhereAmI()

    cli = node.create_client(ListWaypoints, "/list_waypoints")
    if not cli.wait_for_service(timeout_sec=5.0):
        print("✗ /list_waypoints 服務不在 —— 導航堆疊沒起來？")
        return 1
    fut = cli.call_async(ListWaypoints.Request())

    import time
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.grid is not None and node.scan is not None and fut.done():
            break
    if node.grid is None:
        print("✗ 收不到 /map")
        return 1
    if node.scan is None:
        print("✗ 收不到雷射")
        return 1
    if not fut.done():
        print("✗ /list_waypoints 沒回應")
        return 1

    wps = fut.result().waypoints_info
    occupied, pts, geo = build(node.grid, node.scan)

    print("=" * 66)
    print(f"  雷射有效點 {len(pts)}　地圖障礙格 {len(occupied)}")
    print("=" * 66)
    print(f"  {'地點':<10} {'原始分':>7} {'微調後':>7}   微調後位姿")
    print("  " + "-" * 60)

    rows = []
    for w in wps:
        p = w.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        raw = score(occupied, pts, geo, p.position.x, p.position.y, yaw)
        s, bx, by, byaw = local_best(occupied, pts, geo, p.position.x, p.position.y, yaw,
                                     span=SPAN, yaw_span=YAW_SPAN)
        rows.append((s, raw, w.waypoint_name, bx, by, byaw))

    for s, raw, name, bx, by, byaw in sorted(rows, reverse=True):
        print(f"  {name:<10} {raw*100:6.1f}% {s*100:6.1f}%   "
              f"x={bx:+.3f} y={by:+.3f} yaw={math.degrees(byaw):+.1f}°")

    print("  " + "-" * 60)
    rows.sort(reverse=True)
    top, second = rows[0], (rows[1] if len(rows) > 1 else None)
    if top[0] < 0.80:
        print(f"  判定：**不在任何已知地點**（最高只有 {top[0]*100:.1f}%）")
    elif second and top[0] - second[0] < 0.05:
        print(f"  判定：**分不出來** —— {top[2]} {top[0]*100:.1f}% 與 "
              f"{second[2]} {second[0]*100:.1f}% 太接近，走廊對稱歧義")
    else:
        print(f"  判定：車子在 **{top[2]}**（{top[0]*100:.1f}%，"
              f"領先第二名 {(top[0]-second[0])*100:.1f} 個百分點）" if second
              else f"  判定：車子在 **{top[2]}**（{top[0]*100:.1f}%）")
    print("=" * 66)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
