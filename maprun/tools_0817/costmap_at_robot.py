#!/usr/bin/env python3
"""印出全域成本地圖在**車子所在位置**及周圍的實際數值。

★ 為什麼要這支
2026-08-17 出現「規劃器說 Start occupied，但車子物理上很安全」
（前方淨空 1.25 m、定位吻合度 96.3%、兩個條件都過）。
這種時候用推論會猜錯 —— 膨脹半徑、inscribed 半徑、靜態層、障礙層
每一個都可能是兇手。直接把數字讀出來最快。

★★ **`/global_costmap/costmap` 是 OccupancyGrid，發的是 0~100 的縮放值，
   不是 C++ 端 0~254 的原始成本。** 我第一版拿 253 去比一個最大只有 100 的尺度，
   結果只配得到 -1（未知，讀成 255），把真正的牆全部漏掉，
   還因此得出「擋住規劃的是未知區」這個錯誤結論。**先確認尺度再比門檻。**

    OccupancyGrid 值        原始成本      意義
        -1                    255        未知（NO_INFORMATION）
        100                   254        致命障礙 —— 地圖上真的有東西
         99                   253        內切膨脹 —— 車體一定會撞到
        1~98                 1~252       膨脹梯度（越大越靠近障礙）
         0                     0         自由

★ 規劃器把原始成本 >= 253（也就是這裡的 **>= 99**）當成不能走。
  「Start occupied」不只看中心那一格，還會用車體 footprint
  （[-0.09, 0.40] x [±0.185]）做完整碰撞檢查 —— 中心是 60 但側面壓到 99 一樣會被拒。

用法：
    python3 costmap_at_robot.py
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import tf2_ros


class Look(Node):
    def __init__(self):
        super().__init__("costmap_at_robot")
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.cm = None
        self.mp = None
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._cm, qos)
        self.create_subscription(OccupancyGrid, "/map", self._mp, qos)
        self.buf = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.buf, self)

    def _cm(self, m):
        self.cm = m

    def _mp(self, m):
        self.mp = m

    def pose(self):
        try:
            t = self.buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                2.0 * math.atan2(q.z, q.w))


def val(grid, x, y):
    i = grid.info
    gx = int((x - i.origin.position.x) / i.resolution)
    gy = int((y - i.origin.position.y) / i.resolution)
    if not (0 <= gx < i.width and 0 <= gy < i.height):
        return None
    v = grid.data[gy * i.width + gx]
    return v if v >= 0 else -1       # -1 = 未知，不要偷偷換成 255（尺度不同會誤判）


def main():
    rclpy.init()
    n = Look()
    import time
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 15.0:
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.cm is not None and n.pose() is not None:
            break
    if n.cm is None:
        print("✗ 收不到 /global_costmap/costmap"); return 1
    p = n.pose()
    if p is None:
        print("✗ 取不到 map->base_footprint"); return 1

    x, y, yaw = p
    print(f"  車子 x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):+.1f}°")
    print(f"  成本地圖 {n.cm.info.width}x{n.cm.info.height} @ {n.cm.info.resolution} m")
    c = val(n.cm, x, y)
    print(f"\n  ★ 車子中心的成本值：{c}（0~100 尺度）"
          f"　（>= 99 規劃器就當成不能走）")
    if n.mp is not None:
        print(f"    同一點在靜態地圖 /map 上的值：{val(n.mp, x, y)}"
              f"　（100=障礙、0=自由、255=未知）")

    print("\n  周圍 ±0.30 m 的成本（0~100 尺度；99=內切、100=致命、-1=未知）")
    print("      " + "".join(f"{dx:+6.2f}" for dx in [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3]))
    for dy in [0.3, 0.2, 0.1, 0.0, -0.1, -0.2, -0.3]:
        row = f"  {dy:+5.2f}"
        for dx in [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3]:
            v = val(n.cm, x + dx, y + dy)
            row += f"{'  --- ' if v is None else f'{v:6d}'}"
        print(row)

    # 找最近的「不能走」格子，這才知道是被什麼擋住
    best = None
    r = 0.05
    while r <= 1.2 and best is None:
        steps = max(8, int(2 * math.pi * r / 0.05))
        for k in range(steps):
            a = 2 * math.pi * k / steps
            v = val(n.cm, x + r * math.cos(a), y + r * math.sin(a))
            if v is not None and v >= 99:
                best = (r, math.degrees(a), v)
                break
        r += 0.05
    print()
    if best:
        rel = (best[1] - math.degrees(yaw) + 540) % 360 - 180
        side = "正前方" if abs(rel) < 30 else ("正後方" if abs(rel) > 150 else
                                              ("左側" if rel > 0 else "右側"))
        print(f"  最近的『不能走』格子：{best[0]:.2f} m　在車子的{side}（相對 {rel:+.0f}°）"
              f"　成本 {best[2]}")
        print("  ★ 車體 footprint 是 [-0.09, 0.40] x [±0.185]。上面那個距離只要落在"
              " footprint 內，\n    即使中心格是自由的，Start occupied 一樣會成立。")
    else:
        print("  1.2 m 內沒有任何 >= 99 的格子")
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
