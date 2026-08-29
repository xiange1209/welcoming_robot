#!/usr/bin/env python3
"""在**多個候選位姿**上算同一個雷射吻合度，用來抓走廊縱向歧義。

## 為什麼需要這支

`pose_check_cc.py` 只驗「AMCL 現在說的這個位姿」合不合理，
它的條件 A 自己就註明「對走廊縱向位置**沒有分辨力**」——
沿著長軸平移一兩公尺，掃描一樣吻合。

所以「94.2% 通過」不代表「位置對」，只代表「這個位姿不矛盾」。
★ 要抓縱向歧義，得問的是**比較級**的問題：
  「候選 B 是不是比 AMCL 現在說的還吻合？」

單一位姿給不出這個答案，這支就是補那個洞。

用法：
    python3 ~/maprun/pose_compare_cc.py                  # AMCL vs 各個地點
    python3 ~/maprun/pose_compare_cc.py 4.598 0.999      # 再加一個自訂候選
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

# 光達相對 base_footprint 的位置（與 waypoint_service_cc_node.py 同一組值）
_LASER_DX = 0.08874
_LASER_DY = 0.00067
_OCC = 65          # /map 是 0~100 的 OccupancyGrid，不是 C++ 的 0~254


class PoseCompare(Node):
    def __init__(self) -> None:
        super().__init__("pose_compare_cc")
        self.grid = None
        self.info = None
        self.scan = None
        self.amcl = None
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, latched)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self._amcl_cb, 10)

    def _map_cb(self, m):
        self.info, self.grid = m.info, m.data

    def _scan_cb(self, m):
        self.scan = m

    def _amcl_cb(self, m):
        p = m.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.amcl = (p.position.x, p.position.y, yaw)

    def score(self, x, y, yaw):
        """與 waypoint_service_cc_node._scan_match_score 同一套算法。"""
        s, info, grid = self.scan, self.info, self.grid
        if s is None or info is None or grid is None:
            return None
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        w, h = info.width, info.height
        c, sn = math.cos(yaw), math.sin(yaw)
        lx = x + c * _LASER_DX - sn * _LASER_DY
        ly = y + sn * _LASER_DX + c * _LASER_DY

        n = len(s.ranges)
        step = max(1, n // 150)
        hit = total = 0
        for i in range(0, n, step):
            r = s.ranges[i]
            if not (s.range_min < r < s.range_max) or math.isinf(r) or math.isnan(r):
                continue
            a = yaw + s.angle_min + i * s.angle_increment
            px, py = lx + r * math.cos(a), ly + r * math.sin(a)
            gx, gy = int((px - ox) / res), int((py - oy) / res)
            if not (0 <= gx < w and 0 <= gy < h):
                continue          # 地圖外的光束不計入分母
            total += 1
            found = False
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < w and 0 <= ny < h and grid[ny * w + nx] >= _OCC:
                        found = True
                        break
                if found:
                    break
            hit += found
        if total < 40:
            return None
        return hit / total, total


def main() -> int:
    rclpy.init()
    n = PoseCompare()
    t0 = time.time()
    while time.time() - t0 < 15.0 and (n.grid is None or n.scan is None or n.amcl is None):
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.grid is None or n.scan is None:
        print("✗ 收不到 /map 或 /scan", file=sys.stderr)
        return 1
    if n.amcl is None:
        print("✗ 收不到 /amcl_pose（AMCL 靜止時不一定會發，動一下車再試）",
              file=sys.stderr)
        return 1

    ax, ay, ayaw = n.amcl
    cands = [("AMCL 現在說的", ax, ay, ayaw)]

    # 各個已存的地點，用它們自己的朝向
    import json
    import pathlib
    wp = pathlib.Path.home() / ".smartnav" / "waypoint_database" / "waypoints.json"
    cur_map = None
    try:
        ns = pathlib.Path.home() / ".smartnav" / "nav_state.json"
        cur_map = json.loads(ns.read_text(encoding="utf-8")).get("current_map")
    except Exception:
        pass
    try:
        db = json.loads(wp.read_text(encoding="utf-8"))["waypoints_db"]
        for _, v in db.items():
            if cur_map and v.get("map_id") != cur_map:
                continue
            p = v["pose"]["position"]
            q = v["pose"]["orientation"]
            yaw = math.atan2(2.0 * (q["w"] * q["z"]), 1.0 - 2.0 * (q["z"] ** 2))
            cands.append((v["name"], p["x"], p["y"], yaw))
    except Exception as e:  # noqa: BLE001
        print(f"（讀不到地點：{e}）")

    # 命令列附加的候選：用 AMCL 的朝向
    args = sys.argv[1:]
    if len(args) >= 2:
        cands.append(("自訂", float(args[0]), float(args[1]),
                      float(args[2]) if len(args) > 2 else ayaw))

    print("=" * 72)
    print(f"地圖 {n.info.width}x{n.info.height} @ {n.info.resolution} m，"
          f"目前地圖 {cur_map}")
    print(f"AMCL: x={ax:+.3f} y={ay:+.3f} yaw={math.degrees(ayaw):+.1f}°")
    print("-" * 72)
    print(f"{'候選位姿':<16} {'x':>7} {'y':>7} {'yaw':>7} {'吻合度':>8} {'光束':>6} {'離AMCL':>7}")
    best = None
    for name, x, y, yaw in cands:
        r = n.score(x, y, yaw)
        d = math.hypot(x - ax, y - ay)
        if r is None:
            print(f"{name:<16} {x:+7.2f} {y:+7.2f} {math.degrees(yaw):+7.1f} "
                  f"{'有效光束不足':>10} {d:7.2f}")
            continue
        sc, tot = r
        print(f"{name:<16} {x:+7.2f} {y:+7.2f} {math.degrees(yaw):+7.1f} "
              f"{sc*100:7.1f}% {tot:6d} {d:7.2f}")
        if best is None or sc > best[1]:
            best = (name, sc)
    print("-" * 72)
    if best:
        print(f"最高分：**{best[0]}**　{best[1]*100:.1f}%")
        print()
        print("★ 判讀：若某個地點的分數**明顯高於**「AMCL 現在說的」，")
        print("  代表 AMCL 收斂到了走廊上錯誤的縱向位置（沿長軸歧義）。")
        print("★ 分數接近（差幾個百分點）則分辨不出來 —— 那正是這條走廊的性質，")
        print("  不是工具的問題。這種時候要靠條件 B（前後淨空）或人眼。")
    print("=" * 72)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
