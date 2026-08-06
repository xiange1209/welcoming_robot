#!/usr/bin/env python3
"""定位驗收：兩個條件一起量，門檻從地圖計算而不是憑記憶。

## 為什麼需要這支（2026-08-06 的兩次誤判）

**誤判一**：AMCL 沿走廊縱向錯了 3 公尺，而掃描吻合度顯示 90.4%。
走廊沿長軸重複，同一份掃描在任何縱向位置都匹配得上 ——
**吻合度對縱向位置沒有分辨力**。光看它會過關。

**誤判二**：用「起點後方應該 1.3~1.5 m」判定定位失敗。但那個數字是
同一天稍早**門開著時**量的（掃描穿過門口）；門關上之後牆就在 0.62 m，
而從地圖射線算出來是 0.61 m —— 差 1 公分，AMCL 完全正確，錯的是門檻。

★ 所以這支工具做兩件事，而且**門檻一律從地圖射線算**：

    條件 A  掃描吻合度 > 90%          —— 對朝向與側向距離敏感
    條件 B  前後淨空與地圖幾何相符     —— 對縱向位置敏感

兩個條件的敏感方向互補，缺一不可。

## 用法

    python3 ~/maprun/pose_check_cc.py            # 量一次就結束
    python3 ~/maprun/pose_check_cc.py --watch    # 持續量，每 2 秒一次

沒過的話：HMI 的 `POST /api/localize/here` 設到已知地點（帶 waypoint_id），
它會發 /initialpose 再呼叫 /align_pose，然後重量。
★ 不要用 `POST /api/localize`（全域定位）—— 走廊裡它會收斂到錯的那一段，
而且實作是 0.9 m 半徑的圓弧繞行，0.99 m 的走廊會撞牆。
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

# ★ base_footprint -> laser 的縱向偏移。**是 0.089，不是 0.311。**
#   0.311 是「雷達到車頭保險桿」的距離，用在淨空計算。
#   用錯會讓每個雷射點沿車頭方向多推 0.222 m ——
#   而這個誤差在走廊裡是**隱形的**：偏移沿車頭方向，走廊裡車頭平行長軸，
#   兩面側牆在任何縱向位置都對得上。走廊量到 93~95% 一切正常，
#   進了房間才顯形（45%）。8/06 就是這樣被騙過去的。
LASER_DX = 0.089

FRONT_BUMPER = 0.40 - LASER_DX      # 車頭在雷達前方
REAR_BUMPER = 0.09 + LASER_DX       # 車尾在雷達後方

MATCH_TOL_CELLS = 2                 # 掃描點落在幾格內算「對上」
MATCH_THRESHOLD = 0.90              # 條件 A 門檻
CLEARANCE_TOL = 0.30                # 條件 B：實測與地圖差超過這個才算不過
BEAM_HALF_DEG = 10.0                # 前/後淨空取這個扇區內的最小值


class PoseCheck(Node):
    def __init__(self):
        super().__init__("pose_check_cc")
        self._lock = threading.Lock()
        self.grid = None
        self.scan = None

        # /map 是 latched（TRANSIENT_LOCAL），晚訂閱也收得到最後一筆
        self.create_subscription(
            OccupancyGrid, "map", self._map_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))
        # 用 scan_slam（已遮掉車尾的操作者）比 raw scan 乾淨。
        # 它不存在時退回 /scan —— 兩個都訂，用先到的。
        self.create_subscription(LaserScan, "scan_slam", self._scan_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan", self._scan_cb,
                                 qos_profile_sensor_data)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _map_cb(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self.grid = msg

    def _scan_cb(self, msg: LaserScan) -> None:
        with self._lock:
            self.scan = msg

    # ── 位姿 ─────────────────────────────────────────────
    def get_pose(self):
        """從 TF 取 map -> base_footprint。

        ★ 不用 /amcl_pose：它**只在車子移動時發布**，靜止驗收時永遠等不到。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"取不到 map->base_footprint：{exc}")
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return t.x, t.y, math.atan2(siny, cosy)

    # ── 地圖查詢 ─────────────────────────────────────────
    def _occupied(self, grid, mx: int, my: int) -> bool:
        if mx < 0 or my < 0 or mx >= grid.info.width or my >= grid.info.height:
            return False
        return grid.data[my * grid.info.width + mx] > 50

    def _world_to_map(self, grid, x: float, y: float):
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        return int((x - ox) / res), int((y - oy) / res)

    def raycast(self, grid, x: float, y: float, theta: float, max_range: float = 12.0):
        """從 (x, y) 往 theta 射線，回傳到第一個佔用格的距離；沒打到回 None。"""
        res = grid.info.resolution
        step = res * 0.5
        n = int(max_range / step)
        cx, cy = math.cos(theta), math.sin(theta)
        for i in range(1, n + 1):
            d = i * step
            mx, my = self._world_to_map(grid, x + cx * d, y + cy * d)
            if mx < 0 or my < 0 or mx >= grid.info.width or my >= grid.info.height:
                return None
            if self._occupied(grid, mx, my):
                return d
        return None

    # ── 條件 A：掃描吻合度 ────────────────────────────────
    def match_ratio(self, grid, scan, pose):
        x, y, yaw = pose
        # 雷達在 base_footprint 前方 LASER_DX
        lx = x + LASER_DX * math.cos(yaw)
        ly = y + LASER_DX * math.sin(yaw)
        res = grid.info.resolution
        hit = total = 0
        ang = scan.angle_min
        for r in scan.ranges:
            a = ang
            ang += scan.angle_increment
            if not (scan.range_min <= r <= scan.range_max) or r != r:
                continue
            wx = lx + r * math.cos(yaw + a)
            wy = ly + r * math.sin(yaw + a)
            mx, my = self._world_to_map(grid, wx, wy)
            total += 1
            found = False
            for dx in range(-MATCH_TOL_CELLS, MATCH_TOL_CELLS + 1):
                for dy in range(-MATCH_TOL_CELLS, MATCH_TOL_CELLS + 1):
                    if self._occupied(grid, mx + dx, my + dy):
                        found = True
                        break
                if found:
                    break
            if found:
                hit += 1
        return (hit / total if total else 0.0), total

    # ── 條件 B：前後淨空 ─────────────────────────────────
    def measured_clearance(self, scan, forward: bool) -> float:
        """從掃描取車頭（或車尾）方向 ±BEAM_HALF_DEG 扇區的最小距離。"""
        target = 0.0 if forward else math.pi
        half = math.radians(BEAM_HALF_DEG)
        best = float("inf")
        ang = scan.angle_min
        for r in scan.ranges:
            a = ang
            ang += scan.angle_increment
            if not (scan.range_min <= r <= scan.range_max) or r != r:
                continue
            d = abs(math.atan2(math.sin(a - target), math.cos(a - target)))
            if d <= half:
                best = min(best, r)
        return best

    def run_once(self) -> bool:
        with self._lock:
            grid, scan = self.grid, self.scan
        if grid is None or scan is None:
            print("等不到 /map 或 /scan（先確認 run_sensors_cc.sh 與 run_nav_cc.sh 都起來了）")
            return False
        pose = self.get_pose()
        if pose is None:
            return False
        x, y, yaw = pose

        print("=" * 66)
        print("位姿  x=%+.3f  y=%+.3f  yaw=%+.1f 度" % (x, y, math.degrees(yaw)))
        print("地圖  %dx%d 格 @ %.3f m，原點 (%+.2f, %+.2f)" % (
            grid.info.width, grid.info.height, grid.info.resolution,
            grid.info.origin.position.x, grid.info.origin.position.y))
        print("-" * 66)

        # ── 條件 A ──
        ratio, n = self.match_ratio(grid, scan, pose)
        ok_a = ratio >= MATCH_THRESHOLD
        print("條件 A  掃描吻合度 %5.1f%%  (%d 個有效光束，門檻 %.0f%%)   %s" % (
            ratio * 100, n, MATCH_THRESHOLD * 100, "通過" if ok_a else "不通過"))
        print("        ★ 它對朝向與離兩牆的距離敏感，但對走廊縱向位置**沒有分辨力**")

        # ── 條件 B ──
        lx = x + LASER_DX * math.cos(yaw)
        ly = y + LASER_DX * math.sin(yaw)
        rows = []
        ok_b = True
        for label, forward in (("車頭", True), ("車尾", False)):
            theta = yaw if forward else yaw + math.pi
            bumper = FRONT_BUMPER if forward else REAR_BUMPER
            map_d = self.raycast(grid, lx, ly, theta)
            meas_d = self.measured_clearance(scan, forward)
            if map_d is None or meas_d == float("inf"):
                rows.append((label, map_d, meas_d, None))
                continue
            diff = abs(map_d - meas_d)
            if diff > CLEARANCE_TOL:
                ok_b = False
            rows.append((label, map_d, meas_d, diff))

        print("條件 B  前後淨空與地圖幾何比對（門檻：差 > %.2f m 才算不過）" % CLEARANCE_TOL)
        for label, map_d, meas_d, diff in rows:
            md = "%.3f" % map_d if map_d is not None else " 無牆 "
            ms = "%.3f" % meas_d if meas_d != float("inf") else " 無回波"
            ds = "%.3f  %s" % (diff, "OK" if diff <= CLEARANCE_TOL else "★超標") \
                if diff is not None else "無法比對"
            print("        %s  地圖 %s m   實測 %s m   差 %s" % (label, md, ms, ds))
        print("        ★ 它對縱向位置敏感 —— 這正是條件 A 看不出來的那個方向")

        print("-" * 66)
        passed = ok_a and ok_b
        print("判定：%s" % ("★ 兩個條件都通過" if passed else "不通過"))
        if not passed:
            print()
            print("下一步：HMI 的 POST /api/localize/here 設到已知地點（帶 waypoint_id），")
            print("        它會發 /initialpose 再呼叫 /align_pose，然後重跑這支工具。")
            print("        ★ 不要用 POST /api/localize（全域定位）——走廊裡它會收斂到")
            print("          錯的那一段，而且會繞 0.9 m 的圓弧，0.99 m 走廊裡撞牆。")
        print("=" * 66)
        return passed


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", action="store_true", help="持續量，每 2 秒一次")
    args = ap.parse_args()

    rclpy.init()
    node = PoseCheck()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    # 等 /map、/scan 與 TF 都到齊
    for _ in range(100):
        with node._lock:
            ready = node.grid is not None and node.scan is not None
        if ready and node.get_pose() is not None:
            break
        time.sleep(0.1)

    try:
        if args.watch:
            while rclpy.ok():
                node.run_once()
                time.sleep(2.0)
        else:
            ok = node.run_once()
            sys.exit(0 if ok else 1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
