#!/usr/bin/env python3
"""車子停在走廊中間、AMCL 跟丟時，用雷射在地圖上重新找回位姿。

★ 跟既有兩支的差別
    pose_check_cc.py    只回答「現在這個估計對不對」，不給正確答案
    where_am_i.py       只比對**已知地點**，車子不在地點上就沒轍
    /align_pose         要求吻合度提升 5% 才接受，而且走廊長軸幾乎沒有分辨力
這支在目前估計附近做自由搜尋（含完整 360 度），所以救得回「脫困動作把車推歪、
AMCL 整團粒子留在錯的地方」這種情況 —— 2026-08-17 導航到大廳失敗就是這樣。

★ 走廊 180 度對稱是這裡最大的陷阱
長走廊把車頭轉半圈，兩側牆的雷射點照樣落在牆上，分數幾乎一樣。
所以這支**會把前三名一起印出來**，分數接近時明講「分不出來」而不是硬選一個。
真的分不出來時，最可靠的作法仍然是把車推回已知地點再設定位。

用法：
    python3 relocalize.py              # 只搜尋並印出結果，不改任何東西
    python3 relocalize.py --set        # 找到且不歧義時，直接寫進 AMCL
    python3 relocalize.py --set 3.0    # 搜尋範圍改成 +-3.0 m
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import SetInitialPose
import tf2_ros

LX_OFF, LY_OFF = 0.08874, 0.00067      # 雷射相對 base_footprint（robot_model.yaml）

DO_SET = "--set" in sys.argv
# ★★ 2026-08-19：預設只搜「目前朝向 ±90 度」，不再盲搜完整 360 度 ★★
#
# 走廊 180 度對稱害這支工具連續三次回報「分不出來」（100.0% vs 99.3%、
# 兩個解剛好差 180 度）。但那個歧義**只存在於雷射資料裡** ——
# 里程計完全知道車頭朝哪：車子沒有被人抬起來轉半圈，
# 而輪子轉了半圈的話 odom 一定會看到。搜完整 360 度等於主動把這個先驗丟掉。
#
# 所以預設信任目前 tf 的朝向到 ±90 度，只用掃描匹配做細修。
# ★ 什麼時候要用 `--free360`：**車子被人搬動過或抬起來過**（odom 沒看到那段位移），
#   例如開機時、或使用者說「我把它搬回原點了」。那時先驗才是不可信的。
FREE360 = "--free360" in sys.argv
_spans = [a for a in sys.argv[1:] if not a.startswith("--")]
SPAN = float(_spans[0]) if _spans else 2.0


class Relocalize(Node):
    def __init__(self):
        super().__init__("relocalize")
        self.grid = None
        self.scan = None
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, "/map", self._map, qos)
        self.create_subscription(LaserScan, "/scan_filtered", self._sc, 10)
        self.create_subscription(LaserScan, "/scan", self._sc, 10)
        self.buf = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.buf, self)

    def _map(self, m):
        if self.grid is None:
            self.grid = m

    def _sc(self, m):
        self.scan = m

    def current(self):
        try:
            t = self.buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                2.0 * math.atan2(q.z, q.w))


def main():
    rclpy.init()
    n = Relocalize()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 15.0:
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.grid is not None and n.scan is not None and n.current() is not None:
            break
    if n.grid is None or n.scan is None:
        print("✗ 收不到 /map 或雷射"); return 1
    cur = n.current()
    if cur is None:
        print("✗ 取不到 map->base_footprint"); return 1

    info = n.grid.info
    res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    w, h = info.width, info.height
    occ = set()
    for gy in range(h):
        row = gy * w
        for gx in range(w):
            if n.grid.data[row + gx] >= 65:
                occ.add((gx, gy))

    def pts(step):
        out = []
        for i in range(0, len(n.scan.ranges), step):
            r = n.scan.ranges[i]
            if r <= 0.0 or math.isinf(r) or math.isnan(r) or r > 8.0:
                continue
            a = n.scan.angle_min + i * n.scan.angle_increment
            out.append((r * math.cos(a), r * math.sin(a)))
        return out

    coarse_pts, fine_pts = pts(6), pts(3)

    def score(P, bx, by, byaw):
        c, s = math.cos(byaw), math.sin(byaw)
        lx = bx + LX_OFF * c - LY_OFF * s
        ly = by + LX_OFF * s + LY_OFF * c
        hit = 0
        for dx, dy in P:
            wx = lx + dx * c - dy * s
            wy = ly + dx * s + dy * c
            gx = int((wx - ox) / res); gy = int((wy - oy) / res)
            if (gx, gy) in occ or (gx+1, gy) in occ or (gx-1, gy) in occ \
                    or (gx, gy+1) in occ or (gx, gy-1) in occ:
                hit += 1
        return hit / len(P) if P else 0.0

    print(f"  目前估計 x={cur[0]:+.3f} y={cur[1]:+.3f} yaw={math.degrees(cur[2]):+.1f}°"
          f"　吻合度 {score(fine_pts, *cur)*100:.1f}%")
    print(f"  搜尋 ±{SPAN:.1f} m、完整 360°　雷射 {len(coarse_pts)}/{len(fine_pts)} 點…")

    cands = []
    nxy = int(SPAN / 0.10)
    t0 = time.time()
    # 5 度一格。預設 ±90 度（37 格），--free360 才掃完整 360 度（72 格）
    yaw_range = range(72) if FREE360 else range(-18, 19)
    print(f"  朝向搜尋範圍：{'完整 360 度（--free360）' if FREE360 else '目前朝向 ±90 度'}"
          f"　—— 里程計知道車頭朝哪，盲搜 360 度會自己製造 180 度歧義")
    for iy in yaw_range:
        yaw = cur[2] + math.radians(iy * 5.0)
        for ix in range(-nxy, nxy + 1):
            x = cur[0] + ix * 0.10
            for jy in range(-nxy, nxy + 1):
                y = cur[1] + jy * 0.10
                cands.append((score(coarse_pts, x, y, yaw), x, y, yaw))
    cands.sort(reverse=True)
    print(f"  粗搜尋 {len(cands)} 個候選、{time.time()-t0:.0f} 秒")

    # 取彼此距離夠遠的前幾名分別細搜 —— 這樣才看得出對稱歧義
    picks = []
    for s, x, y, yaw in cands:
        if all(math.hypot(x - p[1], y - p[2]) > 0.6 or
               abs(math.degrees(math.atan2(math.sin(yaw - p[3]), math.cos(yaw - p[3])))) > 45
               for p in picks):
            picks.append((s, x, y, yaw))
        if len(picks) >= 3:
            break

    finals = []
    for _, x, y, yaw in picks:
        best = (score(fine_pts, x, y, yaw), x, y, yaw)
        for iy in range(-6, 7):
            yy = yaw + math.radians(iy * 1.0)
            for ix in range(-5, 6):
                for jy in range(-5, 6):
                    xx, yv = x + ix * 0.03, y + jy * 0.03
                    s = score(fine_pts, xx, yv, yy)
                    if s > best[0]:
                        best = (s, xx, yv, yy)
        finals.append(best)
    finals.sort(reverse=True)

    print("\n" + "=" * 66)
    for i, (s, x, y, yaw) in enumerate(finals, 1):
        # 搜尋是從目前估計往上累加角度，不正規化會印出 466.1° 這種看不懂的值
        deg = (math.degrees(yaw) + 180.0) % 360.0 - 180.0
        print(f"  第 {i} 名  {s*100:5.1f}%   x={x:+.3f} y={y:+.3f} yaw={deg:+.1f}°")
    print("-" * 66)
    top = finals[0]
    second = finals[1] if len(finals) > 1 else None
    ok = top[0] >= 0.85 and (second is None or top[0] - second[0] >= 0.08)
    if not ok:
        if second and top[0] - second[0] < 0.08:
            print("  ✗ **分不出來** —— 前兩名太接近，很可能是走廊 180° 對稱。")
        else:
            print(f"  ✗ 最高分只有 {top[0]*100:.1f}%，不夠可信。")
        print("     最可靠的作法：把車推回一個已知地點，再用 where_am_i.py + set_initial_pose。")
    else:
        print(f"  ✓ 可信：{top[0]*100:.1f}%，領先第二名 "
              f"{(top[0]-(second[0] if second else 0))*100:.1f} 個百分點")
        if DO_SET:
            cli = n.create_client(SetInitialPose, "/set_initial_pose")
            if cli.wait_for_service(timeout_sec=5.0):
                req = SetInitialPose.Request()
                p = PoseWithCovarianceStamped()
                p.header.frame_id = "map"
                p.pose.pose.position.x = top[1]
                p.pose.pose.position.y = top[2]
                p.pose.pose.orientation.z = math.sin(top[3] / 2)
                p.pose.pose.orientation.w = math.cos(top[3] / 2)
                cov = [0.0] * 36
                cov[0] = cov[7] = 0.05
                cov[35] = 0.02
                p.pose.covariance = cov
                req.pose = p
                f = cli.call_async(req)
                rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
                print("  ✓ 已寫進 AMCL")
            else:
                print("  ✗ /set_initial_pose 服務不在")
        else:
            print("  （加上 --set 才會真的寫進 AMCL）")
    print("=" * 66)
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
