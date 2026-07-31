#!/usr/bin/env python3
"""手動建圖時的健康監看。

每 15 秒寫一行到 log，盯的是「建圖會不會壞掉」的那幾個訊號，
而不是一般的系統狀態：

  scan      雷射進來的速率。掉下去代表 CPU 被搶走或驅動卡住，
            SLAM 沒有資料就只能靠里程計外推，地圖直接歪掉。
  map       已知格數。**車子在動但這個數字不長 = SLAM 沒在吃新資料**，
            這是最重要的一條，比什麼都準。
  m->o      map->odom_combined 這條 TF 的修正量。SLAM 每次
            scan match 成功就會微調它。**突然跳很大 = 掃描匹配跳到
            錯的解**（長走廊的 180 度對稱歧義就是這樣發作的）。
  tfgap     odom->base 的更新間隔。超過 0.3 秒代表 EKF 被餓到，
            之前實測 333ms 空窗會讓所有導航目標 abort。
  cmd       這 15 秒內有沒有移動指令，用來分辨「車子沒動」和
            「車子在動但地圖沒長」——後者才是故障。

用法：mapping_watch_cc.py [執行分鐘數]   預設 60
輸出：~/maprun/logs/mapping_watch.log
"""
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

LOG = os.path.expanduser("~/maprun/logs/mapping_watch.log")
PERIOD = 15.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    minutes = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init()
    node = rclpy.create_node("mapping_watch_cc")
    buf = Buffer()
    TransformListener(buf, node)

    state = {"scan": 0, "map": None, "cmd": 0}

    node.create_subscription(
        LaserScan, "/scan", lambda m: state.__setitem__("scan", state["scan"] + 1),
        QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST))
    node.create_subscription(
        OccupancyGrid, "/map", lambda m: state.__setitem__("map", m),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST))
    # 底盤是 RELIABLE，要用一樣的 QoS 才看得到真正送達的指令
    node.create_subscription(
        Twist, "/cmd_vel",
        lambda m: state.__setitem__(
            "cmd", state["cmd"] + (1 if abs(m.linear.x) > 1e-6 or abs(m.angular.z) > 1e-6 else 0)),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST))

    fp = open(LOG, "a", buffering=1)
    fp.write(f"\n===== 建圖監看開始 {time.strftime('%H:%M:%S')} =====\n")
    fp.write("時刻      scan   已知格   增量  m->o修正  tfgap  指令  備註\n")

    prev_known = None
    prev_mo = None
    last_tf_ok = time.time()
    end = time.time() + minutes * 60

    while time.time() < end and rclpy.ok():
        t0 = time.time()
        state["scan"] = 0
        state["cmd"] = 0
        while time.time() - t0 < PERIOD and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)

        hz = state["scan"] / PERIOD
        notes = []

        grid = state["map"]
        if grid is None:
            known, grew = 0, 0
            notes.append("沒有 /map")
        else:
            known = sum(1 for v in grid.data if v >= 0)
            grew = 0 if prev_known is None else known - prev_known
            prev_known = known

        # map->odom 的修正量：SLAM 掃描匹配的成果，跳太大就是匹配跑掉
        mo_txt = "  -   "
        try:
            tr = buf.lookup_transform("map", "odom_combined", rclpy.time.Time())
            cur = (tr.transform.translation.x, tr.transform.translation.y,
                   yaw_of(tr.transform.rotation))
            if prev_mo is not None:
                d = math.hypot(cur[0] - prev_mo[0], cur[1] - prev_mo[1])
                dth = abs(math.degrees(cur[2] - prev_mo[2]))
                mo_txt = f"{d:.3f}m"
                if d > 0.25 or dth > 15:
                    notes.append(f"⚠ 掃描匹配跳動 {d:.2f}m/{dth:.0f}°")
            prev_mo = cur
        except Exception:
            notes.append("查不到 map->odom")

        gap = 0.0
        try:
            tr2 = buf.lookup_transform("odom_combined", "base_footprint", rclpy.time.Time())
            stamp = tr2.header.stamp.sec + tr2.header.stamp.nanosec * 1e-9
            gap = max(0.0, time.time() - stamp)
            last_tf_ok = time.time()
        except Exception:
            gap = time.time() - last_tf_ok
            notes.append("odom TF 斷了")

        if hz < 8.0:
            notes.append(f"⚠ 雷射只有 {hz:.1f} Hz")
        if gap > 0.3:
            notes.append(f"⚠ odom TF 落後 {gap:.2f}s")
        # 最重要的一條：在動卻沒在建圖
        if state["cmd"] > 5 and grew == 0 and grid is not None:
            notes.append("⚠ 車子在動但地圖沒長 —— SLAM 可能沒吃到資料")

        fp.write(f"{time.strftime('%H:%M:%S')}  {hz:5.1f}  {known:7d} {grew:+6d}   "
                 f"{mo_txt:>7s}  {gap:5.2f}s  {state['cmd']:4d}  "
                 f"{'; '.join(notes) if notes else 'ok'}\n")

    fp.write(f"===== 監看結束 {time.strftime('%H:%M:%S')} =====\n")
    fp.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
