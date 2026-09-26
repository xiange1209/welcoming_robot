#!/usr/bin/env python3

"""印出目前 /map 的已知面積 —— 比較兩次模擬「建了多少圖」用

    ros2 run smartnav_sim map_area            # 印一次
    ros2 run smartnav_sim map_area --watch 30 # 每 30 秒印一次（Ctrl+C 結束）

輸出：已知空地 m²、牆格數、地圖外框大小。
「已知空地」是最直接的建圖進度指標：探索卡住時它會停止成長，
map_service_cc 的停滯看門狗（240 秒沒增加 0.5 m²）看的就是同一件事。
"""

import argparse
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main(argv=None):
    ap = argparse.ArgumentParser(description="印出 /map 的已知面積")
    ap.add_argument("--watch", type=float, default=0.0, metavar="秒", help="每隔幾秒印一次（0 = 只印一次）")
    a, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Node("map_area")
    latest = []
    # /map 是 transient_local：晚訂閱也拿得到最後一張
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, "/map", lambda m: latest.append(m), qos)
    t0 = time.monotonic()
    try:
        while rclpy.ok():
            deadline = time.monotonic() + 5.0
            latest.clear()
            while not latest and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            if latest:
                m = latest[-1]
                r = m.info.resolution
                free = sum(1 for v in m.data if v == 0)
                occ = sum(1 for v in m.data if v > 50)
                print(f"[{time.monotonic() - t0:6.0f}s] 已知空地 {free * r * r:6.1f} m²、牆 {occ} 格、"
                      f"外框 {m.info.width * r:.1f} × {m.info.height * r:.1f} m", flush=True)
            else:
                print("（5 秒內沒收到 /map —— 建圖模式還沒開始，或 slam_toolbox 沒在跑）", flush=True)
            if a.watch <= 0:
                break
            time.sleep(a.watch)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
