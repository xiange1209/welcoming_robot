#!/usr/bin/env python3
"""把指定角度扇區的光束變成無效值，再發布給 SLAM 用。

## 為什麼需要這個節點

建圖時操作者跟在車子後面走。在 0.99 m 的窄走廊裡，一個人會遮掉
掃描很大一塊角度，而且相對位置隨走動一直變 —— 對 slam_toolbox 來說
這是一團會移動的「牆」，局部匹配被它拖垮之後，很容易掉進錯誤的
迴路閉合（2026-07-31 實測：map->odom 一步跳 14~22 公尺，
比整張地圖還大）。

雷達驅動自己的 `angle_disable_min/max` 參數只在**啟動時**讀一次，
執行中用 `ros2 param set` 改了沒有效果（實測後方 87 根光束 85 根有效）。
所以在 ROS 端過濾。

## 架構決定：只過濾 SLAM 的輸入，不動 /scan 本身

    /scan ──┬── costmap / collision_monitor（看得到人 -> 不會撞到人）
            └── scan_filter_cc ── /scan_slam ── slam_toolbox（看不到人 -> 地圖乾淨）

避障必須看得到人，建圖必須看不到人 —— 這是同一份資料的兩種需求，
所以分成兩個話題，而不是在驅動端把資料弄掉。

## 失效模式

這個節點掛掉的話 /scan_slam 就沒有資料，slam_toolbox 會停止更新地圖
（症狀：車子在動但已知格數不長，mapping_watch 會報警）。
不會安靜地退化成「人被建進地圖」。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFilterCcNode(Node):
    def __init__(self):
        super().__init__("scan_filter_cc_node")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_slam")
        # 車體座標系的角度，度。0 = 車頭正前方，逆時針為正，180 = 正後方。
        # 預設遮 140~220：以正後方為中心各 40 度，蓋住跟車走的人，
        # 但保留兩側 90/270 度附近的走廊牆面（那是窄走廊裡僅有的特徵）。
        self.declare_parameter("mask_min_deg", 140.0)
        self.declare_parameter("mask_max_deg", 220.0)

        self.mask_min = math.radians(self.get_parameter("mask_min_deg").value)
        self.mask_max = math.radians(self.get_parameter("mask_max_deg").value)

        self.pub = self.create_publisher(
            LaserScan, self.get_parameter("output_topic").value, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, self.get_parameter("input_topic").value,
            self._cb, qos_profile_sensor_data
        )
        self._masked_last = 0
        self._log_timer = self.create_timer(30.0, self._report)
        self.get_logger().info(
            f"掃描過濾: {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}，"
            f"遮蔽 {math.degrees(self.mask_min):.0f}~{math.degrees(self.mask_max):.0f} 度（車尾扇區）"
        )

    def _cb(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        ranges = list(msg.ranges)
        masked = 0
        for i in range(len(ranges)):
            # 這顆雷達的 angle_min 是 -180 度，先轉成 0~360 再比
            a = math.degrees(msg.angle_min + i * msg.angle_increment) % 360.0
            if math.degrees(self.mask_min) <= a <= math.degrees(self.mask_max):
                # inf = 「這個方向沒有回波」。slam_toolbox 會直接忽略，
                # 不會把它當成 range_max 處有一面牆。
                ranges[i] = float("inf")
                masked += 1
        out.ranges = ranges
        # intensities 有的驅動不填；長度對不上會讓下游丟掉整筆掃描
        if len(msg.intensities) == len(msg.ranges):
            ints = list(msg.intensities)
            for i in range(len(ints)):
                a = math.degrees(msg.angle_min + i * msg.angle_increment) % 360.0
                if math.degrees(self.mask_min) <= a <= math.degrees(self.mask_max):
                    ints[i] = 0.0
            out.intensities = ints

        self._masked_last = masked
        self.pub.publish(out)

    def _report(self):
        self.get_logger().debug(f"上一筆掃描遮蔽 {self._masked_last} 根光束")


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterCcNode()
    try:
        rclpy.spin(node)   # 單一回呼、純運算，預設單執行緒 executor 就對了
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
