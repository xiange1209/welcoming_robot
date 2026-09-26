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
from std_srvs.srv import SetBool


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

        # ★★ 2026-09-23：光達左右兩側加裝了架高攝影機的鋁擠條 ★★
        #
        # 實測 100 幀 /scan：87~114 度與 252~272 度（= -108~-88）八成以上是無效值，
        # 鋁擠條離光達比 range_min 0.15 m 還近，大部分回波被驅動丟掉，
        # 但邊緣偶爾漏出 0.165 m 之類的回波。slam_toolbox 把它們建成車身旁
        # 0.26~0.38 m 的「障礙物」，planner 於是一直回 Start occupied，車子不動；
        # 車子不動 slam 又不收新掃描（minimum_travel_distance 0.2），假障礙物永遠清不掉。
        #
        # 這兩個扇區本來就被鋁擠條擋住、什麼都看不到，遮掉不損失資訊，
        # 所以**永遠啟用**，不受 set_scan_mask 開關影響（那個開關只管車尾）。
        # 格式：[min1, max1, min2, max2, ...]，各扇區在實測範圍外多留約 3 度。
        self.declare_parameter("self_mask_sectors_deg", [84.0, 118.0, 249.0, 275.0])
        flat = list(self.get_parameter("self_mask_sectors_deg").value)
        self.self_sectors = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]

        # ★★ 2026-08-19：遮蔽改成可即時開關 ★★
        #
        # 使用者指出的（正確）：**遮蔽只有建圖／錄製時需要**——那時操作者跟在車後。
        # **自主運行時沒有人在後面**，繼續遮掉 80 度等於：
        #   - AMCL 少掉 22% 的光束（240 根中約 53 根），窄走廊裡特徵本來就少
        #   - 全域成本地圖的障礙層看不到車後 —— 倒車規劃是瞎的
        #
        # 所以預設仍然是「開」（保持既有建圖行為不變），
        # 但 `path_teach_cc` 在**重播開始時關掉、結束時打開**，
        # 於是自主運行全程是 360 度。
        #
        # ★ 為什麼用服務而不是參數：`ros2 param set` 對這種要即時生效的
        #   開關可以用，但沒有回應可以確認「對方真的收到了」。
        #   服務會回 success，呼叫端才能在失敗時決定要不要繼續。
        #
        # ★ 2026-09-23：預設改成「關」。自動探索建圖時沒有人跟在車後；
        #   要跟車的時機（path_teach_cc 錄製）本來就會自己呼叫 set_scan_mask(True)。
        self.declare_parameter("mask_enabled", False)
        self.mask_enabled = bool(self.get_parameter("mask_enabled").value)
        self.mask_min = math.radians(self.get_parameter("mask_min_deg").value)
        self.mask_max = math.radians(self.get_parameter("mask_max_deg").value)
        self.create_service(SetBool, "set_scan_mask", self._set_mask_cb)

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
            f"固定遮蔽 {', '.join(f'{a:.0f}~{b:.0f}' for a, b in self.self_sectors)} 度（鋁擠條）；"
            f"車尾 {math.degrees(self.mask_min):.0f}~{math.degrees(self.mask_max):.0f} 度"
            f"{'啟用' if self.mask_enabled else '關閉'}"
        )

    def _masked(self, a: float) -> bool:
        """a 是 0~360 度。鋁擠條扇區永遠遮；車尾扇區看 mask_enabled。"""
        if any(lo <= a <= hi for lo, hi in self.self_sectors):
            return True
        return self.mask_enabled and math.degrees(self.mask_min) <= a <= math.degrees(self.mask_max)

    def _set_mask_cb(self, request, response):
        """開關車尾遮蔽。data=True 遮（錄製），False 不遮車尾（鋁擠條扇區不受影響）。"""
        was = self.mask_enabled
        self.mask_enabled = bool(request.data)
        if was != self.mask_enabled:
            self.get_logger().info(
                f"車尾遮蔽 {'啟用（建圖／錄製，擋掉跟車的人）' if self.mask_enabled else '★ 關閉 —— 全 360 度（自主運行）'}")
        response.success = True
        response.message = "masked" if self.mask_enabled else "full_360"
        return response

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
            if self._masked(a):
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
                if self._masked(a):
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
