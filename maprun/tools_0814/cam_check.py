#!/usr/bin/env python3
"""直接訂閱相機話題，繞開 ros2 topic hz。

★ 2026-08-14 教訓：這台機器負載高的時候 `ros2 topic hz` / `topic info` 會逾時，
  而逾時看起來跟「沒有資料」一模一樣。今天我因此誤判「相機一幀都沒發」，
  害使用者白插拔了一次相機 —— 實際上當時是 6.40 Hz。
  **CLI 逾時不是證據，自己開訂閱者才算數。**
"""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0


class Check(Node):
    def __init__(self):
        super().__init__("cam_check")
        self.n = {}
        rel = QoSProfile(depth=5)
        rel.reliability = ReliabilityPolicy.RELIABLE
        specs = [
            ("/camera/color/image_raw", Image, qos_profile_sensor_data, "color BE"),
            ("/camera/color/image_raw", Image, rel, "color REL"),
            ("/camera/color/image_raw/compressed", CompressedImage, qos_profile_sensor_data, "color/comp BE"),
            ("/camera/depth/image_raw", Image, qos_profile_sensor_data, "depth BE"),
        ]
        for topic, typ, qos, label in specs:
            self.n[label] = 0
            self.create_subscription(typ, topic, self._mk(label), qos)

    def _mk(self, label):
        def cb(_):
            self.n[label] += 1
        return cb


def main():
    rclpy.init(); c = Check()
    print(f"  訂閱 {DUR:.0f} 秒…")
    t0 = time.time()
    while time.time() - t0 < DUR:
        rclpy.spin_once(c, timeout_sec=0.05)
    dt = time.time() - t0
    print()
    any_data = False
    for label, n in c.n.items():
        if n:
            any_data = True
        print(f"  {'✓' if n else '✗'} {label:16s} {n:4d} 幀  ({n/dt:5.2f} Hz)")
    print()
    print("  ★ 全部 0 幀 = 驅動有跑但沒抓到影像流" if not any_data else "  ★ 有資料，相機是好的")
    rclpy.try_shutdown()
    return 0 if any_data else 1


if __name__ == "__main__":
    sys.exit(main())
