#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""【未測試】轉向鏈路錄製節點（ROS 2 Jazzy）——只錄、不發任何速度指令。

雲端沒有 ROS 2，這支只做過語法檢查（python3 -m py_compile）。第一次上車請先空轉 30 秒，
看每 5 秒印出的各來源訊息數：任何一個是 0 就先修話題名，不要直接開始實驗。

用法（車上，先 source 環境）：
    python3 record_steering_chain.py --out ~/steer_diag_$(date +%H%M).csv
    # 另開終端機，每換一個實驗階段就打一個標記（標記名稱 = 階段名稱，analyze 用它切段）：
    ros2 topic pub --once /steer_diag_marker std_msgs/msg/String "{data: straight_fwd}"
    ...
    ros2 topic pub --once /steer_diag_marker std_msgs/msg/String "{data: end}"
    # Ctrl+C 結束錄製，然後離線分析：
    python3 analyze_steering_logs.py analyze ~/steer_diag_XXXX.csv

錄的東西（對應 CSV 的 src 欄，格式說明見 analyze_steering_logs.py 檔頭）：
    cmd_smoothed  cmd_vel_smoothed   path_teach 的輸出（取代 velocity_smoother；launch 裡 cmd_topic 就是它）
    cmd_trimmed   cmd_vel_trimmed    steering_trim 之後（零位補償＋0.15 s 低通）
    cmd_prefloor  cmd_vel_prefloor   collision_monitor 之後
    cmd_final     cmd_vel            cmd_vel_floor 之後 = 送給驅動的最後一站（也是 ros2 topic pub 直發的地方）
    odom_raw      odom               驅動由『上行 Vz 積分』得到（HMI 外推用的就是這個）
    odom_ekf      odom_combined      EKF 融合後（AMCL／導航用的里程計）
    imu_raw       imu/data_raw       原始陀螺
    imu_unbiased  imu/data_unbiased  imu_bias_corrector_cc 扣零偏後（EKF 實際吃的）
    amcl          amcl_pose          HMI 優先顯示的位姿（同時記 header 時間，用來量處理延遲）
    tf_map_base   TF map->base_footprint（10 Hz 輪詢；不依賴任何訂閱，是『現在最好的位姿』）
    marker        steer_diag_marker  實驗階段標記
注意：本節點全部以 BEST_EFFORT 訂閱（與 RELIABLE／BEST_EFFORT 的發布端都相容），不會干擾任何現有訂閱。
"""
import argparse
import csv
import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

COLUMNS = ["t", "stamp", "src", "label", "v", "w", "x", "y", "yaw"]


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_of(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class SteeringChainRecorder(Node):
    def __init__(self, args):
        super().__init__("steer_diag_recorder")
        self._lock = threading.Lock()
        os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
        self._fh = open(args.out, "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._fh)
        self._csv.writerow(COLUMNS)
        self._counts = {}
        self._last_flush = time.monotonic()
        qos = qos_profile_sensor_data

        for src, topic in (("cmd_smoothed", args.cmd_smoothed), ("cmd_trimmed", args.cmd_trimmed),
                           ("cmd_prefloor", args.cmd_prefloor), ("cmd_final", args.cmd_final)):
            self.create_subscription(Twist, topic, self._make_twist_cb(src), qos)
        for src, topic in (("odom_raw", args.odom_raw), ("odom_ekf", args.odom_ekf)):
            self.create_subscription(Odometry, topic, self._make_odom_cb(src), qos)
        for src, topic in (("imu_raw", args.imu_raw), ("imu_unbiased", args.imu_unbiased)):
            self.create_subscription(Imu, topic, self._make_imu_cb(src), qos)
        self.create_subscription(PoseWithCovarianceStamped, args.amcl, self._amcl_cb, qos)
        self.create_subscription(String, args.marker, self._marker_cb, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._map_frame, self._base_frame = args.map_frame, args.base_frame
        self.create_timer(0.1, self._tf_poll)
        self.create_timer(5.0, self._report)
        self.get_logger().info(f"錄製中 -> {args.out}（Ctrl+C 結束）。每 5 秒印各來源訊息數，任何一個是 0 先修話題名。")

    # ── 寫檔 ────────────────────────────────────────────────
    def _row(self, src, stamp=None, label="", v=None, w=None, x=None, y=None, yaw=None):
        fmt = lambda a: "" if a is None else (f"{a:.6f}" if isinstance(a, float) else a)
        with self._lock:
            self._csv.writerow([f"{time.time():.4f}", fmt(stamp), src, label, fmt(v), fmt(w), fmt(x), fmt(y), fmt(yaw)])
            self._counts[src] = self._counts.get(src, 0) + 1
            if time.monotonic() - self._last_flush > 1.0:
                self._fh.flush()
                self._last_flush = time.monotonic()

    # ── 回呼 ────────────────────────────────────────────────
    def _make_twist_cb(self, src):
        def cb(msg: Twist):
            self._row(src, v=float(msg.linear.x), w=float(msg.angular.z))
        return cb

    def _make_odom_cb(self, src):
        def cb(msg: Odometry):
            p = msg.pose.pose
            self._row(src, stamp=stamp_of(msg.header), v=float(msg.twist.twist.linear.x), w=float(msg.twist.twist.angular.z),
                      x=float(p.position.x), y=float(p.position.y), yaw=yaw_of(p.orientation))
        return cb

    def _make_imu_cb(self, src):
        def cb(msg: Imu):
            self._row(src, stamp=stamp_of(msg.header), w=float(msg.angular_velocity.z))
        return cb

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        self._row("amcl", stamp=stamp_of(msg.header), x=float(p.position.x), y=float(p.position.y), yaw=yaw_of(p.orientation))

    def _marker_cb(self, msg: String):
        self._row("marker", label=msg.data.strip())
        self.get_logger().info(f"標記：{msg.data.strip()}")

    def _tf_poll(self):
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001  —— 沒定位時查不到是正常的，不要刷屏
            return
        t, r = tf.transform.translation, tf.transform.rotation
        self._row("tf_map_base", stamp=stamp_of(tf.header), x=float(t.x), y=float(t.y), yaw=yaw_of(r))

    def _report(self):
        with self._lock:
            c = dict(self._counts)
        self.get_logger().info("訊息數：" + "  ".join(f"{k}={v}" for k, v in sorted(c.items())))

    def close(self):
        with self._lock:
            self._fh.flush()
            self._fh.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description="轉向鏈路錄製（只錄不發指令）")
    ap.add_argument("--out", default=os.path.expanduser(f"~/steer_diag_{time.strftime('%Y%m%d_%H%M%S')}.csv"))
    ap.add_argument("--cmd-smoothed", default="cmd_vel_smoothed")
    ap.add_argument("--cmd-trimmed", default="cmd_vel_trimmed")
    ap.add_argument("--cmd-prefloor", default="cmd_vel_prefloor")
    ap.add_argument("--cmd-final", default="cmd_vel")
    ap.add_argument("--odom-raw", default="odom")
    ap.add_argument("--odom-ekf", default="odom_combined")
    ap.add_argument("--imu-raw", default="imu/data_raw")
    ap.add_argument("--imu-unbiased", default="imu/data_unbiased")
    ap.add_argument("--amcl", default="amcl_pose")
    ap.add_argument("--marker", default="steer_diag_marker")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_footprint")
    args, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = SteeringChainRecorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
