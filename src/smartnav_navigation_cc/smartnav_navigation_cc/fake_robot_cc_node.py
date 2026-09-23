#!/usr/bin/env python3

"""假機器人節點 (_cc) —— 沒有底座時用來驗證整條導航鏈

這不是 Gazebo，是純 2D 幾何模擬，Pi 4 跑得動。它取代
turn_on_wheeltec_robot 那一整包，提供 nav2 / slam_toolbox 需要的最小輸入：

    訂閱  /cmd_vel          (geometry_msgs/Twist —— 跟 wheeltec_robot_node 同型別)
    發布  /scan             (sensor_msgs/LaserScan，對虛擬房間做光線投射)
    發布  /odom_combined    (nav_msgs/Odometry)
    發布  TF odom_combined -> base_footprint  (阿克曼運動學積分)
    發布  TF base_footprint -> base_link -> laser (靜態，取自 robot_model.yaml)

**阿克曼運動學是刻意模擬的**，不是簡單的差速積分：

    最小轉彎半徑 R = 0.8 m  =>  |ω| <= |v| / R

    所以 v = 0 時 ω 一律被壓成 0 —— 車子無法原地旋轉。
    這一點很重要：舊版的全域定位發原地旋轉指令、恢復行為用 Spin，
    在真車上都是無效動作，但如果模擬器用差速積分就會「假裝成功」，
    測不出問題。這裡照實模擬，跑不動的東西在模擬裡也跑不動。

用法：
    ros2 launch smartnav_navigation_cc fake_robot_cc.launch.py
    # 另一個終端機照常啟動導航
    ros2 launch smartnav_navigation_cc nav_bringup_cc.launch.py start_mode:=mapping
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class FakeRobotCcNode(Node):
    """虛擬阿克曼底盤 + 虛擬雷射"""

    def __init__(self):
        super().__init__("fake_robot_cc_node")

        # ---- 底盤參數 (對齊 senior_akm 實車) ----
        self.declare_parameter("min_turning_radius", 0.80)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("update_rate_hz", 30.0)

        # ---- 雷射參數 (對齊 lslidar x10) ----
        self.declare_parameter("scan_rate_hz", 10.0)
        self.declare_parameter("scan_samples", 360)
        self.declare_parameter("scan_range_max", 12.0)
        self.declare_parameter("scan_range_min", 0.05)
        self.declare_parameter("scan_noise_stddev", 0.01)
        # base_footprint -> laser，取自 turn_on_wheeltec_robot 的 robot_model.yaml
        # senior_akm: base_to_laser: [0.08874, 0.00067, 0.11102, 0, 0, 0]
        self.declare_parameter("laser_offset_x", 0.08874)
        self.declare_parameter("laser_offset_y", 0.00067)
        self.declare_parameter("laser_offset_z", 0.11102)

        # ---- 起始位姿 ----
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        self.min_turning_radius = float(self.get_parameter("min_turning_radius").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.cmd_vel_timeout_sec = float(self.get_parameter("cmd_vel_timeout_sec").value)
        update_rate = float(self.get_parameter("update_rate_hz").value)

        self.scan_samples = int(self.get_parameter("scan_samples").value)
        self.scan_range_max = float(self.get_parameter("scan_range_max").value)
        self.scan_range_min = float(self.get_parameter("scan_range_min").value)
        self.scan_noise_stddev = float(self.get_parameter("scan_noise_stddev").value)
        scan_rate = float(self.get_parameter("scan_rate_hz").value)

        self.laser_offset_x = float(self.get_parameter("laser_offset_x").value)
        self.laser_offset_y = float(self.get_parameter("laser_offset_y").value)
        self.laser_offset_z = float(self.get_parameter("laser_offset_z").value)

        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.yaw = float(self.get_parameter("initial_yaw").value)

        self.v = 0.0
        self.w = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.world = self._build_world()

        # ---- 介面 ----
        # 訂閱 Twist 而不是 TwistStamped：wheeltec_robot_node 收的是 Twist，
        # nav2 的 enable_stamped_cmd_vel 也設成 false，要跟真車一致。
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, "odom_combined", 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        self.last_update = self.get_clock().now()
        self.create_timer(1.0 / update_rate, self._update_motion)
        self.create_timer(1.0 / scan_rate, self._publish_scan)

        self.get_logger().info(
            f"假機器人已啟動 (阿克曼, 最小轉彎半徑 {self.min_turning_radius} m) —— "
            f"起點 ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.yaw):.0f}°)"
        )

    # ==================================================================
    # 虛擬世界
    # ==================================================================
    @staticmethod
    def _build_world():
        """虛擬房間：線段的集合 [(x1, y1, x2, y2), ...]

        一個 10 x 8 m 的房間，中間有隔間與家具，讓 SLAM 有特徵可以匹配，
        也讓規劃器有真的障礙物要繞。走道寬度刻意留 1.2 m 以上
        (senior_akm 車寬 0.37 + 兩側膨脹 0.35，最窄需要約 1.1 m)。
        """
        walls = [
            # 外牆 (-5,-4) 到 (5,4)
            (-5.0, -4.0, 5.0, -4.0),
            (5.0, -4.0, 5.0, 4.0),
            (5.0, 4.0, -5.0, 4.0),
            (-5.0, 4.0, -5.0, -4.0),
            # 中間隔間牆，留一個 1.6 m 的門
            (0.0, -4.0, 0.0, -1.2),
            (0.0, 0.4, 0.0, 4.0),
            # 右側小房間
            (2.5, 1.0, 5.0, 1.0),
            (2.5, 1.0, 2.5, 2.6),
            # 左下角的家具 (一張桌子)
            (-3.5, -2.5, -2.0, -2.5),
            (-2.0, -2.5, -2.0, -1.5),
            (-2.0, -1.5, -3.5, -1.5),
            (-3.5, -1.5, -3.5, -2.5),
        ]
        return walls

    def _raycast(self, ox: float, oy: float, angle: float) -> float:
        """從 (ox, oy) 沿 angle 投射，回傳到最近牆面的距離"""
        dx = math.cos(angle)
        dy = math.sin(angle)
        best = self.scan_range_max

        for x1, y1, x2, y2 in self.world:
            # 射線 (ox,oy)+t*(dx,dy) 與線段 (x1,y1)-(x2,y2) 求交
            sx = x2 - x1
            sy = y2 - y1
            denom = dx * sy - dy * sx
            if abs(denom) < 1e-12:
                continue  # 平行
            t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
            u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
            if t >= 0.0 and 0.0 <= u <= 1.0 and t < best:
                best = t

        return best

    # ==================================================================
    # 運動
    # ==================================================================
    def _cmd_vel_callback(self, msg: Twist) -> None:
        v = max(-self.max_linear_speed, min(self.max_linear_speed, msg.linear.x))
        w = msg.angular.z

        # === 阿克曼約束 ===
        # 轉彎半徑 R = v / ω 必須 >= min_turning_radius，
        # 也就是 |ω| <= |v| / R_min。速度為 0 時 ω 只能是 0：真車無法原地旋轉。
        max_w = abs(v) / self.min_turning_radius
        if abs(w) > max_w:
            w = math.copysign(max_w, w)

        self.v = v
        self.w = w
        self.last_cmd_time = self.get_clock().now()

    def _update_motion(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds / 1e9
        self.last_update = now
        if dt <= 0.0 or dt > 1.0:
            return

        # 指令逾時就停車，跟真實底盤的行為一致
        if (now - self.last_cmd_time).nanoseconds / 1e9 > self.cmd_vel_timeout_sec:
            self.v = 0.0
            self.w = 0.0

        if abs(self.v) > 1e-6 or abs(self.w) > 1e-6:
            # 用中點法積分，轉彎時的軌跡比尤拉法準
            mid_yaw = self.yaw + self.w * dt / 2.0
            self.x += self.v * math.cos(mid_yaw) * dt
            self.y += self.v * math.sin(mid_yaw) * dt
            self.yaw = math.atan2(
                math.sin(self.yaw + self.w * dt), math.cos(self.yaw + self.w * dt)
            )

        stamp = now.to_msg()

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom_combined"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = yaw_to_quaternion(self.yaw)
        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom_combined"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

    # ==================================================================
    # 感測
    # ==================================================================
    def _publish_static_tf(self) -> None:
        """base_footprint -> base_link -> laser

        base_link 也要發：waypoint/navigation 相關節點與 URDF 都以它為基準。
        這裡 base_footprint 與 base_link 只差一個高度，跟實車的 base_to_link 一致。
        """
        base_link = TransformStamped()
        base_link.header.stamp = self.get_clock().now().to_msg()
        base_link.header.frame_id = "base_footprint"
        base_link.child_frame_id = "base_link"
        base_link.transform.translation.z = 0.0423  # robot_model.yaml: senior_akm base_to_link
        base_link.transform.rotation.w = 1.0

        laser = TransformStamped()
        laser.header.stamp = self.get_clock().now().to_msg()
        laser.header.frame_id = "base_footprint"
        laser.child_frame_id = "laser"
        laser.transform.translation.x = self.laser_offset_x
        laser.transform.translation.y = self.laser_offset_y
        laser.transform.translation.z = self.laser_offset_z
        laser.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform([base_link, laser])

    def _publish_scan(self) -> None:
        import random

        # 雷射裝在 base_footprint 前方一點，投射起點要用雷射的世界座標
        lx = self.x + self.laser_offset_x * math.cos(self.yaw) - self.laser_offset_y * math.sin(self.yaw)
        ly = self.y + self.laser_offset_x * math.sin(self.yaw) + self.laser_offset_y * math.cos(self.yaw)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2.0 * math.pi / self.scan_samples
        msg.range_min = self.scan_range_min
        msg.range_max = self.scan_range_max
        msg.scan_time = 0.1
        msg.time_increment = 0.0

        ranges = []
        for i in range(self.scan_samples):
            a = msg.angle_min + i * msg.angle_increment
            d = self._raycast(lx, ly, self.yaw + a)
            if d >= self.scan_range_max:
                ranges.append(float("inf"))
            else:
                if self.scan_noise_stddev > 0.0:
                    d += random.gauss(0.0, self.scan_noise_stddev)
                ranges.append(max(self.scan_range_min, d))

        msg.ranges = ranges
        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobotCcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
