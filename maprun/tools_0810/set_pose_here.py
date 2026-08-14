#!/usr/bin/env python3
"""把 AMCL 的定位直接設到某個已知地點。

★ 走廊裡不要用全域定位（GlobalLocalization）：沿長軸重複，粒子會收斂到錯的那一段，
   而且重定位過程車子會繞 0.9 m 的圓弧，0.99 m 寬的走廊裡會撞牆。
   已知車子在哪的時候，直接發 /initialpose 才是對的做法。
"""
import sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from smartnav_msgs.srv import ListWaypoints

WANT = sys.argv[1] if len(sys.argv) > 1 else "起點"


class Setter(Node):
    def __init__(self):
        super().__init__("set_pose_here")
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.cli = self.create_client(ListWaypoints, "list_waypoints")

    def run(self):
        if not self.cli.wait_for_service(timeout_sec=10.0):
            print("  ✗ list_waypoints 服務沒回應"); return False
        fut = self.cli.call_async(ListWaypoints.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        res = fut.result()
        if res is None:
            print("  ✗ 取地點清單失敗"); return False

        wp = next((w for w in res.waypoints_info if w.waypoint_name == WANT), None)
        if wp is None:
            print(f"  ✗ 找不到地點「{WANT}」"); return False

        p = wp.pose
        print(f"  目標地點：{wp.waypoint_name}")
        print(f"    x={p.position.x:+.3f}  y={p.position.y:+.3f}")
        print(f"    quat z={p.orientation.z:+.4f} w={p.orientation.w:+.4f}")

        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose = p
        # AMCL 的慣例：對角線放 x/y 的變異數與 yaw 的變異數。
        # 0.25 = (0.5 m)^2、0.0685 = (15 度)^2 —— 我們相當確定位置，
        # 但留一點空間讓 AMCL 用掃描把最後幾公分收斂掉。
        cov = [0.0] * 36
        cov[0] = 0.25; cov[7] = 0.25; cov[35] = 0.0685
        m.pose.covariance = cov

        # 發三次：AMCL 的訂閱是 transient_local，但剛啟動時偶爾會漏第一則
        for _ in range(3):
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.3)
            time.sleep(0.4)
        print("  ✓ 已發布 /initialpose ×3")
        return True


def main():
    rclpy.init()
    n = Setter()
    ok = n.run()
    n.destroy_node(); rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
