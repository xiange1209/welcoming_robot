#!/usr/bin/env python3

"""導航動作節點

提供導航的動作
"""

import math
import threading
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped

from smartnav_msgs.srv import GetWaypoint
from smartnav_msgs.action import Navigate


class NavigationServiceNode(Node):
    """導航動作節點類別

    提供導航的 ROS 2 動作
    """

    def __init__(self):
        """初始化導航動作節點"""
        super().__init__("navigation_service_node")

        self.server_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = ReentrantCallbackGroup()

        # 導航動作
        self.navigate_action = ActionServer(
            self, Navigate, "navigate", self.navigate_callback, callback_group=self.server_cb_group
        )

        # Nav2 導航客戶端
        self.nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=self.client_cb_group)
        # 地點查詢客戶端
        self.get_waypoint_client = self.create_client(GetWaypoint, "get_waypoint", callback_group=self.client_cb_group)

        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._amcl_pose_callback, 10, callback_group=self.client_cb_group
        )

        self.current_covariance_norm = float("inf")

        self._init_timer = self.create_timer(0.1, self._init_callback, callback_group=self.client_cb_group)

        self.get_logger().info("導航動作節點已啟動")

    def _init_callback(self) -> None:
        """初始化回調函式"""
        self._init_timer.cancel()
        self._wait_for_services()

    def _wait_for_services(self) -> None:
        """等待所有基礎服務或動作就緒"""
        services = [
            ("get_waypoint", self.get_waypoint_client),
        ]
        actions = [
            ("navigate_to_pose", self.nav2_client),
        ]
        for service_name, client in services:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f"⌛ 等待服務 {service_name}...")
        for action_name, client in actions:
            while not client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(f"⌛ 等待動作 {action_name}...")
        self.get_logger().info("✓ 所有基礎服務與動作已就緒")

    def _wait_for_future(self, future, timeout_sec: float) -> Any:
        """等待服務回應的輔助函式"""
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())

        # 阻塞當前獨立執行緒，直到被喚醒或超時
        if not event.wait(timeout=timeout_sec):
            raise TimeoutError(f"服務請求超時")

        return future.result()

    def navigate_callback(self, goal_handle):
        """處理導航請求"""
        self.get_logger().info(f"收到導航請求")

        request = goal_handle.request
        result = Navigate.Result()

        try:
            if not request.waypoint_id:
                result.success = False
                result.message = "必須提供地點ID以進行導航"
                goal_handle.abort()
                self.get_logger().warning("導航失敗: 未提供地點ID")
                return result

            if self.current_covariance_norm > 0.15:
                result.success = False
                result.message = "當前定位不夠準確，請嘗試全域定位"
                goal_handle.abort()
                self.get_logger().warning("導航失敗: 當前定位不夠準確")
                return result

            req = GetWaypoint.Request()
            req.waypoint_id = request.waypoint_id
            future = self.get_waypoint_client.call_async(req)
            try:
                response = self._wait_for_future(future, timeout_sec=5.0)
            except Exception as e:
                result.success = False
                result.message = f"無法查詢地點座標，導航失敗"
                goal_handle.abort()
                self.get_logger().error(f"查詢地點座標失敗: {str(e)}")
                return result

            if not response.success:
                result.success = False
                result.message = f"查詢地點失敗: {response.message}"
                goal_handle.abort()
                self.get_logger().warning(f"查詢地點失敗，{response.message}")
                return result

            goal = NavigateToPose.Goal()
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.header.frame_id = "map"
            goal.pose.pose = response.waypoint_info.pose
            future = self.nav2_client.send_goal_async(goal)
            try:
                nav2_goal_handle = self._wait_for_future(future, timeout_sec=5.0)
            except Exception as e:
                result.success = False
                result.message = f"無法發送導航請求，導航失敗"
                goal_handle.abort()
                self.get_logger().error(f"發送導航請求失敗: {str(e)}")
                return result

            if not nav2_goal_handle.accepted:
                result.success = False
                result.message = "導航請求被拒絕，導航失敗"
                goal_handle.abort()
                self.get_logger().warning("導航請求被拒絕")
                return result

            self.get_logger().info("Nav2 接受目標，開始導航")

            start_time = self.get_clock().now()
            max_wait_time = Duration(seconds=100.0)
            result_future = nav2_goal_handle.get_result_async()
            while not result_future.done():
                if goal_handle.is_cancel_requested:
                    cancel_future = nav2_goal_handle.cancel_goal_async()
                    try:
                        self._wait_for_future(cancel_future, timeout_sec=5.0)
                        result.success = False
                        result.message = "導航請求已被系統或使用者取消"
                        goal_handle.canceled()
                        self.get_logger().info("導航請求已被系統或使用者取消")
                        return result
                    except Exception as e:
                        result.success = False
                        result.message = f"取消導航請求失敗，請手動停止導航"
                        goal_handle.abort()
                        self.get_logger().error(f"取消導航請求失敗: {str(e)}")
                        return result

                if self.get_clock().now() - start_time > max_wait_time:
                    cancel_future = nav2_goal_handle.cancel_goal_async()
                    try:
                        self._wait_for_future(cancel_future, timeout_sec=5.0)
                        result.success = False
                        result.message = "導航請求超時，已自動取消"
                        goal_handle.canceled()
                        self.get_logger().warning("導航請求超時，已自動取消")
                        return result
                    except Exception as e:
                        result.success = False
                        result.message = f"導航請求超時且取消失敗，請手動停止導航"
                        goal_handle.abort()
                        self.get_logger().error(f"導航請求超時且取消失敗: {str(e)}")
                        return result

                self.get_clock().sleep_until(self.get_clock().now() + Duration(seconds=1.0))

            if result_future.result().status != GoalStatus.STATUS_SUCCEEDED:
                result.success = False
                result.message = "導航過程出現異常，導航失敗"
                goal_handle.abort()
                self.get_logger().error("導航過程出現異常")
                return result

            result.success = True
            result.message = "導航成功"
            goal_handle.succeed()
            self.get_logger().info(f"導航動作成功")
        except Exception as e:
            result.success = False
            result.message = f"系統出現異常，導航失敗"
            goal_handle.abort()
            self.get_logger().error(f"系統出現異常: {str(e)}")

        return result

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        cov_xx = msg.pose.covariance[0]
        cov_xy = msg.pose.covariance[1]
        cov_yy = msg.pose.covariance[7]

        self.current_covariance_norm = (cov_xx**2 + cov_yy**2) ** 0.5

        determinant = (cov_xx * cov_yy) - (cov_xy**2)
        if determinant > 0:
            self.current_entropy = 0.5 * math.log((2 * math.pi * math.e) ** 2 * determinant)
        else:
            self.current_entropy = float("inf")


def main(args=None):
    """導航動作節點進入點"""
    rclpy.init(args=args)
    node = NavigationServiceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
