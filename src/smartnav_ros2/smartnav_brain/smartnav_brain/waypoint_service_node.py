#!/usr/bin/env python3

"""地點服務節點

提供建立地點和查詢地點的服務
"""

import copy
import json
import math
import uuid
import threading
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, TwistStamped, PoseWithCovarianceStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import GetState, ChangeState
from tf2_ros import Buffer, TransformListener

from smartnav_msgs.msg import WaypointInfo
from smartnav_msgs.srv import CreateWaypoint, ListWaypoints, GetWaypoint
from smartnav_msgs.action import GlobalLocalization


class WaypointServiceNode(Node):
    """地點服務節點類別

    提供建立和查詢地點的 ROS 2 服務
    """

    def __init__(self):
        """初始化地點服務節點"""
        super().__init__("waypoint_service_node")

        self.server_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = ReentrantCallbackGroup()

        # 建立地點服務
        self.create_waypoint_srv = self.create_service(
            CreateWaypoint, "create_waypoint", self.create_waypoint_callback, callback_group=self.server_cb_group
        )
        # 列出地點服務
        self.list_waypoints_srv = self.create_service(
            ListWaypoints, "list_waypoints", self.list_waypoints_callback, callback_group=self.server_cb_group
        )
        # 查詢地點服務
        self.get_waypoint_srv = self.create_service(
            GetWaypoint, "get_waypoint", self.get_waypoint_callback, callback_group=self.server_cb_group
        )

        # 全域定位動作
        self.global_localization_action = ActionServer(
            self,
            GlobalLocalization,
            "global_localization",
            self.global_localization_callback,
            callback_group=self.server_cb_group,
        )

        # Nav2 全域定位客戶端
        self.nav2_global_localization_client = self.create_client(
            Empty, "/reinitialize_global_localization", callback_group=self.client_cb_group
        )
        # Nav2 生命週期管理客戶端
        self.amcl_get_state_client = self.create_client(
            GetState, "/amcl/get_state", callback_group=self.client_cb_group
        )
        self.slam_get_state_client = self.create_client(
            GetState, "/slam_toolbox/get_state", callback_group=self.client_cb_group
        )
        self.amcl_change_state_client = self.create_client(
            ChangeState, "/amcl/change_state", callback_group=self.client_cb_group
        )
        self.slam_change_state_client = self.create_client(
            ChangeState, "/slam_toolbox/change_state", callback_group=self.client_cb_group
        )

        # 建立發布/訂閱者
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._amcl_pose_callback, 10, callback_group=self.client_cb_group
        )
        self.current_map_sub = self.create_subscription(
            String, "current_map", self._current_map_callback, qos_profile, callback_group=self.client_cb_group
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self._scan_callback, qos_profile_sensor_data, callback_group=self.client_cb_group
        )
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "cmd_vel", 10, callback_group=self.client_cb_group)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self._current_pose_callback, callback_group=self.client_cb_group)

        # 地點資料庫
        self.waypoints_db = {}

        self.base_dir = Path.home() / ".smartnav" / "waypoint_database"
        self.db_file = self.base_dir / "waypoints.json"

        # 保護地點資料庫的鎖
        self._db_lock = threading.Lock()
        # 嘗試載入地點資料庫
        self._load_waypoints_db()

        self.current_map = ""
        self.current_pose = Pose()
        self.current_covariance_norm = float("inf")
        self.current_entropy = float("inf")
        self.min_front_distance = float("inf")
        self.pose_received = False

        self._init_timer = self.create_timer(0.1, self._init_callback, callback_group=self.client_cb_group)

        self.get_logger().info("地點服務節點已啟動")

    def _init_callback(self) -> None:
        """初始化回調函式"""
        self._init_timer.cancel()
        self._wait_for_services()

    def _wait_for_services(self) -> None:
        """等待所有基礎服務就緒"""
        services = [
            ("amcl_get_state", self.amcl_get_state_client),
            ("slam_get_state", self.slam_get_state_client),
            ("amcl_change_state", self.amcl_change_state_client),
            ("slam_change_state", self.slam_change_state_client),
        ]
        for service_name, client in services:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f"⌛ 等待服務 {service_name}...")
        self.get_logger().info("✓ 所有基礎服務已就緒")

    def _wait_for_future(self, future, timeout_sec: float) -> Any:
        """等待服務回應的輔助函式"""
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())

        # 阻塞當前獨立執行緒，直到被喚醒或超時
        if not event.wait(timeout=timeout_sec):
            raise TimeoutError(f"服務請求超時")

        return future.result()

    def create_waypoint_callback(self, request, response):
        """處理建立地點請求

        Args:
            request: CreateWaypoint 請求，包含 waypoint_name
            response: CreateWaypoint 回應

        Returns:
            CreateWaypoint 回應物件
        """
        self.get_logger().info(f"收到建立地點請求: {request.waypoint_name}")

        try:
            if not request.waypoint_name:
                response.waypoint_info = WaypointInfo()
                response.success = False
                response.message = "必須提供地點名稱以建立地點"
                self.get_logger().warning("建立地點失敗: 未提供地點名稱")
                return response

            if not self.current_map:
                response.waypoint_info = WaypointInfo()
                response.success = False
                response.message = "當前沒有載入地圖，無法建立地點"
                self.get_logger().warning("建立地點失敗: 當前沒有載入地圖")
                return response

            if self.current_covariance_norm > 0.15:
                response.waypoint_info = WaypointInfo()
                response.success = False
                response.message = "當前定位不夠準確，請嘗試全域定位"
                self.get_logger().warning("建立地點失敗: 當前定位不夠準確")
                return response

            if not self.pose_received:
                response.waypoint_info = WaypointInfo()
                response.success = False
                response.message = "當前沒有獲取到位置資訊，無法建立地點"
                self.get_logger().warning("建立地點失敗: 當前沒有獲取到位置資訊")
                return response

            with self._db_lock:
                for waypoint_meta in self.waypoints_db.values():
                    if waypoint_meta["name"] == request.waypoint_name and waypoint_meta["map_id"] == self.current_map:
                        response.waypoint_info = WaypointInfo()
                        response.success = False
                        response.message = f'地點名稱 "{request.waypoint_name}" 已存在，請使用不同的名稱'
                        self.get_logger().warning(f"建立地點失敗: 地點名稱 {request.waypoint_name} 已存在")
                        return response

            # 生成唯一的地點 ID
            waypoint_id = "waypoint_" + uuid.uuid4().hex[:12]

            # 儲存地點資訊
            with self._db_lock:
                self.waypoints_db[waypoint_id] = {
                    "name": request.waypoint_name,
                    "map_id": self.current_map,
                    "pose": copy.deepcopy(self.current_pose),
                }
                self._save_waypoints_db()

            waypoint_info = WaypointInfo()
            waypoint_info.waypoint_id = waypoint_id
            waypoint_info.waypoint_name = request.waypoint_name
            waypoint_info.map_id = self.current_map
            waypoint_info.pose = self.current_pose

            response.waypoint_info = waypoint_info
            response.success = True
            response.message = f'地點 "{request.waypoint_name}" 建立成功'
            self.get_logger().info(
                f"地點建立成功: {waypoint_id} ({request.waypoint_name}) "
                f"位置: ({self.current_pose.position.x}, {self.current_pose.position.y})"
            )
        except Exception as e:
            response.waypoint_info = WaypointInfo()
            response.success = False
            response.message = f"系統出現異常，地點建立失敗"
            self.get_logger().error(f"地點建立錯誤: {str(e)}")

        return response

    def list_waypoints_callback(self, request, response):
        """處理列出地點請求

        Args:
            request: ListWaypoints 請求 (可忽略)
            response: ListWaypoints 回應

        Returns:
            ListWaypoints 回應物件
        """
        self.get_logger().info(f"收到列出地點請求")

        try:
            if not self.current_map:
                response.waypoints_info = []
                response.success = False
                response.message = "當前沒有載入地圖，無法列出地點"
                self.get_logger().warning("列出地點失敗: 當前沒有載入地圖")
                return response

            waypoints_info_list = []
            with self._db_lock:
                for waypoint_id, waypoint_meta in self.waypoints_db.items():
                    # 則只返回當前地圖下的地點
                    if waypoint_meta["map_id"] != self.current_map:
                        continue

                    waypoint_info = WaypointInfo()
                    waypoint_info.waypoint_id = waypoint_id
                    waypoint_info.waypoint_name = waypoint_meta["name"]
                    waypoint_info.map_id = waypoint_meta["map_id"]
                    waypoint_info.pose = waypoint_meta["pose"]
                    waypoints_info_list.append(waypoint_info)

            response.waypoints_info = waypoints_info_list
            response.success = True
            response.message = f"查詢到 {len(response.waypoints_info)} 個地點"
            self.get_logger().info(f"列出地點成功: 找到 {len(response.waypoints_info)} 個地點")
        except Exception as e:
            response.waypoints_info = []
            response.success = False
            response.message = f"系統出現異常，查詢地點列表失敗"
            self.get_logger().error(f"查詢地點列表錯誤: {str(e)}")

        return response

    def get_waypoint_callback(self, request, response):
        """處理查詢地點請求

        Args:
            request: GetWaypoint 請求，包含 waypoint_id
            response: GetWaypoint 回應

        Returns:
            GetWaypoint 回應物件
        """
        self.get_logger().info(f"收到查詢地點請求: {request.waypoint_id}")

        try:
            if not self.current_map:
                response.waypoint_info = WaypointInfo()
                response.success = False
                response.message = "當前沒有載入地圖，無法查詢地點"
                self.get_logger().warning("查詢地點失敗: 當前沒有載入地圖")
                return response

            with self._db_lock:
                waypoint_meta = self.waypoints_db.get(request.waypoint_id)

            if not waypoint_meta or waypoint_meta["map_id"] != self.current_map:
                response.waypoint_info = WaypointInfo()
                response.success = False
                response.message = f'當前地圖下找不到ID為 "{request.waypoint_id}" 的地點'
                self.get_logger().warning(f"查詢地點失敗: 找不到ID為 {request.waypoint_id} 的地點")
                return response

            waypoint_info = WaypointInfo()
            waypoint_info.waypoint_id = request.waypoint_id
            waypoint_info.waypoint_name = waypoint_meta["name"]
            waypoint_info.map_id = waypoint_meta["map_id"]
            waypoint_info.pose = waypoint_meta["pose"]

            response.waypoint_info = waypoint_info
            response.success = True
            response.message = f'地點 "{waypoint_meta["name"]}" 查詢成功'
            self.get_logger().info(f"查詢地點成功: {request.waypoint_id} ({waypoint_meta['name']})")
        except Exception as e:
            response.waypoint_info = WaypointInfo()
            response.success = False
            response.message = f"系統出現異常，查詢地點失敗"
            self.get_logger().error(f"查詢地點錯誤: {str(e)}")

        return response

    def global_localization_callback(self, goal_handle):
        """處理全域定位請求"""
        self.get_logger().info(f"收到全域定位請求")

        result = GlobalLocalization.Result()

        try:
            if not self.current_map:
                result.pose = Pose()
                result.success = False
                result.message = "當前沒有載入地圖，無法進行全域定位"
                goal_handle.abort()
                self.get_logger().error("全域定位失敗: 當前沒有載入地圖")
                return result

            if not self.switch_to_amcl_mode():
                result.pose = Pose()
                result.success = False
                result.message = "無法切換到 AMCL 模式，全域定位失敗"
                goal_handle.abort()
                return result

            req = Empty.Request()
            future = self.nav2_global_localization_client.call_async(req)
            try:
                self._wait_for_future(future, timeout_sec=5.0)
            except:
                result.pose = Pose()
                result.success = False
                result.message = "全域定位請求失敗"
                goal_handle.abort()
                self.get_logger().error("全域定位請求失敗")
                return result

            self.current_covariance_norm = float("inf")
            last_entropy = self.current_entropy
            exploration_state = "ROTATE"
            start_time = self.get_clock().now()
            max_wait_time = Duration(seconds=200.0)
            loop_count = 0
            while True:
                twist_stamped_msg = TwistStamped()
                twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
                twist_stamped_msg.header.frame_id = "base_link"

                if goal_handle.is_cancel_requested:
                    result.pose = Pose()
                    result.success = False
                    result.message = "全域定位請求已被系統或使用者取消"
                    goal_handle.canceled()
                    self.cmd_vel_pub.publish(TwistStamped())
                    self.get_logger().info("全域定位請求已取消")
                    return result

                if self.get_clock().now() - start_time > max_wait_time:
                    result.pose = Pose()
                    result.success = False
                    result.message = "全域定位超時失敗"
                    goal_handle.abort()
                    self.cmd_vel_pub.publish(TwistStamped())
                    self.get_logger().error("全域定位超時失敗")
                    return result

                if self.current_covariance_norm <= 0.15:
                    self.cmd_vel_pub.publish(TwistStamped())
                    break

                entropy_reduction = last_entropy - self.current_entropy
                last_entropy = self.current_entropy

                if exploration_state == "ROTATE":
                    twist_stamped_msg.twist.linear.x = 0.0
                    twist_stamped_msg.twist.angular.z = 0.4

                    if loop_count > 10 and entropy_reduction < 0.05:
                        # 旋轉後的熵減少不明顯，切換到前進探索
                        exploration_state = "FORWARD"
                        loop_count = 0
                elif exploration_state == "FORWARD":
                    if self.min_front_distance < 0.4:
                        # 前方距離過近，切換回旋轉探索
                        exploration_state = "ROTATE"
                        twist_stamped_msg.twist.linear.x = 0.0
                        twist_stamped_msg.twist.angular.z = 0.6
                        loop_count = 0
                    else:
                        twist_stamped_msg.twist.linear.x = 0.2
                        twist_stamped_msg.twist.angular.z = 0.0

                        if loop_count > 10 and entropy_reduction < 0.05:
                            # 前進後的熵減少不明顯，切換回旋轉探索
                            exploration_state = "ROTATE"
                            loop_count = 0

                loop_count += 1
                self.cmd_vel_pub.publish(twist_stamped_msg)
                self.get_clock().sleep_until(self.get_clock().now() + Duration(seconds=0.2))

            result.pose = self.current_pose
            result.success = True
            result.message = f"全域定位成功"
            goal_handle.succeed()
            self.get_logger().info(f"全域定位成功")
        except Exception as e:
            result.pose = Pose()
            result.success = False
            result.message = f"系統出現異常，全域定位失敗"
            goal_handle.abort()
            self.get_logger().error(f"全域定位錯誤: {str(e)}")

        return result

    def _current_map_callback(self, msg):
        self.current_map = msg.data
        self.get_logger().info(f"更新當前地圖: {self.current_map}")

    def _current_pose_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", Time(), timeout=Duration(seconds=0.05))

            # 將 TF 轉換為 Pose 格式
            self.current_pose.position.x = trans.transform.translation.x
            self.current_pose.position.y = trans.transform.translation.y
            self.current_pose.position.z = trans.transform.translation.z
            self.current_pose.orientation = trans.transform.rotation
            self.pose_received = True
        except Exception as e:
            err_msg = str(e)
            if "extrapolation" in err_msg.lower() or "lookup" in err_msg.lower():
                return

            self.get_logger().error(f"無法獲取 TF 座標轉換: {str(e)}")

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

    def _scan_callback(self, msg: LaserScan):
        """處理雷射掃描數據，計算正前方的最短距離"""
        if not msg.ranges:
            return

        min_dist = float("inf")

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(angle) < 0.5:
                if 0.0 < r < float("inf") and not math.isnan(r):
                    min_dist = min(min_dist, r)

        self.min_front_distance = min_dist

    def _save_waypoints_db(self) -> None:
        """將地點資料庫儲存到本地文件"""
        serializable_db = {}
        for waypoint_id, waypoint_meta in self.waypoints_db.items():
            serializable_db[waypoint_id] = {
                "name": waypoint_meta["name"],
                "map_id": waypoint_meta["map_id"],
                "pose": {
                    "position": {
                        "x": waypoint_meta["pose"].position.x,
                        "y": waypoint_meta["pose"].position.y,
                        "z": waypoint_meta["pose"].position.z,
                    },
                    "orientation": {
                        "x": waypoint_meta["pose"].orientation.x,
                        "y": waypoint_meta["pose"].orientation.y,
                        "z": waypoint_meta["pose"].orientation.z,
                        "w": waypoint_meta["pose"].orientation.w,
                    },
                },
            }

        try:
            self.base_dir.mkdir(parents=True, exist_ok=True)
            with open(self.db_file, "w", encoding="utf-8") as f:
                json.dump({"waypoints_db": serializable_db}, f, ensure_ascii=False, indent=4)
            self.get_logger().info(f"成功儲存地點資料庫，包含 {len(self.waypoints_db)} 個地點")
        except Exception as e:
            self.get_logger().error(f"儲存地點資料庫失敗: {str(e)}")

    def _load_waypoints_db(self) -> None:
        """從本地文件載入地點資料庫"""
        try:
            if self.db_file.exists():
                with open(self.db_file, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    raw_db = data.get("waypoints_db", {})
                    self.waypoints_db = {}
                    for waypoint_id, waypoint_meta in raw_db.items():
                        pose_data = waypoint_meta["pose"]
                        pose = Pose()
                        pose.position.x = pose_data["position"]["x"]
                        pose.position.y = pose_data["position"]["y"]
                        pose.position.z = pose_data["position"]["z"]
                        pose.orientation.x = pose_data["orientation"]["x"]
                        pose.orientation.y = pose_data["orientation"]["y"]
                        pose.orientation.z = pose_data["orientation"]["z"]
                        pose.orientation.w = pose_data["orientation"]["w"]

                        self.waypoints_db[waypoint_id] = {
                            "name": waypoint_meta["name"],
                            "map_id": waypoint_meta["map_id"],
                            "pose": pose,
                        }

                self.get_logger().info(f"成功載入地點資料庫，包含 {len(self.waypoints_db)} 個地點")
            else:
                self.get_logger().info("無法載入地點資料庫，將使用空的資料庫")
        except Exception as e:
            self.get_logger().error(f"載入地點資料庫失敗: {str(e)}")

    def get_node_state(self, client) -> str:
        """取得目標生命週期節點的當前狀態"""
        req = GetState.Request()
        future = client.call_async(req)
        try:
            res = self._wait_for_future(future, timeout_sec=2.0)
            return res.current_state.label
        except Exception:
            return "unknown"

    def switch_to_amcl_mode(self) -> bool:
        """切換到 AMCL 定位模式"""
        amcl_state = self.get_node_state(self.amcl_get_state_client)
        slam_state = self.get_node_state(self.slam_get_state_client)

        if amcl_state == "active" and slam_state == "inactive":
            return True

        try:
            if slam_state == "active":
                req = ChangeState.Request()
                req.transition.id = Transition.TRANSITION_DEACTIVATE
                future = self.slam_change_state_client.call_async(req)
                self._wait_for_future(future, timeout_sec=10.0)
            if amcl_state != "active":
                if amcl_state == "unconfigured":
                    req = ChangeState.Request()
                    req.transition.id = Transition.TRANSITION_CONFIGURE
                    future = self.amcl_change_state_client.call_async(req)
                    self._wait_for_future(future, timeout_sec=10.0)

                req = ChangeState.Request()
                req.transition.id = Transition.TRANSITION_ACTIVATE
                future = self.amcl_change_state_client.call_async(req)
                self._wait_for_future(future, timeout_sec=10.0)
            self.get_logger().info("成功切換到 AMCL 定位模式")
        except Exception as e:
            self.get_logger().error(f"切換到 AMCL 定位模式失敗: {str(e)}")
            return False

        return True


def main(args=None):
    """地點服務節點進入點"""
    rclpy.init(args=args)
    node = WaypointServiceNode()
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
