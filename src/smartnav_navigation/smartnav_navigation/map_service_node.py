#!/usr/bin/env python3

"""地圖服務節點

提供建立地圖和查詢地圖的服務
"""

import json
import uuid
import threading
from pathlib import Path
from typing import Optional, Any

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import Log
from std_msgs.msg import String
from nav2_msgs.srv import SaveMap as Nav2SaveMap, LoadMap as Nav2LoadMap
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import GetState, ChangeState
from frontier_exploration_ros2.srv import ControlExploration

from smartnav_msgs.msg import MapInfo
from smartnav_msgs.srv import ListMaps, SwitchMap
from smartnav_msgs.action import CreateMap


class MapServiceNode(Node):
    """地圖服務節點類別

    提供建立和查詢地圖的 ROS 2 服務
    """

    def __init__(self):
        """初始化地圖服務節點"""
        super().__init__("map_service_node")

        self.server_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = ReentrantCallbackGroup()

        # 建立服務/動作/客戶端
        self.list_maps_srv = self.create_service(
            ListMaps, "list_maps", self.list_maps_callback, callback_group=self.server_cb_group
        )
        self.switch_map_srv = self.create_service(
            SwitchMap, "switch_map", self.switch_map_callback, callback_group=self.server_cb_group
        )

        self.create_map_action = ActionServer(
            self, CreateMap, "create_map", self.create_map_callback, callback_group=self.server_cb_group
        )

        self.control_exploration_client = self.create_client(
            ControlExploration, "/control_exploration", callback_group=self.client_cb_group
        )
        self.nav2_save_map_client = self.create_client(
            Nav2SaveMap, "/map_saver/save_map", callback_group=self.client_cb_group
        )
        self.nav2_load_map_client = self.create_client(
            Nav2LoadMap, "/map_server/load_map", callback_group=self.client_cb_group
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
        self.current_map_pub = self.create_publisher(
            String, "current_map", qos_profile, callback_group=self.client_cb_group
        )
        self.rosout_sub = self.create_subscription(
            Log, "/rosout", self.rosout_callback, 10, callback_group=self.client_cb_group
        )

        # 地圖資料庫
        self.maps_db = {}

        self.base_dir = Path.home() / ".smartnav"
        self.map_data_dir = self.base_dir / "map_database" / "data"
        self.db_file = self.base_dir / "map_database" / "maps_db.json"
        self.state_file = self.base_dir / "nav_state.json"

        # 保護地圖資料庫的鎖
        self._db_lock = threading.Lock()
        # 嘗試載入地圖資料庫
        self._load_maps_db()

        # 探索狀態追蹤
        self.exploration_finished = False

        # 執行在節點成功 spin 後的初始化任務
        self._init_timer = self.create_timer(0.1, self._init_callback, callback_group=self.client_cb_group)

        self.get_logger().info("✓ 地圖服務節點已初始化")

    def _init_callback(self) -> None:
        """初始化回調函式，嘗試載入最後使用的地圖"""
        self._init_timer.cancel()
        self.nav2_load_map(None)
        self._wait_for_services()

    def _wait_for_services(self) -> None:
        """等待所有基礎服務就緒"""
        services = [
            ("control_exploration", self.control_exploration_client),
            ("nav2_save_map", self.nav2_save_map_client),
            ("nav2_load_map", self.nav2_load_map_client),
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

    def create_map_callback(self, goal_handle):
        """處理建立地圖請求"""
        self.get_logger().info(f"收到建立地圖請求")

        request = goal_handle.request
        result = CreateMap.Result()

        try:
            if not request.map_name:
                result.map_info = MapInfo()
                result.success = False
                result.message = "必須提供地圖名稱以建立地圖"
                goal_handle.abort()
                self.get_logger().warning("建立失敗: 未提供地圖名稱")
                return result

            with self._db_lock:
                for map_meta in self.maps_db.values():
                    if map_meta["name"] == request.map_name:
                        result.map_info = MapInfo()
                        result.success = False
                        result.message = f'地圖名稱 "{request.map_name}" 已存在，請使用不同的名稱'
                        goal_handle.abort()
                        self.get_logger().warning(f"建立失敗: 地圖名稱 {request.map_name} 已存在")
                        return result

            if not self.switch_to_slam_mode():
                result.map_info = MapInfo()
                result.success = False
                result.message = "無法切換到 SLAM 模式，地圖建立失敗"
                goal_handle.abort()
                self.get_logger().error("切換到 SLAM 模式失敗，地圖建立失敗")
                return result

            # 生成唯一的地圖 ID
            map_id = "map_" + uuid.uuid4().hex[:12]

            req = ControlExploration.Request()
            req.action = ControlExploration.Request.ACTION_START
            future = self.control_exploration_client.call_async(req)
            try:
                self._wait_for_future(future, timeout_sec=5.0)
            except Exception as e:
                result.map_info = MapInfo()
                result.success = False
                result.message = "無法啟動探索服務，地圖建立失敗"
                goal_handle.abort()
                self.get_logger().error(f"啟動探索服務失敗: {str(e)}")
                return result

            start_time = self.get_clock().now()
            max_wait_time = Duration(seconds=400.0)
            while True:
                if goal_handle.is_cancel_requested:
                    req = ControlExploration.Request()
                    req.action = ControlExploration.Request.ACTION_STOP
                    future = self.control_exploration_client.call_async(req)
                    try:
                        self._wait_for_future(future, timeout_sec=5.0)
                        result.message = "地圖建立請求已被系統或使用者取消"
                    except Exception as e:
                        result.message = "取消地圖建立失敗，請手動停止探索"
                        self.get_logger().error(f"取消地圖建立服務失敗: {str(e)}")
                    result.map_info = MapInfo()
                    result.success = False
                    goal_handle.canceled()
                    self.get_logger().info("地圖建立請求已被系統或使用者取消")
                    return result

                if self.get_clock().now() - start_time > max_wait_time:
                    req = ControlExploration.Request()
                    req.action = ControlExploration.Request.ACTION_STOP
                    future = self.control_exploration_client.call_async(req)
                    try:
                        self._wait_for_future(future, timeout_sec=5.0)
                        result.message = "地圖建立超時，已自動停止探索"
                    except Exception as e:
                        result.message = "地圖建立超時，且無法自動停止探索，請手動停止探索"
                        self.get_logger().error(f"地圖建立超時後停止探索服務失敗: {str(e)}")
                    result.map_info = MapInfo()
                    result.success = False
                    goal_handle.abort()
                    self.get_logger().warning("地圖建立超時，已自動停止探索")
                    return result

                if self.exploration_finished:
                    self.exploration_finished = False
                    self.get_logger().info("探索已完成，準備儲存地圖")
                    break

                self.get_clock().sleep_until(self.get_clock().now() + Duration(seconds=1.0))

            self.map_data_dir.mkdir(parents=True, exist_ok=True)
            req = Nav2SaveMap.Request()
            req.map_url = str(self.map_data_dir / map_id)
            future = self.nav2_save_map_client.call_async(req)
            try:
                self._wait_for_future(future, timeout_sec=5.0)
            except Exception as e:
                result.map_info = MapInfo()
                result.success = False
                result.message = "無法啟動地圖儲存服務，地圖建立失敗"
                goal_handle.abort()
                self.get_logger().error(f"啟動地圖儲存服務失敗: {str(e)}")
                return result

            # 儲存地圖資訊
            with self._db_lock:
                self.maps_db[map_id] = {"name": request.map_name}
                self._save_maps_db()
                self._save_map_state(map_id)
                self.nav2_load_map(map_id)

            map_info = MapInfo()
            map_info.map_id = map_id
            map_info.map_name = request.map_name
            result.map_info = map_info
            result.success = True
            result.message = f'地圖 "{request.map_name}" 建立成功'
            goal_handle.succeed()
            self.get_logger().info(f"地圖建立成功: {map_id} ({request.map_name})")
        except Exception as e:
            result.map_info = MapInfo()
            result.success = False
            result.message = f"系統出現異常，地圖建立失敗"
            goal_handle.abort()
            self.get_logger().error(f"地圖建立錯誤: {str(e)}")

        return result

    def list_maps_callback(self, request, response):
        """處理列出所有地圖的請求

        Args:
            request: ListMaps 請求 (可忽略)
            response: ListMaps 回應

        Returns:
            ListMaps 回應物件
        """
        self.get_logger().info("收到列出地圖請求 (list_maps)")

        try:
            maps_info_list = []
            with self._db_lock:
                for map_id, map_meta in self.maps_db.items():
                    map_info = MapInfo()
                    map_info.map_id = map_id
                    map_info.map_name = map_meta["name"]
                    maps_info_list.append(map_info)

            response.maps_info = maps_info_list
            response.success = True
            response.message = f"成功查詢到 {len(response.maps_info)} 個地圖"
            self.get_logger().info(f"查詢地圖成功: 找到 {len(response.maps_info)} 個地圖")
        except Exception as e:
            response.maps_info = []
            response.success = False
            response.message = f"系統出現異常，查詢地圖列表失敗"
            self.get_logger().error(f"查詢地圖列表錯誤: {str(e)}")

        return response

    def switch_map_callback(self, request, response):
        """處理切換地圖請求

        Args:
            request: SwitchMap 請求，包含 map_id
            response: SwitchMap 回應

        Returns:
            SwitchMap 回應物件
        """
        self.get_logger().info("收到切換地圖請求")

        try:
            if not request.map_id:
                response.success = False
                response.message = "必須提供 map_id 以切換到特定地圖"
                self.get_logger().warning("切換失敗: 未提供 map_id")
                return response

            is_valid_map = False
            with self._db_lock:
                if request.map_id in self.maps_db:
                    is_valid_map = True

            # 查詢特定地圖
            if is_valid_map and self.nav2_load_map(request.map_id):
                response.success = True
                response.message = "地圖切換成功"
                self._save_map_state(request.map_id)
                self.get_logger().info(f"地圖切換成功: {request.map_id}")
            else:
                response.success = False
                response.message = f"地圖 {request.map_id} 不存在"
                self.get_logger().info(f"地圖切換失敗: {request.map_id} 不存在")
        except Exception as e:
            response.success = False
            response.message = f"系統出現異常，地圖切換失敗"
            self.get_logger().error(f"地圖切換出現錯誤: {str(e)}")

        return response

    def _save_map_state(self, map_id: str) -> None:
        """儲存當前地圖狀態到本地文件

        Args:
            map_id: 要儲存的地圖 ID
        """
        try:
            self.base_dir.mkdir(parents=True, exist_ok=True)
            data = {"last_active_map": map_id}
            with open(self.state_file, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=4)
            self.get_logger().info(f"成功儲存當前地圖狀態: {map_id}")
        except Exception as e:
            self.get_logger().error(f"儲存當前地圖狀態失敗: {str(e)}")

    def _load_map_state(self) -> Optional[str]:
        """從本地文件載入最後使用的地圖 ID

        Returns:
            最後使用的地圖 ID，如果無法載入則返回 None
        """
        try:
            if self.state_file.exists():
                with open(self.state_file, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    map_id = data.get("last_active_map")
                    self.get_logger().info(f"成功載入最後使用的地圖 ID")
            else:
                self.get_logger().warning("無法載入最後使用的地圖 ID")
                return None
        except Exception as e:
            self.get_logger().error(f"載入最後使用的地圖 ID 失敗: {str(e)}")
            return None

        return map_id

    def _save_maps_db(self) -> None:
        """將地圖資料庫儲存到本地文件"""
        try:
            self.base_dir.mkdir(parents=True, exist_ok=True)
            with open(self.db_file, "w", encoding="utf-8") as f:
                json.dump({"maps_db": self.maps_db}, f, ensure_ascii=False, indent=4)
            self.get_logger().info(f"成功儲存地圖資料庫，包含 {len(self.maps_db)} 個地圖")
        except Exception as e:
            self.get_logger().error(f"儲存地圖資料庫失敗: {str(e)}")

    def _load_maps_db(self) -> None:
        """從本地文件載入地圖資料庫"""
        try:
            if self.db_file.exists():
                with open(self.db_file, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    with self._db_lock:
                        self.maps_db = data.get("maps_db", {})
                self.get_logger().info(f"成功載入地圖資料庫，包含 {len(self.maps_db)} 個地圖")
            else:
                self.get_logger().warning("無法載入地圖資料庫，將使用空的資料庫")
        except Exception as e:
            self.get_logger().error(f"載入地圖資料庫失敗: {str(e)}")

    def nav2_load_map(self, map_id: Optional[str]) -> bool:
        """使用 Nav2 的 LoadMap 服務載入地圖

        Args:
            map_id: 要載入的地圖 ID

        Returns:
            是否成功載入地圖
        """
        try:
            if map_id is None:
                map_id = self._load_map_state()
                if not map_id:
                    self.get_logger().warning("無法獲取最後創建的地圖 ID")
                    return False

            map_file = self.map_data_dir / f"{map_id}.yaml"
            if not (map_file).exists():
                self.get_logger().error(f"地圖文件 {map_file} 不存在，無法載入地圖 {map_id}")
                return False

            req = Nav2LoadMap.Request()
            req.map_url = str(map_file)
            future = self.nav2_load_map_client.call_async(req)
            self._wait_for_future(future, timeout_sec=5.0)
            self.current_map_pub.publish(String(data=map_id))
            self.get_logger().info(f"成功使用 Nav2 LoadMap 服務載入地圖 {map_id}")
        except Exception as e:
            self.get_logger().error(f"使用 Nav2 LoadMap 服務載入地圖 {map_id} 失敗: {str(e)}")
            return False

        return True

    def get_node_state(self, client) -> str:
        """取得目標生命週期節點的當前狀態"""
        req = GetState.Request()
        future = client.call_async(req)
        try:
            res = self._wait_for_future(future, timeout_sec=2.0)
            return res.current_state.label
        except Exception:
            return "unknown"

    def switch_to_slam_mode(self) -> bool:
        """切換到 SLAM 建圖模式"""
        amcl_state = self.get_node_state(self.amcl_get_state_client)
        slam_state = self.get_node_state(self.slam_get_state_client)

        if amcl_state == "inactive" and slam_state == "active":
            return True

        try:
            if amcl_state == "active":
                req = ChangeState.Request()
                req.transition.id = Transition.TRANSITION_DEACTIVATE
                future = self.amcl_change_state_client.call_async(req)
                self._wait_for_future(future, timeout_sec=10.0)

            if slam_state != "active":
                if slam_state == "unconfigured":
                    req = ChangeState.Request()
                    req.transition.id = Transition.TRANSITION_CONFIGURE
                    future = self.slam_change_state_client.call_async(req)
                    self._wait_for_future(future, timeout_sec=10.0)

                req = ChangeState.Request()
                req.transition.id = Transition.TRANSITION_ACTIVATE
                future = self.slam_change_state_client.call_async(req)
                self._wait_for_future(future, timeout_sec=10.0)
            self.get_logger().info("成功切換到 SLAM 建圖模式")
        except Exception as e:
            self.get_logger().error(f"切換到 SLAM 建圖模式失敗: {str(e)}")
            return False

        return True

    def rosout_callback(self, msg: Log) -> None:
        """訂閱 /rosout 日誌消息，監控 explorer 狀態"""
        if msg.name == "frontier_explorer":
            if "Returned to start pose" in msg.msg:
                self.exploration_finished = True
                self.get_logger().info("探索已完成")


def main(args=None):
    """地圖服務節點進入點"""
    rclpy.init(args=args)
    node = MapServiceNode()
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
