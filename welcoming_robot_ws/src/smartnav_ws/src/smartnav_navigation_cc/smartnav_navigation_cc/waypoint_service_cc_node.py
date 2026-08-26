#!/usr/bin/env python3

"""地點服務節點 (_cc 重寫版)

對外介面沿用舊名稱 (/create_waypoint, /list_waypoints, /get_waypoint,
/global_localization)，HMI 與 smartnav_brain 不需修改。

相對舊版的三個關鍵修正：

  1. cmd_vel 的訊息型別錯了。
     舊版全域定位發的是 geometry_msgs/TwistStamped，但 wheeltec_robot_node
     訂閱的是 geometry_msgs/Twist (見 wheeltec_robot.cpp 的 Cmd_Vel_Callback)，
     型別不符 -> 話題根本配不起來 -> 車子完全不動，全域定位必定跑到逾時。

  2. 全域定位的運動策略對阿克曼底盤無效。
     舊版是「原地旋轉」(linear.x = 0, angular.z = 0.4)。
     阿克曼車沒有原地旋轉能力，這種指令下去車子只會空轉方向盤不移動，
     AMCL 的粒子雲永遠收斂不了。現在改成以最小轉彎半徑以上的圓弧慢速繞行，
     前方太近就換方向倒車，這才是這種底盤真正能執行的動作。

  3. 速度不再直接發到 /cmd_vel，改發 cmd_vel_nav。
     這樣會經過 velocity_smoother 與 collision_monitor，
     全域定位期間一樣有防撞保護；直接發 /cmd_vel 等於繞過安全層，
     而且會和 collision_monitor 的輸出互相蓋台。

  4. 不再自己去操作 amcl / slam_toolbox 的生命週期。
     模式仲裁統一由 map_service_cc 負責，這裡改呼叫它的 /ensure_localization。
     舊版兩個節點各自切生命週期，互相搶狀態是很難查的競態來源。
"""

import copy
import json
import math
import threading
import time
import uuid
from pathlib import Path
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from smartnav_msgs.action import GlobalLocalization
from smartnav_msgs.msg import WaypointInfo
from smartnav_msgs.srv import CreateWaypoint, DeleteWaypoint, GetWaypoint, ListWaypoints


# 全域定位時判斷「前方是否太近」的扇形半角 (弧度)，約 ±30°
FRONT_HALF_ANGLE = 0.52


def wait_for_future(future, timeout_sec: float) -> Any:
    """阻塞等待 future 完成"""
    event = threading.Event()
    future.add_done_callback(lambda _f: event.set())
    if not event.wait(timeout=timeout_sec):
        raise TimeoutError("服務請求超時")
    return future.result()


class WaypointServiceCcNode(Node):
    """地點服務節點"""

    def __init__(self):
        super().__init__("waypoint_service_cc_node")

        self.declare_parameter("max_covariance_norm", 0.15)
        self.declare_parameter("global_localization_timeout_sec", 240.0)
        # 全域定位的巡遊速度。轉彎半徑 = linear / angular = 0.18 / 0.20 = 0.9 m，
        # 大於底盤最小轉彎半徑 0.8 m，指令才不會被底盤截斷。
        self.declare_parameter("relocalize_linear_speed", 0.18)
        self.declare_parameter("relocalize_angular_speed", 0.20)
        # 前方障礙物距離小於這個值就換方向
        self.declare_parameter("front_clearance_m", 0.7)
        self.declare_parameter("robot_base_frame", "base_footprint")

        self.max_covariance_norm = float(self.get_parameter("max_covariance_norm").value)
        self.global_localization_timeout_sec = float(
            self.get_parameter("global_localization_timeout_sec").value
        )
        self.relocalize_linear_speed = float(self.get_parameter("relocalize_linear_speed").value)
        self.relocalize_angular_speed = float(self.get_parameter("relocalize_angular_speed").value)
        self.front_clearance_m = float(self.get_parameter("front_clearance_m").value)
        self.robot_base_frame = self.get_parameter("robot_base_frame").value

        self.server_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = ReentrantCallbackGroup()

        # ------------------------------------------------------------------
        # 對外服務 / 動作
        # ------------------------------------------------------------------
        self.create_service(
            CreateWaypoint, "create_waypoint", self._create_waypoint_callback, callback_group=self.server_cb_group
        )
        self.create_service(
            ListWaypoints, "list_waypoints", self._list_waypoints_callback, callback_group=self.server_cb_group
        )
        self.create_service(
            GetWaypoint, "get_waypoint", self._get_waypoint_callback, callback_group=self.server_cb_group
        )
        self.create_service(
            DeleteWaypoint, "delete_waypoint", self._delete_waypoint_callback,
            callback_group=self.server_cb_group,
        )
        self.global_localization_action = ActionServer(
            self,
            GlobalLocalization,
            "global_localization",
            execute_callback=self._global_localization_callback,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self.server_cb_group,
        )

        # ------------------------------------------------------------------
        # 客戶端
        # ------------------------------------------------------------------
        self.reinit_global_localization_client = self.create_client(
            EmptySrv, "/reinitialize_global_localization", callback_group=self.client_cb_group
        )
        self.nomotion_update_client = self.create_client(
            EmptySrv, "/request_nomotion_update", callback_group=self.client_cb_group
        )
        # 模式仲裁交給 map_service_cc，本節點不碰生命週期
        self.ensure_localization_client = self.create_client(
            Trigger, "/ensure_localization", callback_group=self.client_cb_group
        )

        # ------------------------------------------------------------------
        # 發布 / 訂閱
        # ------------------------------------------------------------------
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "amcl_pose",
            self._amcl_pose_callback,
            10,
            callback_group=self.client_cb_group,
        )
        self.create_subscription(
            String, "current_map", self._current_map_callback, latched, callback_group=self.client_cb_group
        )
        # 訂閱地圖只為了拿 info，用來擋掉「建在地圖外」的地點 (見 _pose_in_map)
        self.create_subscription(
            OccupancyGrid, "map", self._map_callback, latched, callback_group=self.client_cb_group
        )
        self.create_subscription(
            LaserScan, "scan", self._scan_callback, qos_profile_sensor_data, callback_group=self.client_cb_group
        )
        # 發到 cmd_vel_nav 而不是 cmd_vel：讓速度經過 velocity_smoother 與
        # collision_monitor，全域定位期間一樣有防撞保護。
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel_nav", 10)

        # TF 訂閱的 QoS depth 從預設 100 降到 20。
        # /tf 有 60 Hz，rclpy 每次 executor 輪詢會把佇列裡積壓的訊息全部取出、
        # 在 Python 端逐一反序列化，depth=100 表示忙起來時一次要處理上百則。
        # 本節點只在 _pose_in_map() 查一次「最新」的 map->base_footprint，
        # 用不到那麼深的佇列。
        #
        # 注意：不要壓到 1。實測 depth=1 反而更貴（32% vs 20%）——
        # 每則訊息各觸發一次 executor 喚醒，失去批次處理的攤提效果。
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=20, static_qos=1)
        # 位姿更新頻率。這個值只給 create_waypoint (在目前位置建點) 用，
        # 是人手動觸發的動作，2 Hz 綽綽有餘。
        # 原本 10 Hz：TF 查詢在 Python 裡不便宜，實測這個 timer 是本節點
        # CPU 的主要來源之一，而多出來的精度完全用不到。
        self.declare_parameter("pose_update_rate_hz", 2.0)
        pose_rate = max(0.2, float(self.get_parameter("pose_update_rate_hz").value))
        self.create_timer(1.0 / pose_rate, self._update_current_pose, callback_group=self.client_cb_group)

        # ------------------------------------------------------------------
        # 狀態
        # ------------------------------------------------------------------
        self.base_dir = Path.home() / ".smartnav" / "waypoint_database"
        self.db_file = self.base_dir / "waypoints.json"
        self.waypoints_db = {}
        self._db_lock = threading.Lock()
        self._load_waypoints_db()

        self.current_map = ""
        self.current_pose = Pose()
        self.pose_received = False
        self.current_covariance_norm = float("inf")
        self.min_front_distance = float("inf")
        self._map_info = None
        # 只有全域定位進行中才需要處理雷射 (見 _scan_callback)
        self._localizing = False
        self._front_idx_n = -1
        self._front_ranges = []
        # ★ 2026-08-25：後方扇形。初值 0.0 而不是 inf —— 還沒量到就當作
        #   後面有東西，倒車要先拿到真的資料才准動。
        self._rear_ranges = []
        self.min_rear_distance = 0.0
        self._scan_stamp = 0.0          # 最後一次收到雷射的單調鐘時刻

        self.get_logger().info("地點服務節點 (_cc) 已啟動")

    # ==================================================================
    # 地點 CRUD
    # ==================================================================
    def _create_waypoint_callback(self, request, response):
        self.get_logger().info(f"收到建立地點請求: {request.waypoint_name}")

        try:
            if not request.waypoint_name:
                return self._waypoint_error(response, "必須提供地點名稱以建立地點")

            if not self.current_map:
                return self._waypoint_error(response, "當前沒有載入地圖，無法建立地點")

            if request.use_given_pose:
                # HMI 點地圖建點：座標本來就是 map frame 的絕對位置，
                # 跟機器人自己定位得準不準無關，跳過定位品質檢查。
                target_pose = copy.deepcopy(request.pose)
            else:
                if not self.pose_received:
                    return self._waypoint_error(response, "當前沒有獲取到位置資訊，無法建立地點")
                if self.current_covariance_norm > self.max_covariance_norm:
                    return self._waypoint_error(response, "當前定位不夠準確，請嘗試全域定位")
                target_pose = copy.deepcopy(self.current_pose)

            # 全零四元數是無效旋轉，會讓 Nav2 的目標朝向計算爆掉
            q = target_pose.orientation
            if abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) < 1e-6:
                target_pose.orientation.w = 1.0

            if not self._pose_in_map(target_pose):
                info = self._map_info
                x0, y0 = info.origin.position.x, info.origin.position.y
                x1 = x0 + info.width * info.resolution
                y1 = y0 + info.height * info.resolution
                return self._waypoint_error(
                    response,
                    f"座標 ({target_pose.position.x:.2f}, {target_pose.position.y:.2f}) "
                    f"在地圖範圍外 (x {x0:.2f}~{x1:.2f}, y {y0:.2f}~{y1:.2f})，"
                    "無法建立地點。要在機器人目前位置建點請設定 use_given_pose=false",
                )

            with self._db_lock:
                for meta in self.waypoints_db.values():
                    if meta["name"] == request.waypoint_name and meta["map_id"] == self.current_map:
                        return self._waypoint_error(
                            response, f'地點名稱 "{request.waypoint_name}" 已存在，請使用不同的名稱'
                        )

            waypoint_id = "waypoint_" + uuid.uuid4().hex[:12]
            with self._db_lock:
                self.waypoints_db[waypoint_id] = {
                    "name": request.waypoint_name,
                    "map_id": self.current_map,
                    "pose": target_pose,
                }
                self._save_waypoints_db()

            info = WaypointInfo()
            info.waypoint_id = waypoint_id
            info.waypoint_name = request.waypoint_name
            info.map_id = self.current_map
            info.pose = target_pose

            response.waypoint_info = info
            response.success = True
            response.message = f'地點 "{request.waypoint_name}" 建立成功'
            self.get_logger().info(
                f"地點建立成功: {waypoint_id} ({request.waypoint_name}) "
                f"位置 ({target_pose.position.x:.2f}, {target_pose.position.y:.2f}) "
                f"來源 {'指定座標' if request.use_given_pose else '機器人當前位置'}"
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"地點建立錯誤: {exc}")
            return self._waypoint_error(response, "系統出現異常，地點建立失敗")

        return response

    def _list_waypoints_callback(self, request, response):
        try:
            map_id = request.map_id or self.current_map
            if not map_id:
                response.waypoints_info = []
                response.success = False
                response.message = "當前沒有載入地圖，無法列出地點"
                return response

            infos = []
            with self._db_lock:
                for waypoint_id, meta in self.waypoints_db.items():
                    if meta["map_id"] != map_id:
                        continue
                    info = WaypointInfo()
                    info.waypoint_id = waypoint_id
                    info.waypoint_name = meta["name"]
                    info.map_id = meta["map_id"]
                    info.pose = meta["pose"]
                    infos.append(info)

            response.waypoints_info = infos
            response.success = True
            response.message = f"查詢到 {len(infos)} 個地點"
        except Exception as exc:  # noqa: BLE001
            response.waypoints_info = []
            response.success = False
            response.message = "系統出現異常，查詢地點列表失敗"
            self.get_logger().error(f"查詢地點列表錯誤: {exc}")
        return response

    def _get_waypoint_callback(self, request, response):
        try:
            if not self.current_map:
                return self._waypoint_error(response, "當前沒有載入地圖，無法查詢地點")

            with self._db_lock:
                meta = self.waypoints_db.get(request.waypoint_id)

            if not meta or meta["map_id"] != self.current_map:
                return self._waypoint_error(
                    response, f'當前地圖下找不到ID為 "{request.waypoint_id}" 的地點'
                )

            info = WaypointInfo()
            info.waypoint_id = request.waypoint_id
            info.waypoint_name = meta["name"]
            info.map_id = meta["map_id"]
            info.pose = meta["pose"]

            response.waypoint_info = info
            response.success = True
            response.message = f'地點 "{meta["name"]}" 查詢成功'
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"查詢地點錯誤: {exc}")
            return self._waypoint_error(response, "系統出現異常，查詢地點失敗")
        return response

    def _delete_waypoint_callback(self, request, response):
        """刪除地點

        跟 create 一樣是「改記憶體 + 立刻寫檔」——地點資料庫只有這個節點會寫，
        寫檔失敗時就把記憶體那份還原，不讓檔案與記憶體分家（下次重啟會冒出
        使用者以為已經刪掉的地點）。

        刻意不限制「只能刪當前地圖的地點」：建錯地圖再建點是常見的誤操作，
        真要清理時反而是切到別張圖之前就想刪掉。用 id 指定不會誤刪。
        """
        try:
            if not request.waypoint_id:
                response.success = False
                response.message = "必須提供地點 ID"
                return response

            with self._db_lock:
                meta = self.waypoints_db.get(request.waypoint_id)
                if meta is None:
                    response.success = False
                    response.message = f'找不到 ID 為 "{request.waypoint_id}" 的地點'
                    return response

                name = meta["name"]
                removed = self.waypoints_db.pop(request.waypoint_id)
                if not self._save_waypoints_db():
                    # 寫檔失敗就還原，不讓記憶體與檔案分家
                    self.waypoints_db[request.waypoint_id] = removed
                    response.success = False
                    response.message = "地點資料庫寫入失敗，刪除已取消"
                    return response

            response.success = True
            response.message = f'地點 "{name}" 已刪除'
            self.get_logger().info(f"地點刪除成功: {request.waypoint_id} ({name})")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"地點刪除錯誤: {exc}")
            response.success = False
            response.message = "系統出現異常，地點刪除失敗"
        return response

    @staticmethod
    def _waypoint_error(response, message: str):
        response.waypoint_info = WaypointInfo()
        response.success = False
        response.message = message
        return response

    # ==================================================================
    # 全域定位
    # ==================================================================
    def _global_localization_callback(self, goal_handle):
        result = GlobalLocalization.Result()
        result.pose = Pose()

        self.get_logger().info("收到全域定位請求")

        try:
            if not self.current_map:
                return self._abort_localization(goal_handle, result, "當前沒有載入地圖，無法進行全域定位")

            if not self._ensure_localization_mode():
                return self._abort_localization(goal_handle, result, "無法切換到定位模式，全域定位失敗")

            if not self.reinit_global_localization_client.wait_for_service(timeout_sec=10.0):
                return self._abort_localization(goal_handle, result, "AMCL 全域定位服務不在線上")

            try:
                wait_for_future(
                    self.reinit_global_localization_client.call_async(EmptySrv.Request()), timeout_sec=10.0
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"全域定位請求失敗: {exc}")
                return self._abort_localization(goal_handle, result, "全域定位請求失敗")

            # 重新撒粒子之後協方差一定很大，先歸零自己的判斷值
            self.current_covariance_norm = float("inf")

            # 逼 AMCL 就地算一次，讓 /amcl_pose 立刻反映新的粒子雲。
            # 少了這一步，收斂判斷會沿用重撒之前的舊協方差，可能一進迴圈就誤判成功。
            if self.nomotion_update_client.wait_for_service(timeout_sec=2.0):
                try:
                    wait_for_future(self.nomotion_update_client.call_async(EmptySrv.Request()), 5.0)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"request_nomotion_update 失敗: {exc}")

            # ★ 2026-08-25：先清掉上一輪的雷射時戳，再打開 _localizing。
            #   不清的話，上一次定位留下的 _scan_stamp 會被誤認成「這一輪的資料」。
            #   _scan_callback 只在 _localizing 為 True 時才更新，所以順序不能反。
            self._scan_stamp = 0.0
            self._localizing = True
            try:
                outcome = self._drive_until_converged(goal_handle)
            finally:
                self._localizing = False
            self._stop_robot()

            if outcome == "cancel":
                result.success = False
                result.message = "全域定位請求已被系統或使用者取消"
                goal_handle.canceled()
                self.get_logger().info(result.message)
                return result

            if outcome != "converged":
                # ★ 2026-08-25：訊息要說出**真正的**失敗原因。
                #   原本不論什麼結果都寫「超時失敗」，而 blocked 是「被障礙擋住」、
                #   跟等了 240 秒是完全不同的兩件事 —— 現場拿錯誤訊息判斷
                #   下一步要做什麼，講錯就會往錯的方向查。
                why = {
                    "blocked": "全域定位失敗：前後都有障礙或雷射沒有資料，"
                               "車子已停下。請把車推到空一點的位置再試",
                    "timeout": f"全域定位失敗：{self.global_localization_timeout_sec:.0f} "
                               f"秒內 AMCL 沒有收斂",
                }.get(outcome, f"全域定位失敗（{outcome}）")
                return self._abort_localization(goal_handle, result, why)

            result.pose = copy.deepcopy(self.current_pose)
            result.success = True
            result.message = "全域定位成功"
            goal_handle.succeed()
            self.get_logger().info("✓ 全域定位成功")
            return result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"全域定位錯誤: {exc}")
            self._localizing = False
            self._stop_robot()
            return self._abort_localization(goal_handle, result, "系統出現異常，全域定位失敗")

    def _drive_until_converged(self, goal_handle) -> str:
        """開著車跑，直到 AMCL 收斂

        阿克曼底盤不能原地轉，只能靠「移動 + 轉向」讓雷射掃到不同的環境特徵。
        策略：以 0.9 m 半徑的圓弧前進；前方太近就倒車並換邊，避免撞牆。
        每一圈換一次轉向，讓粒子有機會看到不同方向的特徵。
        """
        # ★★ 2026-08-25：先等到第一幀雷射再動 ★★
        #   下面每一圈都會檢查「雷射有沒有超過 1 秒沒更新」，但 _scan_stamp
        #   在第一幀到達之前是 0，第一圈就會誤判成掉線。雷射是 10 Hz，
        #   正常情況下這裡最多等 100 ms；等不到才是真的有問題。
        _wait_until = time.monotonic() + 2.0
        while self._scan_stamp <= 0.0 and time.monotonic() < _wait_until:
            if goal_handle.is_cancel_requested:
                return "cancel"
            time.sleep(0.05)
        if self._scan_stamp <= 0.0:
            self.get_logger().error(
                "全域定位中止：等了 2 秒還沒收到任何雷射掃描。"
                "★ 先確認 sensors_cc 有起來、`ros2 topic hz /scan` 約 10 Hz")
            return "blocked"

        deadline = time.monotonic() + self.global_localization_timeout_sec
        direction = 1.0  # 1 = 左轉, -1 = 右轉
        reversing = False
        phase_deadline = time.monotonic() + 8.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return "cancel"

            if time.monotonic() > deadline:
                return "timeout"

            # AMCL 只在移動超過 update_min_d/a 之後才更新，這裡靠實際移動觸發，
            # 但仍每隔一段時間請它強制更新一次，避免慢速下遲遲不收斂。
            if self.current_covariance_norm <= self.max_covariance_norm and self.pose_received:
                return "converged"

            now = time.monotonic()
            if now > phase_deadline:
                # 換一個階段：換轉向，並視前方狀況決定要不要倒車
                direction *= -1.0
                reversing = False
                phase_deadline = now + 8.0

            # ★★★ 2026-08-25：雷射過期就停 ★★★
            #   下面每一個判斷都建立在「掃描是新的」之上。雷射掉線時
            #   min_front/rear_distance 會凍在最後一個值，而這個迴圈會拿著
            #   那個舊值繼續開車。
            if self._scan_stamp <= 0.0 or now - self._scan_stamp > 1.0:
                self._stop_robot()
                self.get_logger().error(
                    "全域定位中止：超過 1 秒沒有收到雷射掃描，"
                    "沒有感測就不移動")
                return "blocked"

            twist = Twist()
            if not reversing and self.min_front_distance < self.front_clearance_m:
                # 前方太近，改倒車並反向打方向盤 (阿克曼車倒車時轉向反向才會擺尾脫困)
                reversing = True
                phase_deadline = now + 4.0

            # ★★★ 2026-08-25：倒車每一圈都要看後方 ★★★
            #
            # 這個迴圈原本**只有正前方 ±30° 一個感測輸入**（FRONT_HALF_ANGLE）。
            # 進入倒車之後會連續 4 秒、每 0.1 秒發一次 -0.18 m/s，
            # 中間**沒有任何一行檢查車後**。而車尾撞牆之後車子不動、
            # 雷射畫面不變 -> AMCL 不會收斂 -> 8 秒換階段 -> 又判定前方太近
            # -> 再倒 4 秒，一路重複到 global_localization_timeout_sec = 240 秒。
            #
            # ★ 這就是使用者回報的「倒車一直撞牆然後還繼續移動沒有停」，
            #   而且它可以持續**四分鐘**。
            #
            # 鏈上也沒有東西會救：PolygonStop 是 enabled: false，
            # PolygonSlow 只乘 0.85，FootprintApproach 只縮放。
            if reversing and self.min_rear_distance < self.front_clearance_m:
                self._stop_robot()
                self.get_logger().warn(
                    f"全域定位中止：後方只剩 {self.min_rear_distance:.2f} m"
                    f"（門檻 {self.front_clearance_m:.2f} m），停止倒車。"
                    f"★ 前後都不通，請把車推到空一點的位置再重新定位")
                return "blocked"

            if reversing:
                twist.linear.x = -self.relocalize_linear_speed
                twist.angular.z = -direction * self.relocalize_angular_speed
            else:
                twist.linear.x = self.relocalize_linear_speed
                twist.angular.z = direction * self.relocalize_angular_speed

            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        return "cancel"

    def _stop_robot(self) -> None:
        for _ in range(3):
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.05)

    def _ensure_localization_mode(self) -> bool:
        """請 map_service_cc 把系統切到定位模式"""
        if not self.ensure_localization_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/ensure_localization 服務不存在 (map_service_cc 沒起來?)")
            return False
        try:
            res = wait_for_future(self.ensure_localization_client.call_async(Trigger.Request()), 60.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"呼叫 /ensure_localization 失敗: {exc}")
            return False

        if not res.success:
            self.get_logger().error(f"切換到定位模式失敗: {res.message}")
        return res.success

    def _abort_localization(self, goal_handle, result, message: str):
        result.success = False
        result.message = message
        goal_handle.abort()
        self.get_logger().error(f"全域定位失敗: {message}")
        return result

    # ==================================================================
    # 訂閱回呼
    # ==================================================================
    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._map_info = msg.info

    def _pose_in_map(self, pose) -> bool:
        """座標是否落在目前地圖範圍內

        為什麼要擋：HMI 的 CreateWaypointRequest 把 use_given_pose 預設成 true，
        前端若只帶了名稱、沒帶座標，就會用預設的 (0, 0) 建出一個地點。
        而地圖原點是建圖起點決定的，(0, 0) 通常根本不在圖裡 ——
        這種地點存得進資料庫、在 HMI 上看得到，但一導航就失敗，
        規劃器回 START/GOAL_OUTSIDE_MAP，使用者只會看到「導航過程出現異常」。
        實測就是這樣建出一個永遠去不了的「接待點」。
        與其讓它變成一顆地雷，不如在建立當下就拒絕。
        """
        info = self._map_info
        if info is None:
            return True  # 還不知道地圖範圍就不擋，避免誤殺
        x = pose.position.x
        y = pose.position.y
        x0 = info.origin.position.x
        y0 = info.origin.position.y
        x1 = x0 + info.width * info.resolution
        y1 = y0 + info.height * info.resolution
        return x0 <= x <= x1 and y0 <= y <= y1

    def _current_map_callback(self, msg: String) -> None:
        if msg.data != self.current_map:
            self.current_map = msg.data
            self.get_logger().info(f"更新當前地圖: {self.current_map or '(無)'}")

    def _update_current_pose(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform("map", self.robot_base_frame, Time())
        except Exception:  # noqa: BLE001
            # 建圖模式或還沒定位好時查不到是正常的，不要洗版
            return

        self.current_pose.position.x = tf.transform.translation.x
        self.current_pose.position.y = tf.transform.translation.y
        self.current_pose.position.z = tf.transform.translation.z
        self.current_pose.orientation = tf.transform.rotation
        self.pose_received = True

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        cov_xx = msg.pose.covariance[0]
        cov_yy = msg.pose.covariance[7]
        self.current_covariance_norm = math.sqrt(cov_xx**2 + cov_yy**2)

    def _scan_callback(self, msg: LaserScan) -> None:
        """算正前方 ±30° 的最短距離

        兩層優化，在 Pi 4 上是實測可見的差別：

        1. 只在全域定位進行中才算。
           min_front_distance 只有 _drive_until_converged 會讀，那是偶發動作。
           但雷射是 10 Hz 持續進來的，原本每一則都全量計算，等於整台車
           從開機到關機都在燒這個 CPU。平常直接 return 成本幾乎為零。

        2. 真的要算時，只掃前方那一段索引，而且不做三角函數。
           原本對 450 個點逐點 atan2(sin, cos) 正規化角度 —— 10 Hz 下
           是每秒 4500 次三角函數，而其中約 85% 的點根本不在 ±30° 內。
           角度到索引的對應只跟雷達參數有關，算一次快取起來即可。
        """
        if not self._localizing:
            return

        n = len(msg.ranges)
        if n == 0:
            return

        if self._front_idx_n != n:
            self._front_idx_n = n
            self._front_ranges = self._compute_sector_index_ranges(msg, n, 0.0)
            # ★★ 2026-08-25：後方扇形。全域定位會倒車，而在這之前
            #   這支節點**只看正前方 ±30°** —— 倒車那 4 秒完全沒有感測。
            self._rear_ranges = self._compute_sector_index_ranges(msg, n, math.pi)

        self.min_front_distance = self._sector_min(msg, self._front_ranges)
        self.min_rear_distance = self._sector_min(msg, self._rear_ranges)
        self._scan_stamp = time.monotonic()

    @staticmethod
    def _sector_min(msg: LaserScan, index_ranges) -> float:
        """扇形內最近的有效回波。**沒有有效回波時回 0.0，不是 inf。**

        ★★ 這是 fail-open 改成 fail-safe ★★

        原本的寫法是 `best = float("inf")`，掃完沒更新就把 inf 寫出去，
        而下游判斷是 `min_front_distance < front_clearance_m` —— inf 永遠不小於
        0.7，於是「量不到」與「很空曠」得到**完全一樣**的結果。

        什麼時候會一個有效點都沒有：玻璃門、深色或鏡面的牆、貼得太近低於
        range_min、雷達正在重連、被車身自己擋住。這些正好都是**危險**的情況，
        卻會被判成安全。改成 0.0（當作貼著障礙），寧可誤停也不要誤衝。

        ★ 這與 maprun/tools_0825/rear_blind_check.py 的立場一致：
          「有沒有量到」與「量到多遠」是兩件事，必須分開。
        """
        best = float("inf")
        n_valid = 0
        for lo, hi in index_ranges:
            for r in msg.ranges[lo:hi]:
                # inf/nan 的比較一律為 False，所以這個條件同時濾掉它們
                if 0.0 < r < float("inf"):
                    n_valid += 1
                    if r < best:
                        best = r
        # 少於 3 點就當作沒資料：單一雜訊點不足以宣告「這個方向是安全的」。
        return best if n_valid >= 3 else 0.0

    @staticmethod
    def _compute_sector_index_ranges(msg: LaserScan, n: int, center: float,
                                     half: float = FRONT_HALF_ANGLE):
        """算出以 center 為中心、±half 的扇形對應的索引區間

        角度不一定落在陣列中央 (取決於 angle_min)，而且扇形可能跨越
        陣列頭尾，所以回傳的是區間列表而不是單一區間。

        ★ center 是參數而不是寫死 0：前方與後方用同一套邏輯，
          後方傳 math.pi 即可（正後方跨頭尾，這個函式本來就處理得了）。
        """
        ranges = []
        lo = None
        for i in range(n):
            a = msg.angle_min + i * msg.angle_increment - center
            a = math.atan2(math.sin(a), math.cos(a))
            inside = abs(a) < half
            if inside and lo is None:
                lo = i
            elif not inside and lo is not None:
                ranges.append((lo, i))
                lo = None
        if lo is not None:
            ranges.append((lo, n))
        return ranges

    # ==================================================================
    # 資料庫
    # ==================================================================
    def _save_waypoints_db(self) -> bool:
        """把地點資料庫寫回 json，回傳有沒有寫成功

        原本失敗只寫進 log 就算了，呼叫端無從得知。刪除地點需要知道結果才能
        在寫檔失敗時把記憶體那份還原——否則畫面上刪掉了、檔案裡還在，
        下次重啟又冒出來。
        """
        serializable = {}
        for waypoint_id, meta in self.waypoints_db.items():
            pose = meta["pose"]
            serializable[waypoint_id] = {
                "name": meta["name"],
                "map_id": meta["map_id"],
                "pose": {
                    "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
                    "orientation": {
                        "x": pose.orientation.x,
                        "y": pose.orientation.y,
                        "z": pose.orientation.z,
                        "w": pose.orientation.w,
                    },
                },
            }

        try:
            self.base_dir.mkdir(parents=True, exist_ok=True)
            with open(self.db_file, "w", encoding="utf-8") as fp:
                json.dump({"waypoints_db": serializable}, fp, ensure_ascii=False, indent=4)
            self.get_logger().info(f"已儲存地點資料庫，共 {len(self.waypoints_db)} 個")
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"儲存地點資料庫失敗: {exc}")
            return False

    def _load_waypoints_db(self) -> None:
        try:
            if not self.db_file.exists():
                self.get_logger().info("找不到地點資料庫，使用空資料庫")
                return

            with open(self.db_file, "r", encoding="utf-8") as fp:
                raw = json.load(fp).get("waypoints_db", {})

            loaded = {}
            for waypoint_id, meta in raw.items():
                pose_data = meta["pose"]
                pose = Pose()
                pose.position.x = pose_data["position"]["x"]
                pose.position.y = pose_data["position"]["y"]
                pose.position.z = pose_data["position"]["z"]
                pose.orientation.x = pose_data["orientation"]["x"]
                pose.orientation.y = pose_data["orientation"]["y"]
                pose.orientation.z = pose_data["orientation"]["z"]
                pose.orientation.w = pose_data["orientation"]["w"]
                loaded[waypoint_id] = {"name": meta["name"], "map_id": meta["map_id"], "pose": pose}

            with self._db_lock:
                self.waypoints_db = loaded
            self.get_logger().info(f"已載入地點資料庫，共 {len(loaded)} 個")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"載入地點資料庫失敗: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointServiceCcNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
