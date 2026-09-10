#!/usr/bin/env python3

"""地圖服務節點 (_cc 重寫版)

這個節點是整套定位/建圖的「唯一模式仲裁者」。amcl、map_server、map_saver、
slam_toolbox 四個生命週期節點只由它一個人操作，其他節點想切模式必須呼叫
本節點提供的服務，不可以自己去打 /amcl/change_state。

兩種模式互斥，任何時刻都成立：

    建圖模式 (mapping)
        slam_toolbox  = active     -> 發布 /map 與 map -> odom_combined
        map_server    = unconfigured
        amcl          = unconfigured

    定位模式 (localization)
        map_server    = active     -> 發布 /map
        amcl          = active     -> 發布 map -> odom_combined
        slam_toolbox  = unconfigured

舊版壞在兩件事上：
  1. 建完圖之後只把 map_server 拉回 active，slam_toolbox 卻留在 active。
     於是 /map 有兩個發布者、map -> odom_combined 也有兩個發布者。
     global_costmap 的 static_layer 在不同大小的地圖之間反覆 resize，
     車子走出舊地圖範圍就變成 "Robot is out of bounds of the costmap"，
     接著規劃器一路回報 "Start occupied"，每一個導航目標都失敗。
  2. 探索完成是靠訂閱 /rosout 比對字串 "Exploration finished"，
     字串一變或那個分支沒被執行到，建圖就只能等 timeout。
     現在改用 frontier_explorer 正式發布的 /exploration_complete 事件。

對外介面刻意沿用舊名稱，HMI 與 smartnav_brain 不需要任何修改：
    /create_map   (action  smartnav_msgs/CreateMap)
    /list_maps    (service smartnav_msgs/ListMaps)
    /switch_map   (service smartnav_msgs/SwitchMap)
    /current_map  (topic   std_msgs/String, transient_local)

新增的輔助介面：
    /finish_map            (std_srvs/Trigger) 手動結束建圖並存檔
    /ensure_localization   (std_srvs/Trigger) 確保切回 AMCL 定位模式
    /get_nav_mode          (std_srvs/Trigger) 查詢目前模式
"""

import json
import math
import threading
import time
import uuid
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import Trigger
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav2_msgs.srv import LoadMap as Nav2LoadMap
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.srv import SaveMap as Nav2SaveMap
from tf2_ros import Buffer, TransformListener

from frontier_exploration_ros2.srv import ControlExploration
from smartnav_msgs.action import CreateMap
from smartnav_msgs.msg import MapInfo
from smartnav_msgs.srv import DeleteMap, ListMaps, SwitchMap

from smartnav_navigation_cc.lifecycle_helper_cc import (
    ACTIVE,
    UNCONFIGURED,
    LifecycleNodeClient,
    wait_for_future,
)

MODE_UNKNOWN = "unknown"
MODE_MAPPING = "mapping"
MODE_LOCALIZATION = "localization"


class MapServiceCcNode(Node):
    """地圖服務 / 模式仲裁節點"""

    def __init__(self):
        super().__init__("map_service_cc_node")

        # ------------------------------------------------------------------
        # 參數
        # ------------------------------------------------------------------
        self.declare_parameter("start_mode", "auto")
        self.declare_parameter("use_exploration", True)
        self.declare_parameter("nav2_lifecycle_manager", "lifecycle_manager_navigation_cc")
        # 自動探索的最長時間。舊版寫 1200 秒，實測整層樓跑不完就被砍掉。
        self.declare_parameter("exploration_timeout_sec", 1800.0)
        # 等 map -> base_footprint 出現的時間 (Pi 4 冷開機時 slam 第一張圖要好幾秒)
        self.declare_parameter("tf_ready_timeout_sec", 60.0)
        self.declare_parameter("lifecycle_transition_timeout_sec", 25.0)
        # map_saver 的 save_map_timeout 是 15 秒，這裡必須留得比它寬
        self.declare_parameter("save_map_timeout_sec", 25.0)
        # 定位模式下強制 AMCL 就地更新的週期 (秒)，<=0 表示關閉。
        # 這不是可有可無的優化，是這台車的必要補償，原因見 _amcl_refresh_tick。
        self.declare_parameter("amcl_refresh_period_sec", 3.0)
        # 進入定位模式時，用當前雷射與地圖做一次對齊修正 seed 位姿
        self.declare_parameter("align_pose_on_localize", True)
        self.declare_parameter("align_search_yaw_deg", 50.0)
        self.declare_parameter("align_search_xy_m", 0.30)
        # 單次對齊允許的最大角度修正。超過就拒絕 —— 見 _align_pose_with_scan 的說明。
        self.declare_parameter("align_max_correction_deg", 45.0)

        self.start_mode = self.get_parameter("start_mode").value
        self.use_exploration = bool(self.get_parameter("use_exploration").value)
        self.nav2_lcm = self.get_parameter("nav2_lifecycle_manager").value
        self.exploration_timeout_sec = float(self.get_parameter("exploration_timeout_sec").value)
        self.tf_ready_timeout_sec = float(self.get_parameter("tf_ready_timeout_sec").value)
        self.save_map_timeout_sec = float(self.get_parameter("save_map_timeout_sec").value)
        self.amcl_refresh_period_sec = float(self.get_parameter("amcl_refresh_period_sec").value)
        self.align_pose_on_localize = bool(self.get_parameter("align_pose_on_localize").value)
        self.align_search_yaw_deg = float(self.get_parameter("align_search_yaw_deg").value)
        self.align_search_xy_m = float(self.get_parameter("align_search_xy_m").value)
        self.align_max_correction_deg = float(self.get_parameter("align_max_correction_deg").value)
        transition_timeout = float(self.get_parameter("lifecycle_transition_timeout_sec").value)

        # ------------------------------------------------------------------
        # 回呼群組
        #   server_cb_group: 對外的服務/動作，彼此序列化
        #   client_cb_group: 對內的客戶端呼叫，可重入 (才能在動作裡面等服務回應)
        # ------------------------------------------------------------------
        self.server_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = ReentrantCallbackGroup()

        # ------------------------------------------------------------------
        # 生命週期遙控器 —— 這四個節點只有本節點能動
        # ------------------------------------------------------------------
        self.lc_map_server = LifecycleNodeClient(
            self, "map_server", self.client_cb_group, transition_timeout
        )
        self.lc_map_saver = LifecycleNodeClient(
            self, "map_saver", self.client_cb_group, transition_timeout
        )
        self.lc_amcl = LifecycleNodeClient(self, "amcl", self.client_cb_group, transition_timeout)
        self.lc_slam = LifecycleNodeClient(
            self, "slam_toolbox", self.client_cb_group, transition_timeout
        )

        # ------------------------------------------------------------------
        # 客戶端
        # ------------------------------------------------------------------
        self.load_map_client = self.create_client(
            Nav2LoadMap, "/map_server/load_map", callback_group=self.client_cb_group
        )
        self.save_map_client = self.create_client(
            Nav2SaveMap, "/map_saver/save_map", callback_group=self.client_cb_group
        )
        self.amcl_set_params_client = self.create_client(
            SetParameters, "/amcl/set_parameters", callback_group=self.client_cb_group
        )
        # 逼 amcl 就地更新一次並發布 /amcl_pose (不移動的話它預設不會發)
        self.nomotion_update_client = self.create_client(
            EmptySrv, "/request_nomotion_update", callback_group=self.client_cb_group
        )
        self.control_exploration_client = self.create_client(
            ControlExploration, "/control_exploration", callback_group=self.client_cb_group
        )
        self.nav2_manage_client = self.create_client(
            ManageLifecycleNodes, f"/{self.nav2_lcm}/manage_nodes", callback_group=self.client_cb_group
        )

        # ------------------------------------------------------------------
        # 發布 / 訂閱
        # ------------------------------------------------------------------
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.current_map_pub = self.create_publisher(String, "current_map", latched)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)

        # frontier_explorer 的完成事件是 transient_local，訂閱當下就會收到上一輪
        # 留在那裡的舊事件，所以一定要用 _exploration_epoch 過濾。
        # 訂閱 /map 只為了記住 info (寬高/解析度/origin)，給 _map_center_pose 當退路用
        self.create_subscription(
            OccupancyGrid, "/map", self._map_info_callback, latched,
            callback_group=self.client_cb_group,
        )
        self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data,
            callback_group=self.client_cb_group,
        )
        self.exploration_complete_sub = self.create_subscription(
            Empty,
            "/exploration_complete",
            self._exploration_complete_callback,
            latched,
            callback_group=self.client_cb_group,
        )

        # TF 訂閱的 QoS depth 從預設 100 降到 20。
        #
        # /tf 有 60 Hz，而 rclpy 每次 executor 輪詢會把佇列裡積壓的訊息**全部**
        # 取出來、在 Python 端逐一反序列化成物件樹再塞進 buffer。depth=100 表示
        # Pi 4 忙起來時一次要處理上百則，這個節點因此吃掉約 25% CPU。
        #
        # 但本節點只在三個地方查 TF，而且都是查「最新」的姿態
        # （can_transform/lookup_transform 都用 Time()）：
        #   486 行  確認 map->base_footprint 是否已經建立
        #   574 行  確認 odom_combined->base_footprint 是否已經建立
        #   597 行  每 30 秒存一次位姿
        # 中間那些被丟掉的 TF 訊息對這些用途完全沒有影響。
        #
        # 注意：不要壓到 1。實測 depth=1 反而更貴（32% vs 20%）——
        # 每則訊息各觸發一次 executor 喚醒，失去批次處理的攤提效果。
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=20, static_qos=1)

        # ------------------------------------------------------------------
        # 對外服務
        # ------------------------------------------------------------------
        self.create_service(ListMaps, "list_maps", self._list_maps_callback, callback_group=self.server_cb_group)
        self.create_service(SwitchMap, "switch_map", self._switch_map_callback, callback_group=self.server_cb_group)
        self.create_service(
            DeleteMap, "delete_map", self._delete_map_callback,
            callback_group=self.server_cb_group,
        )
        self.create_service(Trigger, "finish_map", self._finish_map_callback, callback_group=self.client_cb_group)
        self.create_service(
            Trigger, "ensure_localization", self._ensure_localization_callback, callback_group=self.server_cb_group
        )
        self.create_service(
            Trigger, "get_nav_mode", self._get_nav_mode_callback, callback_group=self.client_cb_group
        )
        # 導航前重新對齊位姿。IMU 零偏會讓車子靜置時 yaw 一直漂 (見 _amcl_refresh_tick)，
        # 導航起步前對一次，才不會照著錯的角度規劃。
        self.create_service(
            Trigger, "align_pose", self._align_pose_callback, callback_group=self.client_cb_group
        )
        # 手動開始自動探索。用於「窄走廊先遙控、進到寬敞區域再自動探索」的流程：
        # 阿克曼車在 0.99 m 走廊裡無法掉頭 (轉彎直徑 1.6 m)，自動探索一定卡死，
        # 但進到房間後就完全可行。
        self.create_service(
            Trigger, "start_exploration", self._start_exploration_callback,
            callback_group=self.client_cb_group,
        )
        self.create_map_action = ActionServer(
            self,
            CreateMap,
            "create_map",
            execute_callback=self._create_map_callback,
            goal_callback=self._create_map_goal_callback,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self.server_cb_group,
        )

        # ------------------------------------------------------------------
        # 狀態
        # ------------------------------------------------------------------
        self.base_dir = Path.home() / ".smartnav"
        self.map_data_dir = self.base_dir / "map_database" / "data"
        self.db_file = self.base_dir / "map_database" / "maps_db.json"
        self.state_file = self.base_dir / "nav_state.json"

        self.maps_db = {}
        self._db_lock = threading.Lock()
        # 模式切換一次只能有一個人在做
        self._mode_lock = threading.RLock()
        self.mode = MODE_UNKNOWN
        self.current_map_id = ""

        # 探索完成事件的過濾用：只認「本輪開始之後」收到的事件
        self._last_map_info = None
        self._last_map = None
        self._last_scan = None
        self._exploration_epoch = 0
        self._exploration_active = False
        self._exploration_done_epoch = -1
        # /finish_map 觸發旗標
        self._finish_requested = False

        self._load_maps_db()
        self.current_map_pub.publish(String(data=""))

        # 定期把位姿寫回 nav_state.json，這樣下次開機才接得回來。
        # 30 秒一次：夠頻繁到重啟不會差太多，又不會一直寫 SD 卡。
        self.create_timer(30.0, self._persist_pose_tick, callback_group=self.client_cb_group)

        if self.amcl_refresh_period_sec > 0:
            self.create_timer(
                self.amcl_refresh_period_sec, self._amcl_refresh_tick,
                callback_group=self.client_cb_group,
            )

        # 開機流程會阻塞好幾十秒，丟到獨立執行緒，不要卡住 executor
        self._startup_thread = threading.Thread(target=self._startup_sequence, daemon=True)
        self._startup_thread.start()

        self.get_logger().info("地圖服務節點 (_cc) 已初始化")

    # ==================================================================
    # 開機流程
    # ==================================================================
    def _startup_sequence(self) -> None:
        """決定初始模式，模式就緒後才放行 nav2"""
        try:
            self.get_logger().info("等待 amcl / map_server / map_saver / slam_toolbox 的生命週期服務...")
            for client in (self.lc_map_server, self.lc_map_saver, self.lc_amcl, self.lc_slam):
                while rclpy.ok() and not client.wait_until_available(timeout_sec=2.0):
                    self.get_logger().warn(f"⌛ 等待 {client.node_name} 的生命週期服務...")
            self.get_logger().info("✓ 四個生命週期節點都在線上")

            # map_saver 跟模式互斥無關，先拉到 active 備用
            if not self.lc_map_saver.ensure_state(ACTIVE):
                self.get_logger().error("map_saver 無法啟用，之後存地圖會失敗")

            target_map = self._resolve_startup_map()
            if self.start_mode == "mapping" or target_map is None:
                if self.start_mode == "localization":
                    self.get_logger().warn("指定了 localization 模式但沒有可用的地圖，改用建圖模式待命")
                self._enter_mapping_mode()
            else:
                # 用上次記錄的位姿開機。沒有的話 _enter_localization_mode 會退而
                # 求其次用地圖中心 —— 就是不能用 (0,0)，那多半在地圖外。
                startup_pose = self._load_last_pose()
                if startup_pose is not None:
                    self.get_logger().info(
                        f"沿用上次位姿 ({startup_pose[0]:.2f}, {startup_pose[1]:.2f}, "
                        f"{math.degrees(startup_pose[2]):.0f}°)"
                    )
                if not self._enter_localization_mode(target_map, initial_pose=startup_pose):
                    self.get_logger().warn("進入定位模式失敗，退回建圖模式待命")
                    self._enter_mapping_mode()

            # 模式就緒 (map -> odom_combined 已經有唯一的發布者) 之後才啟動 nav2。
            # 這樣 global_costmap 在 activate 時一定拿得到 map -> base_footprint，
            # 不會出現舊版那種要靠 initial_transform_timeout 硬等的競態。
            self._startup_nav2()
        except Exception as exc:  # noqa: BLE001 - 開機執行緒不能讓例外逃掉
            self.get_logger().error(f"開機流程發生例外: {exc}")

    def _resolve_startup_map(self) -> Optional[str]:
        """開機要載入哪張地圖；沒有可用地圖回 None"""
        if self.start_mode == "mapping":
            return None

        map_id = self._load_map_state()
        if map_id and self._map_file(map_id).exists():
            return map_id

        # nav_state.json 沒有或指向不存在的檔案時，退而求其次找資料庫裡任何一張存在的
        with self._db_lock:
            candidates = list(self.maps_db.keys())
        for candidate in candidates:
            if self._map_file(candidate).exists():
                self.get_logger().warn(f"nav_state.json 無效，改用資料庫中的地圖 {candidate}")
                return candidate

        self.get_logger().info("找不到任何已存的地圖，進入建圖模式待命")
        return None

    def _startup_nav2(self) -> None:
        """呼叫 nav2 的 lifecycle_manager 做 STARTUP"""
        self.get_logger().info(f"等待 /{self.nav2_lcm}/manage_nodes ...")
        while rclpy.ok() and not self.nav2_manage_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"⌛ 等待 /{self.nav2_lcm}/manage_nodes ...")

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.STARTUP
        try:
            # nav2 全部節點 configure + activate 在 Pi 4 上要一分鐘以上
            res = wait_for_future(self.nav2_manage_client.call_async(req), timeout_sec=180.0)
            if res.success:
                self.get_logger().info("✓ Nav2 已啟動")
            else:
                self.get_logger().error("Nav2 啟動失敗 (lifecycle_manager 回報 success=false)")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"呼叫 Nav2 lifecycle_manager 失敗: {exc}")

    # ==================================================================
    # 模式切換
    # ==================================================================
    def _enter_mapping_mode(self) -> bool:
        """進入建圖模式：只有 slam_toolbox 在發 /map 與 map -> odom_combined"""
        with self._mode_lock:
            if self.mode == MODE_MAPPING:
                return True

            self.get_logger().info("切換到建圖模式...")

            # 順序很重要：先讓舊的地圖來源徹底下線，再讓 slam 上線。
            # 反過來做會有一小段時間兩個 /map 發布者並存，static_layer 會抖動。
            if not self.lc_amcl.ensure_state(UNCONFIGURED):
                self.get_logger().error("amcl 無法下線，中止切換")
                return False
            # 一定要退到 unconfigured 而不是 inactive：
            # inactive 時 map_server 的 transient_local publisher 還活著，
            # 舊地圖會繼續留在 /map 上，map_saver 一訂閱就先收到那張舊圖。
            if not self.lc_map_server.ensure_state(UNCONFIGURED):
                self.get_logger().error("map_server 無法下線，中止切換")
                return False
            if not self.lc_slam.ensure_state(ACTIVE):
                self.get_logger().error("slam_toolbox 無法啟用，中止切換")
                return False

            if not self._wait_for_map_tf():
                self.get_logger().error("slam_toolbox 已 active 但等不到 map -> base_footprint")
                return False

            self.mode = MODE_MAPPING
            self.current_map_id = ""
            self.current_map_pub.publish(String(data=""))
            self.get_logger().info("✓ 已進入建圖模式")
            return True

    def _enter_localization_mode(self, map_id: str, initial_pose=None) -> bool:
        """進入定位模式：只有 map_server 發 /map、只有 amcl 發 map -> odom_combined

        Args:
            map_id: 要載入的地圖 ID
            initial_pose: (x, y, yaw)，None 表示用原點
        """
        with self._mode_lock:
            self.get_logger().info(f"切換到定位模式 (地圖 {map_id})...")

            map_file = self._map_file(map_id)
            if not map_file.exists():
                self.get_logger().error(f"地圖檔不存在: {map_file}")
                return False

            # 這一步就是舊版漏掉的：slam_toolbox 沒下線的話，
            # 它會繼續發 /map 跟 map -> odom_combined，和 map_server / amcl 打架。
            if not self.lc_slam.ensure_state(UNCONFIGURED):
                self.get_logger().error("slam_toolbox 無法下線，中止切換")
                return False

            # amcl 的初始位姿必須在 configure 之前寫進參數，
            # on_configure 才讀得到 (activate 之後再改就只能靠 /initialpose)。
            # pose_known = 位姿是「查出來的」而不是預設原點：
            #   建完圖時直接沿用 SLAM 的最後位姿 -> 可以用很緊的協方差；
            #   切換地圖 / 開機沒有依據時 -> 落在原點，協方差要放寬，等全域定位收斂。
            pose_known = initial_pose is not None
            if pose_known:
                x, y, yaw = initial_pose
            else:
                # 絕對不要用 (0,0)：地圖原點由建圖起點決定，(0,0) 常常在圖外，
                # 會讓規劃器直接回 START_OUTSIDE_MAP。地圖中心至少一定在圖內。
                fallback = self._map_center_pose()
                if fallback is None:
                    x, y, yaw = 0.0, 0.0, 0.0
                else:
                    x, y, yaw = fallback
                    self.get_logger().warn(
                        f"沒有已知位姿，改用地圖中心 ({x:.2f}, {y:.2f})，"
                        "請執行全域定位讓 AMCL 收斂"
                    )
            self._set_amcl_initial_pose_params(x, y, yaw)

            if not self.lc_map_server.ensure_state(ACTIVE):
                self.get_logger().error("map_server 無法啟用，中止切換")
                return False
            if not self._load_map(map_id):
                self.get_logger().error("載入地圖失敗，中止切換")
                return False
            if not self.lc_amcl.ensure_state(ACTIVE):
                self.get_logger().error("amcl 無法啟用，中止切換")
                return False

            # 在餵給 amcl 之前，先用雷射跟地圖對一次。
            # nav_state.json 存的位姿可能已經被 IMU 零偏帶歪 (見 _align_pose_with_scan)，
            # 直接拿去 seed 等於一開機就定位錯誤。
            if pose_known and self.align_pose_on_localize:
                aligned = self._align_pose_with_scan((x, y, yaw))
                if aligned is not None:
                    x, y, yaw = aligned

            # 用話題再送一次初始位姿並逼 amcl 立刻回報。
            # 參數只能決定「位置」，協方差一律是 amcl 內建的寬值 (norm ≈ 0.354)，
            # 會直接卡死 navigation_action_cc 的 0.15 定位門檻，所以位姿明確時
            # 一定要靠 /initialpose 補一個緊的協方差進去。
            self._seed_amcl_pose(x, y, yaw, tight=pose_known)

            if not self._wait_for_map_tf():
                self.get_logger().error("amcl 已 active 但等不到 map -> base_footprint")
                return False

            self.mode = MODE_LOCALIZATION
            self.current_map_id = map_id
            self.current_map_pub.publish(String(data=map_id))
            self._save_map_state(map_id)
            self.get_logger().info(f"✓ 已進入定位模式 (地圖 {map_id})")
            return True

    def _wait_for_map_tf(self) -> bool:
        """等 map -> base_footprint 出現"""
        deadline = time.monotonic() + self.tf_ready_timeout_sec
        while time.monotonic() < deadline:
            if not rclpy.ok():
                return False
            if self.tf_buffer.can_transform("map", "base_footprint", Time()):
                return True
            time.sleep(0.2)
        return False

    def _set_amcl_initial_pose_params(self, x: float, y: float, yaw: float) -> None:
        """在 configure 之前把初始位姿寫進 amcl 的參數"""
        if not self.amcl_set_params_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("/amcl/set_parameters 不在線上，改用 /initialpose 話題")
            return

        def _double(name, value):
            return Parameter(
                name=name,
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)),
            )

        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name="set_initial_pose",
                value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True),
            ),
            _double("initial_pose.x", x),
            _double("initial_pose.y", y),
            _double("initial_pose.z", 0.0),
            _double("initial_pose.yaw", yaw),
        ]
        try:
            wait_for_future(self.amcl_set_params_client.call_async(req), timeout_sec=5.0)
            self.get_logger().info(f"已設定 amcl 初始位姿: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"設定 amcl 初始位姿參數失敗: {exc}")

    def _publish_initial_pose(self, x: float, y: float, yaw: float, tight: bool = False) -> None:
        """發布 /initialpose 讓 amcl 重新初始化粒子雲

        tight=False：位置變異 0.25 m² (std 0.5 m)。給「大概位置」用 (例如切換地圖、
                     開機落在原點)，之後要靠 global_localization 收斂。
        tight=True ：位置變異 0.01 m² (std 0.1 m)。給「明確已知位置」用 (剛建完圖，
                     位姿直接沿用 SLAM 的結果)。
                     這一項很關鍵：navigation_action_cc 的定位門檻是
                     sqrt(cov_xx² + cov_yy²) <= 0.15。loose 的 0.25 換算 norm ≈ 0.354，
                     一定超標 —— 於是「剛建完圖、其實定位很準」卻無法馬上導航，
                     被迫先跑一次全域定位。tight 的 0.01 換算 norm ≈ 0.014，遠低於門檻，
                     建完圖可以直接導航。
        """
        pos_var = 0.01 if tight else 0.25
        yaw_var = 0.00685 if tight else 0.06853891945200942

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = pos_var
        msg.pose.covariance[7] = pos_var
        msg.pose.covariance[35] = yaw_var
        self.initial_pose_pub.publish(msg)

    def _seed_amcl_pose(self, x: float, y: float, yaw: float, tight: bool) -> None:
        """amcl 剛 active 後，把已知位姿餵進去並逼它立刻回報協方差

        amcl 只在機器人移動超過 update_min_d/a 之後才會主動發 /amcl_pose，
        所以剛切到定位模式時 navigation_action_cc 會一直等不到 pose、或拿到的是
        重新初始化前的舊協方差。這裡做兩件事：
          1. 連發幾次 /initialpose (amcl 剛 activate 時訂閱可能還沒接上，多送幾次比較保險)
          2. 呼叫 /request_nomotion_update 逼 amcl 就地跑一次更新並發布 /amcl_pose

        發之前一定要等 base_footprint -> odom_combined 的 TF buffer 有料。
        amcl 收到 initialpose 後會拿訊息的 stamp 去查這條 TF，剛 activate 時
        buffer 是空的，於是前幾則會被丟掉：
            Failed to transform initial pose in time (Lookup would require
            extrapolation into the past ... the earliest data is at time ...)
        丟掉的話粒子雲不會被重設，協方差維持在初始的寬值，
        navigation_action_cc 的定位門檻就會擋下第一次導航。

        註：amcl 剛 activate 時它自己的 TF buffer 是全新的，所以即使等到
        can_transform 成立，前幾則仍可能落在 buffer 最早資料之前而印出這句 WARN。
        **這是良性的** —— 實測 WARN 後面緊接著就是
            [amcl] Setting pose (...): 1.273 -0.049 -0.042
        amcl 會退回直接採用訊息裡的位姿，協方差實測 0.0091 (門檻 0.15) 有生效。
        看到這句 WARN 不用處理，要確認的是後面那句 Setting pose 有沒有出現。
        """
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if self.tf_buffer.can_transform("odom_combined", "base_footprint", Time()):
                break
            time.sleep(0.1)
        else:
            self.get_logger().warn("等不到 odom_combined -> base_footprint，initialpose 可能被丟棄")

        # buffer 剛出現第一筆資料時，用「現在」當 stamp 仍可能落在資料之前，
        # 多留一點餘裕再發。
        time.sleep(0.5)

        for _ in range(3):
            self._publish_initial_pose(x, y, yaw, tight=tight)
            time.sleep(0.2)

        if self.nomotion_update_client.wait_for_service(timeout_sec=2.0):
            try:
                wait_for_future(self.nomotion_update_client.call_async(EmptySrv.Request()), timeout_sec=5.0)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"request_nomotion_update 失敗: {exc}")

    def _current_robot_pose(self):
        """讀 map -> base_footprint，回 (x, y, yaw)，失敗回 None"""
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
        except Exception:  # noqa: BLE001
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    # ==================================================================
    # 地圖檔案操作
    # ==================================================================
    def _map_file(self, map_id: str) -> Path:
        return self.map_data_dir / f"{map_id}.yaml"

    def _load_map(self, map_id: str) -> bool:
        """呼叫 /map_server/load_map"""
        if not self.load_map_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/map_server/load_map 服務不存在")
            return False

        req = Nav2LoadMap.Request()
        req.map_url = str(self._map_file(map_id))
        try:
            res = wait_for_future(self.load_map_client.call_async(req), timeout_sec=20.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"載入地圖 {map_id} 失敗: {exc}")
            return False

        if res.result != Nav2LoadMap.Response.RESULT_SUCCESS:
            self.get_logger().error(f"載入地圖 {map_id} 失敗，map_server 回傳 result={res.result}")
            return False

        self.get_logger().info(f"✓ 已載入地圖 {map_id}")
        return True

    def _save_map(self, map_id: str) -> bool:
        """呼叫 /map_saver/save_map 把 slam_toolbox 目前的 /map 存檔"""
        self.map_data_dir.mkdir(parents=True, exist_ok=True)

        if not self.save_map_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/map_saver/save_map 服務不存在")
            return False

        # 每個欄位都明確填值。留空會落到 map_saver 的預設值推導邏輯，
        # 不同版本行為不一致 (例如 map_mode 空字串會走例外分支)。
        req = Nav2SaveMap.Request()
        req.map_topic = "map"
        req.map_url = str(self.map_data_dir / map_id)
        req.image_format = "pgm"
        req.map_mode = "trinary"
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65

        try:
            res = wait_for_future(self.save_map_client.call_async(req), timeout_sec=self.save_map_timeout_sec)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"存檔請求失敗: {exc}")
            return False

        if not res.result:
            self.get_logger().error("map_saver 回報存檔失敗")
            return False

        if not self._map_file(map_id).exists():
            self.get_logger().error(f"map_saver 回報成功但找不到 {self._map_file(map_id)}")
            return False

        self.get_logger().info(f"✓ 地圖已存檔: {self._map_file(map_id)}")
        return True

    # ==================================================================
    # create_map 動作
    # ==================================================================
    def _create_map_goal_callback(self, goal_request):
        if not goal_request.map_name:
            self.get_logger().warning("建立地圖被拒絕: 未提供地圖名稱")
            return GoalResponse.REJECT

        # 已經有建圖作業在跑就拒絕。
        #
        # 下面那段名稱檢查只看**已存檔**的地圖（maps_db），擋不住
        # 「同一個名字連按兩次」——第二次按的時候第一張還沒存檔，
        # 名字自然不在 db 裡。2026-07-31 實測：同時跑起三個 create_map 目標，
        # 三個都在等 /finish_map，按下結束時會同時搶存檔與模式切換。
        #
        # 建圖作業本質上是獨佔的（獨佔 slam_toolbox、獨佔模式），
        # 所以這裡用 _exploration_active 直接擋掉第二個。
        if self._exploration_active:
            self.get_logger().warning(
                f"建立地圖被拒絕: 已經有建圖作業在進行中"
                f"（要重新開始請先按「結束並儲存」或取消目前的作業）"
            )
            return GoalResponse.REJECT

        with self._db_lock:
            for meta in self.maps_db.values():
                if meta["name"] == goal_request.map_name:
                    self.get_logger().warning(f"建立地圖被拒絕: 名稱 {goal_request.map_name} 已存在")
                    return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _create_map_callback(self, goal_handle):
        """建立地圖：切到建圖模式 -> 探索/遙控 -> 存檔 -> 切回定位模式"""
        request = goal_handle.request
        result = CreateMap.Result()
        result.map_info = MapInfo()
        map_id = "map_" + uuid.uuid4().hex[:12]

        self.get_logger().info(f"收到建立地圖請求: {request.map_name} -> {map_id}")

        try:
            if not self._enter_mapping_mode():
                return self._abort(goal_handle, result, "無法切換到建圖模式，地圖建立失敗")

            self._finish_requested = False
            if not self._start_exploration():
                return self._abort(goal_handle, result, "無法啟動探索服務，地圖建立失敗")

            outcome = self._wait_for_mapping_done(goal_handle)
            self._stop_exploration()

            if outcome == "cancel":
                result.success = False
                result.message = "地圖建立請求已被系統或使用者取消"
                goal_handle.canceled()
                self.get_logger().info(result.message)
                self._restore_after_failed_mapping()
                return result

            if outcome == "timeout":
                # 逾時不代表沒東西可存，但沿用舊版語意回報失敗，
                # 讓呼叫端知道這張圖不完整。
                return self._abort(goal_handle, result, "地圖建立超時，已自動停止探索")

            # ---- 探索完成，開始存檔 ----
            self.get_logger().info("探索結束，準備存檔")
            end_pose = self._current_robot_pose()

            if not self._save_map(map_id):
                self._restore_after_failed_mapping()
                return self._abort(goal_handle, result, "地圖存檔失敗")

            with self._db_lock:
                self.maps_db[map_id] = {"name": request.map_name}
                self._save_maps_db()

            if not self._enter_localization_mode(map_id, initial_pose=end_pose):
                return self._abort(goal_handle, result, "地圖已存檔，但切回定位模式失敗")

            map_info = MapInfo()
            map_info.map_id = map_id
            map_info.map_name = request.map_name
            result.map_info = map_info
            result.success = True
            result.message = f'地圖 "{request.map_name}" 建立成功'
            goal_handle.succeed()
            self.get_logger().info(f"✓ 地圖建立成功: {map_id} ({request.map_name})")
            return result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"地圖建立發生例外: {exc}")
            self._stop_exploration()
            self._restore_after_failed_mapping()
            return self._abort(goal_handle, result, "系統出現異常，地圖建立失敗")

    def _abort(self, goal_handle, result, message: str):
        result.success = False
        result.message = message
        goal_handle.abort()
        self.get_logger().warning(message)
        return result

    def _restore_after_failed_mapping(self) -> None:
        """建圖失敗時盡量把系統帶回可用狀態

        有舊地圖就切回定位模式；沒有就留在建圖模式待命 —— 不管哪一種，
        map -> odom_combined 都必須有且只有一個發布者，不能讓系統停在真空狀態。
        """
        target = self._resolve_startup_map()
        if target is None:
            self.get_logger().info("沒有可回復的地圖，留在建圖模式待命")
            return
        if not self._enter_localization_mode(target):
            self.get_logger().error("回復定位模式失敗，留在建圖模式")
            self._enter_mapping_mode()

    def _wait_for_mapping_done(self, goal_handle) -> str:
        """等建圖結束，回傳 'done' / 'cancel' / 'timeout'"""
        deadline = time.monotonic() + self.exploration_timeout_sec
        epoch = self._exploration_epoch

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return "cancel"

            if self._finish_requested:
                self.get_logger().info("收到 /finish_map，提前結束建圖")
                return "done"

            if self.use_exploration and self._exploration_done_epoch == epoch:
                self.get_logger().info("收到 /exploration_complete，探索完成")
                return "done"

            if time.monotonic() > deadline:
                return "timeout"

            time.sleep(0.5)

        return "cancel"

    def _start_exploration(self) -> bool:
        """啟動自動探索；手動建圖模式下直接回 True"""
        self._exploration_epoch += 1
        self._exploration_active = True

        if not self.use_exploration:
            self.get_logger().info("未啟用自動探索，請用遙控把環境走一遍，完成後呼叫 /finish_map")
            return True

        if not self.control_exploration_client.wait_for_service(timeout_sec=15.0):
            # 服務不在 = frontier_explorer 節點根本沒啟動。
            # 這種情況下硬失敗是最糟的選擇：使用者按了「開始建圖」，
            # 15 秒後才失敗、而且錯誤只進 log 不上畫面，於是他以為建圖已經
            # 開始、走完一整趟才發現存不了（2026-07-31 實測發生過）。
            #
            # 節點不在就是不可能自動探索，退回遙控建圖是唯一合理的行為。
            # 講清楚原因就好，不要把整個建圖作業斃掉。
            self.get_logger().warning(
                "/control_exploration 服務不存在（frontier_explorer 沒啟動），"
                "自動探索無法使用 —— 已改為遙控建圖模式，"
                "請用遙控把環境走一遍，完成後按「結束並儲存」"
            )
            self.use_exploration = False
            return True

        req = ControlExploration.Request()
        req.action = ControlExploration.Request.ACTION_START
        try:
            res = wait_for_future(self.control_exploration_client.call_async(req), timeout_sec=10.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"啟動探索失敗: {exc}")
            return False

        if not res.accepted:
            self.get_logger().error(f"探索服務拒絕啟動: {res.message}")
            return False

        self.get_logger().info("✓ 自動探索已啟動")
        return True

    def _stop_exploration(self) -> None:
        self._exploration_active = False
        if not self.use_exploration:
            return
        if not self.control_exploration_client.service_is_ready():
            return

        req = ControlExploration.Request()
        req.action = ControlExploration.Request.ACTION_STOP
        try:
            wait_for_future(self.control_exploration_client.call_async(req), timeout_sec=10.0)
            self.get_logger().info("已停止自動探索")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"停止探索失敗: {exc}")

    def _align_pose_with_scan(self, pose):
        """用當前雷射與地圖做一次對齊，回傳修正後的 (x, y, yaw)

        為什麼需要：這台車的 IMU 有陀螺儀零偏 (見 _amcl_refresh_tick)，
        車子靜置時 odom 的 yaw 一直漂，於是我們存進 nav_state.json 的「上次位姿」
        本身就已經是漂掉的值。拿它去 seed AMCL，等於一開機就給錯的起點 ——
        實測開機後位姿偏 38 度，雷射與地圖吻合度只有 28%，
        規劃器把「車頭前方一公尺」算到牆裡，導航完全不能用。

        AMCL 自己不一定救得回來：粒子雲整團在錯的地方，只靠重新加權跳不出去。
        所以在餵給 AMCL 之前，先用雷射跟地圖直接對一次 —— 這其實就是手動做一次
        scan matching。實測可以從 28% 的吻合度找回 99.8%。

        搜尋策略是粗掃再細掃，避免在 Pi 4 上算太久：
        先用大步距掃過整個範圍，再在最佳解附近用小步距收斂。
        """
        grid = self._last_map
        scan = self._last_scan
        if grid is None or scan is None or pose is None:
            return pose

        info = grid.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        w, h = info.width, info.height

        occupied = set()
        for gy in range(h):
            row = gy * w
            for gx in range(w):
                if grid.data[row + gx] >= 65:
                    occupied.add((gx, gy))
        if not occupied:
            return pose

        # 雷射點取樣：每 3 點取 1 個就夠對齊用，全取在 Pi 上太慢
        pts = []
        for i in range(0, len(scan.ranges), 3):
            r = scan.ranges[i]
            if r <= 0.0 or math.isinf(r) or math.isnan(r) or r > 8.0:
                continue
            a = scan.angle_min + i * scan.angle_increment
            pts.append((r * math.cos(a), r * math.sin(a)))
        if len(pts) < 20:
            return pose

        # 雷射相對 base_footprint 的位移 (robot_model.yaml: senior_akm base_to_laser)
        lx_off, ly_off = 0.08874, 0.00067

        def score(bx, by, byaw):
            c, sn = math.cos(byaw), math.sin(byaw)
            lx = bx + lx_off * c - ly_off * sn
            ly = by + lx_off * sn + ly_off * c
            hit = 0
            for dx, dy in pts:
                wx = lx + dx * c - dy * sn
                wy = ly + dx * sn + dy * c
                gx = int((wx - ox) / res)
                gy = int((wy - oy) / res)
                if (gx, gy) in occupied or (gx + 1, gy) in occupied or (gx - 1, gy) in occupied \
                        or (gx, gy + 1) in occupied or (gx, gy - 1) in occupied:
                    hit += 1
            return hit / len(pts)

        x0, y0, yaw0 = pose
        best = (score(x0, y0, yaw0), x0, y0, yaw0)
        base_score = best[0]

        def sweep(cx, cy, cyaw, yaw_span, yaw_step, xy_span, xy_step):
            nonlocal best
            yaw_n = max(1, int(yaw_span / yaw_step))
            xy_n = max(0, int(xy_span / xy_step))
            for iy in range(-yaw_n, yaw_n + 1):
                yy = cyaw + math.radians(iy * yaw_step)
                for ix in range(-xy_n, xy_n + 1):
                    for jy in range(-xy_n, xy_n + 1):
                        sc = score(cx + ix * xy_step, cy + jy * xy_step, yy)
                        if sc > best[0]:
                            best = (sc, cx + ix * xy_step, cy + jy * xy_step, yy)

        # 粗掃：角度大範圍、位置粗步距
        sweep(x0, y0, yaw0, self.align_search_yaw_deg, 4.0, self.align_search_xy_m, 0.15)
        # 細掃：在粗掃最佳解附近收斂
        sweep(best[1], best[2], best[3], 6.0, 1.0, 0.10, 0.05)

        if best[0] < base_score + 0.05:
            self.get_logger().info(f"位姿對齊：維持原值 (吻合度 {base_score * 100:.0f}%)")
            return pose

        # 角度修正量的合理性檢查。
        #
        # 長直走廊前後看起來幾乎一樣，scan matching 有嚴重的 180 度對稱歧義：
        # 把車頭轉半圈，兩側牆的雷射點照樣落在牆上，吻合度一樣很高。
        # 實測就發生過「對齊到 yaw=166.5 度」——真實方向是 -15.6 度，
        # 整整反了一圈，於是「前方一公尺」變成後方，規劃器回 no valid path。
        #
        # 車子是靜止的，物理上不可能真的轉那麼多。修正量過大只有兩種可能：
        # 對稱誤匹配，或是定位已經完全跑掉。兩種都不該靠這裡硬修 ——
        # 前者會修錯方向，後者該走全域定位。所以拒絕並保留原值。
        delta = math.degrees(math.atan2(math.sin(best[3] - yaw0), math.cos(best[3] - yaw0)))
        if abs(delta) > self.align_max_correction_deg:
            self.get_logger().warn(
                f"位姿對齊：修正量 {delta:+.1f}° 超過上限 "
                f"{self.align_max_correction_deg:.0f}°，判定為走廊前後對稱造成的誤匹配，"
                "維持原值。若車子確實被搬動過，請執行全域定位。"
            )
            return pose

        self.get_logger().info(
            f"位姿對齊：吻合度 {base_score * 100:.0f}% -> {best[0] * 100:.0f}%，"
            f"角度修正 {delta:+.1f}°"
        )
        return best[1], best[2], best[3]

    def _amcl_refresh_tick(self) -> None:
        """定位模式下定期逼 AMCL 就地重新定位

        為什麼非做不可 —— 這台車的 IMU 陀螺儀有零偏：

            車子完全靜止時實測 z 軸角速度 +0.00132 rad/s = +4.54 度/分鐘，
            odom_combined -> base_footprint 的 yaw 就以同樣速率持續漂。

        而 AMCL 只有在移動超過 update_min_d / update_min_a 之後才會更新，
        車子靜止時它完全不動作，map -> odom 這段修正量就一直凍結在舊值。
        於是 map -> base = (凍結的 map->odom) x (一直在漂的 odom->base)
        整個跟著 odom 漂 —— 實測靜止 60 秒偏掉 4.74 度，靜止五分鐘就是 23 度。

        後果不是「位置顯示不準」而已：導航目標常以車頭方向推算，
        角度偏 38 度時「前方一公尺」會指到牆裡，規劃器直接回 no valid path。
        實測就是這樣連續失敗的。

        request_nomotion_update 會讓 AMCL 用當前雷射重跑一次觀測更新，
        把粒子雲拉回正確位姿，等於持續抵消陀螺儀零偏。
        成本很低 (3 秒一次)，但沒有它這台車靜置後就無法導航。
        """
        if self.mode != MODE_LOCALIZATION:
            return
        if not self.nomotion_update_client.service_is_ready():
            return
        try:
            wait_for_future(
                self.nomotion_update_client.call_async(EmptySrv.Request()), timeout_sec=2.0
            )
        except Exception:  # noqa: BLE001
            pass  # 偶爾逾時無所謂，下一輪會再試

    def _persist_pose_tick(self) -> None:
        """定位模式下定期記錄位姿，供下次開機接續"""
        if self.mode != MODE_LOCALIZATION or not self.current_map_id:
            return
        pose = self._current_robot_pose()
        if pose is not None:
            self._save_map_state(self.current_map_id, pose)

    def _map_info_callback(self, msg: OccupancyGrid) -> None:
        self._last_map_info = msg.info
        self._last_map = msg

    def _scan_callback(self, msg: LaserScan) -> None:
        self._last_scan = msg

    def _exploration_complete_callback(self, _msg: Empty) -> None:
        """frontier_explorer 的完成事件

        這個話題是 transient_local，訂閱當下就會補送上一輪的舊事件，
        所以一定要檢查 _exploration_active，不然一開機就會被誤判成完成。
        """
        if not self._exploration_active:
            self.get_logger().debug("忽略探索完成事件 (目前沒有在探索)")
            return
        self._exploration_done_epoch = self._exploration_epoch

    # ==================================================================
    # 其他服務
    # ==================================================================
    def _start_exploration_callback(self, _request, response):
        """建圖進行中，手動把後續交給自動探索"""
        if self.mode != MODE_MAPPING or not self._exploration_active:
            response.success = False
            response.message = "目前不在建圖中，無法啟動探索"
            return response

        if not self.control_exploration_client.wait_for_service(timeout_sec=10.0):
            response.success = False
            response.message = "/control_exploration 服務不存在 (frontier_explorer 沒啟動?)"
            return response

        req = ControlExploration.Request()
        req.action = ControlExploration.Request.ACTION_START
        try:
            res = wait_for_future(self.control_exploration_client.call_async(req), timeout_sec=10.0)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"啟動探索失敗: {exc}"
            return response

        if not res.accepted:
            response.success = False
            response.message = f"探索服務拒絕啟動: {res.message}"
            return response

        # 讓 _wait_for_mapping_done 開始認 /exploration_complete
        self.use_exploration = True
        self.get_logger().info("✓ 已手動啟動自動探索")
        response.success = True
        response.message = "自動探索已開始，完成後會自動存檔"
        return response

    def _finish_map_callback(self, _request, response):
        if self.mode != MODE_MAPPING or not self._exploration_active:
            response.success = False
            response.message = "目前不在建圖中，/finish_map 無效"
            return response

        self._finish_requested = True
        response.success = True
        response.message = "已要求結束建圖，正在存檔"
        return response

    def _ensure_localization_callback(self, _request, response):
        """給 waypoint_service_cc 用：全域定位之前要確保在 AMCL 模式"""
        if self.mode == MODE_LOCALIZATION:
            response.success = True
            response.message = "已經在定位模式"
            return response

        target = self._resolve_startup_map()
        if target is None:
            response.success = False
            response.message = "沒有可用的地圖，無法切換到定位模式"
            return response

        if self._enter_localization_mode(target):
            response.success = True
            response.message = f"已切換到定位模式 (地圖 {target})"
        else:
            response.success = False
            response.message = "切換到定位模式失敗"
        return response

    def _align_pose_callback(self, _request, response):
        """用雷射與地圖重新對齊目前位姿，並把結果餵回 AMCL

        給 navigation_action_cc 在每次導航開始前呼叫。

        為什麼導航前一定要對：走廊的角度可觀測性很差 (車子轉個十幾度，
        兩側牆的雷射點仍然落在牆上，AMCL 分不出來)，而 IMU 零偏又持續
        把 yaw 往一個方向推。車子靜置越久偏得越多，一旦帶著錯誤的角度
        起步，「前方一公尺」就會被算到牆裡，導航不是失敗就是走歪。
        """
        if self.mode != MODE_LOCALIZATION:
            response.success = False
            response.message = "不在定位模式，無法對齊"
            return response

        current = self._current_robot_pose()
        if current is None:
            response.success = False
            response.message = "讀不到目前位姿"
            return response

        aligned = self._align_pose_with_scan(current)
        if aligned is None or aligned == current:
            response.success = True
            response.message = "位姿無需修正"
            return response

        with self._mode_lock:
            self._seed_amcl_pose(aligned[0], aligned[1], aligned[2], tight=True)

        response.success = True
        response.message = (
            f"已對齊到 ({aligned[0]:.2f}, {aligned[1]:.2f}, "
            f"{math.degrees(aligned[2]):.1f}°)"
        )
        return response

    def _get_nav_mode_callback(self, _request, response):
        response.success = self.mode != MODE_UNKNOWN
        response.message = self.mode
        return response

    def _list_maps_callback(self, _request, response):
        try:
            maps_info = []
            with self._db_lock:
                for map_id, meta in self.maps_db.items():
                    info = MapInfo()
                    info.map_id = map_id
                    info.map_name = meta["name"]
                    maps_info.append(info)
            response.maps_info = maps_info
            response.success = True
            response.message = f"成功查詢到 {len(maps_info)} 個地圖"
        except Exception as exc:  # noqa: BLE001
            response.maps_info = []
            response.success = False
            response.message = "系統出現異常，查詢地圖列表失敗"
            self.get_logger().error(f"查詢地圖列表錯誤: {exc}")
        return response

    def _delete_map_callback(self, request, response):
        """刪除一張地圖（資料庫紀錄 + 磁碟上的 .yaml/.pgm）

        建圖失敗或建壞的地圖以前只能手動改 json 再刪檔案，很容易改壞。

        兩條硬性拒絕（不是提醒，是直接不做）：

        1. **create_map 工作階段進行中不可刪任何地圖**。
           中途動資料庫會讓 create_map 結束存檔時的目標對不上。

           注意這裡看的是 `_exploration_active`（有沒有 create_map 在跑），
           **不是** `mode == MODE_MAPPING`。一開始寫成後者，結果是
           「導航堆疊只要跑在建圖模式就不准刪任何地圖」——
           但手動建圖時本來就一直處於建圖模式，那正是操作者最想
           清掉先前建壞的圖的時候。兩者是不同的事：
           「模式」是 slam 有沒有 active，「工作階段」才是有沒有在寫某張圖。

        2. **不可刪除目前正在使用的地圖**。
           map_server 已經把它載入記憶體，刪掉磁碟檔案不會讓它消失，
           但下一次切換或重啟就會變成「資料庫說有、檔案卻不見」的破碎狀態，
           而 _pick_startup_map 會挑到一個載不起來的 map_id。
           要刪的話請先切換到別張地圖。
        """
        try:
            if not request.map_id:
                response.success = False
                response.message = "必須提供 map_id"
                return response

            if self._exploration_active:
                response.success = False
                response.message = "建圖工作階段進行中，無法刪除地圖；請先完成存檔或取消建圖"
                self.get_logger().warn(f"拒絕在 create_map 進行中刪除地圖 {request.map_id}")
                return response

            if request.map_id == self.current_map_id:
                response.success = False
                response.message = "不能刪除目前使用中的地圖，請先切換到其他地圖"
                return response

            with self._db_lock:
                meta = self.maps_db.get(request.map_id)
                if meta is None:
                    response.success = False
                    response.message = f"地圖 {request.map_id} 不存在"
                    return response
                map_name = meta.get("name", request.map_id)
                self.maps_db.pop(request.map_id, None)
                self._save_maps_db()

            # 檔案刪不掉不算失敗：資料庫紀錄已經移除，地圖不會再被選到。
            # 殘留檔案只是佔空間，比「資料庫還在但檔案沒了」安全得多，
            # 所以順序刻意是「先移除紀錄、再刪檔案」。
            removed, failed = [], []
            for suffix in (".yaml", ".pgm", ".png"):
                path = self.map_data_dir / f"{request.map_id}{suffix}"
                try:
                    if path.exists():
                        path.unlink()
                        removed.append(path.name)
                except OSError as exc:
                    failed.append(f"{path.name}({exc.strerror})")

            msg = f"已刪除地圖「{map_name}」"
            if removed:
                msg += f"，移除檔案 {len(removed)} 個"
            if failed:
                msg += f"；但有檔案刪不掉：{', '.join(failed)}"
            self.get_logger().warn(f"{msg} (id={request.map_id})")
            response.success = True
            response.message = msg
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = "系統出現異常，刪除地圖失敗"
            self.get_logger().error(f"刪除地圖錯誤: {exc}")
        return response

    def _switch_map_callback(self, request, response):
        try:
            if not request.map_id:
                response.success = False
                response.message = "必須提供 map_id 以切換地圖"
                return response

            # 建圖進行中不准切地圖。
            #
            # 實測 (2026-07-29)：HMI 重啟後會還原「上次選的地圖」而呼叫 switch_map，
            # 剛好撞上正在啟動的 create_map，於是：
            #     ✓ 已進入建圖模式
            #     切換到定位模式 (地圖 map_02368f43c62a)      <- 41 毫秒後就被切走
            #     ✓ 自動探索已啟動                            <- 對著已經不存在的建圖模式啟動
            #     沒有已知位姿，改用地圖中心 (3.30, 3.46)
            # 結果 slam_toolbox 被關掉、amcl 被拉起來套用舊地圖，
            # 機器人位姿被設到舊地圖的中心 —— 那個位置在新 costmap 界外，
            # planner 直接噴 "Robot is out of bounds of the costmap"，
            # 整個建圖工作階段報廢。
            #
            # 建圖是獨佔的操作，要換地圖請先 /finish_map 或取消 create_map。
            if self._exploration_active or self.mode == MODE_MAPPING:
                response.success = False
                response.message = "建圖進行中，無法切換地圖；請先完成或取消建圖"
                self.get_logger().warn(
                    f"拒絕在建圖進行中切換地圖 (要求切到 {request.map_id})"
                )
                return response

            with self._db_lock:
                known = request.map_id in self.maps_db
            if not known or not self._map_file(request.map_id).exists():
                response.success = False
                response.message = f"地圖 {request.map_id} 不存在"
                return response

            # 切換地圖之後機器人的位置是未知的，初始位姿回到原點，
            # 需要的話再由 HMI 觸發全域定位。
            if self._enter_localization_mode(request.map_id):
                response.success = True
                response.message = "地圖切換成功"
                self.get_logger().info(f"地圖切換成功: {request.map_id}")
            else:
                response.success = False
                response.message = "地圖切換失敗"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = "系統出現異常，地圖切換失敗"
            self.get_logger().error(f"地圖切換錯誤: {exc}")
        return response

    # ==================================================================
    # 資料庫
    # ==================================================================
    def _save_maps_db(self) -> None:
        try:
            self.db_file.parent.mkdir(parents=True, exist_ok=True)
            with open(self.db_file, "w", encoding="utf-8") as fp:
                json.dump({"maps_db": self.maps_db}, fp, ensure_ascii=False, indent=4)
            self.get_logger().info(f"已儲存地圖資料庫，共 {len(self.maps_db)} 張")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"儲存地圖資料庫失敗: {exc}")

    def _load_maps_db(self) -> None:
        try:
            if self.db_file.exists():
                with open(self.db_file, "r", encoding="utf-8") as fp:
                    data = json.load(fp)
                with self._db_lock:
                    self.maps_db = data.get("maps_db", {})
                self.get_logger().info(f"已載入地圖資料庫，共 {len(self.maps_db)} 張")
            else:
                self.get_logger().info("找不到地圖資料庫，使用空資料庫")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"載入地圖資料庫失敗: {exc}")

    def _save_map_state(self, map_id: str, pose=None) -> None:
        """把目前地圖與位姿寫進 nav_state.json

        pose 一起存是必要的，不是加值功能。只存 map_id 的話，重新啟動導航時
        AMCL 的初始位姿只能落在原點 (0, 0)，而地圖的座標原點是建圖起點決定的 ——
        如果建圖時車子不是從地圖原點出發，(0,0) 很可能根本不在地圖範圍內，
        規劃器會直接回 START_OUTSIDE_MAP (error_code 203)：

            GridBased plugin failed to plan from (0.00, 0.00) ... 起點在地圖外

        實測就踩到這個：地圖範圍 x 3.69~16.59，車子沒被搬動過，
        卻因為重啟而被定位到 (0,0)，整個導航直接不能用。
        """
        data = {"last_active_map": map_id}
        if pose is None:
            pose = self._current_robot_pose()
        if pose is not None:
            data["last_pose"] = {"x": pose[0], "y": pose[1], "yaw": pose[2]}
        try:
            self.base_dir.mkdir(parents=True, exist_ok=True)
            with open(self.state_file, "w", encoding="utf-8") as fp:
                json.dump(data, fp, ensure_ascii=False, indent=4)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"儲存目前地圖狀態失敗: {exc}")

    def _load_map_state(self) -> Optional[str]:
        try:
            if not self.state_file.exists():
                return None
            with open(self.state_file, "r", encoding="utf-8") as fp:
                return json.load(fp).get("last_active_map")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"載入目前地圖狀態失敗: {exc}")
            return None

    def _load_last_pose(self):
        """讀上次記錄的位姿，回 (x, y, yaw)；沒有就回 None"""
        try:
            if not self.state_file.exists():
                return None
            with open(self.state_file, "r", encoding="utf-8") as fp:
                p = json.load(fp).get("last_pose")
            if not p:
                return None
            return float(p["x"]), float(p["y"]), float(p["yaw"])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"讀取上次位姿失敗: {exc}")
            return None

    def _map_center_pose(self):
        """地圖中心，當成最後的退路

        連上次位姿都沒有時 (例如手動刪過 nav_state.json)，用地圖中心也遠比
        用原點安全 —— 至少保證起點在地圖範圍內，AMCL 有機會靠全域定位收斂，
        而不是一開始就 START_OUTSIDE_MAP 直接死掉。
        """
        info = self._last_map_info
        if info is None:
            return None
        cx = info.origin.position.x + info.width * info.resolution / 2.0
        cy = info.origin.position.y + info.height * info.resolution / 2.0
        return cx, cy, 0.0


def main(args=None):
    rclpy.init(args=args)
    node = MapServiceCcNode()
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
