#!/usr/bin/env python3
"""人機介面伺服器節點。

這個檔案只負責**節點本身**：ROS 的訂閱／發布／服務／動作、依用戶端在場
與否開關高頻訂閱、遙控看門狗執行緒的啟動、節點開關、把 FastAPI 跑起來，
以及把各業務模組的元件組起來、把 ROS callback 轉發給它們。

真正的業務邏輯都在 core/ 與 modules/ 底下，各自成為不依賴 ROS 節點、
可以單獨讀完也可以單獨測試的元件：

    core/auth.py              管理者登入（權杖表與帳密）
    core/constants.py         跨模組共用的常數（節點分組、動作逾時、型別對照）
    core/state.py             HmiState —— ROS 執行緒寫、asyncio 讀的共享狀態
    core/schemas/common.py    HTTP 請求格式 (pydantic)

    modules/system/           系統與網路資訊、節點健康檢查、資源監看
    modules/hardware/         周邊硬體偵測
    modules/imaging/          影像串流、地圖渲染、位姿追蹤、幾何換算
    modules/stats/            到訪統計（唯讀 sqlite）
    modules/navigation/       地圖切換/建圖、地點/導航、教導路徑、遙控與急停
    modules/jobs/             長時間 ROS 服務/動作的共用呼叫機制與 job 登記簿
    modules/users/            人臉註冊（背景採樣執行緒）
    modules/chat/             LLM 對話串流追蹤
    modules/session/          管理者登入路由
    modules/system_control/   子系統開關與測試情境
    modules/realtime/         狀態快照與 WebSocket 推播

    api/main.py                 build_app(node)：組裝 FastAPI，掛上所有
                                 modules/<topic>/router.py
"""

import time
import uvicorn
import threading
from pathlib import Path
from fastapi import FastAPI
from typing import Any, Dict, List

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, Empty, Float32, String
from std_srvs.srv import Trigger
from nav_msgs.msg import Path as RosPath

from smartnav_msgs.msg import RegistrationProgress, UserIdentity
from smartnav_msgs.srv import (
    CreateWaypoint,
    DeleteMap,
    DeleteUser,
    DeleteTaughtPath,
    DeleteWaypoint,
    ListMaps,
    ListUsers,
    ListTaughtPaths,
    ListWaypoints,
    PlanTaughtPath,
    RecordPath,
    RegisterFace,
    RegisterFacePhoto,
    SwitchMap,
    UpdateUser,
)
from smartnav_msgs.action import CreateMap, FollowTaughtPath, GlobalLocalization, Navigate

from .api.main import build_app
from .core.auth import AdminAuth, resolve_password
from .core.constants import USER_TYPE_NAMES
from .core.state import HmiState
from .modules.chat.tracker import ChatTracker
from .modules.hardware.hardware import HardwareProbe
from .modules.imaging.map_render import MapRenderer
from .modules.imaging.pose import PoseTracker
from .modules.imaging.video import VideoStreamer
from .modules.jobs.runner import JobRunner
from .modules.navigation.teleop import TeleopController
from .modules.system.health import NodeHealthChecker
from .modules.system.netinfo import detect_lan_ips
from .modules.system_control.manager import SystemControlManager
from .modules.users.registration import RegistrationRunner


class HmiServerNode(Node):
    """HMI 伺服器節點"""

    # 送到前端畫的路徑最多幾點。地圖畫布只有幾百像素寬，超過這個數的細節
    # 眼睛看不出來，卻要每次狀態推播都重傳一遍。
    MAX_PATH_POINTS = 120

    def __init__(self):
        super().__init__("hmi_server_node")

        # ── 參數 ──────────────────────────────────────────
        self.declare_parameter("host", "0.0.0.0", ParameterDescriptor(description="HTTP 綁定位址"))
        self.declare_parameter("port", 8080, ParameterDescriptor(description="HTTP 連接埠"))
        self.declare_parameter("admin_username", "admin", ParameterDescriptor(description="管理者帳號"))
        # 預設留空，密碼不寫死在原始碼裡 —— 這個 repo 是公開的，寫死等於公開密碼。
        # 取得順序：launch 參數 > 環境變數 SMARTNAV_ADMIN_PASSWORD > 隨機生成。
        # 三者都沒有時會隨機生成一組並印在啟動 log，所以不會有「登不進去」的情況。
        self.declare_parameter(
            "admin_password",
            "",
            ParameterDescriptor(
                description="管理者密碼（留空則讀環境變數 SMARTNAV_ADMIN_PASSWORD，仍為空則隨機生成並印在啟動 log）"
            ),
        )
        self.declare_parameter("admin_session_hours", 12.0, ParameterDescriptor(description="管理者登入有效時數"))
        # 節點跑在哪台機器是靠掃本機 /proc 判定的，遇到判斷不準的節點可以在這裡釘死
        self.declare_parameter("pi_nodes", [""], ParameterDescriptor(description="強制歸類為本機（Pi）的節點名稱"))
        self.declare_parameter("edge_nodes", [""], ParameterDescriptor(description="強制歸類為邊緣裝置的節點名稱"))
        self.declare_parameter(
            "image_capture_topic", "/camera/color/image_raw/compressed", ParameterDescriptor(description="相機影像話題")
        )
        self.declare_parameter(
            "user_identity_topic", "user_identity", ParameterDescriptor(description="身份辨識結果話題")
        )
        self.declare_parameter("map_topic", "map", ParameterDescriptor(description="佔據柵格地圖話題"))
        self.declare_parameter("map_frame", "map", ParameterDescriptor(description="地圖座標系名稱"))
        self.declare_parameter(
            "robot_frame",
            "base_link",
            ParameterDescriptor(description="機器人本體座標系。WHEELTEC 鏈可能要設 base_footprint"),
        )
        # Pi4 的瓶頸是 CPU（實測全節點 load average 19.36，4 核超載 4.8 倍）。
        # 純迎賓展示不需要地圖，關掉可省下 /map 訂閱、PNG 渲染與 TF 監聽三份負擔。
        self.declare_parameter(
            "enable_map",
            True,
            ParameterDescriptor(description="是否啟用地圖功能（訂閱 /map、渲染 PNG、監聽 TF）"),
        )
        self.declare_parameter(
            "map_render_interval",
            2.0,
            ParameterDescriptor(description="地圖最快多久重繪一次（秒）。建圖時地圖每秒更新多次，重繪不是免費的"),
        )
        self.declare_parameter(
            "pose_update_rate",
            2.0,
            ParameterDescriptor(description="更新機器人位姿的頻率（Hz）。調高只會讓箭頭更順，代價是 CPU"),
        )
        # 位姿與地圖渲染都只有「地圖分頁開著」時才有人看得到。前端在地圖分頁時
        # 每隔幾秒打一次 /api/map/watch，這個值是那個心跳的有效期限：
        # 超過就把 amcl_pose／TF 訂閱整個拆掉，讓節點回到零高頻訂閱的狀態。
        # 設得比前端心跳間隔（5 秒）寬鬆，避免單次封包遺失就閃斷。
        self.declare_parameter(
            "pose_watch_ttl",
            12.0,
            ParameterDescriptor(description="沒有人看地圖多久後就停止訂閱位姿來源（秒）"),
        )
        self.declare_parameter("video_fps", 6.0, ParameterDescriptor(description="MJPEG 串流上限幀率"))
        self.declare_parameter("video_quality", 55, ParameterDescriptor(description="JPEG 壓縮品質 1-100"))
        self.declare_parameter("video_width", 640, ParameterDescriptor(description="MJPEG 輸出寬度，0 表示不縮放"))
        self.declare_parameter("service_timeout", 8.0, ParameterDescriptor(description="呼叫 ROS 服務的等待秒數"))
        # ── 用戶端在場偵測 ──────────────────────────────
        # 「最後一次 HTTP 請求」之後還要把節點當成「有人在看」多久。
        # WebSocket 活著時這個值用不到（活連線本身就是在場證明），它是給
        # 「ws 剛斷、正在重連」那個空窗用的，所以要比前端的重連退避上限
        # （index.html: Math.min(8000, ...) = 8 秒）寬。
        self.declare_parameter(
            "client_idle_ttl",
            25.0,
            ParameterDescriptor(description="沒有 WebSocket 時，最後一次 HTTP 請求後仍視為有人在看的秒數"),
        )
        # MJPEG 串流結束後仍保留影像訂閱的秒數。純粹是防抖：切分頁、重新整理
        # 都會讓串流斷一下再接回來，沒有這個緩衝就會一直建立／銷毀訂閱，
        # 每次都要重跑一輪 DDS 探索，反而更貴。
        self.declare_parameter(
            "video_idle_ttl",
            10.0,
            ParameterDescriptor(description="沒有人看影像多久後就取消影像訂閱（秒）"),
        )
        # WebSocket 存活探測。★ 用的是 **協定層** 的 ping/pong，不是自己寫的
        # 心跳：瀏覽器的網路層會自動回 pong，就算分頁在背景、JS 計時器被節流
        # 甚至整個暫停也照回不誤。所以「平板放著沒操作」不會被誤踢，
        # 而「平板走出 WiFi 範圍」會在 interval+timeout 內被判定並關閉。
        self.declare_parameter(
            "ws_ping_interval",
            25.0,
            ParameterDescriptor(description="WebSocket 協定層 ping 間隔（秒），<=0 關閉"),
        )
        self.declare_parameter(
            "ws_ping_timeout",
            20.0,
            ParameterDescriptor(description="WebSocket pong 逾時（秒），逾時即關閉該連線"),
        )
        self.declare_parameter(
            "image_transport",
            "auto",
            ParameterDescriptor(
                description=(
                    "影像傳輸方式：auto（依話題是否以 /compressed 結尾判斷）、raw（sensor_msgs/Image）、"
                    "compressed（sensor_msgs/CompressedImage）。兩者是不同的訊息型別，選錯就完全收不到。"
                )
            ),
        )

        self.host = self.get_parameter("host").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        # 登入整組交給 AdminAuth：權杖表、到期、帳密比對都在那裡，
        # 這個節點只負責把參數餵給它（見 core/auth.py 對「這是操作閘門不是資安機制」的說明）
        admin_username = self.get_parameter("admin_username").get_parameter_value().string_value
        self.admin_session_sec = self.get_parameter("admin_session_hours").get_parameter_value().double_value * 3600.0
        self.auth = AdminAuth(
            username=admin_username,
            password=resolve_password(
                self.get_parameter("admin_password").get_parameter_value().string_value,
                admin_username,
            ),
            session_sec=self.admin_session_sec,
        )
        self.pi_nodes = {n for n in self.get_parameter("pi_nodes").get_parameter_value().string_array_value if n}
        self.edge_nodes = {n for n in self.get_parameter("edge_nodes").get_parameter_value().string_array_value if n}
        image_topic = self.get_parameter("image_capture_topic").get_parameter_value().string_value
        identity_topic = self.get_parameter("user_identity_topic").get_parameter_value().string_value
        map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self.robot_frame = self.get_parameter("robot_frame").get_parameter_value().string_value
        self.enable_map = self.get_parameter("enable_map").get_parameter_value().bool_value
        self.map_render_interval = self.get_parameter("map_render_interval").get_parameter_value().double_value
        pose_update_rate = self.get_parameter("pose_update_rate").get_parameter_value().double_value
        self.pose_watch_ttl = self.get_parameter("pose_watch_ttl").get_parameter_value().double_value
        self.video_fps = self.get_parameter("video_fps").get_parameter_value().double_value
        self.video_quality = self.get_parameter("video_quality").get_parameter_value().integer_value
        self.video_width = self.get_parameter("video_width").get_parameter_value().integer_value
        self.service_timeout = self.get_parameter("service_timeout").get_parameter_value().double_value
        self.client_idle_ttl = self.get_parameter("client_idle_ttl").get_parameter_value().double_value
        self.video_idle_ttl = self.get_parameter("video_idle_ttl").get_parameter_value().double_value
        self.ws_ping_interval = self.get_parameter("ws_ping_interval").get_parameter_value().double_value
        self.ws_ping_timeout = self.get_parameter("ws_ping_timeout").get_parameter_value().double_value

        transport = self.get_parameter("image_transport").get_parameter_value().string_value.strip().lower()
        if transport == "auto":
            self.image_compressed = image_topic.endswith("/compressed")
        else:
            self.image_compressed = transport == "compressed"
        # 影像訂閱改成按需建立（見 _client_tick），所以話題名要留著
        self.image_topic = image_topic

        # ── 狀態 ──────────────────────────────────────────
        self.state = HmiState()
        # 電壓／充電電流各自節流到 1 Hz（見對應 callback）
        self._last_voltage_at = 0.0
        self._last_charge_cur_at = 0.0

        cb = ReentrantCallbackGroup()
        self._cb = cb  # nav2 的 get_state 客戶端要用到才建，屆時沿用同一個群組

        # 影像走 BEST_EFFORT——相機是 sensor data，掉幀比塞住好
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        # 影像 QoS 留著給按需訂閱用（odom 車速也是同一套：depth=1、掉了就掉了）
        self._sensor_qos = sensor_qos
        # 地圖與 current_map 都是 latched（TRANSIENT_LOCAL），晚加入也收得到最後一筆
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── 業務模組元件 ──────────────────────────────────
        self.service_clients: Dict[str, Any] = {}
        self.action_clients: Dict[str, ActionClient] = {}

        self.jobs = JobRunner(
            self,
            self.state,
            self.service_clients,
            self.action_clients,
            self.service_timeout,
            cancel_callback_group=cb,
        )
        self.registration = RegistrationRunner(self, self.state, self.jobs)
        self.chat = ChatTracker(self, self.state)
        # 硬體偵測連同它的 3 秒快取都在 HardwareProbe 裡；
        # 這裡只把「怎麼拿到 system 快照」注入進去，它本身不認識 ROS。
        self.hardware = HardwareProbe(lambda: self.state.snapshot().get("system", {}))
        self.health = NodeHealthChecker(self, self.state, self.pi_nodes, self.edge_nodes, cb)
        self.system_control = SystemControlManager(self, self.jobs)

        self.video = VideoStreamer(
            self,
            self.state,
            image_topic=image_topic,
            image_compressed=self.image_compressed,
            sensor_qos=sensor_qos,
            video_fps=self.video_fps,
            video_quality=self.video_quality,
            video_width=self.video_width,
            video_idle_ttl=self.video_idle_ttl,
        )
        self.map_render = MapRenderer(self, self.state, self.map_render_interval)
        self.pose_tracker = PoseTracker(
            self,
            self.state,
            self.map_render,
            map_frame=self.map_frame,
            robot_frame=self.robot_frame,
            pose_watch_ttl=self.pose_watch_ttl,
        )

        # ── 用戶端在場追蹤 ──────────────────────────────────
        self._client_lock = threading.Lock()
        self._ws_clients = 0  # 活著的 WebSocket 連線數
        self._last_http_at = 0.0  # 最後一次 HTTP 請求（monotonic）
        self._clients_seen = False  # 上一輪的判定，只在變化時記 log
        self._odom_sub = None  # 按需建立，見 _client_tick

        # ── 訂閱 ──────────────────────────────────────────
        #   「有 MJPEG 串流在跑時才訂」
        self.create_subscription(UserIdentity, identity_topic, self._identity_cb, 10, callback_group=cb)
        self.create_subscription(
            RegistrationProgress,
            "registration_progress",
            self.registration.on_progress,
            10,
            callback_group=cb,
        )
        self.create_subscription(String, "user_text", self.chat.on_user_text, 10, callback_group=cb)
        self.create_subscription(String, "partial_text", self.chat.on_partial_text, 10, callback_group=cb)
        self.create_subscription(String, "llm_response", self.chat.on_llm_response, 10, callback_group=cb)
        self.create_subscription(String, "llm_stream", self.chat.on_llm_stream, 10, callback_group=cb)
        self.create_subscription(Empty, "llm_stream_reset", self.chat.on_llm_stream_reset, 10, callback_group=cb)
        # 模型名稱是 latched，HMI 比 LLM 晚啟動也收得到
        self.create_subscription(String, "llm_model", self.chat.on_llm_model, latched_qos, callback_group=cb)
        self.create_subscription(String, "speech_text", self.chat.on_speech_text, 10, callback_group=cb)
        self.create_subscription(Bool, "playback_status", self.chat.on_playback, 10, callback_group=cb)

        self.playback_pub = self.create_publisher(Bool, "playback_status", 10)
        if self.enable_map:
            self.create_subscription(OccupancyGrid, map_topic, self.map_render.on_grid, latched_qos, callback_group=cb)
        # map_service_node 發布的是 std_msgs/String（內容為 map_id），不是 MapInfo
        self.create_subscription(String, "current_map", self._current_map_cb, latched_qos, callback_group=cb)
        # 電池電壓由 WHEELTEC 底盤發布，沒跑底盤時單純不會有資料。
        # 用 depth=1 + BEST_EFFORT：電壓只要「最新值」，堆 10 則舊值沒有意義，
        # 而且 HMI 忙起來時 RELIABLE 會逼 DDS 重送我們根本不看的舊資料。
        # （BEST_EFFORT 訂閱者相容於 RELIABLE 發布者，底盤那端不用改。）
        self.create_subscription(Float32, "PowerVoltage", self._voltage_cb, sensor_qos, callback_group=cb)
        # 充電狀態也由底盤發布，跟電壓同一條線：充電中電壓會被充電器拉高，
        # 只看電壓會把「正在充電」誤讀成「電池很飽」，所以兩個要一起顯示。
        self.create_subscription(Bool, "robot_charging_flag", self._charging_cb, sensor_qos, callback_group=cb)
        self.create_subscription(
            Float32, "robot_charging_current", self._charge_current_cb, sensor_qos, callback_group=cb
        )
        self.create_timer(5.0, self.health.resource_tick, callback_group=cb)

        # ── 規劃路徑（使用者要求在 HMI 上看得到）──
        #   taught_path  path_teach_cc 發的：規劃完成與重播開始時（latched）
        #   plan         nav2 planner_server 的全域路徑（走 MPPI 那條時才有）
        # 兩條都畫，用 source 欄位區分顏色。latched 讓晚開的平板也拿得到。
        self.create_subscription(
            RosPath, "taught_path", lambda m: self._nav_path_cb(m, "taught"), latched_qos, callback_group=cb
        )
        self.create_subscription(RosPath, "plan", lambda m: self._nav_path_cb(m, "nav2"), 10, callback_group=cb)

        # ── 發布 ──────────────────────────────────────────
        # 網頁打字送出的文字進 user_text，與語音辨識、人臉事件走同一條路進 LLM
        self.user_text_pub = self.create_publisher(String, "user_text", 10)
        # 直接讓機器人說話（跳過 LLM），展示與測試 TTS 用
        self.speech_text_pub = self.create_publisher(String, "speech_text", 10)
        # 通知 LLM 一併清掉對話記憶，避免畫面清空了模型卻還記得上一位客戶
        self.clear_conversation_pub = self.create_publisher(Empty, "clear_conversation", 10)

        # ── 遙控建圖 ──────────────────────────────────────
        #
        # 發到 cmd_vel_trimmed 而不是 cmd_vel：這樣指令仍然會經過
        # collision_monitor 才到底盤，遙控時一樣有防撞保護。
        # （鏈路：teleop -> cmd_vel_trimmed -> collision_monitor -> cmd_vel -> 底盤）
        # 導航堆疊沒開時 collision_monitor 不存在，這時要改發 cmd_vel，
        # 所以做成參數。
        self.declare_parameter("teleop_cmd_topic", "cmd_vel_trimmed")
        # 看門狗：手機遙控最危險的情境是「連線斷掉但車子還在跑」——
        # 走出 WiFi 範圍、鎖螢幕、切到別的 App、瀏覽器分頁被系統回收，
        # 都會讓指令突然停止送達。因此不是「收到停止才停」，
        # 而是「超過這個時間沒收到新指令就自動停」。
        # 前端每 200 ms 送一次，0.6 秒的容忍度可以容納兩次丟包。
        self.declare_parameter("teleop_watchdog_sec", 0.6)
        # 雷達節點在 /x10 命名空間底下，不是頂層的 /lslidar_driver_node。
        # 做成參數而不是寫死，換雷達或改命名空間時不用改程式。
        self.declare_parameter("lidar_node_name", "/x10/lslidar_driver_node")
        # 到訪統計頁要讀的資料庫。
        self.declare_parameter("visit_log_path", str(Path.home() / ".smartnav" / "visit_log.db"))
        self.visit_log_path = self.get_parameter("visit_log_path").get_parameter_value().string_value
        teleop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.teleop_pub = self.create_publisher(
            Twist, self.get_parameter("teleop_cmd_topic").get_parameter_value().string_value, teleop_qos
        )
        # 中繼節點不在時的直通路徑，理由見 TeleopController._publish_teleop
        self.teleop_direct_pub = self.create_publisher(Twist, "cmd_vel", teleop_qos)
        self.teleop = TeleopController(
            self,
            self.teleop_pub,
            self.teleop_direct_pub,
            watchdog_sec=float(self.get_parameter("teleop_watchdog_sec").get_parameter_value().integer_value),
            lidar_node_name_param="lidar_node_name",
        )
        self.teleop.start()

        # ── 服務／動作客戶端 ───────────────────────────────
        self.service_clients.update(
            {
                "register_face": self.create_client(RegisterFace, "register_face", callback_group=cb),
                "register_face_photo": self.create_client(RegisterFacePhoto, "register_face_photo", callback_group=cb),
                "list_users": self.create_client(ListUsers, "list_users", callback_group=cb),
                "delete_user": self.create_client(DeleteUser, "delete_user", callback_group=cb),
                "update_user": self.create_client(UpdateUser, "update_user", callback_group=cb),
                "list_maps": self.create_client(ListMaps, "list_maps", callback_group=cb),
                "switch_map": self.create_client(SwitchMap, "switch_map", callback_group=cb),
                # 刪掉建壞的地圖。安全檢查在 map_service_cc 那端做（建圖中、使用中都拒絕）
                "delete_map": self.create_client(DeleteMap, "delete_map", callback_group=cb),
                # 建圖頁面要顯示「現在是建圖還是定位」
                "get_nav_mode": self.create_client(Trigger, "/get_nav_mode", callback_group=cb),
                # 遙控建圖用：沒有 /exploration_complete 事件，要由操作者手動結束
                "finish_map": self.create_client(Trigger, "/finish_map", callback_group=cb),
                "create_waypoint": self.create_client(CreateWaypoint, "create_waypoint", callback_group=cb),
                "list_waypoints": self.create_client(ListWaypoints, "list_waypoints", callback_group=cb),
                # 掃描對齊：不需要移動車子的局部位姿修正（map_service_cc 提供）。
                # 走廊裡優先用它，不要用會繞圈的全域定位——0.9 m 半徑的圓弧
                # 在 0.99 m 的走廊會撞牆。
                "align_pose": self.create_client(Trigger, "/align_pose", callback_group=cb),
            }
        )
        # AMCL 的初始位姿入口（給 POST /api/localize/here 用）。
        # 用 VOLATILE 而不是 TRANSIENT_LOCAL：latched 的初始位姿會在 AMCL
        # 重啟時把它推回一個可能早已過時的位置，那比收不到更難查。
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        self.service_clients["delete_waypoint"] = self.create_client(
            DeleteWaypoint, "delete_waypoint", callback_group=cb
        )
        # 教導-重現路徑
        self.service_clients["record_path"] = self.create_client(RecordPath, "record_path", callback_group=cb)
        self.service_clients["list_taught_paths"] = self.create_client(
            ListTaughtPaths, "list_taught_paths", callback_group=cb
        )
        self.service_clients["delete_taught_path"] = self.create_client(
            DeleteTaughtPath, "delete_taught_path", callback_group=cb
        )
        self.service_clients["plan_taught_path"] = self.create_client(
            PlanTaughtPath, "plan_taught_path", callback_group=cb
        )
        self.action_clients.update(
            {
                "create_map": ActionClient(self, CreateMap, "create_map", callback_group=cb),
                "navigate": ActionClient(self, Navigate, "navigate", callback_group=cb),
                "global_localization": ActionClient(self, GlobalLocalization, "global_localization", callback_group=cb),
            }
        )
        self.action_clients["follow_taught_path"] = ActionClient(
            self, FollowTaughtPath, "follow_taught_path", callback_group=cb
        )
        # 急停用的「取消全部」client 趁現在建好（JobRunner 建構時 action_clients 還是空的）。
        # 急停當下才建的話，第一次 service_is_ready() 幾乎一定是 False 而被跳過
        self.jobs._ensure_cancel_clients()

        # ── 位姿來源（畫機器人在地圖上的位置）───────────────
        if self.enable_map:
            self.create_timer(1.0 / max(pose_update_rate, 0.1), self.pose_tracker.pose_timer_cb, callback_group=cb)

        # 看門狗：等不到 llm_response 時把扣住的 speech_text 放出來
        self.create_timer(1.0, lambda: self.chat.flush_pending_speech(), callback_group=cb)

        # 用戶端在場 → 高頻訂閱的開關
        self.create_timer(1.0, self._client_tick, callback_group=cb)

        # ── FastAPI ───────────────────────────────────────
        self.app: FastAPI = build_app(self)
        self._server_thread = threading.Thread(target=self._run_server, daemon=True)
        self._server_thread.start()

        # ★ 把影像參數印進啟動 log。專案鐵則：
        #   `ros2 param get` 不算驗證，要看啟動 log 或實際行為。
        self.get_logger().info(f"  平板影像：{self.video_fps:.0f} fps、JPEG 品質 {self.video_quality}")
        self.get_logger().info("✓ HMI 伺服器節點已初始化")
        urls = self.access_urls()
        if urls:
            # 第一個是預設路由那張網卡，平板通常就是連這個；其餘併成一行免得洗版
            self.get_logger().info(f"  網頁介面: {urls[0]}")
            if len(urls) > 1:
                self.get_logger().info(f"            其他網卡: {'、'.join(urls[1:5])}")
        else:
            self.get_logger().warn(f"  網頁介面: 連接埠 {self.port}，但抓不到本機 IP，請用 `hostname -I` 查")
        self.get_logger().info(
            f"  影像來源: {image_topic}" f"（{'CompressedImage' if self.image_compressed else 'Image'}）"
        )
        self.get_logger().info(f"  身份來源: {identity_topic}")
        if self.enable_map:
            self.get_logger().info(
                f"  地圖來源: {map_topic}（{self.map_frame} → {self.robot_frame}）"
                f"，位姿與地圖渲染在無人觀看 {self.pose_watch_ttl:.0f} 秒後自動停用"
            )
        else:
            self.get_logger().info("  地圖功能已關閉（enable_map:=false），省下 /map 訂閱與 TF 監聽")

    # ── 簡單的 ROS callback（篇幅小、不值得獨立成模組的，留在節點上）───

    def _identity_cb(self, msg: UserIdentity) -> None:
        self.state.set_identity(
            user_name=msg.user_name,
            user_type=USER_TYPE_NAMES.get(msg.user_type.type, "GUEST"),
            recognized=bool(msg.recognized),
            similarity=round(float(msg.similarity), 3),
            description=msg.description,
            bbox=[round(float(v), 1) for v in msg.bbox],
        )

    def _current_map_cb(self, msg: String) -> None:
        self.state.set_system(map_id=msg.data)
        # 換地圖後舊的 amcl_pose 是另一張圖的座標，留著會讓箭頭停在錯的位置，
        # 而且會擋住 TF 退路。丟掉，等 amcl 重新定位後自然會有新的。
        self.pose_tracker.on_map_switched()

    def _voltage_cb(self, msg: Float32) -> None:
        """電池電壓"""
        now = time.monotonic()
        if now - self._last_voltage_at < 1.0:
            return
        self._last_voltage_at = now
        self.state.set_system(voltage=round(float(msg.data), 2))

    def _charging_cb(self, msg: Bool) -> None:
        """充電旗標。只在真的變了才寫，否則等於每則都推播一次整包狀態。"""
        val = bool(msg.data)
        if self.state.system.get("charging") is not val:
            self.state.set_system(charging=val)

    def _charge_current_cb(self, msg: Float32) -> None:
        now = time.monotonic()
        if now - self._last_charge_cur_at < 1.0:
            return
        self._last_charge_cur_at = now
        self.state.set_system(charge_current=round(float(msg.data), 2))

    def _nav_path_cb(self, msg, source: str) -> None:
        """把規劃路徑送給前端畫，取樣到最多 MAX_PATH_POINTS 點"""
        pts = [(round(p.pose.position.x, 3), round(p.pose.position.y, 3)) for p in msg.poses]
        if len(pts) > self.MAX_PATH_POINTS:
            step = len(pts) / float(self.MAX_PATH_POINTS)
            idx = sorted({int(i * step) for i in range(self.MAX_PATH_POINTS)})
            if idx[-1] != len(pts) - 1:
                idx.append(len(pts) - 1)
            pts = [pts[i] for i in idx]
        self.state.set_nav_path(source, pts)

    # ── 用戶端在場 → 高頻訂閱的開關 ───────────────────────

    def note_client(self) -> None:
        """記下「剛剛有用戶端來過」。由 HTTP middleware 與 WebSocket 呼叫。"""
        with self._client_lock:
            self._last_http_at = time.monotonic()

    def clients_present(self) -> bool:
        with self._client_lock:
            if self._ws_clients > 0:
                return True
            return (time.monotonic() - self._last_http_at) < self.client_idle_ttl

    def _acquire_odom_sub(self) -> None:
        if self._odom_sub is not None:
            return
        self._odom_sub = self.create_subscription(
            Odometry, "odom", self.pose_tracker.odom_speed_cb, self._sensor_qos, callback_group=self._cb
        )

    def _release_odom_sub(self) -> None:
        if self._odom_sub is None:
            return
        self.destroy_subscription(self._odom_sub)
        self._odom_sub = None
        # 訂閱沒了就不會有新車速，明確清成 None，免得畫面停在一個舊數字
        self.state.set_system(speed=None)

    def _client_tick(self) -> None:
        present = self.clients_present()
        if present != self._clients_seen:
            self._clients_seen = present
            self.get_logger().info(
                "偵測到用戶端，恢復 /odom（車速）訂閱"
                if present
                else f"沒有任何用戶端連線（閒置 {self.client_idle_ttl:.0f} 秒），" "停用 /odom 與影像訂閱以節省 CPU"
            )

        if present:
            self._acquire_odom_sub()
        else:
            self._release_odom_sub()

        if self.clients_present() and self.video.wants_stream():
            self.video.acquire()
        else:
            self.video.release()

    def access_urls(self) -> List[str]:
        """平板可以打開的網址清單"""
        if self.host not in ("0.0.0.0", "::", ""):
            return [f"http://{self.host}:{self.port}"]
        return [f"http://{ip}:{self.port}" for ip in detect_lan_ips()]

    # ── FastAPI 伺服器 ────────────────────────────────────

    def _run_server(self) -> None:
        """啟動 uvicorn"""
        config = uvicorn.Config(
            self.app,
            host=self.host,
            port=self.port,
            log_level="warning",
            ws_ping_interval=self.ws_ping_interval if self.ws_ping_interval > 0 else None,
            ws_ping_timeout=self.ws_ping_timeout if self.ws_ping_timeout > 0 else None,
            timeout_keep_alive=15,
        )
        server = uvicorn.Server(config)
        try:
            server.run()
        except Exception as e:
            self.get_logger().error(f"HMI 伺服器結束: {e}")


def main(args=None):
    """人機介面伺服器節點進入點"""
    rclpy.init(args=args)
    node = HmiServerNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
