#!/usr/bin/env python3
"""人機介面伺服器節點"""

import asyncio
import itertools
import json
import base64
import binascii
import math
import os
import re
import secrets
import socket
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import rclpy
import uvicorn
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from fastapi import Depends, FastAPI, Header, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from starlette.exceptions import HTTPException as StarletteHTTPException
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from pydantic import BaseModel
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Empty, Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from smartnav_msgs.msg import RegistrationProgress, UserIdentity, UserType
from smartnav_msgs.srv import (
    CreateWaypoint,
    DeleteUser,
    ListMaps,
    ListUsers,
    ListWaypoints,
    RegisterFace,
    RegisterFacePhoto,
    SwitchMap,
    UpdateUser,
)
from smartnav_msgs.action import CreateMap, GlobalLocalization, Navigate

# DeleteWaypoint 是後加的介面。smartnav_msgs 是 CMake 套件，重建它會換掉執行中節點
# 已經載入的 .so，所以不能跟著 HMI 一起隨手 build。這裡容許它還不存在：
# 沒有的話 HMI 照常啟動，只有刪除地點那支端點回報「需要重建 smartnav_msgs」，
# 而不是整個節點在 import 就掛掉、連迎賓畫面都開不起來。
try:
    from smartnav_msgs.srv import DeleteWaypoint
except ImportError:  # pragma: no cover - 取決於 smartnav_msgs 有沒有重建
    DeleteWaypoint = None

USER_TYPE_NAMES = {UserType.GUEST: "GUEST", UserType.VIP: "VIP", UserType.ADMIN: "ADMIN"}

# 健康頁的「預期節點」清單。列在這裡的節點沒啟動時會顯示成「未啟動」，
# 沒列到的只要有在跑就會出現在「其他」那組——兩者合起來才看得出「該在卻不在」。
#
# SmartNav 這組取自各套件 super().__init__() 的實際節點名稱；
# nav2 兩組取自 nav2_bringup 的 localization_launch.py 與 navigation_launch.py
# 裡的 lifecycle_nodes，跟著上游走才不會漏。
NODE_GROUPS = [
    (
        "SmartNav",
        [
            "hmi_server_node",
            "user_auth_node",
            "face_embedding_node",
            "llm_service_node",
            "voice_trigger_node",
            "speech_recognizer_node",
            "speech_synthesizer_node",
            "audio_playback_node",
            "map_service_node",
            "waypoint_service_node",
            "navigation_service_node",
        ],
    ),
    ("nav2 定位", ["map_server", "amcl"]),
    (
        "nav2 導航",
        [
            "controller_server",
            "smoother_server",
            "planner_server",
            "route_server",
            "behavior_server",
            "velocity_smoother",
            "collision_monitor",
            "bt_navigator",
            "waypoint_follower",
            "docking_server",
        ],
    ),
]
OTHER_GROUP = "其他執行中的節點"

# 由其他節點內部自動建立、無法對應到單一行程的節點。
# 它們跟著父行程走，光看名字判不出在哪台機器，所以不硬猜。
INTERNAL_NODE_RE = re.compile(r"^(transform_listener_impl_|_ros2cli_daemon_|.*_rclcpp_node$)")

# 比對 speech_text 與 LLM 回覆時用來抹平標點差異。
# llm_service_node 送給 TTS 的句子已經去過標點，原文則保留標點。
NON_WORD_RE = re.compile(r"[^\w一-龥]")

# 照片註冊的上限。base64 會讓內容膨脹約 1/3，這裡限制的是解碼後的位元組數。
MAX_PHOTO_BYTES = 8 * 1024 * 1024
MAX_PHOTOS = 20


def decode_photo(payload: str) -> CompressedImage:
    """把 base64 或 data URL 字串轉成 CompressedImage

    照片走 JSON+base64 而不是 multipart，是為了不必在 Pi 上多裝 python-multipart。
    """
    data = payload.split(",", 1)[1] if payload.startswith("data:") else payload
    try:
        raw = base64.b64decode(data, validate=True)
    except (binascii.Error, ValueError):
        raise ValueError("照片不是有效的 base64 資料")
    if not raw:
        raise ValueError("照片內容是空的")
    if len(raw) > MAX_PHOTO_BYTES:
        raise ValueError(f"單張照片超過 {MAX_PHOTO_BYTES // (1024 * 1024)} MB 上限")

    msg = CompressedImage()
    msg.format = "jpeg"
    msg.data = raw
    return msg

# 各動作的等待上限（秒）。比動作節點自己的逾時再多留一點餘裕——
# 這裡的值只在動作伺服器整個掛掉時才會用到，避免留下永不結束的執行緒。
ACTION_TIMEOUTS = {
    "create_map": 480.0,          # 節點內建 400 秒
    "global_localization": 260.0,  # 節點內建 200 秒
    "navigate": 160.0,             # 節點內建 100 秒
}


def detect_lan_ips() -> List[str]:
    """列出本機可供平板連線的 IPv4 位址（排除 loopback）

    刻意不依賴 netifaces／psutil 之類的額外套件——Pi 上多裝一個 pip 套件
    就多一個部署時會忘記裝的東西。兩段式偵測：

      1. 開一個「連到外部位址」的 UDP socket，問路由表該走哪張網卡。
         UDP 的 connect 不會真的送出封包，**沒有網路也能用**，
         拿到的是預設路由那張網卡的位址——通常就是平板連得到的那個。
      2. 再從主機名稱解析補上其他網卡：機器人常同時接有線與 WiFi，
         平板可能只連得到其中一邊。

    Returns:
        List[str]: 依可用性排序的 IPv4 位址，偵測不到時為空 list
    """
    ips: List[str] = []

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        ips.append(probe.getsockname()[0])
    except OSError:
        pass  # 沒有預設路由（例如只接了一條沒設閘道的網線）
    finally:
        probe.close()

    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            # Debian 系的 /etc/hosts 常把主機名指到 127.0.1.1，那個對平板沒用
            if ip not in ips and not ip.startswith("127."):
                ips.append(ip)
    except OSError:
        pass

    return ips


def yaw_to_quaternion(yaw: float) -> Dict[str, float]:
    """把平面角度轉成四元數（只繞 Z 軸）"""
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """從四元數取出平面角度（只看 Z 軸分量）"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def make_pose(x: float, y: float, yaw: float) -> Pose:
    """由平面座標與角度組出 geometry_msgs/Pose"""
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = 0.0
    q = yaw_to_quaternion(float(yaw))
    pose.orientation.x = q["x"]
    pose.orientation.y = q["y"]
    pose.orientation.z = q["z"]
    pose.orientation.w = q["w"]
    return pose


class HmiState:
    """HMI 共享狀態

    ROS callback（多執行緒）寫入、asyncio（WebSocket）讀取。
    每次寫入遞增 version，WebSocket 端靠比對 version 決定是否推播。
    """

    def __init__(self, max_messages: int = 40):
        self._lock = threading.Lock()
        self._version = 0
        # 對話紀錄另外計版本。LLM 串流時每個 token 都會讓 _version 前進，
        # 若每次都把整份對話重送，等於在 LLM 回應的當下對 Pi 加上 10 Hz 的
        # JSON 序列化負擔——而 Pi4 的瓶頸正是 CPU（實測 load average 19）。
        self._messages_version = 0
        self.max_messages = max_messages

        self.identity: Dict[str, Any] = {
            "user_name": "",
            "user_type": "GUEST",
            "recognized": False,
            "similarity": 0.0,
            "description": "",
            "bbox": [],
            # 0.0 代表從沒收過辨識結果。user_auth_node 只在「偵測到人臉」時才
            # 發布，沒人時不會發「沒人」——所以人在不在只能靠這個時戳的新鮮度判斷。
            "updated_at": 0.0,
        }
        self.messages: List[Dict[str, Any]] = []
        self.partial_text: str = ""
        self.llm_streaming: str = ""
        self.system: Dict[str, Any] = {
            "speaking": False,
            "voltage": None,
            "map_id": "",
            "camera_fps": 0.0,
            "started_at": time.time(),
            # llm_service_node 以 latched 話題發布，LLM 沒啟動時就一直是空字串
            "llm_model": "",
        }
        self.map_meta: Optional[Dict[str, Any]] = None
        self.robot_pose: Optional[Dict[str, float]] = None
        self.jobs: Dict[str, Dict[str, Any]] = {}
        # 人臉註冊進度。放在共用狀態而不是前端區域變數，這樣平板、筆電
        # 等所有連線中的裝置看到的採樣倒數與結果都是同一份。
        self.registration: Dict[str, Any] = {}

    # ── 寫入端（ROS callback）──────────────────────────────

    def _bump(self) -> None:
        self._version += 1

    def set_identity(self, **kwargs) -> None:
        with self._lock:
            self.identity.update(kwargs)
            self.identity["updated_at"] = time.time()
            self._bump()

    def add_message(self, role: str, text: str, stats: Optional[Dict[str, Any]] = None) -> None:
        text = (text or "").strip()
        if not text:
            return
        with self._lock:
            # 同一角色連續送出完全相同的內容就不重複記（TTS 與 LLM 常會重疊）
            if self.messages and self.messages[-1]["role"] == role and self.messages[-1]["text"] == text:
                return
            entry: Dict[str, Any] = {"role": role, "text": text, "ts": time.time()}
            if stats:
                entry["stats"] = stats
            self.messages.append(entry)
            del self.messages[: max(0, len(self.messages) - self.max_messages)]
            self._messages_version += 1
            self._bump()

    def clear_messages(self) -> None:
        """清空對話紀錄與串流中的殘句

        串流中的文字一併清掉，否則清完畫面後上一輪還沒收尾的半句會馬上又冒出來。
        """
        with self._lock:
            if not self.messages and not self.llm_streaming and not self.partial_text:
                return
            self.messages.clear()
            self.llm_streaming = ""
            self.partial_text = ""
            self._messages_version += 1
            self._bump()

    def append_stream(self, token: str) -> None:
        """把串流 token 接到目前這段回覆後面

        llm_stream 話題送的是「單一 token」不是累積字串，直接覆蓋的話
        畫面上只會看到最新那一個字，不會有逐字浮現的效果。
        """
        if not token:
            return
        with self._lock:
            self.llm_streaming += token
            self._bump()

    def set_partial(self, text: str) -> None:
        with self._lock:
            if self.partial_text != text:
                self.partial_text = text
                self._bump()

    def set_stream(self, text: str) -> None:
        with self._lock:
            if self.llm_streaming != text:
                self.llm_streaming = text
                self._bump()

    def set_system(self, **kwargs) -> None:
        with self._lock:
            changed = any(self.system.get(k) != v for k, v in kwargs.items())
            self.system.update(kwargs)
            if changed:
                self._bump()

    def set_map_meta(self, meta: Optional[Dict[str, Any]]) -> None:
        with self._lock:
            self.map_meta = meta
            self._bump()

    def set_robot_pose(self, pose: Optional[Dict[str, float]]) -> None:
        """更新機器人位姿

        位姿以 5 Hz 更新，若每次都遞增版本號會讓 WebSocket 一直推整包狀態。
        因此只有移動超過 2 公分或轉超過 2 度才視為「變了」。
        """
        with self._lock:
            old = self.robot_pose
            if pose is None:
                changed = old is not None
            elif old is None:
                changed = True
            else:
                changed = (
                    abs(pose["x"] - old["x"]) > 0.02
                    or abs(pose["y"] - old["y"]) > 0.02
                    or abs(pose["yaw"] - old["yaw"]) > 0.035
                )
            self.robot_pose = pose
            if changed:
                self._bump()

    def set_job(self, job_id: str, **kwargs) -> None:
        with self._lock:
            job = self.jobs.setdefault(job_id, {"job_id": job_id})
            job.update(kwargs)
            # 只保留最近 20 筆，避免長時間運行後無限增長
            if len(self.jobs) > 20:
                for stale in sorted(self.jobs, key=lambda k: self.jobs[k].get("started_at", 0))[:-20]:
                    del self.jobs[stale]
            self._bump()

    def set_registration(self, payload: Optional[Dict[str, Any]]) -> None:
        """整份取代註冊狀態。傳 None 表示清除"""
        with self._lock:
            new = dict(payload) if payload else {}
            if new == self.registration:
                return
            self.registration = new
            self._bump()

    def merge_registration(self, payload: Dict[str, Any]) -> None:
        """疊加更新註冊狀態

        進度來自 registration_progress 話題，但倒數要用的 started_at/deadline
        是發起端點寫的——用疊加才不會被進度訊息洗掉。
        """
        with self._lock:
            new = dict(self.registration)
            new.update(payload)
            if new == self.registration:
                return
            self.registration = new
            self._bump()

    def registration_active(self) -> bool:
        with self._lock:
            return self.registration.get("status") == "running"

    # ── 讀取端（asyncio）──────────────────────────────────

    def snapshot(self, since_messages_version: Optional[int] = None) -> Dict[str, Any]:
        """取得狀態快照

        傳入 `since_messages_version` 時，只有對話真的變過才會帶上 `messages`
        欄位——前端沿用自己那份即可。省下的是 LLM 串流期間每秒 10 次、
        每次數十則訊息的序列化成本。
        """
        with self._lock:
            payload = {
                "version": self._version,
                # 伺服器時鐘基準。前端要判斷 identity 有多舊，但平板時鐘不一定
                # 跟機器人同步，所以送出當下的伺服器時間讓前端算差值而不是絕對值。
                "now": time.time(),
                "identity": dict(self.identity),
                "messages_version": self._messages_version,
                "partial_text": self.partial_text,
                "llm_streaming": self.llm_streaming,
                "system": dict(self.system),
                "map_meta": dict(self.map_meta) if self.map_meta else None,
                "robot_pose": dict(self.robot_pose) if self.robot_pose else None,
                "jobs": [dict(j) for j in self.jobs.values()],
                "registration": dict(self.registration) if self.registration else None,
            }
            if since_messages_version is None or since_messages_version != self._messages_version:
                payload["messages"] = list(self.messages)
            return payload

    @property
    def version(self) -> int:
        with self._lock:
            return self._version

    @property
    def messages_version(self) -> int:
        with self._lock:
            return self._messages_version

    def map_meta_copy(self) -> Optional[Dict[str, Any]]:
        """只取地圖中繼資料

        原本 /api/map/meta 是呼叫 snapshot() 再挑一個欄位出來，等於為了拿六個
        數字把整份對話紀錄也複製一遍。
        """
        with self._lock:
            return dict(self.map_meta) if self.map_meta else None

    def jobs_copy(self) -> List[Dict[str, Any]]:
        """只取作業清單（理由同 map_meta_copy）"""
        with self._lock:
            return [dict(j) for j in self.jobs.values()]


# ── 請求格式 ─────────────────────────────────────────────


class LoginRequest(BaseModel):
    """POST /api/login"""

    username: str = ""
    password: str = ""


class RegisterPhotoRequest(BaseModel):
    """POST /api/users/register-photo

    照片用 base64 傳而不是 multipart，因為 multipart 需要額外安裝
    python-multipart，而這個專案刻意不在 Pi 上多裝 pip 套件。
    photos 接受純 base64 或 "data:image/jpeg;base64,..." 兩種寫法。
    """

    user_name: str
    user_type: int = 0
    description: str = ""
    photos: List[str] = []


class RegisterRequest(BaseModel):
    """POST /api/users/register"""

    user_name: str
    user_type: int = 0
    description: str = ""
    num_samples: int = 10


class UpdateUserRequest(BaseModel):
    """PUT /api/users/{uuid}"""

    user_name: str = ""
    user_type: int = 0
    description: str = ""


class SayRequest(BaseModel):
    """POST /api/say（丟給 LLM）與 /api/speak（直接播報）"""

    text: str


class TeleopRequest(BaseModel):
    # 正值前進、負值後退 (m/s)。上限由 hmi_server 端再夾一次，
    # 不信任前端傳來的數值。
    linear: float = 0.0
    # 正值左轉 (rad/s)
    angular: float = 0.0


class RearMaskRequest(BaseModel):
    enabled: bool = True
    # 以車尾正後方為中心、往兩側各遮這麼多度
    half_angle_deg: float = 35.0


class SwitchMapRequest(BaseModel):
    """POST /api/maps/switch"""

    map_id: str


class CreateMapRequest(BaseModel):
    """POST /api/maps/create"""

    map_name: str


class CreateWaypointRequest(BaseModel):
    """POST /api/waypoints

    x/y/yaw 皆為 map frame 座標。use_given_pose=false 時忽略座標，
    改用機器人當前位置建點。
    """

    waypoint_name: str
    use_given_pose: bool = True
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class NavigateRequest(BaseModel):
    """POST /api/navigate

    waypoint_id 有值時導航到既有地點；否則導航到 x/y/yaw 指定的座標。
    """

    waypoint_id: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    target_name: str = ""


class HmiServerNode(Node):
    """HMI 伺服器節點"""

    # 對齊 user_auth_node._register_face_callback 裡的 threading.Timer(20.0)。
    # 那支計時器一到就會把樣本不足的使用者刪掉，這裡不能等得比它久。
    REG_TIMEOUT_SEC = 20.0
    # 輪詢註冊進度的間隔。1 秒足以看到張數跳動，又不會把 list_users 打爆。
    REG_POLL_SEC = 1.0

    # 最後一個 token 之後多久內，仍視為 LLM 還在產生這一輪的回覆。
    # agent 多步驟決策時兩次 invoke 之間會停頓，值太小會把中段的句子放出去。
    STREAM_ACTIVE_SEC = 5.0
    # 扣住的句子等不到 llm_response 就直接顯示，避免 LLM 掛掉時訊息消失
    SPEECH_HOLD_SEC = 12.0

    def __init__(self):
        super().__init__("hmi_server_node")

        # ── 參數 ──────────────────────────────────────────
        self.declare_parameter("host", "0.0.0.0", ParameterDescriptor(description="HTTP 綁定位址"))
        self.declare_parameter("port", 8080, ParameterDescriptor(description="HTTP 連接埠"))
        self.declare_parameter(
            "admin_username", "admin", ParameterDescriptor(description="管理者帳號")
        )
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
        self.declare_parameter(
            "admin_session_hours", 12.0, ParameterDescriptor(description="管理者登入有效時數")
        )
        # 節點跑在哪台機器是靠掃本機 /proc 判定的，遇到判斷不準的節點可以在這裡釘死
        self.declare_parameter(
            "pi_nodes", [""], ParameterDescriptor(description="強制歸類為本機（Pi）的節點名稱")
        )
        self.declare_parameter(
            "edge_nodes", [""], ParameterDescriptor(description="強制歸類為邊緣裝置的節點名稱")
        )
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
        self.declare_parameter("video_fps", 12.0, ParameterDescriptor(description="MJPEG 串流上限幀率"))
        self.declare_parameter("video_quality", 70, ParameterDescriptor(description="JPEG 壓縮品質 1-100"))
        self.declare_parameter(
            "video_width", 640, ParameterDescriptor(description="MJPEG 輸出寬度，0 表示不縮放")
        )
        self.declare_parameter(
            "service_timeout", 8.0, ParameterDescriptor(description="呼叫 ROS 服務的等待秒數")
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
        self.admin_username = self.get_parameter("admin_username").get_parameter_value().string_value
        self.admin_password = self._resolve_admin_password()
        self.admin_session_sec = (
            self.get_parameter("admin_session_hours").get_parameter_value().double_value * 3600.0
        )
        self.pi_nodes = {
            n for n in self.get_parameter("pi_nodes").get_parameter_value().string_array_value if n
        }
        self.edge_nodes = {
            n for n in self.get_parameter("edge_nodes").get_parameter_value().string_array_value if n
        }
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

        transport = self.get_parameter("image_transport").get_parameter_value().string_value.strip().lower()
        if transport == "auto":
            self.image_compressed = image_topic.endswith("/compressed")
        else:
            self.image_compressed = transport == "compressed"

        # ── 狀態與緩衝 ─────────────────────────────────────
        self.state = HmiState()
        self.bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_jpeg: Optional[bytes] = None
        self._last_encode_time = 0.0
        self._frame_times: List[float] = []
        # 壓縮影像是否能直接轉發位元組（None = 還沒判斷過，用第一張影格決定）
        self._passthrough: Optional[bool] = None
        # 管理者登入權杖 -> 到期時間戳。存在記憶體，節點重啟即全部失效。
        self._tokens: Dict[str, float] = {}
        self._token_lock = threading.Lock()
        # lifecycle 查詢用的 get_state 客戶端。只在 _node_status() 一次查詢期間存在，
        # 查完就銷毀——留著會永久墊高執行器每一輪 wait set 的實體數（見 _node_status）
        self._state_clients: Dict[str, Any] = {}
        self._nodes_cache: List[Dict[str, Any]] = []
        self._nodes_cache_at = 0.0
        self._nodes_lock = threading.Lock()
        # 這一輪對話的計時與去重狀態
        self._turn_lock = threading.Lock()
        self._turn_started = 0.0
        self._first_token_at = 0.0
        self._last_llm_text = ""
        self._last_llm_at = 0.0
        self._last_token_at = 0.0
        # LLM 這一輪期間收到的 speech_text 先扣住，等完整回覆到了再比對。
        # 不能當下就判斷：llm_stream 與 speech_text 是兩個話題，DDS 不保證
        # 跨話題順序，整句常比構成它的 token 更早抵達。
        self._pending_speech: List[tuple] = []

        self._map_lock = threading.Lock()
        self._map_png: Optional[bytes] = None
        self._map_version = 0
        self._last_map_render = 0.0
        # 最新一則還沒渲染的 OccupancyGrid。/map 的回呼只把訊息「放這裡」，
        # 真正的 PNG 編碼延到 _render_map_if_needed()（有人在看地圖時才做）。
        # 原因：建圖時 slam_toolbox 每 3 秒重發整張地圖，在 ROS 執行緒上直接
        # 做 numpy 轉換＋cv2.imencode 會把執行緒卡住數十毫秒，而九成的時間
        # 根本沒有人打開地圖分頁。訊息只存參考，不複製，成本接近零。
        self._pending_grid: Optional[OccupancyGrid] = None

        # ── 位姿來源（按需開啟）──────────────────────────
        # 有人在看地圖的期限（time.monotonic()）。0 = 沒人在看。
        self._pose_watch_until = 0.0
        # amcl_pose 訂閱與最近一次收到的位姿。None 代表還沒收到過。
        self._amcl_sub = None
        self._amcl_pose: Optional[Dict[str, float]] = None
        # 這一輪觀看是否已經拿到過初始位置（見 _pose_timer_cb）
        self._pose_seeded = False
        self._pose_lock = threading.Lock()
        # 電壓節流用（見 _voltage_cb）
        self._last_voltage_at = 0.0

        self._job_counter = itertools.count(1)
        # 進行中的動作 goal handle 登記簿，供取消使用
        self._job_handles: Dict[str, Any] = {}
        self._job_handles_lock = threading.Lock()

        cb = ReentrantCallbackGroup()
        self._cb = cb   # nav2 的 get_state 客戶端要用到才建，屆時沿用同一個群組

        # 影像走 BEST_EFFORT——相機是 sensor data，掉幀比塞住好
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1
        )
        # 地圖與 current_map 都是 latched（TRANSIENT_LOCAL），晚加入也收得到最後一筆
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── 訂閱 ──────────────────────────────────────────
        if self.image_compressed:
            self.create_subscription(
                CompressedImage, image_topic, self._compressed_image_cb, sensor_qos, callback_group=cb
            )
        else:
            self.create_subscription(Image, image_topic, self._image_cb, sensor_qos, callback_group=cb)
        self.create_subscription(UserIdentity, identity_topic, self._identity_cb, 10, callback_group=cb)
        self.create_subscription(
            RegistrationProgress, "registration_progress", self._registration_progress_cb, 10,
            callback_group=cb,
        )
        self.create_subscription(String, "user_text", self._user_text_cb, 10, callback_group=cb)
        self.create_subscription(String, "partial_text", self._partial_text_cb, 10, callback_group=cb)
        self.create_subscription(String, "llm_response", self._llm_response_cb, 10, callback_group=cb)
        self.create_subscription(String, "llm_stream", self._llm_stream_cb, 10, callback_group=cb)
        # 模型名稱是 latched，HMI 比 LLM 晚啟動也收得到
        self.create_subscription(String, "llm_model", self._llm_model_cb, latched_qos, callback_group=cb)
        self.create_subscription(String, "speech_text", self._speech_text_cb, 10, callback_group=cb)
        self.create_subscription(Bool, "playback_status", self._playback_cb, 10, callback_group=cb)
        if self.enable_map:
            self.create_subscription(OccupancyGrid, map_topic, self._map_cb, latched_qos, callback_group=cb)
        # map_service_node 發布的是 std_msgs/String（內容為 map_id），不是 MapInfo
        self.create_subscription(String, "current_map", self._current_map_cb, latched_qos, callback_group=cb)
        # 電池電壓由 WHEELTEC 底盤發布，沒跑底盤時單純不會有資料。
        # 用 depth=1 + BEST_EFFORT：電壓只要「最新值」，堆 10 則舊值沒有意義，
        # 而且 HMI 忙起來時 RELIABLE 會逼 DDS 重送我們根本不看的舊資料。
        # （BEST_EFFORT 訂閱者相容於 RELIABLE 發布者，底盤那端不用改。）
        self.create_subscription(Float32, "PowerVoltage", self._voltage_cb, sensor_qos, callback_group=cb)

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
        self.teleop_pub = self.create_publisher(
            Twist, self.get_parameter("teleop_cmd_topic").value, 10
        )
        self._teleop_watchdog_sec = float(self.get_parameter("teleop_watchdog_sec").value)
        self._teleop_last_cmd = 0.0      # 最後一次收到遙控指令的時間 (monotonic)
        self._teleop_active = False      # 是否還需要送停止命令
        self._teleop_linear = 0.0
        self._teleop_angular = 0.0
        self._teleop_lock = threading.Lock()
        # 10 Hz：只負責「補送指令」與「逾時歸零」，不是主要的指令來源
        self.create_timer(0.1, self._teleop_tick, callback_group=cb)

        # ── 服務／動作客戶端 ───────────────────────────────
        self.service_clients: Dict[str, Any] = {
            "register_face": self.create_client(RegisterFace, "register_face", callback_group=cb),
            "register_face_photo": self.create_client(
                RegisterFacePhoto, "register_face_photo", callback_group=cb
            ),
            "list_users": self.create_client(ListUsers, "list_users", callback_group=cb),
            "delete_user": self.create_client(DeleteUser, "delete_user", callback_group=cb),
            "update_user": self.create_client(UpdateUser, "update_user", callback_group=cb),
            "list_maps": self.create_client(ListMaps, "list_maps", callback_group=cb),
            "switch_map": self.create_client(SwitchMap, "switch_map", callback_group=cb),
            # 遙控建圖用：沒有 /exploration_complete 事件，要由操作者手動結束
            "finish_map": self.create_client(Trigger, "/finish_map", callback_group=cb),
            "create_waypoint": self.create_client(CreateWaypoint, "create_waypoint", callback_group=cb),
            "list_waypoints": self.create_client(ListWaypoints, "list_waypoints", callback_group=cb),
        }
        # smartnav_msgs 還沒重建時 DeleteWaypoint 會是 None，此時不建客戶端，
        # 由端點回報要重建；其餘功能完全不受影響。
        if DeleteWaypoint is not None:
            self.service_clients["delete_waypoint"] = self.create_client(
                DeleteWaypoint, "delete_waypoint", callback_group=cb
            )
        self.action_clients: Dict[str, ActionClient] = {
            "create_map": ActionClient(self, CreateMap, "create_map", callback_group=cb),
            "navigate": ActionClient(self, Navigate, "navigate", callback_group=cb),
            "global_localization": ActionClient(
                self, GlobalLocalization, "global_localization", callback_group=cb
            ),
        }

        # ── 位姿來源（畫機器人在地圖上的位置）───────────────
        #
        # 這裡原本是「開機就建一個 TransformListener」，那是本節點最大的 CPU 來源：
        #
        #   ・TransformListener 訂閱 /tf，預設 QoS depth=100。導航跑起來時 /tf 上有
        #     ekf_node（約 50 Hz）、robot_state_publisher、amcl，建圖時再加上
        #     slam_toolbox 的 transform_publish_period=0.02（50 Hz），合計輕鬆破 100 Hz。
        #   ・rclpy 對「每一則」訊息都要在 Python 端把整個 TFMessage 反序列化成物件樹
        #     （每個 transform ≈ 7 個 Python 物件），再逐一 set_transform() 轉回 C++。
        #     這條路徑在 Pi4 上每則約 1～3 ms——實測 ROS 執行緒 19.4% CPU，
        #     其中絕大部分就花在這裡，而畫面上只需要 2 Hz 的一個箭頭。
        #
        # 改成兩層，兩層都是「有人在看地圖才開」（見 _pose_timer_cb）：
        #   1. 優先訂閱 amcl_pose：amcl 直接給 map→base_link，只在機器人真的移動、
        #      粒子重採樣時才發（約 10 Hz 上限，靜止時完全不發），而且是一則小訊息，
        #      沒有 TF buffer 的插入成本。導航中這層就夠用。
        #   2. TF 只在兩種時候出現：剛開始看、還需要一個初始位置，或者建圖模式
        #      根本沒有 amcl。前者拿到位置就立刻把監聽拆掉。
        #
        # 沒有人打開地圖分頁時（迎賓展示的常態）兩層都不存在，節點對 /tf 完全零訂閱。
        self.tf_buffer = None
        self.tf_listener = None
        if self.enable_map:
            self.create_timer(1.0 / max(pose_update_rate, 0.1), self._pose_timer_cb, callback_group=cb)

        # 看門狗：等不到 llm_response 時把扣住的 speech_text 放出來
        self.create_timer(1.0, lambda: self._flush_pending_speech(), callback_group=cb)

        # ── FastAPI ───────────────────────────────────────
        self.app = self._build_app()
        self._server_thread = threading.Thread(target=self._run_server, daemon=True)
        self._server_thread.start()

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
            f"  影像來源: {image_topic}"
            f"（{'CompressedImage' if self.image_compressed else 'Image'}）"
        )
        self.get_logger().info(f"  身份來源: {identity_topic}")
        if self.enable_map:
            self.get_logger().info(
                f"  地圖來源: {map_topic}（{self.map_frame} → {self.robot_frame}）"
                f"，位姿與地圖渲染在無人觀看 {self.pose_watch_ttl:.0f} 秒後自動停用"
            )
        else:
            self.get_logger().info("  地圖功能已關閉（enable_map:=false），省下 /map 訂閱與 TF 監聽")

    # ── ROS callbacks ────────────────────────────────────

    def _should_process_frame(self) -> Optional[float]:
        """依 video_fps 節流。要處理這一影格時回傳當下時間，否則回 None"""
        now = time.monotonic()
        if now - self._last_encode_time < 1.0 / max(self.video_fps, 1.0):
            return None
        self._last_encode_time = now
        return now

    def _encode_frame(self, frame: np.ndarray) -> Optional[bytes]:
        """縮放（如有需要）並編成 JPEG"""
        if self.video_width and frame.shape[1] != self.video_width:
            scale = self.video_width / frame.shape[1]
            frame = cv2.resize(frame, (self.video_width, int(frame.shape[0] * scale)))
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.video_quality])
        return buf.tobytes() if ok else None

    def _publish_frame(self, payload: bytes, now: float) -> None:
        """存下最新影格並更新幀率統計（只留最近兩秒）"""
        with self._frame_lock:
            self._latest_jpeg = payload

        self._frame_times.append(now)
        self._frame_times = [t for t in self._frame_times if now - t <= 2.0]
        if len(self._frame_times) >= 2:
            span = self._frame_times[-1] - self._frame_times[0]
            if span > 0:
                self.state.set_system(camera_fps=round((len(self._frame_times) - 1) / span, 1))

    def _image_cb(self, msg: Image) -> None:
        """原始影像（sensor_msgs/Image）：解成 BGR → 縮放 → 編 JPEG"""
        now = self._should_process_frame()
        if now is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            payload = self._encode_frame(frame)
        except Exception as e:  # 影像壞掉不該弄垮整個 HMI
            self.get_logger().warn(f"影像轉換失敗: {e}", throttle_duration_sec=5.0)
            return
        if payload is not None:
            self._publish_frame(payload, now)

    def _compressed_image_cb(self, msg: CompressedImage) -> None:
        """壓縮影像（sensor_msgs/CompressedImage）

        上游已經是 JPEG 而且尺寸就是我們要輸出的尺寸時，**直接把位元組轉發出去**——
        解碼再重新編碼一次只是白白浪費 Pi 的 CPU，還會多一次有損壓縮。
        能不能走這條捷徑用第一張影格判斷一次就好，相機解析度不會中途改變。
        """
        now = self._should_process_frame()
        if now is None:
            return

        try:
            if self._passthrough is None:
                self._passthrough = self._decide_passthrough(msg)

            if self._passthrough:
                payload = bytes(msg.data)
            else:
                frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    self.get_logger().warn("壓縮影像解碼失敗", throttle_duration_sec=5.0)
                    return
                payload = self._encode_frame(frame)
        except Exception as e:
            self.get_logger().warn(f"壓縮影像處理失敗: {e}", throttle_duration_sec=5.0)
            return

        if payload is not None:
            self._publish_frame(payload, now)

    def _decide_passthrough(self, msg: CompressedImage) -> bool:
        """用第一張影格決定壓縮影像能否直接轉發

        兩個條件都要成立：
          ・格式是 JPEG——MJPEG 串流每個分段都宣告 image/jpeg，塞 PNG 進去瀏覽器不吃
          ・尺寸已經等於 video_width（或設定為不縮放）
        """
        fmt = (msg.format or "").lower()
        if "jpeg" not in fmt and "jpg" not in fmt:
            self.get_logger().info(f"壓縮格式為「{msg.format}」非 JPEG，將解碼後重新編碼")
            return False

        if not self.video_width:
            self.get_logger().info("壓縮影像直接轉發（video_width=0，不縮放）——省下解碼與重新編碼")
            return True

        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("壓縮影像解碼失敗，改走解碼路徑")
            return False

        if frame.shape[1] == self.video_width:
            self.get_logger().info(f"壓縮影像直接轉發（來源已是 {self.video_width}px 寬）——省下解碼與重新編碼")
            return True

        self.get_logger().info(
            f"來源寬度 {frame.shape[1]}px 與 video_width={self.video_width} 不同，需解碼縮放。"
            f"想走零解碼捷徑可設 video_width:=0"
        )
        return False

    def _map_cb(self, msg: OccupancyGrid) -> None:
        """收下最新的佔據柵格，但**不在這裡渲染**

        原本這支直接做 numpy 轉換 + cv2.imencode，等於在 ROS 執行緒上同步跑數十
        毫秒的影像編碼。/map 是 latched 話題，重點是「最後一則」而不是每一則，
        所以這裡只留參考（不複製，成本接近零），實際渲染交給 _render_map_if_needed()：
        有人在看地圖時才做，而且照樣受 map_render_interval 節流。

        這樣建圖模式下 slam_toolbox 每 3 秒重發整張地圖也不再有 CPU 尖峰。
        """
        self._pending_grid = msg

    def _render_map_if_needed(self) -> None:
        """有人在看地圖時，把最新的柵格渲染成 PNG

        由 _pose_timer_cb 呼叫（跟位姿同一個節拍），沒有新地圖或還沒到節流間隔
        就直接返回，正常情況下是一個 if 就結束。
        """
        msg = self._pending_grid
        if msg is None:
            return

        now = time.monotonic()
        if now - self._last_map_render < self.map_render_interval:
            return
        self._last_map_render = now

        try:
            info = msg.info
            width, height = int(info.width), int(info.height)
            if width <= 0 or height <= 0:
                return

            grid = np.asarray(msg.data, dtype=np.int8).reshape(height, width)

            # -1（未知）→ 中灰；0~100（佔據機率）→ 白到黑的連續灰階
            img = np.full((height, width), 127, dtype=np.uint8)
            known = grid >= 0
            img[known] = (255 - grid[known].astype(np.int16) * 255 // 100).astype(np.uint8)

            # ROS 柵格的 row 0 在地圖「下方」，影像的 row 0 在上方，所以要上下翻轉
            img = np.flipud(img)

            ok, buf = cv2.imencode(".png", img)
            if not ok:
                return

            with self._map_lock:
                self._map_png = buf.tobytes()
                self._map_version += 1
                version = self._map_version

            origin = info.origin
            self.state.set_map_meta(
                {
                    "width": width,
                    "height": height,
                    "resolution": float(info.resolution),
                    "origin_x": float(origin.position.x),
                    "origin_y": float(origin.position.y),
                    "origin_yaw": quaternion_to_yaw(
                        origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w
                    ),
                    "version": version,
                }
            )
            # 渲染成功才丟掉來源。/map 是 latched 話題，map_server 載入後可能
            # 一整天都不會再發第二則——中途失敗就把它清掉的話，這張地圖
            # 就永遠畫不出來了。留著，下一個間隔會再試一次。
            self._pending_grid = None
        except Exception as e:
            self.get_logger().warn(f"地圖渲染失敗: {e}", throttle_duration_sec=10.0)

    def note_map_interest(self) -> None:
        """記下「現在有人在看地圖」

        由前端的 /api/map/watch 心跳與所有跟地圖有關的端點呼叫。位姿訂閱與
        地圖渲染都掛在這個訊號上——沒人看就一律不做，這是本節點省 CPU 的關鍵。
        """
        self._pose_watch_until = time.monotonic() + self.pose_watch_ttl

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        """amcl 直接給的 map→base_link，取代 TF 查詢

        amcl 只在粒子重採樣時發布，機器人靜止時完全不發——所以「舊」的位姿
        其實就是「現在」的位姿，這裡不做時效判斷（見 _pose_timer_cb）。
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self._pose_lock:
            self._amcl_pose = {
                "x": float(p.x),
                "y": float(p.y),
                "yaw": quaternion_to_yaw(q.x, q.y, q.z, q.w),
            }

    def _acquire_pose_sources(self) -> None:
        """開始觀看時建立 amcl_pose 訂閱（TF 留到真的需要才建）"""
        if self._amcl_sub is not None:
            return
        # depth=1：只要最新一筆，補送舊位姿對畫面沒有意義
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._amcl_pose_cb,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._cb,
        )

    def _acquire_tf_listener(self) -> None:
        """建立 TF 監聽（只在 amcl_pose 收不到時才走到這裡）"""
        if self.tf_listener is not None:
            return
        # cache_time 從預設的 10 秒縮到 2 秒：我們永遠只查「最新」的轉換，
        # 留著十秒歷史只是讓每次 set_transform 都往更大的 buffer 裡插。
        self.tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        # QoS depth 從 tf2_ros 預設的 100 降到 10。堆 100 則的唯一效果是
        # 我們落後時要補做 100 則的 Python 反序列化，只會讓塞車更嚴重。
        self.tf_listener = TransformListener(
            self.tf_buffer, self,
            qos=QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
        self.get_logger().info("收不到 amcl_pose，改用 TF 取位姿（建圖模式的正常情形）")

    def _drop_tf_listener(self) -> None:
        """拆掉 TF 監聽（訂閱一消失，/tf 就再也不會喚醒這個節點）"""
        if self.tf_listener is None:
            return
        self.tf_listener.unregister()
        self.tf_listener = None
        self.tf_buffer = None

    def _release_pose_sources(self) -> None:
        """沒人在看地圖了，把位姿相關的訂閱整個拆掉

        這才是省下來的地方：訂閱不存在，rclpy 就完全不會被 /tf 喚醒，
        也不用為每一則訊息付反序列化的代價。
        """
        self._drop_tf_listener()
        self._pose_seeded = False
        if self._amcl_sub is not None:
            self.destroy_subscription(self._amcl_sub)
            self._amcl_sub = None
        with self._pose_lock:
            self._amcl_pose = None

    def _pose_timer_cb(self) -> None:
        """更新機器人位姿，並在需要時渲染地圖

        沒人在看地圖時這支只花一次 monotonic() 比較就返回，是刻意的：
        迎賓展示時平板停在迎賓分頁，這條路徑一整天都不該做任何事。
        """
        watching = time.monotonic() < self._pose_watch_until

        if not watching:
            if self._amcl_sub is not None or self.tf_listener is not None:
                self._release_pose_sources()
                # 訂閱都拆了就沒有新位姿，明確清成 None，免得前端一直畫著
                # 一個早就不再更新的箭頭
                self.state.set_robot_pose(None)
            return

        self._acquire_pose_sources()
        self._render_map_if_needed()

        # amcl 在不在，看圖上有沒有人發布 amcl_pose 就知道。這是本機的圖快取
        # 查詢，不是服務呼叫，成本可以忽略。
        amcl_alive = self.count_publishers("amcl_pose") > 0

        with self._pose_lock:
            pose = self._amcl_pose
        if pose is not None:
            # amcl 有資料就絕不碰 TF——這是整個改動的重點
            self._drop_tf_listener()
            self._pose_seeded = True
            self.state.set_robot_pose(dict(pose))
            return

        if amcl_alive and self._pose_seeded:
            # amcl 活著但沒發新位姿 = 機器人靜止（amcl 只在粒子重採樣時發布）。
            # 位置既然不會變，就沒有理由為了「確認它沒變」養著 /tf 訂閱。
            return

        # 走到這裡只有兩種情況：
        #   (a) 剛開始看，還沒有任何初始位置可畫；
        #   (b) 建圖模式，根本沒有 amcl，只能靠 TF。
        self._acquire_tf_listener()
        try:
            # timeout 一定要留 0：tf2_ros 的 can_transform() 在有 timeout 時是
            # **忙等**（每 20 ms 醒一次），而且會在 ROS 執行緒上原地卡住。
            # 沒有導航時 map→base_link 本來就查不到，原本每次都白等滿 50 ms。
            # 這裡是 2 Hz 的輪詢，等不到就下次再查，不需要等。
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time(), timeout=Duration()
            )
        except Exception:
            # 沒跑導航時本來就沒有 map→base_link，這是正常狀態不是錯誤
            self.state.set_robot_pose(None)
            return

        r = trans.transform.rotation
        self.state.set_robot_pose(
            {
                "x": trans.transform.translation.x,
                "y": trans.transform.translation.y,
                "yaw": quaternion_to_yaw(r.x, r.y, r.z, r.w),
            }
        )
        self._pose_seeded = True
        if amcl_alive:
            # 初始位置拿到了，之後的更新交給 amcl_pose，TF 可以收掉。
            # 這條路徑對應「導航跑著、但機器人停著沒動」——最常見的看地圖情境。
            self._drop_tf_listener()

    def _identity_cb(self, msg: UserIdentity) -> None:
        self.state.set_identity(
            user_name=msg.user_name,
            user_type=USER_TYPE_NAMES.get(msg.user_type.type, "GUEST"),
            recognized=bool(msg.recognized),
            similarity=round(float(msg.similarity), 3),
            description=msg.description,
            bbox=[round(float(v), 1) for v in msg.bbox],
        )

    def _user_text_cb(self, msg: String) -> None:
        self.state.set_partial("")
        self.state.add_message("user", msg.data)
        # 一句話送出去就是一輪的起點，用來算這輪 LLM 花了多久
        with self._turn_lock:
            self._turn_started = time.time()
            self._first_token_at = 0.0
        self.state.set_stream("")

    def _partial_text_cb(self, msg: String) -> None:
        self.state.set_partial(msg.data)

    def _llm_response_cb(self, msg: String) -> None:
        now = time.time()
        with self._turn_lock:
            started, first = self._turn_started, self._first_token_at
            self._turn_started = 0.0
            self._first_token_at = 0.0
            # llm_response 之後才送達的 speech_text 仍屬於這輪，留著比對
            self._last_llm_text = msg.data
            self._last_llm_at = now

        stats: Dict[str, Any] = {"chars": len(msg.data.strip())}
        if started:
            stats["total"] = round(now - started, 2)
            if first:
                # 首字延遲＝送出到第一個 token，中間可能包含工具呼叫
                stats["think"] = round(first - started, 2)
                stats["generate"] = round(now - first, 2)
                if now - first > 0.05:
                    stats["cps"] = round(stats["chars"] / (now - first), 1)

        # 扣住的句子先處理掉，屬於這段回覆的丟棄，其餘（例如導航播報插進來）保留
        self._flush_pending_speech(final_text=msg.data)
        self.state.set_stream("")
        self.state.add_message("robot", msg.data, stats=stats)

    def _llm_stream_cb(self, msg: String) -> None:
        now = time.time()
        with self._turn_lock:
            if not self._first_token_at:
                self._first_token_at = now
            self._last_token_at = now
        self.state.append_stream(msg.data)

    def _speech_text_cb(self, msg: String) -> None:
        """語音輸出也記進對話流——但要濾掉 LLM 回覆的回音

        llm_service_node 把同一段回覆送兩個地方：逐句去標點後發到 speech_text
        給 TTS，最後再把完整版發到 llm_response。兩個都收的話，一次回答會變成
        「數個殘句 + 一句完整」重複顯示。

        判斷不能在收到當下做：llm_stream 與 speech_text 是兩個話題，DDS 不保證
        跨話題順序，實測整句常比構成它的 token 更早抵達，此時串流緩衝區還沒有
        那段字，比對就會漏。所以 LLM 這一輪期間先扣住，等 llm_response 來了再用
        完整回覆比對。非 LLM 來源（導航播報、語音測試頁）照舊立即顯示。
        """
        text = (msg.data or "").strip()
        if not text:
            return

        now = time.time()
        with self._turn_lock:
            active = now - self._last_token_at < self.STREAM_ACTIVE_SEC
            if active:
                self._pending_speech.append((now, text))
                return
            last_text, last_at = self._last_llm_text, self._last_llm_at

        # 不在 LLM 產生期間。仍可能是上一輪的尾句，給一小段寬限期再比對一次。
        if last_text and now - last_at < 15.0:
            if NON_WORD_RE.sub("", text) in NON_WORD_RE.sub("", last_text):
                return
        self.state.add_message("robot", text)

    def _flush_pending_speech(self, final_text: str = "") -> None:
        """處理扣住的 speech_text

        傳入 final_text 時（llm_response 到了）：屬於回覆一部分的丟掉，其餘顯示。
        沒傳時（看門狗）：只放行等太久的，避免 LLM 掛掉導致訊息永遠不出現。
        """
        now = time.time()
        with self._turn_lock:
            if final_text:
                ready = [t for _ts, t in self._pending_speech]
                self._pending_speech = []
            else:
                ready = [t for ts, t in self._pending_speech if now - ts > self.SPEECH_HOLD_SEC]
                self._pending_speech = [
                    (ts, t) for ts, t in self._pending_speech if now - ts <= self.SPEECH_HOLD_SEC
                ]

        norm_final = NON_WORD_RE.sub("", final_text) if final_text else ""
        for text in ready:
            if norm_final and NON_WORD_RE.sub("", text) in norm_final:
                continue
            self.state.add_message("robot", text)

    def _playback_cb(self, msg: Bool) -> None:
        self.state.set_system(speaking=bool(msg.data))

    def _current_map_cb(self, msg: String) -> None:
        self.state.set_system(map_id=msg.data)
        # 換地圖後舊的 amcl_pose 是另一張圖的座標，留著會讓箭頭停在錯的位置，
        # 而且會擋住 TF 退路。丟掉，等 amcl 重新定位後自然會有新的。
        with self._pose_lock:
            self._amcl_pose = None

    def _llm_model_cb(self, msg: String) -> None:
        self.state.set_system(llm_model=msg.data)

    def _voltage_cb(self, msg: Float32) -> None:
        """電池電壓

        底盤每 11 個控制迴圈發一次，實測約 9 Hz。電壓在負載下第二位小數一直抖，
        每則都寫進 state 就等於每則都遞增版本號、把整包狀態透過 WebSocket 推給
        每一台連線的裝置。畫面上這是一個「12.4 V」的字，1 Hz 完全夠。
        """
        now = time.monotonic()
        if now - self._last_voltage_at < 1.0:
            return
        self._last_voltage_at = now
        self.state.set_system(voltage=round(float(msg.data), 2))

    def access_urls(self) -> List[str]:
        """平板可以打開的網址清單

        綁定到特定位址時就只有那一個入口，不必猜；綁在 0.0.0.0 才需要偵測。
        """
        if self.host not in ("0.0.0.0", "::", ""):
            return [f"http://{self.host}:{self.port}"]
        return [f"http://{ip}:{self.port}" for ip in detect_lan_ips()]

    # ── ROS 服務／動作呼叫（阻塞版，給執行緒池用）─────────

    def _call_service(self, name: str, request: Any, timeout: Optional[float] = None) -> Any:
        """同步呼叫服務並回傳 response

        這個函式會阻塞，必須從 asyncio 的執行緒池（asyncio.to_thread）呼叫，
        不能直接在事件迴圈裡跑。
        """
        client = self.service_clients[name]
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"服務 {name} 未就緒（對應節點沒啟動？）")

        limit = self.service_timeout if timeout is None else timeout
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=limit):
            future.cancel()
            raise TimeoutError(f"服務 {name} 呼叫逾時（{limit:.0f} 秒）")
        return future.result()

    @staticmethod
    def _local_ros_processes() -> tuple:
        """掃 /proc 取得本機 ROS 行程的完整指令列與執行檔名

        ROS 2 的圖 API 不提供「節點在哪台機器」，只能反過來從本機行程推。
        回傳 (cmdline 字串集合, 執行檔基本名集合)。
        """
        cmdlines, exes = set(), set()
        try:
            pids = os.listdir("/proc")
        except OSError:
            return cmdlines, exes

        for pid in pids:
            if not pid.isdigit():
                continue
            try:
                with open(f"/proc/{pid}/cmdline", "rb") as fh:
                    parts = [p for p in fh.read().decode(errors="replace").split("\0") if p]
            except OSError:
                continue
            if not parts:
                continue
            joined = " ".join(parts)
            if "/opt/ros/" not in joined and "/install/" not in joined:
                continue
            cmdlines.add(joined)
            for token in parts[:4]:
                if token and not token.startswith("-"):
                    base = os.path.basename(token)
                    # python3／ros2 這種包裝行程的名字對判定沒有幫助
                    if base not in ("python3", "python", "ros2", "sh", "bash"):
                        exes.add(base)
        return cmdlines, exes

    def _device_of(self, name: str, cmdlines: set, exes: set) -> str:
        """判斷節點跑在本機（pi）還是其他裝置（edge）

        判定順序：參數釘死 > 自己 > 內部節點不猜 > 本機行程比對 > 其餘視為遠端。
        """
        if name in self.pi_nodes:
            return "pi"
        if name in self.edge_nodes:
            return "edge"
        if name == self.get_name():
            return "pi"
        if INTERNAL_NODE_RE.match(name):
            return "unknown"
        # launch 指定名稱時 cmdline 會有 __node:=<name>
        if any(name in c for c in cmdlines):
            return "pi"
        # 節點名常等於執行檔名或多／少一個 _node 後綴
        for exe in exes:
            if len(exe) < 5:
                continue
            if exe == name or name.startswith(exe) or exe.startswith(name):
                return "pi"
        return "edge"

    def _node_status(self) -> List[Dict[str, Any]]:
        """列出圖上所有節點，lifecycle 節點另外回報生命週期狀態

        只看節點在不在會誤導：實測 bt_navigator 停在 inactive、amcl 停在
        unconfigured 時節點都「存在」，但 navigate_to_pose 根本沒有 server。
        所以凡是 lifecycle 節點都要看它是不是 active。

        哪些是 lifecycle 節點不寫死，而是看圖上有沒有對應的 get_state 服務——
        這樣 slam_toolbox 之類的也會自動被納入。

        這個函式會阻塞（要等服務回應），必須從執行緒池呼叫。
        """
        now = time.time()
        with self._nodes_lock:
            if self._nodes_cache and now - self._nodes_cache_at < 2.0:
                return self._nodes_cache

        # bare name -> 完整路徑。預期清單用 bare name 比對，但顯示要用完整路徑，
        # 否則 /x10/lslidar_driver_node 會被寫成 lslidar_driver_node 而看不出在哪個命名空間。
        present: Dict[str, str] = {}
        try:
            for name, ns in self.get_node_names_and_namespaces():
                present[name] = name if ns in ("", "/") else f"{ns.rstrip('/')}/{name}"
        except Exception as e:
            self.get_logger().warn(f"讀取節點清單失敗: {e}", throttle_duration_sec=30.0)

        # 從服務清單反推哪些是 lifecycle 節點
        lifecycle: Dict[str, str] = {}
        try:
            for srv, types in self.get_service_names_and_types():
                if srv.endswith("/get_state") and any(t.endswith("GetState") for t in types):
                    lifecycle[srv[: -len("/get_state")].split("/")[-1]] = srv
        except Exception as e:
            self.get_logger().warn(f"讀取服務清單失敗: {e}", throttle_duration_sec=30.0)

        # 一次把所有請求送出去再一起等。逐一等待的話，十幾個節點各等 1 秒
        # 會讓這支端點慢到不能用。
        pending: Dict[str, Any] = {}
        for name, srv in lifecycle.items():
            if name not in present:
                continue
            client = self._state_clients.get(srv)
            if client is None:
                client = self.create_client(GetState, srv, callback_group=self._cb)
                self._state_clients[srv] = client
            if client.service_is_ready():
                pending[name] = client.call_async(GetState.Request())

        deadline = time.time() + 1.5
        while time.time() < deadline and not all(f.done() for f in pending.values()):
            time.sleep(0.05)

        # 用完就拆掉。這些 get_state 客戶端原本是「建了就永久留著」，導航跑起來
        # 時 lifecycle 節點有十幾個，等於在執行器的 wait set 裡永久多出十幾個實體——
        # 而 rclpy 每一輪等待都要重新走過所有實體、逐一 enter 它們的 handle。
        # 健康頁是使用者按按鈕才刷新的，每次重建的成本遠低於一直掛著。
        # 用 popitem 而不是先 items() 再 clear()：兩個健康頁請求同時進來時
        # 每個客戶端只會被其中一邊取走一次，不會重複銷毀。
        while self._state_clients:
            try:
                _srv, client = self._state_clients.popitem()
                self.destroy_client(client)
            except Exception:  # 已被另一邊取走（KeyError）、或節點正在關閉
                pass

        cmdlines, exes = self._local_ros_processes()

        def row(name: str, group: str) -> Dict[str, Any]:
            label_name = present.get(name, name)
            # 沒在跑的節點談不上「在哪台機器」，另外歸類，不要塞進任一欄誤導
            device = "down" if name not in present else self._device_of(name, cmdlines, exes)
            base = {"name": label_name, "group": group, "device": device}
            if name not in present:
                return dict(base, state="未啟動", ok=False)
            if name not in lifecycle:
                return dict(base, state="執行中", ok=True)
            future = pending.get(name)
            if future is None or not future.done() or future.result() is None:
                return dict(base, state="無回應", ok=False)
            label = future.result().current_state.label
            return dict(base, state=label, ok=label == "active")

        rows: List[Dict[str, Any]] = []
        listed = set()
        for group, names in NODE_GROUPS:
            for name in names:
                rows.append(row(name, group))
                listed.add(name)

        # 沒列在預期清單裡、但確實在跑的節點（底盤、感測器、TF、launch 等）
        for name in sorted(set(present) - listed, key=lambda n: present[n]):
            rows.append(row(name, OTHER_GROUP))

        with self._nodes_lock:
            self._nodes_cache = rows
            self._nodes_cache_at = time.time()
        return rows

    # ── 管理者登入 ────────────────────────────────────────
    #
    # 這是「操作閘門」不是資安機制：共用平板上避免訪客誤按到使用者管理或
    # 導航，並讓管理端點需要登入才能呼叫。權杖存在記憶體、走明文 HTTP，
    # 同一個區網內有心人仍可側錄——不要把它當成真正的存取控制。

    def _issue_token(self) -> str:
        token = secrets.token_urlsafe(24)
        now = time.time()
        with self._token_lock:
            # 順手清掉過期的，免得長時間執行後無限增長
            self._tokens = {t: exp for t, exp in self._tokens.items() if exp > now}
            self._tokens[token] = now + self.admin_session_sec
        return token

    def _token_valid(self, token: str) -> bool:
        if not token:
            return False
        with self._token_lock:
            expiry = self._tokens.get(token)
            if expiry is None:
                return False
            if expiry <= time.time():
                del self._tokens[token]
                return False
        return True

    def _revoke_token(self, token: str) -> None:
        with self._token_lock:
            self._tokens.pop(token, None)

    def _resolve_admin_password(self) -> str:
        """決定管理者密碼，順序：launch 參數 > 環境變數 > 隨機生成

        密碼不寫死在原始碼裡：這個 repo 是公開的，寫死等於公開密碼，
        而且大家很容易拿平常在用的密碼當預設值（這正是先前的情況——
        預設值跟 Pi 的 SSH 密碼是同一組）。

        三個來源都沒有時**不會拒絕啟動**，而是隨機生成一組印在 log 上。
        車子在現場沒網路、沒鍵盤時，「起不來」比「密碼要去 log 裡看」麻煩得多。
        """
        pw = self.get_parameter("admin_password").get_parameter_value().string_value
        if pw:
            return pw

        pw = os.environ.get("SMARTNAV_ADMIN_PASSWORD", "")
        if pw:
            self.get_logger().info("管理者密碼來自環境變數 SMARTNAV_ADMIN_PASSWORD")
            return pw

        # token_urlsafe 會產生 URL 安全字元，不會有需要跳脫的符號，平板上好輸入
        pw = secrets.token_urlsafe(6)
        self.get_logger().warn(
            "\n"
            "════════════════════════════════════════════\n"
            " 未指定管理者密碼，已隨機生成一組：\n"
            f"     帳號 {self.admin_username}    密碼 {pw}\n"
            "\n"
            " 這組密碼每次重啟都會變。要固定的話擇一：\n"
            "   ros2 launch ... admin_password:=你的密碼\n"
            "   export SMARTNAV_ADMIN_PASSWORD=你的密碼\n"
            "════════════════════════════════════════════"
        )
        return pw

    def _check_credentials(self, username: str, password: str) -> bool:
        # compare_digest 避免用字串比較洩漏長度/前綴資訊
        return secrets.compare_digest(username, self.admin_username) and secrets.compare_digest(
            password, self.admin_password
        )

    def _fail_registration(self, name: str, message: str, collected: int = 0) -> None:
        self.state.set_registration(
            {
                "status": "failed",
                "user_name": name,
                "message": message,
                "collected": collected,
                "finished_at": time.time(),
            }
        )

    def _registration_progress_cb(self, msg: RegistrationProgress) -> None:
        """user_auth_node 每收一張樣本就發一次，取代原本每秒輪詢 list_users"""
        payload: Dict[str, Any] = {
            "status": msg.status,
            "user_name": msg.user_name,
            "user_uuid": msg.user_uuid,
            "collected": int(msg.collected_samples),
            "target": int(msg.target_samples),
            "message": msg.message,
        }
        if msg.active:
            payload["finished_at"] = 0.0
        else:
            payload["finished_at"] = time.time()
        self.state.merge_registration(payload)

        # 採樣期間名單的 num_samples 一直在變，結束時再刷一次讓前端拿到定案值
        if not msg.active and msg.status == "succeeded":
            self.get_logger().info(f"註冊完成: {msg.message}")

    def _run_registration(self, name: str, user_type: int, description: str, num_samples: int) -> None:
        """在背景執行緒發起人臉註冊，並看守它有沒有正常結束

        register_face 服務只負責「開始」——它同步建好使用者資料列就立刻回應，
        樣本是之後由 user_auth_node 在人臉回呼裡一張張累積的。實際進度由
        registration_progress 話題推送（見 _registration_progress_cb），
        這裡只負責發起與看門狗。
        """
        try:
            request = RegisterFace.Request()
            request.user_name = name
            request.user_type = UserType(type=user_type)
            request.description = description
            request.num_samples = num_samples
            res = self._call_service("register_face", request)
            if not bool(res.success):
                self._fail_registration(name, res.message)
                return
        except Exception as e:
            self.get_logger().error(f"人臉註冊錯誤: {e}")
            self._fail_registration(name, f"註冊失敗: {e}")
            return

        # 進度由 registration_progress 話題推送，這裡只當看門狗：
        # user_auth_node 若中途掛掉就不會再有任何訊息，狀態會永遠卡在 running。
        deadline = time.time() + self.REG_TIMEOUT_SEC + 5.0
        while time.time() < deadline:
            time.sleep(0.5)
            if not self.state.registration_active():
                return

        self._fail_registration(
            name, "註冊逾時且沒有收到進度回報，請確認 user_auth_node 是否正常"
        )

    def _start_action(self, name: str, goal: Any, label: str) -> str:
        """送出動作目標，回傳 job_id；實際等待在背景執行緒進行"""
        job_id = f"job{next(self._job_counter)}"
        self.state.set_job(
            job_id,
            action=name,
            label=label,
            status="pending",
            message="等待動作伺服器接受目標…",
            started_at=time.time(),
        )
        threading.Thread(target=self._run_action, args=(job_id, name, goal), daemon=True).start()
        return job_id

    def _run_action(self, job_id: str, name: str, goal: Any) -> None:
        """在背景執行緒跑完整個動作生命週期並持續更新 job 狀態"""
        client = self.action_clients[name]
        try:
            if not client.wait_for_server(timeout_sec=3.0):
                self.state.set_job(job_id, status="failed", message=f"動作 {name} 未就緒（對應節點沒啟動？）")
                return

            send_future = client.send_goal_async(goal)
            goal_handle = self._await_future(send_future, timeout=10.0)
            if goal_handle is None or not goal_handle.accepted:
                self.state.set_job(job_id, status="rejected", message="動作目標被拒絕")
                return

            with self._job_handles_lock:
                self._job_handles[job_id] = goal_handle
            self.state.set_job(job_id, status="running", message="執行中…")

            result_future = goal_handle.get_result_async()
            # 動作本身有自己的逾時（建圖 400 秒、全域定位 200 秒、導航 100 秒），
            # 這裡的上限只是為了在動作伺服器整個掛掉時不要留下永遠不結束的執行緒。
            wrapped = self._await_future(result_future, timeout=ACTION_TIMEOUTS.get(name, 300.0))
            result = getattr(wrapped, "result", None)
            status = getattr(wrapped, "status", None)

            if status == GoalStatus.STATUS_CANCELED:
                self.state.set_job(job_id, status="cancelled", message="已取消")
                return

            success = bool(getattr(result, "success", False))
            message = getattr(result, "message", "") or ("完成" if success else "失敗")
            self.state.set_job(job_id, status="succeeded" if success else "failed", message=message)
        except Exception as e:
            self.state.set_job(job_id, status="failed", message=f"動作執行錯誤: {e}")
            self.get_logger().error(f"動作 {name} 執行錯誤: {e}")
        finally:
            with self._job_handles_lock:
                self._job_handles.pop(job_id, None)

    def cancel_job(self, job_id: str) -> bool:
        """要求取消進行中的動作"""
        with self._job_handles_lock:
            goal_handle = self._job_handles.get(job_id)
        if goal_handle is None:
            return False
        goal_handle.cancel_goal_async()
        self.state.set_job(job_id, status="cancelling", message="取消中…")
        return True

    # ------------------------------------------------------------------
    # 遙控建圖
    # ------------------------------------------------------------------
    # 速度上限。遙控建圖要慢——slam_toolbox 的 minimum_travel_distance 是 0.2 m，
    # 開太快等於每兩幀之間跳過一大段，scan matching 會失準。
    # 也刻意比導航時的 vx_max (0.25) 更保守。
    TELEOP_MAX_LINEAR = 0.18
    TELEOP_MAX_ANGULAR = 0.45

    def apply_teleop(self, linear: float, angular: float) -> None:
        """套用一次遙控指令並重置看門狗"""
        try:
            lin = float(linear)
            ang = float(angular)
        except (TypeError, ValueError):
            return
        # 不信任前端傳來的數值：夾在安全範圍內，也擋掉 NaN
        if lin != lin or ang != ang:  # NaN
            lin = ang = 0.0
        lin = max(-self.TELEOP_MAX_LINEAR, min(self.TELEOP_MAX_LINEAR, lin))
        ang = max(-self.TELEOP_MAX_ANGULAR, min(self.TELEOP_MAX_ANGULAR, ang))

        with self._teleop_lock:
            self._teleop_linear = lin
            self._teleop_angular = ang
            self._teleop_last_cmd = time.monotonic()
            self._teleop_active = True

        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.teleop_pub.publish(msg)

    def _teleop_tick(self) -> None:
        """看門狗：太久沒收到新指令就把車子停住。

        這是手機遙控的安全核心。連線中斷、鎖螢幕、切 App、瀏覽器分頁被回收，
        都會讓指令停止送達卻沒有任何「停止」訊息 —— 只靠「收到停止才停」
        會讓車子在失聯後繼續跑。
        """
        with self._teleop_lock:
            if not self._teleop_active:
                return
            idle = time.monotonic() - self._teleop_last_cmd
            if idle < self._teleop_watchdog_sec:
                return
            self._teleop_active = False
            self._teleop_linear = 0.0
            self._teleop_angular = 0.0

        # 連送三次零速：DDS 掉一則封包不能變成「車子繼續跑」
        stop = Twist()
        for _ in range(3):
            self.teleop_pub.publish(stop)
        self.get_logger().warn(f"遙控逾時 {idle:.1f} 秒未收到指令，已停車")

    def set_lidar_rear_mask(self, enabled: bool, half_angle_deg: float) -> tuple:
        """開關雷達後方扇形遮罩

        lslidar 驅動的 angle_disable_min/max 單位是 0.01 度，
        裁掉的是 [min, max] 這個區間。車尾是 180 度。
        """
        try:
            half = max(5.0, min(80.0, float(half_angle_deg)))
        except (TypeError, ValueError):
            half = 35.0

        if enabled:
            lo = int((180.0 - half) * 100)
            hi = int((180.0 + half) * 100)
        else:
            # 兩個都設 0 = 不裁切（驅動的預設值）
            lo = hi = 0

        node_name = self.get_parameter("lidar_node_name").value
        results = []
        # 這兩個參數的型別是 integer_array 而不是 integer ——
        # 驅動支援同時遮蔽多段區間，所以即使只有一段也要包成陣列。
        for name, value in (("angle_disable_min", lo), ("angle_disable_max", hi)):
            ok = self._set_remote_int_array_param(node_name, name, [value])
            results.append(ok)

        if not all(results):
            return False, "設定雷達參數失敗（節點名稱可能不同）"

        # 參數寫得進去，但**驅動只在啟動時讀它** —— 實測寫入
        # angle_disable_min=[14500]、angle_disable_max=[21500] 之後，
        # 後方 43 束雷射仍然 100% 有回波，完全沒有被裁掉。
        # 所以這裡不能回報「已生效」，那會讓操作者以為自己被遮住了而放心站在車後。
        if enabled:
            return True, (
                f"參數已寫入（車尾 ±{half:.0f}°），但雷達驅動只在啟動時讀取，"
                "**本次不會生效**。要真的遮蔽請重啟感測器。"
                "在那之前請不要站在車子正後方。"
            )
        return True, "已清除遮罩參數（下次啟動感測器時生效）"

    def _set_remote_int_array_param(self, node_name: str, param: str, values: list) -> bool:
        """對別的節點設定一個整數陣列參數"""
        client = self.create_client(SetParameters, f"{node_name}/set_parameters")
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn(f"{node_name} 的參數服務不存在")
                return False
            req = SetParameters.Request()
            p = Parameter()
            p.name = param
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                integer_array_value=[int(v) for v in values],
            )
            req.parameters = [p]
            res = self._await_future(client.call_async(req), timeout=5.0)
            return bool(res and res.results and res.results[0].successful)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"設定 {node_name}/{param} 失敗: {exc}")
            return False
        finally:
            self.destroy_client(client)

    @staticmethod
    def _await_future(future, timeout: Optional[float]) -> Any:
        """阻塞等待 rclpy future

        用 threading.Event 而不是輪詢——future 的完成由 executor 執行緒觸發，
        在這裡自旋只是白燒 CPU（Pi4 上 load average 本來就吃緊）。
        """
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            raise TimeoutError("動作等待逾時")
        return future.result()

    # ── FastAPI 應用 ──────────────────────────────────────

    def _build_app(self) -> FastAPI:  # noqa: C901 - 路由表本來就長
        app = FastAPI(title="SmartNav HMI", docs_url=None, redoc_url=None)
        web_dir = Path(get_package_share_directory("smartnav_hmi")) / "web"

        if (web_dir / "static").is_dir():
            app.mount("/static", StaticFiles(directory=str(web_dir / "static")), name="static")

        @app.exception_handler(StarletteHTTPException)
        async def http_error(_request: Request, exc: StarletteHTTPException) -> JSONResponse:
            """前端只認 success/message 兩個欄位，別讓 FastAPI 丟原生的 detail"""
            return JSONResponse(
                {"success": False, "message": str(exc.detail)}, status_code=exc.status_code
            )

        # ── 管理者登入 ────────────────────────────────────

        def require_admin(authorization: str = Header(default="")) -> None:
            """保護管理端點。權杖從 Authorization: Bearer <token> 取得"""
            token = authorization[7:].strip() if authorization.lower().startswith("bearer ") else ""
            if not self._token_valid(token):
                raise HTTPException(status_code=401, detail="需要管理者登入")

        admin_only = [Depends(require_admin)]

        @app.post("/api/login")
        async def api_login(req: LoginRequest) -> JSONResponse:
            if not self._check_credentials(req.username, req.password):
                self.get_logger().warn(f"管理者登入失敗（帳號 {req.username!r}）")
                return JSONResponse(
                    {"success": False, "message": "帳號或密碼錯誤"}, status_code=401
                )
            token = self._issue_token()
            self.get_logger().info("管理者登入成功")
            return JSONResponse(
                {
                    "success": True,
                    "message": "登入成功",
                    "token": token,
                    "expires_in": self.admin_session_sec,
                }
            )

        @app.post("/api/logout")
        async def api_logout(authorization: str = Header(default="")) -> JSONResponse:
            token = authorization[7:].strip() if authorization.lower().startswith("bearer ") else ""
            self._revoke_token(token)
            return JSONResponse({"success": True, "message": "已登出"})

        @app.get("/api/session")
        async def api_session(authorization: str = Header(default="")) -> JSONResponse:
            """前端重新整理後用這支確認手上的權杖還有效"""
            token = authorization[7:].strip() if authorization.lower().startswith("bearer ") else ""
            return JSONResponse({"success": True, "admin": self._token_valid(token)})

        # ── 頁面與狀態 ────────────────────────────────────

        @app.get("/", response_class=HTMLResponse)
        async def index() -> HTMLResponse:
            index_path = web_dir / "index.html"
            if not index_path.exists():
                return HTMLResponse("<h1>找不到 index.html</h1>", status_code=500)
            return HTMLResponse(index_path.read_text(encoding="utf-8"))

        @app.get("/api/state")
        async def api_state() -> JSONResponse:
            return JSONResponse(self.state.snapshot())

        @app.get("/api/health")
        async def api_health() -> JSONResponse:
            with self._frame_lock:
                has_frame = self._latest_jpeg is not None
            with self._map_lock:
                has_map = self._map_png is not None
            services = {name: c.service_is_ready() for name, c in self.service_clients.items()}
            actions = {name: c.server_is_ready() for name, c in self.action_clients.items()}
            # 查 lifecycle 會阻塞，丟去執行緒池免得卡住事件迴圈（影像串流也在上面跑）
            nodes = await asyncio.to_thread(self._node_status)
            return JSONResponse(
                {
                    "ok": True,
                    "node": self.get_name(),
                    "urls": self.access_urls(),
                    "camera": has_frame,
                    "map": has_map,
                    "services": services,
                    "actions": actions,
                    "nodes": nodes,
                    "host": socket.gethostname(),
                }
            )

        @app.websocket("/ws")
        async def ws_endpoint(websocket: WebSocket) -> None:
            await websocket.accept()
            last_version = -1
            last_messages_version = None  # None 代表第一次，會帶上完整對話
            try:
                while True:
                    current = self.state.version
                    if current != last_version:
                        last_version = current
                        payload = self.state.snapshot(since_messages_version=last_messages_version)
                        last_messages_version = payload["messages_version"]
                        await websocket.send_text(json.dumps(payload, ensure_ascii=False))
                    await asyncio.sleep(0.1)
            except WebSocketDisconnect:
                pass
            except Exception as e:
                self.get_logger().debug(f"WebSocket 結束: {e}")

        # ── 影像與地圖 ────────────────────────────────────

        @app.get("/video")
        async def video() -> StreamingResponse:
            return StreamingResponse(
                self._mjpeg_generator(),
                media_type="multipart/x-mixed-replace; boundary=frame",
            )

        @app.get("/api/map.png")
        async def api_map_png() -> Response:
            self.note_map_interest()
            with self._map_lock:
                png = self._map_png
            if png is None:
                return JSONResponse({"error": "尚未收到地圖"}, status_code=404)
            # 前端用 ?v=<version> 破快取，這裡明確禁止快取避免拿到舊圖
            return Response(content=png, media_type="image/png", headers={"Cache-Control": "no-store"})

        @app.get("/api/map/meta")
        async def api_map_meta() -> JSONResponse:
            self.note_map_interest()
            meta = self.state.map_meta_copy()
            if meta is None:
                return JSONResponse({"error": "尚未收到地圖"}, status_code=404)
            return JSONResponse(meta)

        @app.post("/api/map/watch")
        async def api_map_watch() -> JSONResponse:
            """地圖分頁的心跳

            位姿訂閱（amcl_pose／TF）與地圖 PNG 渲染都很貴，而且只有地圖分頁
            看得到。前端在地圖分頁時每 5 秒打一次這支，後端才開這些來源；
            切走或關掉頁面就不再有心跳，十幾秒後自動全部收掉。

            刻意不設成管理端點：它不吐任何資料，只是一個「有人在看」的訊號，
            要求權杖只會讓沒登入的頁面看不到地圖更新。
            """
            self.note_map_interest()
            return JSONResponse({"success": True})

        # ── 地圖管理 ──────────────────────────────────────

        @app.get("/api/maps", dependencies=admin_only)
        async def api_maps() -> JSONResponse:
            def work():
                res = self._call_service("list_maps", ListMaps.Request())
                return {
                    "success": bool(res.success),
                    "message": res.message,
                    "maps": [{"map_id": m.map_id, "map_name": m.map_name} for m in res.maps_info],
                }

            return await self._guard(work)

        @app.post("/api/maps/switch", dependencies=admin_only)
        async def api_switch_map(req: SwitchMapRequest) -> JSONResponse:
            def work():
                request = SwitchMap.Request()
                request.map_id = req.map_id
                res = self._call_service("switch_map", request)
                return {"success": bool(res.success), "message": res.message}

            return await self._guard(work)

        @app.post("/api/maps/create", dependencies=admin_only)
        async def api_create_map(req: CreateMapRequest) -> JSONResponse:
            if not req.map_name.strip():
                return JSONResponse({"success": False, "message": "請提供地圖名稱"}, status_code=400)
            goal = CreateMap.Goal()
            goal.map_name = req.map_name.strip()
            job_id = self._start_action("create_map", goal, f"建立地圖「{goal.map_name}」")
            return JSONResponse({"success": True, "message": "建圖已開始", "job_id": job_id})

        # ── 地點與導航 ────────────────────────────────────

        @app.post("/api/maps/finish", dependencies=admin_only)
        async def api_finish_map() -> JSONResponse:
            """結束建圖並存檔（遙控建圖用）

            自動探索模式下 map_service_cc 會自己在收到 /exploration_complete
            之後存檔；遙控建圖沒有那個事件，要由操作者按下按鈕才知道走完了。
            """
            client = self.service_clients.get("finish_map")
            if client is None or not client.wait_for_service(timeout_sec=3.0):
                return JSONResponse(
                    {"success": False, "message": "/finish_map 服務不存在（導航堆疊沒啟動？）"},
                    status_code=503,
                )
            try:
                res = self._await_future(client.call_async(Trigger.Request()), timeout=30.0)
            except Exception as exc:  # noqa: BLE001
                return JSONResponse({"success": False, "message": f"存檔失敗：{exc}"}, status_code=500)
            ok = bool(res and res.success)
            return JSONResponse(
                {"success": ok, "message": (res.message if res else "無回應")},
                status_code=200 if ok else 500,
            )

        @app.get("/api/waypoints", dependencies=admin_only)
        async def api_waypoints() -> JSONResponse:
            # 會打這支就代表地圖分頁被打開了。舊版前端沒有 /api/map/watch 心跳，
            # 靠這裡至少還能讓地圖先渲染出來。
            self.note_map_interest()

            def work():
                res = self._call_service("list_waypoints", ListWaypoints.Request())
                waypoints = []
                for w in res.waypoints_info:
                    q = w.pose.orientation
                    waypoints.append(
                        {
                            "waypoint_id": w.waypoint_id,
                            "waypoint_name": w.waypoint_name,
                            "map_id": w.map_id,
                            "x": w.pose.position.x,
                            "y": w.pose.position.y,
                            "yaw": quaternion_to_yaw(q.x, q.y, q.z, q.w),
                        }
                    )
                return {"success": bool(res.success), "message": res.message, "waypoints": waypoints}

            return await self._guard(work)

        @app.post("/api/waypoints", dependencies=admin_only)
        async def api_create_waypoint(req: CreateWaypointRequest) -> JSONResponse:
            if not req.waypoint_name.strip():
                return JSONResponse({"success": False, "message": "請提供地點名稱"}, status_code=400)

            def work():
                request = CreateWaypoint.Request()
                request.waypoint_name = req.waypoint_name.strip()
                request.use_given_pose = bool(req.use_given_pose)
                if req.use_given_pose:
                    request.pose = make_pose(req.x, req.y, req.yaw)
                res = self._call_service("create_waypoint", request)
                return {"success": bool(res.success), "message": res.message}

            return await self._guard(work)

        @app.delete("/api/waypoints/{waypoint_id}", dependencies=admin_only)
        async def api_delete_waypoint(waypoint_id: str) -> JSONResponse:
            """刪除地點

            走 waypoint_service 的 delete_waypoint 服務而不是自己動 json 檔：
            地點資料庫是那個節點的記憶體狀態，繞過它直接改檔案，下一次建點時
            它會用自己那份記憶體整份覆寫回去，剛刪掉的又活過來。
            """
            if DeleteWaypoint is None:
                return JSONResponse(
                    {
                        "success": False,
                        "message": "刪除地點需要重建 smartnav_msgs（colcon build --packages-select "
                                   "smartnav_msgs smartnav_navigation_cc）後重啟",
                    },
                    status_code=501,
                )

            def work():
                request = DeleteWaypoint.Request()
                request.waypoint_id = waypoint_id
                res = self._call_service("delete_waypoint", request)
                return {"success": bool(res.success), "message": res.message}

            return await self._guard(work)

        @app.post("/api/navigate", dependencies=admin_only)
        async def api_navigate(req: NavigateRequest) -> JSONResponse:
            goal = Navigate.Goal()
            if req.waypoint_id:
                goal.waypoint_id = req.waypoint_id
                label = f"導航到地點 {req.target_name or req.waypoint_id}"
            else:
                goal.use_target_pose = True
                goal.target_pose = make_pose(req.x, req.y, req.yaw)
                goal.target_name = req.target_name or "指定位置"
                label = f"導航到 ({req.x:.2f}, {req.y:.2f})"
            job_id = self._start_action("navigate", goal, label)
            return JSONResponse({"success": True, "message": "導航已開始", "job_id": job_id})

        @app.post("/api/localize", dependencies=admin_only)
        async def api_localize() -> JSONResponse:
            job_id = self._start_action("global_localization", GlobalLocalization.Goal(), "全域定位")
            return JSONResponse({"success": True, "message": "全域定位已開始", "job_id": job_id})

        # ── 遙控建圖 ─────────────────────────────────────

        @app.post("/api/teleop", dependencies=admin_only)
        async def api_teleop(req: TeleopRequest) -> JSONResponse:
            """收一次遙控指令。前端要持續送，停止送 = 車子停。

            刻意不做成「按下去開始、放開才停」的狀態機：那樣一旦放開的那則
            請求沒送達（手機遙控很常見），車子就會一直跑下去。
            改成前端持續心跳、後端逾時自動歸零，斷線就是最安全的狀態。
            """
            self.apply_teleop(req.linear, req.angular)
            return JSONResponse({"success": True})

        @app.post("/api/teleop/stop", dependencies=admin_only)
        async def api_teleop_stop() -> JSONResponse:
            """明確的停止。看門狗本來就會停，這條只是讓停止更即時。"""
            self.apply_teleop(0.0, 0.0)
            return JSONResponse({"success": True, "message": "已停止"})

        @app.post("/api/teleop/rearmask", dependencies=admin_only)
        async def api_teleop_rearmask(req: RearMaskRequest) -> JSONResponse:
            """開關雷達後方扇形遮罩。

            遙控建圖時操作者常常走在車子後方，會被雷達掃進去變成移動的假障礙，
            污染地圖也干擾 scan matching。lslidar 驅動本身支援
            angle_disable_min/max（單位 0.01 度），直接在驅動裡裁掉最便宜。
            """
            ok, msg = self.set_lidar_rear_mask(req.enabled, req.half_angle_deg)
            return JSONResponse({"success": ok, "message": msg}, status_code=200 if ok else 500)

        # ── 作業（長時間動作）─────────────────────────────

        @app.get("/api/jobs", dependencies=admin_only)
        async def api_jobs() -> JSONResponse:
            return JSONResponse({"jobs": self.state.jobs_copy()})

        @app.delete("/api/jobs/{job_id}", dependencies=admin_only)
        async def api_cancel_job(job_id: str) -> JSONResponse:
            ok = self.cancel_job(job_id)
            return JSONResponse(
                {"success": ok, "message": "已送出取消要求" if ok else "找不到進行中的作業"},
                status_code=200 if ok else 404,
            )

        # ── 使用者管理 ────────────────────────────────────

        @app.get("/api/users", dependencies=admin_only)
        async def api_users() -> JSONResponse:
            def work():
                res = self._call_service("list_users", ListUsers.Request())
                users = [
                    {
                        "user_uuid": u.user_uuid,
                        "user_name": u.user_name,
                        "user_type": int(u.user_type.type),
                        "user_type_name": USER_TYPE_NAMES.get(u.user_type.type, "GUEST"),
                        "description": u.description,
                        "created_at": u.created_at,
                        "num_samples": int(u.num_samples),
                    }
                    for u in res.users
                ]
                return {"success": bool(res.success), "message": res.message, "users": users}

            return await self._guard(work)

        @app.post("/api/users/register", dependencies=admin_only)
        async def api_register(req: RegisterRequest) -> JSONResponse:
            """立刻回應並在背景採樣

            不用 _guard 阻塞 20～30 秒，是因為註冊進度要讓「所有」裝置看得到：
            狀態寫進 self.state 後由 WebSocket 廣播，發起註冊的那台平板
            並不特別。
            """
            name = req.user_name.strip()
            if not name:
                return JSONResponse({"success": False, "message": "請提供使用者名稱"}, status_code=400)
            if self.state.registration_active():
                return JSONResponse(
                    {"success": False, "message": "已有註冊進行中，請等它結束"}, status_code=409
                )

            num_samples = max(1, int(req.num_samples))
            now = time.time()
            # 先擺一個 running 讓所有裝置立刻有反應；真正的進度與期限由
            # _run_registration 輪詢 list_users 後覆寫。
            self.state.set_registration(
                {
                    "status": "running",
                    "user_name": name,
                    "num_samples": num_samples,
                    "target": num_samples,
                    "collected": 0,
                    "started_at": now,
                    "deadline": now + self.REG_TIMEOUT_SEC,
                    "message": f"正在為「{name}」採樣，請正對鏡頭",
                }
            )
            threading.Thread(
                target=self._run_registration,
                args=(name, int(req.user_type), req.description, num_samples),
                daemon=True,
            ).start()
            return JSONResponse({"success": True, "message": f"開始為「{name}」註冊"})

        @app.get("/api/frame.jpg")
        async def api_frame() -> Response:
            """抓當下這一張影格。前端「從畫面拍照」用這支"""
            with self._frame_lock:
                jpeg = self._latest_jpeg
            if jpeg is None:
                return JSONResponse({"success": False, "message": "目前沒有相機影像"}, status_code=404)
            return Response(content=jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

        @app.post("/api/users/register-photo", dependencies=admin_only)
        async def api_register_photo(req: RegisterPhotoRequest) -> JSONResponse:
            """用現成照片註冊。與即時採樣不同，這支是同步的，做完才回應"""
            name = req.user_name.strip()
            if not name:
                return JSONResponse({"success": False, "message": "請提供使用者名稱"}, status_code=400)
            if not req.photos:
                return JSONResponse({"success": False, "message": "請至少提供一張照片"}, status_code=400)
            if len(req.photos) > MAX_PHOTOS:
                return JSONResponse(
                    {"success": False, "message": f"一次最多 {MAX_PHOTOS} 張照片"}, status_code=400
                )
            if self.state.registration_active():
                return JSONResponse(
                    {"success": False, "message": "已有註冊進行中，請等它結束"}, status_code=409
                )

            try:
                photos = [decode_photo(p) for p in req.photos]
            except ValueError as e:
                return JSONResponse({"success": False, "message": str(e)}, status_code=400)

            def work():
                request = RegisterFacePhoto.Request()
                request.user_name = name
                request.user_type = UserType(type=int(req.user_type))
                request.description = req.description
                request.photos = photos
                # 每張照片都要跑一次 InsightFace，Pi 上不快，給比預設寬鬆的時限
                res = self._call_service(
                    "register_face_photo", request, timeout=30.0 + 5.0 * len(photos)
                )
                return {
                    "success": bool(res.success),
                    "message": res.message,
                    "accepted": int(res.accepted),
                    "rejected": int(res.rejected),
                }

            return await self._guard(work)

        @app.put("/api/users/{user_uuid}", dependencies=admin_only)
        async def api_update_user(user_uuid: str, req: UpdateUserRequest) -> JSONResponse:
            def work():
                request = UpdateUser.Request()
                request.user_uuid = user_uuid
                request.user_name = req.user_name
                request.user_type = UserType(type=int(req.user_type))
                request.description = req.description
                res = self._call_service("update_user", request)
                return {"success": bool(res.success), "message": res.message}

            return await self._guard(work)

        @app.delete("/api/users/{user_uuid}", dependencies=admin_only)
        async def api_delete_user(user_uuid: str) -> JSONResponse:
            def work():
                request = DeleteUser.Request()
                request.user_uuid = user_uuid
                res = self._call_service("delete_user", request)
                return {"success": bool(res.success), "message": res.message}

            return await self._guard(work)

        # ── 對話 ──────────────────────────────────────────

        @app.post("/api/say")
        async def api_say(req: SayRequest) -> JSONResponse:
            """把網頁打的字丟進 user_text，等同於對機器人講話"""
            text = req.text.strip()
            if not text:
                return JSONResponse({"success": False, "message": "內容不可為空"}, status_code=400)
            self.user_text_pub.publish(String(data=text))
            return JSONResponse({"success": True, "message": "已送出"})

        @app.post("/api/speak")
        async def api_speak(req: SayRequest) -> JSONResponse:
            """跳過 LLM，直接讓機器人念出這段文字（測試 TTS 用）"""
            text = req.text.strip()
            if not text:
                return JSONResponse({"success": False, "message": "內容不可為空"}, status_code=400)
            self.speech_text_pub.publish(String(data=text))
            return JSONResponse({"success": True, "message": "已送出"})

        @app.post("/api/chat/clear")
        async def api_chat_clear() -> JSONResponse:
            """清除所有對話

            畫面上的對話紀錄與 LLM 的對話記憶要一起清，否則模型還會沿用
            上一位客戶的上下文回答下一位客戶。
            """
            self.state.clear_messages()
            self.clear_conversation_pub.publish(Empty())
            self.get_logger().info("🧹 已清除對話紀錄")
            return JSONResponse({"success": True, "message": "已清除對話"})

        return app

    async def _guard(self, work) -> JSONResponse:
        """把阻塞的服務呼叫丟到執行緒池，並把例外轉成前端看得懂的 JSON

        服務未就緒回 503（對應節點沒啟動），逾時回 504，其餘 500。
        前端只要看 success 欄位即可，不必解析錯誤字串。
        """
        try:
            payload = await asyncio.to_thread(work)
            return JSONResponse(payload)
        except TimeoutError as e:
            return JSONResponse({"success": False, "message": str(e)}, status_code=504)
        except RuntimeError as e:
            return JSONResponse({"success": False, "message": str(e)}, status_code=503)
        except Exception as e:
            self.get_logger().error(f"HTTP 處理錯誤: {e}")
            return JSONResponse({"success": False, "message": f"伺服器錯誤: {e}"}, status_code=500)

    # 沒有相機影像時，佔位圖的重送間隔（秒）
    PLACEHOLDER_INTERVAL_SEC = 1.0

    async def _mjpeg_generator(self):
        """產生 multipart MJPEG 串流

        相機關掉時（迎賓機目前的常態）原本照樣以 video_fps=12 一直重送同一張
        「NO CAMERA SIGNAL」佔位圖——每秒 12 次穿過 starlette 的 chunked 編碼，
        內容還完全一樣。這裡改成沒有真影格時降到 1 fps：畫面上看不出差別
        （本來就是靜態圖），HTTP 執行緒的工作量降到十二分之一。
        """
        interval = 1.0 / max(self.video_fps, 1.0)
        placeholder = self._placeholder_jpeg()
        while True:
            with self._frame_lock:
                frame = self._latest_jpeg
            payload = frame if frame is not None else placeholder
            yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
            yield str(len(payload)).encode()
            yield b"\r\n\r\n"
            yield payload
            yield b"\r\n"
            await asyncio.sleep(interval if frame is not None else self.PLACEHOLDER_INTERVAL_SEC)

    @staticmethod
    def _placeholder_jpeg() -> bytes:
        """相機還沒有資料時顯示的佔位圖——比讓 <img> 一直轉圈好判讀"""
        img = np.full((360, 640, 3), 30, dtype=np.uint8)
        cv2.putText(img, "NO CAMERA SIGNAL", (120, 190), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (90, 90, 200), 2)
        ok, buf = cv2.imencode(".jpg", img)
        return buf.tobytes() if ok else b""

    def _run_server(self) -> None:
        config = uvicorn.Config(self.app, host=self.host, port=self.port, log_level="warning")
        server = uvicorn.Server(config)
        try:
            server.run()
        except Exception as e:
            self.get_logger().error(f"HMI 伺服器結束: {e}")


def main(args=None):
    """人機介面伺服器節點進入點

    這裡原本是「建一個 MultiThreadedExecutor、add_node，然後呼叫 rclpy.spin(node)」，
    但 rclpy.spin() 會拿**全域的 SingleThreadedExecutor** 再 add_node 一次，
    而 Node.executor 的 setter 會把節點從前一個執行器移除——也就是說那個
    MultiThreadedExecutor 從來沒有真的跑過，一直都是單執行緒。

    這裡順著實際行為改成明確的單執行緒，而不是「修好」多執行緒：
      ・會阻塞的工作（服務呼叫、動作等待、健康檢查）本來就跑在 FastAPI 的
        執行緒池與各自的背景執行緒上，不需要執行器出手。
      ・MultiThreadedExecutor 每一則訊息都要多一次執行緒池派工，在 Pi4 上
        對高頻話題反而更貴。
    寫明白也避免下一個人以為多執行緒有在生效而依賴它。
    """
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
