#!/usr/bin/env python3
"""人機介面伺服器節點"""

import asyncio
import itertools
import json
import base64
import binascii
import math
import glob
import os
import re
import secrets
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import rclpy
import uvicorn
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from fastapi import Depends, FastAPI, Header, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from starlette.exceptions import HTTPException as StarletteHTTPException
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
# ★ 改名匯入：這個檔案第 17 行已經有 `from pathlib import Path`，
#   直接 `from nav_msgs.msg import Path` 會把它蓋掉，而 web_dir 那邊還在用。
from nav_msgs.msg import Path as RosPath
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
    DeleteMap,
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

# 教導-重現路徑（2026-08-01 新增），同樣容許還沒重建。
# 這五個是一組的：缺任何一個就把整組視為不可用，避免出現
# 「列表看得到但按了重播沒反應」這種更難查的半殘狀態。
try:
    from smartnav_msgs.srv import DeleteTaughtPath, ListTaughtPaths, PlanTaughtPath, RecordPath
    from smartnav_msgs.action import FollowTaughtPath
    TAUGHT_PATH_AVAILABLE = True
except ImportError:  # pragma: no cover
    DeleteTaughtPath = ListTaughtPaths = PlanTaughtPath = RecordPath = None
    FollowTaughtPath = None
    TAUGHT_PATH_AVAILABLE = False

# ★★ 2026-08-14 補上 BLACKLIST —— 它從 8/01 就存在於 UserType.msg，這裡卻漏了。★★
#
# 後果是**系統對評審說謊**：下面兩處都用 `.get(type, "GUEST")`，
# 所以辨識到黑名單時，平板橫幅會顯示成「訪客 GUEST」。
# 語音與 Telegram 通報其實都正確觸發了（bank_reception_node 讀的是原始
# uint8，不經過這張表），**只有畫面是錯的** —— 這種「後端對、前端錯」
# 最難發現，因為功能看起來是好的。
#
# 三個角色是整個專題的骨架（docs/完整故事線_報告骨架.md），
# 其中一個在平板上被標成另一個，是不能上場的。
USER_TYPE_NAMES = {
    UserType.GUEST: "GUEST",
    UserType.VIP: "VIP",
    UserType.ADMIN: "ADMIN",
    UserType.BLACKLIST: "BLACKLIST",
}

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
            # ★ 2026-08-10：這三個原本寫的是 map_service_node /
            #   waypoint_service_node / navigation_service_node —— 那是**舊套件**
            #   smartnav_navigation 的節點名，實機從不啟動，於是健康頁上
            #   永遠有三行紅色的「尚未啟動」，久了就被當背景雜訊忽略，
            #   真的有東西沒起來時反而看不出來。
            #   實機 nav_bringup_cc.launch.py 起的是 _cc 版：
            "map_service_cc_node",
            "waypoint_service_cc_node",
            "navigation_action_cc_node",
            "path_teach_cc_node",
            "scan_filter_cc_node",
            "steering_trim_cc_node",
            # 迎賓劇本沒起來時要看得出來——它是完整故事線的核心
            "bank_reception_node",
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
# 這些值必須**大於**對應節點自己的逾時。小於的話 HMI 會先放棄等待，
# 而車子還在動 —— 見下面 navigate 那條的血淚。
ACTION_TIMEOUTS = {
    "create_map": 480.0,          # 節點內建 400 秒
    "global_localization": 260.0,  # 節點內建 200 秒
    # ★★ 2026-08-12：160 -> 340（原註解「節點內建 100 秒」早就過期了）★★
    #
    # navigation_action_cc_node 的 navigation_timeout_sec 預設是 **300 秒**
    # （見該檔 :63，當初從 100 放大就是因為「阿克曼車要繞路又要倒車，
    # 100 秒常常只夠走到一半」）。HMI 這邊卻還停在替 100 秒配的 160。
    #
    # 超過 160 秒會發生什麼（連鎖三段，全部在展示中看得到）：
    #   1. _await_future 逾時 -> 作業被標成「失敗／動作執行錯誤」，
    #      但**車子還在開**，因為導航節點根本沒被通知
    #   2. finally 把 goal handle 從 _job_handles 移除
    #   3. ★ 此後按「緊急停止」找不到任何進行中的作業，只會回
    #      「已緊急停止（當時沒有進行中的作業）」—— 停不了正在跑的導航
    #
    # 而 160 秒非常容易超過：7 m 教導路徑 @ 0.15 m/s 已經 47 秒，
    # 再加上讓行等待（每次最多 20 秒）與脫困（三段），破 160 是常態。
    # 340 = 節點的 300 秒再加 40 秒餘裕，維持「HMI 永遠比節點晚放棄」。
    "navigate": 340.0,             # 節點內建 300 秒（navigation_timeout_sec）
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


def normalize_angle(a: float) -> float:
    """把角度收斂到 (-pi, pi]。

    ★ 2026-08-24：位姿外推用。不加這個的話 yaw 相減會在 ±pi 交界處
      跳出 2pi 的假位移 —— 車子在地圖上會瞬間轉半圈。
    """
    return math.atan2(math.sin(a), math.cos(a))


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
            # ── 上方列的延伸資訊（2026-08-10 加）──────────────
            # 全部維持 None 代表「沒有資料」，前端就顯示 —，
            # 這樣底盤沒跑的時候不會謊報成 0。
            "charging": None,        # 底盤 robot_charging_flag
            "charge_current": None,  # 底盤 robot_charging_current（安培）
            "speed": None,           # /odom 的前進速度（m/s，帶正負號）
            "cpu_temp": None,        # Pi 核心溫度（°C）——導航時會逼近降頻門檻
            "mem_avail_mb": None,    # ★ 可用記憶體。這個歸零才是真的會死機
            "mem_used_pct": None,
            "cpu_load": None,        # 1 分鐘平均負載；Pi 4 是四核，> 4 就是滿載
        }
        self.map_meta: Optional[Dict[str, Any]] = None
        self.robot_pose: Optional[Dict[str, float]] = None
        # 要畫在地圖上的規劃路徑（見 set_nav_path）
        self.nav_path: Optional[Dict[str, Any]] = None
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
            #
            # ★ 2026-08-10：加上 3 秒的時間窗 ★
            # 原本是無時限的「連續相同就丟」。但迎賓詞是**固定樣板**
            # （bank_reception_node:97「{name}貴賓您好，歡迎蒞臨…」），
            # 同一位 VIP 每次產生的字串完全一樣；而冷卻是 VIP 60 秒／訪客 300 秒，
            # 也就是「過了冷卻就該再迎賓一次」正是設計意圖。
            # 中間沒有別的訊息時，第二次迎賓詞會被這道閘直接丟掉 ——
            # `/speech_text` 看得到、對話卻不更新、平板也不會唸。
            # 錄影時反覆走進走出必定踩到。
            #
            # 3 秒遠小於 60 秒冷卻：同一輪 speech_text 與 llm_response 的重疊
            # 照樣擋得掉，隔一輪的迎賓詞則放行。
            if (self.messages and self.messages[-1]["role"] == role
                    and self.messages[-1]["text"] == text
                    and time.time() - self.messages[-1].get("ts", 0.0) < 3.0):
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

    def set_nav_path(self, source: str, pts: list) -> None:
        """更新要畫在地圖上的路徑（2026-08-14 新增）

        使用者要求：「我想要在 HMI 看到它規劃的路徑（按下導航或錄製重現）」。
        資料本來就存在，只是從來沒有被送到前端 ——
          `taught_path`  path_teach_cc 在重播開始與規劃完成時發（latched）
          `plan`         nav2 planner_server 的全域路徑

        ★ 這裡**不做座標轉換**。前端已經有 map_meta（resolution/origin）與現成的
          世界座標→像素轉換（畫機器人用的那個），所以送原始 map frame 的公尺值
          最單純，也不會因為換地圖而失效。

        ★ 空清單代表「清掉畫面上的路徑」（導航結束時）。
        """
        with self._lock:
            new = {"source": source, "points": pts} if pts else None
            old = self.nav_path
            # 點數與來源都一樣時就不算變 —— 這個東西會 5 Hz 進來，
            # 每次都 _bump() 會讓 WebSocket 一直推整包狀態。
            same = (
                (old is None and new is None)
                or (old is not None and new is not None
                    and old["source"] == new["source"]
                    and len(old["points"]) == len(new["points"]))
            )
            self.nav_path = new
            if not same:
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
                "nav_path": dict(self.nav_path) if self.nav_path else None,
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


class SetPoseRequest(BaseModel):
    """POST /api/localize/here —— 把定位直接設到一個已知位置

    為什麼需要這支（2026-08-06）：
    原本只有 `/api/localize`，它做的是 **GlobalLocalization**——把粒子撒滿
    整張圖再收斂。走廊沿長軸重複，同一份掃描在任何縱向位置都匹配得上，
    所以全域定位很可能收斂到**另一段走廊**。8/06 實測 AMCL 就沿縱向錯了
    3 公尺，而掃描吻合度還顯示 90.4%。
    而且全域定位的實作是 0.9 m 半徑的圓弧繞行——0.99 m 的走廊裡會撞牆。

    已知車子在哪（例如剛從起點出發、或人工推回起點）時，直接把位姿設過去
    再用 `/align_pose` 做局部修正，比讓它自己找可靠得多，也不需要移動。

    waypoint_id 有值就用該地點的座標；否則用 x/y/yaw。
    """

    waypoint_id: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    align: bool = True          # 設完之後是否呼叫 /align_pose 做掃描對齊


class NavigateRequest(BaseModel):
    """POST /api/navigate

    waypoint_id 有值時導航到既有地點；否則導航到 x/y/yaw 指定的座標。
    """

    waypoint_id: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    target_name: str = ""


class RecordPathRequest(BaseModel):
    """POST /api/paths/record —— 教導路徑的錄製控制"""

    action: str = "start"          # start / stop / cancel
    name: str = ""                 # stop 時必填


class FollowPathRequest(BaseModel):
    """POST /api/paths/follow —— 重播教導路徑"""

    path_id: str = ""
    name: str = ""                 # 只用於作業標籤的顯示
    reverse: bool = False          # 反向走完整條路徑（原路折返回起點）
    speed_scale: float = 0.0       # 0 或負值 = 用節點的預設速度


class PlanPathRequest(BaseModel):
    """POST /api/paths/plan —— 把地圖上點選的位置規劃成教導路徑"""

    name: str = ""
    start_from_robot: bool = True
    points: List[Dict[str, float]] = []    # [{"x":.., "y":.., "yaw":..}, ...]


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
        # ── 用戶端在場偵測（2026-08-17）──────────────────────
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
        self._hw_lock = threading.Lock()
        self._hw_cache: list = []
        self._hw_cache_at = 0.0
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

        # ── 用戶端在場追蹤（2026-08-17）────────────────────
        #
        # 起因：使用者說「HMI 已連接設備如果沒在連線的話可以踢掉連線 省 CPU」。
        #
        # ★ 實測（py-spy 取樣 hmi_server 主執行緒，20 秒、100 Hz）發現 CPU
        #   **不是**花在用戶端身上，而是花在 rclpy 的執行器上：
        #     99.0%  _spin_once_impl → wait_for_ready_callbacks
        #     77.6%  wait_set.wait()（DDS 等待本身）
        #      0.7%  executors.py:592 handler ← 我們自己的回呼只有這麼多
        #   也就是說幾乎全部的 CPU 都在「每次被喚醒就重建一次 wait set」。
        #   成本 ≈ 喚醒次數 × wait set 內的實體數，跟連了幾台平板無關。
        #
        # ★ 對照實驗（獨立小節點，同一台機器）：
        #     訂閱數 0、閒置服務客戶端 0  → 1.10%
        #     訂閱 /odom（22 Hz）        → 6.45%
        #     /odom + 20 個閒置客戶端     → 10.30%
        #     只有 40 個閒置客戶端、無訂閱 → 0.50%
        #   結論：**閒著的實體幾乎不要錢，被高頻話題叫醒才要錢。**
        #
        # 所以「省 CPU」的正解不是踢連線本身，而是「沒人在看就不要訂高頻話題」。
        # 這裡追蹤用戶端在場與否，讓 /odom（20 Hz，只為了顯示車速）與影像
        # 訂閱都掛在這個訊號上——跟既有的地圖／位姿按需訂閱同一個套路。
        self._client_lock = threading.Lock()
        self._ws_clients = 0           # 活著的 WebSocket 連線數
        self._video_viewers = 0        # 進行中的 /video MJPEG 串流數
        self._last_http_at = 0.0       # 最後一次 HTTP 請求（monotonic）
        self._last_frame_req_at = 0.0  # 最後一次 /api/frame.jpg（拍照用）
        self._clients_seen = False     # 上一輪的判定，只在變化時記 log
        self._odom_sub = None          # 按需建立，見 _client_tick
        self._image_sub = None         # 同上

        # ── 位姿來源（按需開啟）──────────────────────────
        # 有人在看地圖的期限（time.monotonic()）。0 = 沒人在看。
        self._pose_watch_until = 0.0
        # amcl_pose 訂閱與最近一次收到的位姿。None 代表還沒收到過。
        self._amcl_sub = None
        self._amcl_pose: Optional[Dict[str, float]] = None
        # ★★ 2026-08-24：里程計外推，修「HMI 顯示的位置落後現實」★★
        #
        # 使用者回報「HMI 顯示的車輛有時候不是實際位置」。根因是 amcl_pose
        # 是**上一次雷射更新時**的估計，而 AMCL 的更新門檻是
        # update_min_d 0.10 m / update_min_a 0.10 rad（nav2 yaml），
        # 也就是車子要移動 10 cm 或轉 5.7 度才會發下一則
        # -> 顯示最多落後 10 cm、5.7 度，0.15 m/s 時約 0.67 秒。
        #
        # 正解是 map→base_footprint 的 TF，但 rclpy 的 TF 訂閱在 Pi 4 上
        # 吃 19.4% CPU，本節點當初就是為此才改用 amcl_pose 的。
        #
        # 折衷：amcl_pose 當「map→odom 的修正」、/odom 做外推。
        # /odom 本來就已經訂了（_odom_speed_cb 拿它算車速），
        # 這裡只是在節流**之前**多存三個浮點數，成本可以忽略。
        self._odom_pose: Optional[Dict[str, float]] = None
        self._odom_at_amcl: Optional[Dict[str, float]] = None
        # 這一輪觀看是否已經拿到過初始位置（見 _pose_timer_cb）
        self._pose_seeded = False
        self._pose_lock = threading.Lock()
        # 電壓節流用（見 _voltage_cb）
        self._last_voltage_at = 0.0
        # 充電電流與車速也各自節流到 1 Hz（見對應 callback）
        self._last_charge_cur_at = 0.0
        self._last_speed_at = 0.0

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

        # 影像 QoS 留著給按需訂閱用（見 _acquire_image_sub）
        self._sensor_qos = sensor_qos

        # ── 訂閱 ──────────────────────────────────────────
        # ★ 影像不在這裡訂。相機開著時它是全節點最高頻的話題（30 Hz），
        #   而且不管有沒有人在看 /video 都照收、照 JPEG 編碼。改成
        #   「有 MJPEG 串流在跑時才訂」，見 _client_tick / _acquire_image_sub。
        self.create_subscription(UserIdentity, identity_topic, self._identity_cb, 10, callback_group=cb)
        self.create_subscription(
            RegistrationProgress, "registration_progress", self._registration_progress_cb, 10,
            callback_group=cb,
        )
        self.create_subscription(String, "user_text", self._user_text_cb, 10, callback_group=cb)
        self.create_subscription(String, "partial_text", self._partial_text_cb, 10, callback_group=cb)
        self.create_subscription(String, "llm_response", self._llm_response_cb, 10, callback_group=cb)
        self.create_subscription(String, "llm_stream", self._llm_stream_cb, 10, callback_group=cb)
        # ★ 2026-08-17：串流作廢訊號。llm_service_node 在「這一輪要去呼叫工具」時發，
        #   用來丟掉已經串到畫面上、但永遠不會成為正式回覆的那段旁白。
        #   不用 llm_response 代替：那會多出一則已完成訊息，前端會把它唸出來。
        self.create_subscription(
            Empty, "llm_stream_reset", self._llm_stream_reset_cb, 10, callback_group=cb
        )
        # 模型名稱是 latched，HMI 比 LLM 晚啟動也收得到
        self.create_subscription(String, "llm_model", self._llm_model_cb, latched_qos, callback_group=cb)
        self.create_subscription(String, "speech_text", self._speech_text_cb, 10, callback_group=cb)
        self.create_subscription(Bool, "playback_status", self._playback_cb, 10, callback_group=cb)
        # ★★ 2026-08-19：平板瀏覽器回報真實的 TTS 起訖 ★★
        #
        # 車上沒有喇叭，出聲是平板的 `speechSynthesis`，所以
        # `playback_status` 這個話題**從來沒有人發布過** ——
        # ASR 那端訂了它（speech_recognizer_node.py:262）卻永遠收不到，
        # 於是「機器人聽到自己講話」只能靠字數估時間（0.22 秒/字）擋。
        #
        # 估計有兩個真實的壞處：
        #   1. 100 字的回覆 -> 靜音 23 秒，這段期間客人講什麼都聽不到
        #   2. **平板沒連線時 TTS 根本沒響，卻照樣靜音** —— 白聾一場
        #
        # 瀏覽器的 SpeechSynthesisUtterance 有 onstart/onend，那是真訊號。
        # 讓它回報，這裡轉發成 ROS 話題，估計值降級成「收不到 onend 時的天花板」。
        self.playback_pub = self.create_publisher(Bool, "playback_status", 10)
        if self.enable_map:
            self.create_subscription(OccupancyGrid, map_topic, self._map_cb, latched_qos, callback_group=cb)
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
        self.create_subscription(Float32, "robot_charging_current", self._charge_current_cb, sensor_qos, callback_group=cb)
        # 車速取自 odom。上方列要回答的是「車子現在到底有沒有在動」——
        # 這在脫困、防撞死鎖那類問題上是第一個要看的量（發了指令但車不動）。
        #
        # ★ 2026-08-17：**改成按需訂閱**（見 _client_tick）。
        #   /odom 是底盤以 20 Hz 發的，而畫面上只需要 1 Hz 的一個數字——
        #   回呼裡本來就節流到 1 Hz 了，但**節流省不到重點**：訊息照樣抵達、
        #   照樣把執行器叫醒、照樣讓 rclpy 在 Python 端重建整個 wait set。
        #   實測這一條訂閱在本機要價約 6% 的一顆核心（見上面的對照實驗），
        #   而迎賓展示時九成的時間根本沒有平板連著。
        #   ★ 安全性不受影響：遙控看門狗用的是時間，不是 odom；
        #     緊急停止、防撞、導航都不經過 HMI 的這條訂閱。

        # 資源監看：★ 獨立計時器，不掛在任何話題上。
        # 掛話題的話「沒開底盤就沒有 CPU 資訊」，而最需要看的時候
        # （相機＋人臉＋LLM 全開、底盤沒開）剛好就是沒有底盤的時候。
        self.create_timer(5.0, self._resource_tick, callback_group=cb)

        # ── 規劃路徑（2026-08-14 新增，使用者要求在 HMI 上看得到）──
        #   taught_path  path_teach_cc 發的：規劃完成與重播開始時（latched）
        #   plan         nav2 planner_server 的全域路徑（走 MPPI 那條時才有）
        # 兩條都畫，用 source 欄位區分顏色。latched 讓晚開的平板也拿得到。
        self.create_subscription(
            RosPath, "taught_path",
            lambda m: self._nav_path_cb(m, "taught"), latched_qos, callback_group=cb)
        self.create_subscription(
            RosPath, "plan",
            lambda m: self._nav_path_cb(m, "nav2"), 10, callback_group=cb)

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
        # 控制指令用 RELIABLE + depth 1。
        #
        # depth=1 + KEEP_LAST 才是解決「指令過期」的關鍵：訂閱端稍慢時只留
        # 最新一則，不會像預設的 depth=10 那樣排隊重送幾百毫秒前的方向。
        #
        # ★ reliability 一定要 RELIABLE，不能圖方便用 BEST_EFFORT ★
        # 2026-07-31 實測：底盤 `wheeltec_robot` 訂閱 /cmd_vel 用的是 RELIABLE，
        # collision_monitor 訂閱 /cmd_vel_trimmed 也是。DDS 的相容規則是
        # 「發布端的保證不得低於訂閱端的要求」，所以 BEST_EFFORT 發布端對上
        # RELIABLE 訂閱端 **完全建立不了連線**，訊息一則都不會送達：
        #
        #     [WARN] New subscription discovered on topic 'cmd_vel',
        #            requesting incompatible QoS. No messages will be sent to it.
        #
        # 這就是「HMI 按了車子不動」的真因，跟速度、頻率、看門狗都無關。
        # 反方向是相容的（RELIABLE 發布端可以餵 BEST_EFFORT 訂閱端），
        # 所以發布端選 RELIABLE 對兩種訂閱者都通。
        #
        # 當初的 T1 驗證會過關是因為測試節點自己用 BEST_EFFORT 訂閱，
        # 只證明了「訊息有發出去」，沒證明「底盤收得到」。
        # 以後測資料流要訂在**真正的消費端**上，或直接比對 topic info -v 的
        # Reliability 欄位。
        teleop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.teleop_pub = self.create_publisher(
            Twist, self.get_parameter("teleop_cmd_topic").value, teleop_qos
        )
        # 中繼節點不在時的直通路徑，理由見 _publish_teleop
        self.teleop_direct_pub = self.create_publisher(Twist, "cmd_vel", teleop_qos)
        self._teleop_fallback_warned = False
        self._teleop_watchdog_sec = float(self.get_parameter("teleop_watchdog_sec").value)
        self._teleop_last_cmd = 0.0      # 最後一次收到遙控指令的時間 (monotonic)
        self._teleop_active = False      # 是否還需要送停止命令
        self._teleop_linear = 0.0
        self._teleop_angular = 0.0
        self._teleop_lock = threading.Lock()
        self._teleop_tick_count = 0
        self._teleop_tick_t0 = time.monotonic()
        self._teleop_loop_count = 0
        self._teleop_loop_t0 = time.monotonic()
        # 20 Hz 補送：這才是底盤的指令來源。HTTP 只負責更新設定值，
        # 網路抖動因此不會傳導成底盤的 1.0 秒逾時停車。
        #
        # **用獨立執行緒而不是 ROS timer。** 原本寫成
        #     self.create_timer(0.05, self._teleop_tick, callback_group=cb)
        # 但實測完全沒有作用：加了計數器之後，持續遙控 12 秒（20 Hz 應有 240 次）
        # 一次補送都沒發生，而同一個函式裡的看門狗分支卻會觸發 ——
        # 代表 timer 的實際間隔遠大於 0.6 秒，每次被執行到時都已經逾時了。
        # 這個節點的 SingleThreadedExecutor 要同時處理 HTTP 進來的服務呼叫、
        # 訂閱與動作回呼，0.05 秒的 timer 根本排不上。
        #
        # 遙控是安全相關的即時路徑，不能讓它跟一般回呼搶執行器。
        # 獨立執行緒不受執行器排程影響，而 rclpy 的 publish 本身可以跨執行緒呼叫。
        self._teleop_thread = threading.Thread(
            target=self._teleop_loop, name="teleop_resend", daemon=True
        )
        self._teleop_thread.start()

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
        # smartnav_msgs 還沒重建時 DeleteWaypoint 會是 None，此時不建客戶端，
        # 由端點回報要重建；其餘功能完全不受影響。
        if DeleteWaypoint is not None:
            self.service_clients["delete_waypoint"] = self.create_client(
                DeleteWaypoint, "delete_waypoint", callback_group=cb
            )
        # 教導-重現路徑。整組一起加或一起不加，理由見檔案上方的 import。
        if TAUGHT_PATH_AVAILABLE:
            self.service_clients["record_path"] = self.create_client(
                RecordPath, "record_path", callback_group=cb
            )
            self.service_clients["list_taught_paths"] = self.create_client(
                ListTaughtPaths, "list_taught_paths", callback_group=cb
            )
            self.service_clients["delete_taught_path"] = self.create_client(
                DeleteTaughtPath, "delete_taught_path", callback_group=cb
            )
            self.service_clients["plan_taught_path"] = self.create_client(
                PlanTaughtPath, "plan_taught_path", callback_group=cb
            )
        # ★ 2026-08-17：緊急停止用的「取消全部」服務客戶端，延遲建立（見 _cancel_all_action_goals）
        self._cancel_srv_clients: Dict[str, Any] = {}
        self._cancel_cb_group = cb
        self.action_clients: Dict[str, ActionClient] = {
            "create_map": ActionClient(self, CreateMap, "create_map", callback_group=cb),
            "navigate": ActionClient(self, Navigate, "navigate", callback_group=cb),
            "global_localization": ActionClient(
                self, GlobalLocalization, "global_localization", callback_group=cb
            ),
        }
        if TAUGHT_PATH_AVAILABLE:
            self.action_clients["follow_taught_path"] = ActionClient(
                self, FollowTaughtPath, "follow_taught_path", callback_group=cb
            )

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

        # 用戶端在場 → 高頻訂閱的開關（2026-08-17）
        self.create_timer(1.0, self._client_tick, callback_group=cb)

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

    # ── 用戶端在場 → 高頻訂閱的開關 ───────────────────────

    def note_client(self) -> None:
        """記下「剛剛有用戶端來過」。由 HTTP middleware 與 WebSocket 呼叫。"""
        with self._client_lock:
            self._last_http_at = time.monotonic()

    def note_video_interest(self) -> None:
        """記下「剛剛有人要影格」（/api/frame.jpg 拍照用）"""
        with self._client_lock:
            self._last_frame_req_at = time.monotonic()

    def clients_present(self) -> bool:
        """現在有沒有用戶端

        判定順序刻意是「先看活連線，再看時間」：
          ・有 WebSocket 活著 → 一定算在場。★ 這是不誤踢平板的關鍵：
            連線活不活由**協定層 pong** 決定（uvicorn 的 ws_ping_interval），
            瀏覽器網路層自動回覆，分頁在背景、JS 被節流都照回。
            「平板放著沒人操作」在這個判準下是「在場」。
          ・沒有 WebSocket 時退回看最後一次 HTTP。這是給「ws 斷了正在重連」
            的空窗用的，TTL 設得比前端重連退避上限（8 秒）寬很多。
        """
        with self._client_lock:
            if self._ws_clients > 0:
                return True
            return (time.monotonic() - self._last_http_at) < self.client_idle_ttl

    def _video_wanted(self) -> bool:
        """要不要維持影像訂閱

        兩個條件要**同時**成立：
          ・有 /video 串流在跑，或剛剛有人抓過影格
          ・而且確實有用戶端在場
        第二個條件不能省。MJPEG 是一條長連線，對端如果是「悄悄消失」
        （平板走出 WiFi 範圍、直接斷電），TCP 只會把資料塞進送出緩衝區，
        伺服器這邊要很久才會發現，_video_viewers 就一直掛著不歸零 ——
        一條這樣的殭屍串流就足以讓相機訂閱永遠關不掉。
        用 clients_present() 交叉驗證：那台平板的 WebSocket 會先被 pong 逾時
        判死，在場訊號跟著消失，串流再怎麼賴著也留不住訂閱。
        """
        if not self.clients_present():
            return False
        with self._client_lock:
            if self._video_viewers > 0:
                return True
            return (time.monotonic() - self._last_frame_req_at) < self.video_idle_ttl

    def _acquire_image_sub(self) -> None:
        if self._image_sub is not None:
            return
        if self.image_compressed:
            self._image_sub = self.create_subscription(
                CompressedImage, self.image_topic, self._compressed_image_cb,
                self._sensor_qos, callback_group=self._cb,
            )
        else:
            self._image_sub = self.create_subscription(
                Image, self.image_topic, self._image_cb, self._sensor_qos, callback_group=self._cb,
            )

    def _release_image_sub(self) -> None:
        if self._image_sub is None:
            return
        self.destroy_subscription(self._image_sub)
        self._image_sub = None
        # ★ 一定要把快取的影格丟掉。留著的話 /api/frame.jpg 會回一張
        #   不知道多久以前的照片 —— 人臉註冊拿它當樣本就會註冊到錯的人。
        with self._frame_lock:
            self._latest_jpeg = None
        self._frame_times = []
        self.state.set_system(camera_fps=0.0)

    def _acquire_odom_sub(self) -> None:
        if self._odom_sub is not None:
            return
        self._odom_sub = self.create_subscription(
            Odometry, "odom", self._odom_speed_cb, self._sensor_qos, callback_group=self._cb
        )

    def _release_odom_sub(self) -> None:
        if self._odom_sub is None:
            return
        self.destroy_subscription(self._odom_sub)
        self._odom_sub = None
        # 訂閱沒了就不會有新車速，明確清成 None，免得畫面停在一個舊數字
        self.state.set_system(speed=None)

    def _client_tick(self) -> None:
        """1 Hz：依用戶端在場與否開關高頻訂閱

        ★ 一定要在執行器執行緒上做（也就是 ROS timer 裡），不能在 HTTP
          執行緒上做：create_subscription / destroy_subscription 會動到執行器
          正在走訪的實體集合。
        1 Hz 是刻意的——它自己就是一個喚醒源，太快就把省下來的又吃回去。
        """
        present = self.clients_present()
        if present != self._clients_seen:
            self._clients_seen = present
            self.get_logger().info(
                "偵測到用戶端，恢復 /odom（車速）訂閱" if present
                else f"沒有任何用戶端連線（閒置 {self.client_idle_ttl:.0f} 秒），"
                     "停用 /odom 與影像訂閱以節省 CPU"
            )

        if present:
            self._acquire_odom_sub()
        else:
            self._release_odom_sub()

        if self._video_wanted():
            self._acquire_image_sub()
        else:
            self._release_image_sub()

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
            # ★ 記下「這一則 amcl 對應到哪個里程計位置」。之後的外推
            #   就是拿現在的里程計減掉這個基準。
            self._odom_at_amcl = dict(self._odom_pose) if self._odom_pose else None

    # 外推的上限。超過就代表里程計或 amcl 有一邊不對勁（打滑、重定位、
    # 里程計重置），寧可顯示「舊但正確」也不要顯示「新但編出來」的位置。
    EXTRAP_MAX_M = 1.0
    EXTRAP_MAX_RAD = 1.0

    def _extrapolate(self, amcl, odom_ref, odom_now) -> Dict[str, float]:
        """用里程計把 amcl 位姿外推到「現在」。

        ★ 為什麼需要（2026-08-24）
        `amcl_pose` 是**上一次雷射更新時**的估計。AMCL 的更新門檻是
        `update_min_d: 0.10` / `update_min_a: 0.10 rad`，也就是車子要移動
        10 cm 或轉 5.7 度才會發下一則 —— 顯示最多落後 10 cm、5.7 度，
        0.15 m/s 時約 0.67 秒，再加上雷射→濾波→發布的處理延遲。

        ★ 數學
        amcl 給的是 map→base，odom 給的是 odom→base。兩則 amcl 之間，
        map→odom 這個修正量是**固定的**（amcl 沒發新的就沒有新的修正），
        所以「現在的 map→base」＝「上次的 map→base」⊕「這段期間的 odom 位移」，
        而位移要先從 odom 座標系轉到 map 座標系（差一個 yaw）。

        ★ 使用者提過「粒子重採樣調快一點」—— **那個方向是錯的**：
          調快只是增加 CPU 與位姿抖動，而且**避障根本不看 AMCL**
          （`_body_margin` / `_forward_clearance` / `_arc_clearance`
          讀的都是即時 `/scan`）。顯示落後與避障安全是兩件事。
        """
        if not odom_ref or not odom_now:
            return dict(amcl)          # 沒有里程計就照舊，不要編
        dx = odom_now["x"] - odom_ref["x"]
        dy = odom_now["y"] - odom_ref["y"]
        dyaw = normalize_angle(odom_now["yaw"] - odom_ref["yaw"])
        if math.hypot(dx, dy) > self.EXTRAP_MAX_M or abs(dyaw) > self.EXTRAP_MAX_RAD:
            # 位移大到不合理：打滑、重定位、或里程計被重置過。
            # 這種時候 amcl 馬上就會發新的，等它就好。
            return dict(amcl)
        # 把 odom 座標系的位移轉進 map 座標系（兩者差 amcl.yaw - odom_ref.yaw）
        th = amcl["yaw"] - odom_ref["yaw"]
        c, sn = math.cos(th), math.sin(th)
        return {
            "x": amcl["x"] + c * dx - sn * dy,
            "y": amcl["y"] + sn * dx + c * dy,
            "yaw": normalize_angle(amcl["yaw"] + dyaw),
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
            odom_now = dict(self._odom_pose) if self._odom_pose else None
            odom_ref = dict(self._odom_at_amcl) if self._odom_at_amcl else None
        if pose is not None:
            # amcl 有資料就絕不碰 TF——這是整個改動的重點
            self._drop_tf_listener()
            self._pose_seeded = True
            self.state.set_robot_pose(self._extrapolate(pose, odom_ref, odom_now))
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

    def _llm_stream_reset_cb(self, msg: Empty) -> None:
        """作廢目前串流中的灰字（這一輪只是去呼叫工具，不是要講給客戶聽的答案）

        只清畫面緩衝，**不**動 _turn_started / _first_token_at ——
        那兩個是整輪（含工具執行時間）的計時起點，清掉的話最終回覆的
        think/generate/cps 統計就沒了。也不碰 _pending_speech，
        那條線本來就由 llm_response 或看門狗負責。
        """
        self.state.set_stream("")

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
        """把規劃路徑送給前端畫。

        ★ 降採樣：規劃路徑動輒 30~150 點、教導路徑可以到 20000 點
          （record_spacing 5 cm × 長路徑）。整包塞進 WebSocket 每次狀態推播
          都要重傳，Pi 4 撐不住也沒必要 —— 畫在 300 px 寬的地圖上，
          相鄰兩點差幾公分根本看不出來。
          取樣到最多 MAX_PATH_POINTS 點，**但保證留下最後一點**（終點位置
          是操作者最想確認的東西，被降採樣切掉就白畫了）。
        """
        pts = [(round(p.pose.position.x, 3), round(p.pose.position.y, 3))
               for p in msg.poses]
        if len(pts) > self.MAX_PATH_POINTS:
            step = len(pts) / float(self.MAX_PATH_POINTS)
            idx = sorted({int(i * step) for i in range(self.MAX_PATH_POINTS)})
            if idx[-1] != len(pts) - 1:
                idx.append(len(pts) - 1)
            pts = [pts[i] for i in idx]
        self.state.set_nav_path(source, pts)

    def _odom_speed_cb(self, msg: Odometry) -> None:
        """車速。1 Hz 節流（odom 本身是 20 Hz）。

        ★ 2026-08-14：CPU 溫度／負載已經搬到 _resource_tick 的獨立計時器。
          原本它們搭在這班車上，代價是「沒開底盤就沒有 CPU 資訊」——
          而 8/14 把整台機器跑到 load 51、SSH 被擠掉的那次，正好是
          相機＋人臉＋LLM 全開而底盤沒開。最需要看的時候剛好看不到。
        """
        # ★ 位姿要在節流**之前**存：外推需要最新的里程計，
        #   而車速 1 Hz 就夠看。存三個浮點數的成本可以忽略。
        _p = msg.pose.pose.position
        _q = msg.pose.pose.orientation
        with self._pose_lock:
            self._odom_pose = {
                "x": float(_p.x),
                "y": float(_p.y),
                "yaw": quaternion_to_yaw(_q.x, _q.y, _q.z, _q.w),
            }

        now = time.monotonic()
        if now - self._last_speed_at < 1.0:
            return
        self._last_speed_at = now
        self.state.set_system(speed=round(float(msg.twist.twist.linear.x), 3))

    def _resource_tick(self) -> None:
        """獨立的資源監看（2026-08-14 新增）

        使用者要求 ASR 常駐等待，而常駐服務最怕的是**慢性資源漂移**：
        當下看起來都好，跑兩小時之後記憶體被吃光、SSH 進不去、只能斷電重開。
        8/14 就發生過一次（load average **51**、四核心、SSH 被擠掉）。

        ★ 三個量的分工不同，缺一不可：
          cpu_load   反應快，但**負載高不一定會死** —— 排隊而已
          mem_avail  ★ 這個才是致命的那個。可用記憶體歸零 = OOM killer
                     開始亂殺行程，或瘋狂 swap 導致整機失去回應
          cpu_temp   Pi 4 到 80°C 會降頻，症狀是「什麼都變慢」而不是壞掉

        ★ 為什麼讀 MemAvailable 而不是 MemFree：Linux 會把空閒記憶體拿去當
          快取，MemFree 常態就很低，看它會天天誤報。MemAvailable 是核心自己
          估的「不觸發 swap 能給出多少」，才是真正該看的數字。

        跨過門檻時**寫進 log**（不只是更新畫面）—— 事後查當機原因時，
        沒有人會有當時的平板畫面，但 log 一定留著。
        """
        fields: Dict[str, Any] = {}
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as fh:
                fields["cpu_temp"] = round(int(fh.read().strip()) / 1000.0, 1)
        except (OSError, ValueError):
            pass
        try:
            fields["cpu_load"] = round(os.getloadavg()[0], 2)
        except OSError:
            pass
        try:
            avail = total = None
            with open("/proc/meminfo", "r") as fh:
                for line in fh:
                    if line.startswith("MemAvailable:"):
                        avail = int(line.split()[1]) / 1024.0          # kB -> MB
                    elif line.startswith("MemTotal:"):
                        total = int(line.split()[1]) / 1024.0
                    if avail is not None and total is not None:
                        break
            if avail is not None:
                fields["mem_avail_mb"] = round(avail)
                if total:
                    fields["mem_used_pct"] = round(100.0 * (1.0 - avail / total))
        except (OSError, ValueError, IndexError):
            pass

        if fields:
            self.state.set_system(**fields)

        # ── 門檻警告 ────────────────────────────────────────
        # throttle_duration_sec 讓它最多每分鐘唸一次，不會洗版。
        # ★★ 2026-08-17：load average **不能**單獨拿來判斷「現在忙不忙」★★
        #
        # 實測：load average 顯示 36.6 的同一時刻，vmstat 的 r（真正可執行的
        # 執行緒數）只有 0~1、CPU 閒置 48~55%、IO 等待 0%。**系統根本沒在排隊。**
        # 原因是 load average 是 1/5/15 分鐘的指數加權移動平均，**它落後現實好幾分鐘**：
        # 當時反映的是稍早那段有四份 stuck_detector_cc、三份 scan_filter_cc
        # 殘留的狀況（stop_nav_cc.sh 收不乾淨造成，已修）。
        #
        # 照舊邏輯，操作者會被叫去「關掉深度相機或導航堆疊」——
        # 而真正該做的是把重複的節點收掉。**錯的警告比沒有警告更糟。**
        #
        # 改法：load 高**只是候選條件**，要再看一個即時指標才報。
        # 這裡用 /proc/stat 兩次取樣算出的 CPU 忙碌率（非 idle 佔比）。
        load = fields.get("cpu_load")
        if load is not None and load >= self.LOAD_WARN:
            busy = self._cpu_busy_ratio()
            if busy is None or busy >= 0.85:
                self.get_logger().warn(
                    f"⚠ 系統負載 {load:.1f}"
                    + (f"、CPU 忙碌 {busy*100:.0f}%" if busy is not None else "")
                    + "——考慮關掉深度相機或導航堆疊",
                    throttle_duration_sec=60.0)
            else:
                # 這種情形通常代表「剛剛很忙、現在已經好了」，或有殘留節點被收掉了。
                self.get_logger().info(
                    f"系統負載 {load:.1f} 偏高，但 CPU 忙碌只有 {busy*100:.0f}%"
                    "——是移動平均的殘影，現在沒有在排隊",
                    throttle_duration_sec=300.0)
        mem = fields.get("mem_avail_mb")
        if mem is not None and mem <= self.MEM_WARN_MB:
            self.get_logger().error(
                f"⚠⚠ 可用記憶體只剩 {mem} MB（<{self.MEM_WARN_MB}）"
                "——再下去 OOM killer 會開始殺行程、SSH 會進不來，請立刻停掉非必要節點",
                throttle_duration_sec=60.0)
        temp = fields.get("cpu_temp")
        if temp is not None and temp >= self.TEMP_WARN:
            self.get_logger().warn(
                f"⚠ CPU {temp:.1f}°C（>{self.TEMP_WARN:.0f} 會開始降頻，症狀是全部變慢）",
                throttle_duration_sec=60.0)

    def _cpu_busy_ratio(self) -> Optional[float]:
        """CPU 忙碌率（0~1）—— 兩次 /proc/stat 取樣的差值，是**即時**指標。

        ★ 跟 load average 的差別：load 是 1/5/15 分鐘的移動平均，會把幾分鐘前的
        尖峰一直帶著；這個看的是「上次呼叫到現在」這段區間真的用掉多少 CPU。
        第一次呼叫沒有前一筆可以比，回 None（呼叫端會退回只看 load）。
        """
        try:
            with open("/proc/stat") as f:
                parts = f.readline().split()
            vals = [int(x) for x in parts[1:11]]
            total = sum(vals)
            idle = vals[3] + vals[4]            # idle + iowait
        except Exception:
            return None
        prev = getattr(self, "_cpu_stat_prev", None)
        self._cpu_stat_prev = (total, idle)
        if prev is None:
            return None
        dt, di = total - prev[0], idle - prev[1]
        if dt <= 0:
            return None
        return max(0.0, min(1.0, 1.0 - di / dt))

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

        密碼不寫死在原始碼裡：這個 repo 是公開的，寫死等於公開密碼。
        而且寫死的預設值很容易就是平常在用的那組——一旦寫進 git，
        之後就算改掉，舊值仍然留在歷史裡永遠拿得到。

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
            # 動作本身有自己的逾時（見 ACTION_TIMEOUTS 的註解），
            # 這裡的上限只是為了在動作伺服器整個掛掉時不要留下永遠不結束的執行緒。
            action_timeout = ACTION_TIMEOUTS.get(name, 300.0)
            try:
                wrapped = self._await_future(result_future, timeout=action_timeout)
            except TimeoutError:
                # ★ 2026-08-12：等不到結果時**主動叫停**，不能只是自己放棄。
                #
                # 這條路只有在「動作節點沒有依約回報」時才會走到，而那正是最危險
                # 的情形：HMI 認定作業結束、finally 把 goal handle 從登記簿移除，
                # 於是連緊急停止都找不到它 —— 而車子完全沒被通知過，還在照原本的
                # 路徑開。送出取消至少讓節點端的取消檢查有機會把車停穩（導航是
                # navigation_action_cc 的 _wait_taught_result，重播是 _follow_loop）。
                #
                # ⚠ 這裡一定要接 TimeoutError 而不是判斷回傳值：_await_future 逾時
                #   是 **raise** 不是回傳 None（見該方法 :2544）。
                self.get_logger().error(
                    f"動作 {name} 等待結果逾時（{action_timeout:.0f} 秒），"
                    "主動送出取消以免車子繼續移動"
                )
                try:
                    goal_handle.cancel_goal_async()
                except Exception as e:  # noqa: BLE001
                    self.get_logger().error(f"逾時後送出取消也失敗: {e}")
                self.state.set_job(job_id, status="failed",
                                   message="等待結果逾時，已送出取消（請確認車輛已停止）")
                return

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

    # ★★ 2026-08-17：緊急停止原本停不住「不是 HMI 發起的」導航 ★★
    #
    # 實機發生過：使用者用語音叫車子做全域定位，車子開始繞 0.9 m 圓弧，
    # 按平板的緊急停止，log 回「已送零速，取消 **0 個**作業」而車子照走。
    #
    # 原因：`cancel_job()` 只能取消 `_job_handles` 裡的 goal handle，
    # 那是 HMI 自己送出去才會登記的。LLM 的工具是
    # `llm_service_node` -> `/navigate` 直接送，HMI 從頭到尾不知道有這件事。
    # 而 `apply_teleop(0,0)` 送的零速會跟控制器 10~20 Hz 的真指令交錯，
    # 車子只會頓一下，**不會停**（那條「遙控看門狗 0.6 秒」的註解，
    # 前提是沒有別人在發指令，有控制器在跑時不成立）。
    #
    # ★ 客人只要開口就能讓車子動，而平板上的紅色大按鈕停不住它 —— 這是安全問題。
    #
    # 解法：直接對 action 的 cancel 服務送「取消全部」。
    # ROS 2 action 規格：goal_id 全零 + stamp 全零 = 取消該伺服器上**所有**目標，
    # 與是誰送出的無關。這條路不需要 goal handle，所以繞得過登記簿。
    ACTION_CANCEL_ALL = ("navigate", "follow_taught_path", "global_localization", "create_map")

    def _cancel_all_action_goals(self) -> List[str]:
        """對所有會讓車子移動的 action 送『取消全部』。回傳成功送出的動作名。"""
        sent: List[str] = []
        for name in self.ACTION_CANCEL_ALL:
            client = self.action_clients.get(name)
            if client is None:
                continue
            try:
                srv = self._cancel_srv_clients.get(name)
                if srv is None:
                    # action 的取消服務固定是 <action_name>/_action/cancel_goal
                    srv = self.create_client(
                        CancelGoal, f"{client._action_name}/_action/cancel_goal",
                        callback_group=self._cancel_cb_group)
                    self._cancel_srv_clients[name] = srv
                if not srv.service_is_ready():
                    continue
                # 全零 goal_id + 全零 stamp = 取消全部
                srv.call_async(CancelGoal.Request())
                sent.append(name)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"送出 {name} 的取消全部失敗: {e}")
        return sent

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
    # 送到前端畫的路徑最多幾點。地圖畫布只有幾百像素寬，超過這個數的細節
    # 眼睛看不出來，卻要每次狀態推播都重傳一遍。
    MAX_PATH_POINTS = 120

    # 資源警戒線（見 _resource_tick）。四核心 Pi 4：
    #   load 4 = 剛好滿載；8/14 那次量到 51，SSH 已經進不去
    #   記憶體 8 GB，留 600 MB 是「還救得回來」的下限
    LOAD_WARN = 6.0
    MEM_WARN_MB = 600
    TEMP_WARN = 78.0

    TELEOP_MAX_LINEAR = 0.18
    TELEOP_MAX_ANGULAR = 0.45

    def _publish_teleop(self, lin: float, ang: float) -> None:
        """把速度指令送到底盤，必要時自動繞過不存在的中繼節點

        teleop_cmd_topic 預設是 cmd_vel_trimmed，指令會經過 collision_monitor
        才到底盤，遙控時一樣有防撞保護。但這只在完整導航堆疊有起來時成立：

            mapping_manual_cc.launch.py  只起 slam_toolbox
            sensors_cc.launch.py         只起底盤與雷達
            nav_bringup_cc 剛啟動的頭幾分鐘  collision_monitor 還在 unconfigured

        這三種情況下 cmd_vel_trimmed **沒有任何訂閱者**，指令發出去就消失，
        HTTP 卻照樣回成功 —— 從操作者的角度就是「按了完全沒反應」。

        所以發布前先看有沒有人在聽；沒有就直接發 cmd_vel（底盤自己訂的話題）。
        這是刻意的降級而不是預設值改動：有防撞層時仍然走防撞層。
        """
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.teleop_pub.publish(msg)

        if self.teleop_pub.topic_name.lstrip("/") == "cmd_vel":
            return
        if self.count_subscribers(self.teleop_pub.topic_name) > 0:
            self._teleop_fallback_warned = False
            return

        if not self._teleop_fallback_warned:
            self._teleop_fallback_warned = True
            self.get_logger().warn(
                f"{self.teleop_pub.topic_name} 沒有訂閱者（collision_monitor 未啟動？），"
                f"遙控指令改直接發 cmd_vel —— 此時沒有防撞保護，請放慢速度"
            )
        self.teleop_direct_pub.publish(msg)

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
            # 收到零速就直接收工，不要讓看門狗在 0.6 秒後再多噴一次逾時警告
            self._teleop_active = lin != 0.0 or ang != 0.0

        self._publish_teleop(lin, ang)

    def _teleop_loop(self) -> None:
        """20 Hz 補送迴圈（獨立執行緒）

        用 monotonic 推算下一次的絕對時間點而不是固定 sleep(0.05)，
        這樣單次處理慢一點也不會讓整體頻率一路往下掉。
        """
        period = 0.05
        next_at = time.monotonic()
        while rclpy.ok():
            next_at += period
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                # 落後太多就重新對時，不要補一堆遲到的指令
                next_at = time.monotonic()
            try:
                self._teleop_tick()
            except Exception as exc:  # noqa: BLE001
                # 這條執行緒掛掉等於看門狗失效，車子可能停不下來 —— 絕不能讓它死
                self.get_logger().error(f"遙控補送迴圈異常：{exc}")

            # 量的是**迴圈本身**的頻率，不是「有補送」的次數：
            # 看門狗一觸發就把 _teleop_active 設回 False，之後的 tick 會提早 return，
            # 只計補送次數會把「執行緒沒在跑」和「沒東西要送」混為一談。
            self._teleop_loop_count += 1
            if self._teleop_loop_count % 200 == 0:
                span = time.monotonic() - self._teleop_loop_t0
                if span > 0:
                    self.get_logger().info(
                        f"遙控迴圈 {200.0 / span:.1f} Hz（目標 20），"
                        f"其中實際補送 {self._teleop_tick_count} 次"
                    )
                self._teleop_loop_t0 = time.monotonic()

    def _teleop_tick(self) -> None:
        """維持指令流 ＋ 看門狗

        **補送指令**：底盤韌體有 1.0 秒指令逾時，沒收到新指令就自己停。
        如果發布頻率等於 HTTP 到達頻率（前端標稱 5 Hz），那麼 WiFi 抖動、
        TCP 重傳、Pi4 CPU 排隊只要讓連續幾則請求慢下來，底盤就會看到
        超過 1 秒的空窗 → 停車 → 下一則到達又起步，表現就是一頓一頓。
        所以這裡以固定頻率重送目前的設定值，讓網路抖動不會傳導到底盤。

        **看門狗**：連線中斷、鎖螢幕、切 App、瀏覽器分頁被回收，都會讓
        指令停止送達卻沒有任何「停止」訊息 —— 只靠「收到停止才停」
        會讓車子在失聯後繼續跑。
        """
        with self._teleop_lock:
            if not self._teleop_active:
                return
            idle = time.monotonic() - self._teleop_last_cmd
            if idle < self._teleop_watchdog_sec:
                # 還在有效期內：重送目前的設定值，維持指令流不中斷
                lin, ang = self._teleop_linear, self._teleop_angular
                timed_out = False
            else:
                self._teleop_active = False
                self._teleop_linear = 0.0
                self._teleop_angular = 0.0
                lin = ang = 0.0
                timed_out = True

        if not timed_out:
            self._publish_teleop(lin, ang)
            self._teleop_tick_count += 1
            # 每 100 次補送（20 Hz 下約 5 秒）報一次實際速率。
            # 這不是除錯殘留：補送頻率掉下來就代表執行器被塞住，
            # 而症狀會是「車子一頓一頓」，從外面很難判斷是網路還是排程。
            if self._teleop_tick_count % 100 == 0:
                now_s = time.monotonic()
                span = now_s - self._teleop_tick_t0
                if span > 0:
                    self.get_logger().info(
                        f"遙控補送 {self._teleop_tick_count} 次，實際 {100.0 / span:.1f} Hz（目標 20）"
                    )
                self._teleop_tick_t0 = now_s
            return

        # 連送三次零速：DDS 掉一則封包不能變成「車子繼續跑」
        for _ in range(3):
            self._publish_teleop(0.0, 0.0)
        self.get_logger().warn(f"遙控逾時 {idle:.1f} 秒未收到指令，已停車")

    # ------------------------------------------------------------------
    # 系統節點開關
    # ------------------------------------------------------------------
    # 只允許這張表裡的項目，而且對應的是**既有的啟動腳本**而不是任意指令。
    # 這是刻意的：HMI 是有網頁介面的服務，如果讓它執行前端傳來的字串，
    # 等於把 shell 開放給任何拿到管理者權杖的人。
    #
    # detect  用來判斷是否在跑（比對行程指令列，不是 pgrep -f 以免誤殺）
    # start   啟動腳本；stop 停止腳本（沒有停止腳本的用 detect 找 PID 送 TERM）
    SYSTEM_UNITS = {
        "sensors": {
            "label": "底盤 + 雷達 + IMU",
            "detect": ["lslidar_driver_node", "wheeltec_robot_node"],
            "start": ["/home/user/maprun/run_sensors_cc.sh"],
            "stop": ["/home/user/maprun/kill_sensors_cc.sh"],
            # 啟動後要靜止校準 IMU，時間較長
            "start_hint": "啟動後約 20 秒完成 IMU 零偏校準，期間車子必須靜止",
        },
        "camera": {
            "label": "深度相機（避障點雲）",
            "detect": ["astra_camera_node", "depth_obstacle_cc"],
            "start": ["/home/user/maprun/run_camera_obstacle_cc.sh", "false", "160", "120", "5.0", "2"],
            "stop": ["/home/user/maprun/kill_camera_cc.sh"],
            "start_hint": "160x120 深度 + 降頻點雲，約佔 30% CPU",
        },
        # 導航堆疊只有一個單元，但有三種啟動方式。
        #
        # 一開始寫成 nav_mapping / nav_explore / nav_localization 三個獨立單元，
        # 結果是災難：三者共用同一批行程（map_service_cc、planner_server、amcl…），
        # 只靠比對行程名稱根本分不出來，於是啟動建圖模式之後
        # 「定位＋導航」也顯示執行中、「建圖＋探索」顯示部分 —— UI 在騙人。
        #
        # 真正的模式是 map_service_cc 的內部狀態，要問 /get_nav_mode 才知道，
        # 所以改成「一個單元 + 三個啟動選項」，模式由 /api/maps/status 另外顯示。
        "nav": {
            "label": "導航堆疊",
            "detect": ["map_service_cc"],
            "stop": ["/home/user/maprun/stop_nav_cc.sh"],
            "start_hint": "啟動約需 60 秒。模式請看建圖頁的狀態列",
            "variants": {
                "mapping": {
                    "label": "建圖（遙控）",
                    "cmd": ["/home/user/maprun/run_nav_cc.sh", "mapping", "false"],
                },
                "explore": {
                    "label": "建圖 + 自動探索",
                    "cmd": ["/home/user/maprun/run_nav_cc.sh", "mapping", "true"],
                    "warn": "車子會自己跑動",
                },
                "localization": {
                    "label": "定位 + 導航",
                    "cmd": ["/home/user/maprun/run_nav_cc.sh", "localization"],
                },
            },
        },
        # ── 迎賓流程的功能模組 ──────────────────────────
        # 流程：待機（人臉辨識）-> 認出貴賓 -> LLM 對話 -> 觸發導航 -> 到達 -> 恢復辨識
        #
        # 這些都是 `ros2 run` 起的單一節點，沒有 launch 檔，
        # 所以統一透過 run_node_cc.sh（負責補上 ROS 與 DDS 環境）。
        #
        # requires 欄位是**前置條件檢查**：與其讓操作者按了沒反應、
        # 還要自己去翻 log，不如在按鈕旁邊直接說明缺什麼。
        "face": {
            "label": "人臉辨識",
            "detect": ["face_embedding"],
            "start": ["/home/user/maprun/run_node_cc.sh", "smartnav_vision", "face_embedding"],
            "stop": ["/home/user/maprun/kill_node_cc.sh", "smartnav_vision", "face_embedding"],
            "start_hint": "迎賓流程的觸發點。需要深度相機的彩色影像",
            "requires": {"module": "insightface"},
        },
        "user_auth": {
            "label": "使用者認證 / 決策",
            "detect": ["user_auth"],
            "start": ["/home/user/maprun/run_node_cc.sh", "smartnav_brain", "user_auth"],
            "stop": ["/home/user/maprun/kill_node_cc.sh", "smartnav_brain", "user_auth"],
            "start_hint": "辨識到人之後決定要不要迎賓、走哪個流程",
        },
        # ★ 2026-08-10 補上：面板原本沒有這一項，而它正是完整故事線的核心。
        #   其餘 10 項都能從平板一鍵開，唯獨迎賓劇本要 ssh 進去手打——
        #   發表當天最不該需要鍵盤的就是這一個。
        "bank_reception": {
            "label": "迎賓劇本（銀行）",
            "detect": ["bank_reception"],
            "start": ["/home/user/maprun/run_node_cc.sh", "smartnav_brain", "bank_reception"],
            "stop": ["/home/user/maprun/kill_node_cc.sh", "smartnav_brain", "bank_reception"],
            "start_hint": "訂 /user_identity 出迎賓詞到 /speech_text；需先開「使用者認證 / 決策」",
        },
        "llm": {
            "label": "LLM 對話",
            "detect": ["llm_service"],
            "start": ["/home/user/maprun/run_node_cc.sh", "smartnav_llm", "llm_service"],
            "stop": ["/home/user/maprun/kill_node_cc.sh", "smartnav_llm", "llm_service"],
            "start_hint": "連遠端 Ollama（預設 192.168.137.1:11434 = 筆電），不是跑在這台 Pi 上",
        },
        # ★★ 2026-08-14：原本這裡是「語音喚醒」與「語音辨識」兩顆分開的按鈕，
        #    兩顆都走 run_node_cc.sh —— 而那條路**啟動出來的 ASR 收不到人聲**。
        #
        #    缺的三件事（8/14 實機才查出來，全部不報錯）：
        #      1. Astra S 的 ALSA 卡號**每次開機都會變**（那天一天內 1 -> 2 -> 3），
        #         裝置索引必須當場問 sounddevice，不能沿用預設
        #      2. 增益有 'Mic',0 與 'Mic',1 **兩個**控制項，只設一個沒有用；
        #         而預設的 48（24 dB）底噪 -35 dBFS、人聲埋在裡面 -> VAD 一次都沒觸發
        #      3. voice_trigger 要收到 `device:=<索引>`，否則抓到別張音效卡
        #
        #    這三件事都在 `~/maprun/run_asr_cc.sh` 裡（8/14 實測校準過，增益 66 = 33 dB）。
        #
        # ★ 為什麼合併成一顆而不是修兩顆：這兩個節點**只有一起跑才有意義** ——
        #   voice_trigger 只在 VAD 判定有人講話時才送 /audio_in，
        #   speech_recognizer 收到才吐 /user_text。分開按只會製造「開了一半」的狀態。
        #   而且發表當天沒有鍵盤，**面板上一顆會動的按鈕勝過兩顆不會動的**。
        #
        # detect 兩個都列：只起來一個時前端會顯示 partial（半亮），一眼看得出不對。
        "asr_chain": {
            "label": "語音輸入（麥克風 → 文字）",
            "detect": ["voice_trigger", "speech_recognizer"],
            "start": ["/home/user/maprun/run_asr_cc.sh"],
            "stop": ["/home/user/maprun/run_asr_cc.sh", "stop"],
            "requires": {"module": "sounddevice"},
            "start_hint": "麥克風長在 Astra S 相機裡；相機沒插好就沒有它。載入模型約 10~25 秒",
        },
        # ★ 這兩個是「車上出聲」用的，而**車上沒有喇叭**（2026-08-07 決定不採購）。
        #   實際的語音輸出走平板瀏覽器的 speechSynthesis：
        #       /speech_text -> HMI -> 平板唸出來
        #   保留按鈕是因為之後若真的加了喇叭就能直接用，但要在面板上寫清楚，
        #   否則操作者會在「沒聲音」時來按這兩顆，然後以為是它們壞了。
        "speech_synthesizer": {
            "label": "語音合成（需車上喇叭）",
            "detect": ["speech_synthesizer"],
            "start": ["/home/user/maprun/run_node_cc.sh", "smartnav_audio", "speech_synthesizer"],
            "stop": ["/home/user/maprun/kill_node_cc.sh", "smartnav_audio", "speech_synthesizer"],
            "requires": {"audio_output": True},
            "hide_when_missing": True,
            "start_hint": "車上沒有喇叭，展示時的語音輸出是平板瀏覽器唸的，不需要開這個",
        },
        "voice_playback": {
            "label": "語音播放（需車上喇叭）",
            "detect": ["voice_playback"],
            "start": ["/home/user/maprun/run_node_cc.sh", "smartnav_audio", "voice_playback"],
            "stop": ["/home/user/maprun/kill_node_cc.sh", "smartnav_audio", "voice_playback"],
            "requires": {"module": "sounddevice", "audio_output": True},
            "hide_when_missing": True,
            "start_hint": "同上：車上沒有喇叭。要平板出聲請開右上角的朗讀開關",
        },
    }

    @staticmethod
    def _proc_cmdlines() -> list:
        """讀出所有行程的指令列

        用 /proc 而不是 pgrep：pgrep -f 會匹配到「指令列裡含有該關鍵字的
        呼叫端自己」，在這個專案已經造成過三次自殺（exit 144）。
        """
        out = []
        for entry in os.scandir("/proc"):
            if not entry.name.isdigit():
                continue
            try:
                with open(f"/proc/{entry.name}/cmdline", "rb") as fp:
                    cmd = fp.read().replace(b"\0", b" ").decode("utf-8", "replace")
                if cmd.strip():
                    out.append((int(entry.name), cmd))
            except (OSError, ValueError):
                continue
        return out

    # Python 模組是否存在的結果會被快取：狀態頁每幾秒就打一次，
    # 而模組裝沒裝在一次執行期間不會變。
    _requires_cache: Dict[str, str] = {}

    # ── 周邊硬體偵測（2026-08-14 新增）────────────────────────
    #
    # 專案鐵則：**硬體問題用列舉指令回答，不要用文件回答**。
    # 「驅動套件建置過」不等於「硬體在車上」——8/07 就是靠這條抓到
    # 「以為有 6 麥陣列，其實只有驅動」。這支端點把那些列舉搬到平板上。
    #
    # ★ 兩層都要看，只看一層會得到錯的結論：
    #     核心層  裝置節點在不在（插了沒、驅動認得嗎）
    #     資料層  話題有沒有在發（認得到 ≠ 有資料，相機常態是關的）
    #   只看核心層 -> 相機插著但沒啟動，會誤報「正常」
    #   只看資料層 -> 節點沒開時全部紅字，分不出「沒開」還是「沒插」
    #
    # ★ 全部走 /proc 與 /sys，不開子行程：這支會被平板輪詢，
    #   每次 fork 一個 lsusb 在 Pi 4 上是不必要的負擔。
    @staticmethod
    def _read(path: str) -> str:
        try:
            with open(path, "r", encoding="utf-8", errors="ignore") as fh:
                return fh.read()
        except OSError:
            return ""

    def _usb_products(self) -> list:
        """USB 裝置的產品名清單（相當於 lsusb，但不開子行程）"""
        names = []
        for f in glob.glob("/sys/bus/usb/devices/*/product"):
            v = self._read(f).strip()
            if v:
                names.append(v)
        return names

    def hardware_status(self) -> list:
        now = time.time()
        with self._hw_lock:
            if self._hw_cache and now - self._hw_cache_at < 3.0:
                return self._hw_cache

        pcm = self._read("/proc/asound/pcm")
        usb = self._usb_products()
        usb_l = " | ".join(usb).lower()
        videos = sorted(glob.glob("/dev/video*"))
        serials = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
                         + glob.glob("/dev/wheeltec_*"))
        sysinfo = self.state.snapshot().get("system", {})

        def item(key, label, present, detail, hint=""):
            return {"key": key, "label": label, "present": bool(present),
                    "detail": detail, "hint": hint}

        out = [
            item("camera", "深度相機 Astra S",
                 any("astra" in u.lower() or "orbbec" in u.lower() for u in usb) or bool(videos),
                 (f"{len(videos)} 個 /dev/video*" if videos else "找不到 /dev/video*")
                 + (f"；USB: {[u for u in usb if 'astra' in u.lower() or 'orbbec' in u.lower()]}"
                    if any('astra' in u.lower() or 'orbbec' in u.lower() for u in usb) else ""),
                 "沒抓到就把 Astra S 手動插拔一次 —— 它有開機列舉失敗的老問題，"
                 "而且**麥克風長在同一顆裝置上，會一起消失**"),
            item("mic", "麥克風（在相機裡）",
                 "capture" in pcm,
                 "有錄音裝置" if "capture" in pcm else "/proc/asound/pcm 沒有 capture",
                 "跟相機同一顆 USB 裝置。ASR 要用它"),
            item("speaker", "喇叭",
                 "playback" in pcm,
                 "有播放裝置" if "playback" in pcm else "沒有播放裝置（正常）",
                 "★ 車上刻意不裝喇叭（2026-08-07 決定）。語音輸出走平板瀏覽器，"
                 "所以這一項紅色是**預期的**，不用處理"),
            item("serial", "序列埠（底盤／光達）",
                 bool(serials),
                 "、".join(serials) if serials else "找不到任何 ttyUSB/ttyACM",
                 "底盤 STM32 與 N10 光達都走 USB 序列埠。少一個就會有一個節點起不來"),
        ]

        # ── 資料層：話題有沒有在發 ──
        # 這幾個量本來就在 state 裡（由既有訂閱更新），直接借用，不另外訂閱。
        out.append(item(
            "chassis_data", "底盤回報（電壓）",
            sysinfo.get("voltage") is not None,
            f"{sysinfo.get('voltage')} V" if sysinfo.get("voltage") is not None else "沒收到 /PowerVoltage",
            "★ 有序列埠但沒有電壓 = 線接上了但節點沒起來（或起來了但通訊失敗）"))

        with self._hw_lock:
            self._hw_cache = out
            self._hw_cache_at = now
        return out

    def _check_requires(self, requires: Optional[dict]) -> str:
        """回傳「缺什麼」的說明，都齊全就回空字串"""
        if not requires:
            return ""
        # ★ 2026-08-14：硬體檢查。使用者要求「HMI 要自己把沒在用的設備剔除」。
        #   做成偵測而不是刪掉單元 —— 日後真的裝了喇叭，按鈕會自己回來。
        if requires.get("audio_output"):
            cached = self._requires_cache.get("__aplay__")
            if cached is None:
                try:
                    # 問核心而不是看設定檔。/proc/asound/pcm 每行結尾是
                    # "playback 1" / "capture 1"，只有播放裝置才有 playback。
                    txt = open("/proc/asound/pcm", encoding="utf-8", errors="ignore").read()
                    cached = "" if "playback" in txt else "車上沒有喇叭（找不到播放裝置）"
                except OSError:
                    cached = "車上沒有喇叭（找不到播放裝置）"
                self._requires_cache["__aplay__"] = cached
            if cached:
                return cached

        module = requires.get("module")
        if not module:
            return ""
        cached = self._requires_cache.get(module)
        if cached is None:
            try:
                import importlib.util

                cached = "" if importlib.util.find_spec(module) else f"缺少 Python 模組 {module}"
            except (ImportError, ValueError):
                cached = f"缺少 Python 模組 {module}"
            self._requires_cache[module] = cached
        return cached

    def system_status(self) -> list:
        """回報每個單元是否在跑"""
        procs = self._proc_cmdlines()
        mypid = os.getpid()
        units = []
        for key, spec in self.SYSTEM_UNITS.items():
            # ★ 缺硬體而且標了 hide_when_missing 的單元直接不列出來。
            #   面板上一顆永遠按不動的按鈕，比沒有這顆按鈕更糟 ——
            #   操作者會在「沒聲音」時去按它，然後以為是它壞了。
            if spec.get("hide_when_missing") and self._check_requires(spec.get("requires")):
                continue
            found = []
            for needle in spec["detect"]:
                for pid, cmd in procs:
                    # 排除自己，也排除 shell 包裝（bash -c "..." 會含有關鍵字）
                    if pid == mypid or cmd.lstrip().startswith("/bin/bash"):
                        continue
                    if needle in cmd:
                        found.append(needle)
                        break
            units.append(
                {
                    "key": key,
                    "label": spec["label"],
                    "running": len(found) == len(spec["detect"]),
                    "partial": 0 < len(found) < len(spec["detect"]),
                    "hint": spec.get("start_hint", ""),
                    # 缺什麼就先講，不要讓操作者按了沒反應才去翻 log
                    "blocked": self._check_requires(spec.get("requires")),
                    # 有 variants 的單元由前端畫成多顆啟動按鈕
                    "variants": [
                        {"key": vk, "label": v["label"], "warn": v.get("warn", "")}
                        for vk, v in spec.get("variants", {}).items()
                    ],
                }
            )
        return units

    def system_control(self, unit: str, action: str) -> tuple:
        """啟動或停止一個單元

        action 是 "stop"，或 "start"（單一啟動方式）／"start:<variant>"（多選一）。
        """
        spec = self.SYSTEM_UNITS.get(unit)
        if spec is None:
            return False, f"未知的單元：{unit}"

        variant_key = None
        if action.startswith("start:"):
            action, variant_key = "start", action.split(":", 1)[1]
        if action not in ("start", "stop"):
            return False, f"未知的動作：{action}"

        if action == "start" and spec.get("variants"):
            if variant_key is None:
                return False, f"{spec['label']} 需要指定啟動方式"
            variant = spec["variants"].get(variant_key)
            if variant is None:
                return False, f"未知的啟動方式：{variant_key}"
            cmd = variant["cmd"]
        else:
            cmd = spec.get(action)
        if not cmd:
            return False, f"{spec['label']} 不支援 {action}"

        # 重複啟動的防呆。
        #
        # 導航堆疊要 60 秒才會有可見的變化，操作者按了沒反應很自然會再按一次，
        # 於是同時跑起兩套 Nav2 —— 兩個 controller_server、兩個 planner_server、
        # 兩個 slam_toolbox 搶同一個 /map 與 map->odom TF，比沒啟動還糟，
        # 而且外顯症狀（地圖亂跳、目標一直 abort）看起來完全不像是「按了兩次」。
        # 2026-07-31 實測發生過一次，兩套疊跑了四分鐘。
        if action == "start":
            for u in self.system_status():
                if u["key"] != unit:
                    continue
                if u["running"]:
                    return False, f"{spec['label']} 已經在執行中，不需要再啟動一次"
                if u["partial"]:
                    return False, (
                        f"{spec['label']} 正在啟動或只起來一半，"
                        "請先按停止再重新啟動，不要重複按啟動"
                    )
                break

        try:
            if action == "start":
                # setsid + 完全脫離：HMI 服務重啟時不能把這些節點一起帶走
                subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    stdin=subprocess.DEVNULL,
                    start_new_session=True,
                )
                what = spec["label"]
                if variant_key:
                    what += f" — {spec['variants'][variant_key]['label']}"
                hint = spec.get("start_hint", "")
                return True, f"已啟動 {what}" + (f"（{hint}）" if hint else "")
            # stop 要等它跑完才知道結果
            res = subprocess.run(
                cmd, capture_output=True, text=True, timeout=60, stdin=subprocess.DEVNULL
            )
            tail = (res.stdout or res.stderr or "").strip().splitlines()
            return True, f"已停止 {spec['label']}" + (f"：{tail[-1]}" if tail else "")
        except subprocess.TimeoutExpired:
            return False, f"{action} {spec['label']} 逾時"
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"system_control({unit},{action}) 失敗: {exc}")
            return False, f"執行失敗：{exc}"

    def _query_nav_mode(self) -> str:
        """問 map_service_cc 目前是建圖還是定位模式"""
        client = self.service_clients.get("get_nav_mode")
        if client is None or not client.wait_for_service(timeout_sec=1.0):
            return "unknown"
        try:
            res = self._await_future(client.call_async(Trigger.Request()), timeout=5.0)
        except Exception:  # noqa: BLE001
            return "unknown"
        if not res:
            return "unknown"
        # /get_nav_mode 把模式字串放在 message 裡
        msg = (res.message or "").strip()
        for token in ("mapping", "localization", "unknown"):
            if token in msg:
                return token
        return msg or "unknown"

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

        @app.middleware("http")
        async def track_client(request: Request, call_next):
            """任何一個 HTTP 請求都算「有用戶端在」

            放在 middleware 而不是逐一在端點裡呼叫：漏掉一個端點的代價是
            節點在有人操作時把訂閱收掉，那種 bug 很難查。
            對 StreamingResponse（/video）不會造成阻塞——call_next 在送出
            表頭時就回來了，串流本體是之後才逐段產生的。
            """
            self.note_client()
            return await call_next(request)

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
            # ★ 2026-08-20：補禁快取標頭。
            #   Starlette 的 HTMLResponse 只設 content-type 與 content-length，
            #   **沒有 Cache-Control、沒有 ETag、也沒有 Last-Modified** ——
            #   連條件式請求都發不出來，瀏覽器只能重抓或直接吃自己那份。
            #   同一支檔案的地圖 PNG 與 frame.jpg 都明確帶了 no-store，就這裡漏了。
            #   而整個前端只有 index.html 一個檔（見 setup.py 的 data_files），
            #   **一次快取失手 = 整個前端是舊版**，包含所有修正。
            #   iOS Safari 在「加入主畫面」模式下對 HTML 的重用特別積極。
            return HTMLResponse(
                index_path.read_text(encoding="utf-8"),
                headers={"Cache-Control": "no-store, must-revalidate"},
            )

        @app.get("/api/state")
        async def api_state() -> JSONResponse:
            return JSONResponse(self.state.snapshot())

        @app.get("/api/health")
        async def api_health() -> JSONResponse:
            with self._frame_lock:
                has_frame = self._latest_jpeg is not None
            # 影像訂閱按需建立之後，「手上沒有影格」不再等於「相機沒影像」——
            # 沒人在看的時候本來就沒有。改問話題上有沒有發布者，這其實比
            # 原本的判斷更準：它回答的是「相機節點在不在發」。
            if not has_frame:
                has_frame = self.count_publishers(self.image_topic) > 0
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
            """狀態推播

            ★ 2026-08-17：加上「斷線就收掉」。原本這個迴圈只有在 send 失敗時
              才會結束，而 send 只在狀態版本變動時才發生——狀態不動的期間
              （迎賓機閒置時的常態）連線已經死了也沒人知道，迴圈就以 10 Hz
              空轉下去，而且沒有上限：平板每重新整理一次就多留一條。

              兩道防線：
              1. **協定層 ping/pong**（uvicorn 的 ws_ping_interval/timeout，
                 見 _run_server）。沒有 pong 就由伺服器主動關閉連線。
              2. 這裡的 receive 監看任務。連線一被關閉（不論是對方關的、
                 還是第 1 條判死的），它馬上收到 websocket.disconnect，
                 推播迴圈下一輪就結束——不必等到有東西要送。
            """
            await websocket.accept()
            self.note_client()
            with self._client_lock:
                self._ws_clients += 1
            last_version = -1
            last_messages_version = None  # None 代表第一次，會帶上完整對話

            async def watch_disconnect() -> None:
                """只為了偵測斷線而收訊息。前端目前不送任何東西，這是刻意的：
                能不能活由協定層的 pong 決定，不依賴前端要記得送心跳
                （JS 計時器在背景分頁會被節流，拿它當存活判準會誤踢）。"""
                while True:
                    message = await websocket.receive()
                    if message.get("type") == "websocket.disconnect":
                        return
                    # 收到什麼都算一次「還活著」，以後前端要加心跳也不必改這裡
                    self.note_client()

            watcher = asyncio.ensure_future(watch_disconnect())
            try:
                while not watcher.done():
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
            finally:
                watcher.cancel()
                with self._client_lock:
                    self._ws_clients = max(0, self._ws_clients - 1)

        # ── 影像與地圖 ────────────────────────────────────

        @app.get("/video")
        async def video(request: Request) -> StreamingResponse:
            return StreamingResponse(
                self._mjpeg_generator(request),
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

        @app.get("/api/hardware")
        async def api_hardware() -> JSONResponse:
            """周邊硬體偵測。★ 刻意不要求登入 ——
            這是純唯讀的診斷資訊，而「東西壞了看不出來」比「被人看到有幾個 USB」嚴重。"""
            items = await asyncio.get_running_loop().run_in_executor(None, self.hardware_status)
            return JSONResponse({"success": True, "items": items})

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

        @app.delete("/api/maps/{map_id}", dependencies=admin_only)
        async def api_delete_map(map_id: str) -> JSONResponse:
            """刪除地圖（建壞的、建到一半失敗的都可以刪）

            安全檢查刻意放在 map_service_cc 那端而不是這裡：
            它才知道現在是不是在建圖、目前載入的是哪張。
            HMI 這層只負責轉發與呈現結果。
            """
            client = self.service_clients.get("delete_map")
            if client is None or not client.wait_for_service(timeout_sec=3.0):
                return JSONResponse(
                    {"success": False, "message": "delete_map 服務不存在（導航堆疊沒啟動？）"},
                    status_code=503,
                )
            req = DeleteMap.Request()
            req.map_id = map_id
            try:
                res = self._await_future(client.call_async(req), timeout=15.0)
            except Exception as exc:  # noqa: BLE001
                return JSONResponse({"success": False, "message": f"刪除失敗：{exc}"}, status_code=500)
            ok = bool(res and res.success)
            return JSONResponse(
                {"success": ok, "message": (res.message if res else "無回應")},
                status_code=200 if ok else 409,
            )

        @app.get("/api/maps/status", dependencies=admin_only)
        async def api_map_status() -> JSONResponse:
            """建圖頁面用的綜合狀態

            把「操作者想知道的事」湊成一包，前端只要打這一支就好：
            現在是建圖還是定位、載入哪張圖、圖多大、建了多少格、有沒有建圖作業在跑。
            """
            meta = self.state.map_meta_copy()
            mode = self._query_nav_mode()
            jobs = self.state.jobs_copy()
            # 欄位名是 "action" 不是 "kind"（jobs_copy() 產出的結構）。
            # 之前寫成 kind，篩選永遠回 None：畫面上「建圖作業」一直顯示
            # 「尚未開始」，使用者連按三次都失敗卻毫不知情，走完才發現存不了。
            mapping_job = next(
                (j for j in jobs if j.get("action") == "create_map" and j.get("status") == "running"),
                None,
            )
            # 沒有進行中的作業時，把最近一次失敗帶出去，讓畫面能講出原因，
            # 而不是只顯示「尚未開始」——失敗和沒開始是兩件完全不同的事。
            last_failed = None
            if mapping_job is None:
                fails = [j for j in jobs
                         if j.get("action") == "create_map" and j.get("status") == "failed"]
                if fails:
                    last_failed = max(fails, key=lambda j: j.get("started_at") or 0)
            known = None
            if meta and meta.get("known_cells") is not None:
                known = meta.get("known_cells")
            return JSONResponse(
                {
                    "mode": mode,                       # mapping / localization / unknown
                    "current_map": (self.state.snapshot().get("system") or {}).get("map_id"),
                    "mapping_job": mapping_job,         # None 表示沒有建圖作業在跑
                    "last_failed_job": last_failed,     # 最近一次失敗，讓畫面講得出原因
                    "map_meta": meta,                   # 寬高、解析度、原點
                    "known_cells": known,
                }
            )

        # ── 系統節點開關 ─────────────────────────────────

        @app.get("/api/system/status", dependencies=admin_only)
        async def api_system_status() -> JSONResponse:
            return JSONResponse({"units": self.system_status()})

        @app.post("/api/system/{unit}/{action:path}", dependencies=admin_only)
        async def api_system_control(unit: str, action: str) -> JSONResponse:
            # action 可能是 "start"、"stop"，或 "start:mapping" 這種帶啟動方式的形式。
            # 用 {action:path} 而不是 {action}，冒號才不會被路由切掉。
            ok, msg = self.system_control(unit, action)
            return JSONResponse({"success": ok, "message": msg}, status_code=200 if ok else 400)

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
            """全域定位：把粒子撒滿整張圖再收斂。

            ⚠️ 走廊裡不要用這支，用下面的 /api/localize/here。
            理由見 SetPoseRequest 的說明（沿軸歧義 ＋ 會繞圈撞牆）。
            """
            job_id = self._start_action("global_localization", GlobalLocalization.Goal(), "全域定位")
            return JSONResponse({"success": True, "message": "全域定位已開始", "job_id": job_id})

        @app.post("/api/localize/here", dependencies=admin_only)
        async def api_localize_here(req: SetPoseRequest) -> JSONResponse:
            """把定位設到一個已知位置，不做全域搜尋（2026-08-06 新增）。

            兩步：發 /initialpose 給 AMCL 一個先驗，再用 /align_pose 做
            不需移動的掃描對齊。走廊裡這是唯一可靠的定位重設方式——
            全域定位會沿長軸收斂到錯的那一段（8/06 實測錯 3 公尺，
            而掃描吻合度還顯示 90.4%）。
            """

            def work():
                # ── 1. 決定目標位姿 ──
                if req.waypoint_id:
                    res = self._call_service("list_waypoints", ListWaypoints.Request())
                    hit = None
                    for w in res.waypoints_info:
                        if w.waypoint_id == req.waypoint_id:
                            hit = w
                            break
                    if hit is None:
                        return {"success": False,
                                "message": f"找不到地點 {req.waypoint_id}"}
                    pose = hit.pose
                    where = hit.waypoint_name or req.waypoint_id
                else:
                    pose = make_pose(req.x, req.y, req.yaw)
                    where = f"({req.x:.2f}, {req.y:.2f})"

                # ── 2. 發初始位姿 ──
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = "map"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.pose = pose
                # 對角線：x/y 各 0.25（≈0.5 m 標準差）、yaw 0.0685（≈15 度）。
                # 這是 RViz「2D Pose Estimate」的預設值——表達「知道大概在哪
                # 但不精確」。設太小 AMCL 會拒絕修正自己的誤差，設太大等於沒設。
                msg.pose.covariance[0] = 0.25
                msg.pose.covariance[7] = 0.25
                msg.pose.covariance[35] = 0.0685
                self.initialpose_pub.publish(msg)

                if not req.align:
                    return {"success": True, "message": f"已把定位設到「{where}」"}

                # ── 3. 掃描對齊 ──
                # 等 AMCL 把 initialpose 吸收進粒子群再對齊，否則對齊的是舊位姿。
                time.sleep(0.5)
                client = self.service_clients.get("align_pose")
                if client is None or not client.service_is_ready():
                    return {"success": True,
                            "message": f"已把定位設到「{where}」；"
                                       "/align_pose 服務不在，略過掃描對齊"}
                ares = self._call_service("align_pose", Trigger.Request())
                return {"success": True,
                        "message": f"已把定位設到「{where}」；掃描對齊：{ares.message}"}

            return await self._guard(work)

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

        # ★★ 2026-08-14：拿掉 admin_only —— 緊急停止不可以需要登入。★★
        #
        # 原本它跟導航、建圖一起被 admin_only 保護。那個分類是錯的：
        # 其他端點保護的是「不要讓路人亂開車」，而這一顆的作用是**讓車停下來**。
        # 登入 12 小時後過期（admin_session_hours），而展示當天平板可能整天開著
        # —— 真的要急停時跳出登入框，是最糟的失敗方式。
        #
        # 濫用風險評估過：最壞情況是有人惡意讓車停下來，那正是安全的方向。
        # 對照「該停停不下來」，這個代價完全值得。
        @app.post("/api/estop")
        async def api_estop() -> JSONResponse:
            """全域緊急停止：**一個動作停下所有會讓車子移動的來源**

            先前唯一的停止鍵在遙控頁，而且只停遙控——導航中或重播教導路徑
            時按它完全沒用，操作者得先切到地圖分頁、找到作業清單、按取消，
            而那期間車子還在開。展示時評審站在旁邊，這個延遲不能接受。

            這裡一次處理三個來源，而且**順序有意義**：
              1. 先送零速（最快讓輪子停下，不必等任何服務回應）
              2. 再取消所有進行中的作業（導航、建圖、教導路徑重播）
              3. 最後回報哪些被取消了

            即使取消服務沒回應，第 1 步的零速加上遙控看門狗（0.6 秒）
            也會讓車停住——安全動作不可以依賴任何一個會失敗的呼叫。
            """
            self.apply_teleop(0.0, 0.0)

            cancelled, failed = [], []
            for job in self.state.jobs_copy():
                if job.get("status") in ("running", "pending"):
                    jid = job.get("job_id", "")
                    (cancelled if self.cancel_job(jid) else failed).append(
                        job.get("label") or jid
                    )

            # ★ 2026-08-17：登記簿之外的目標也要停（語音發起的導航就在這裡）
            broadcast = self._cancel_all_action_goals()

            self.get_logger().warn(
                f"緊急停止：已送零速，取消 {len(cancelled)} 個作業"
                + (f"，另對 {len(broadcast)} 個動作送出取消全部（{'、'.join(broadcast)}）" if broadcast else "")
                + (f"，{len(failed)} 個取消失敗" if failed else "")
            )
            if cancelled:
                msg = "已緊急停止，並取消：" + "、".join(cancelled)
            else:
                msg = "已緊急停止（當時沒有進行中的作業）"
            if failed:
                msg += f"　⚠ 這些取消失敗：{'、'.join(failed)}"
            return JSONResponse({"success": True, "message": msg,
                                 "cancelled": cancelled, "failed": failed})

        @app.post("/api/teleop/rearmask", dependencies=admin_only)
        async def api_teleop_rearmask(req: RearMaskRequest) -> JSONResponse:
            """開關雷達後方扇形遮罩。

            遙控建圖時操作者常常走在車子後方，會被雷達掃進去變成移動的假障礙，
            污染地圖也干擾 scan matching。lslidar 驅動本身支援
            angle_disable_min/max（單位 0.01 度），直接在驅動裡裁掉最便宜。
            """
            ok, msg = self.set_lidar_rear_mask(req.enabled, req.half_angle_deg)
            return JSONResponse({"success": ok, "message": msg}, status_code=200 if ok else 500)


        # ── 教導-重現路徑 ──────────────────────────────────
        #
        # 這台車最可靠的移動方式：錄下人開過的位姿，之後用純追蹤重播。
        # 路徑是人開過的所以物理上保證可行，繞開了自動規劃在窄走廊的
        # 各種麻煩（MPPI 視野不足、容差疊加、K-turn 做不出來）。

        def _taught_unavailable() -> JSONResponse:
            return JSONResponse(
                {
                    "success": False,
                    "message": "教導路徑需要重建 smartnav_msgs（colcon build --packages-select "
                               "smartnav_msgs smartnav_navigation_cc）後重啟所有節點",
                },
                status_code=501,
            )

        @app.get("/api/paths", dependencies=admin_only)
        async def api_list_paths() -> JSONResponse:
            if not TAUGHT_PATH_AVAILABLE:
                return _taught_unavailable()
            req = ListTaughtPaths.Request()
            req.current_map_only = True
            resp = await asyncio.to_thread(self._call_service, "list_taught_paths", req)
            if resp is None:
                return JSONResponse(
                    {"success": False, "message": "教導路徑節點沒有回應（path_teach_cc 有啟動嗎？）",
                     "paths": []},
                    status_code=503,
                )
            return JSONResponse({
                "success": bool(resp.success),
                "message": resp.message,
                "paths": [
                    {
                        "path_id": p.path_id, "name": p.name, "map_id": p.map_id,
                        "num_points": int(p.num_points), "length_m": round(float(p.length_m), 2),
                        "num_cusps": int(p.num_cusps), "source": p.source,
                        "created_at": p.created_at,
                    }
                    for p in resp.paths
                ],
            })

        @app.post("/api/paths/record", dependencies=admin_only)
        async def api_record_path(req: RecordPathRequest) -> JSONResponse:
            """錄製控制：start / stop / cancel

            錄製期間節點只是被動記錄位姿，不送任何速度指令，所以跟遙控
            完全不衝突——使用者照常用方向鍵或鍵盤把車開一遍。
            """
            if not TAUGHT_PATH_AVAILABLE:
                return _taught_unavailable()
            actions = {
                "start": RecordPath.Request.START,
                "stop": RecordPath.Request.STOP,
                "cancel": RecordPath.Request.CANCEL,
            }
            if req.action not in actions:
                return JSONResponse(
                    {"success": False, "message": f"未知的動作 {req.action}"}, status_code=400
                )
            if req.action == "stop" and not (req.name or "").strip():
                return JSONResponse(
                    {"success": False, "message": "請先輸入路徑名稱再結束錄製"}, status_code=400
                )
            r = RecordPath.Request()
            r.action = actions[req.action]
            r.name = (req.name or "").strip()
            resp = await asyncio.to_thread(self._call_service, "record_path", r)
            if resp is None:
                return JSONResponse(
                    {"success": False, "message": "教導路徑節點沒有回應（path_teach_cc 有啟動嗎？）"},
                    status_code=503,
                )
            out = {"success": bool(resp.success), "message": resp.message}
            if req.action == "stop" and resp.success:
                out.update({"path_id": resp.path_id, "num_points": int(resp.num_points),
                            "length_m": round(float(resp.length_m), 2)})
            return JSONResponse(out, status_code=200 if resp.success else 400)

        @app.delete("/api/paths/{path_id}", dependencies=admin_only)
        async def api_delete_path(path_id: str) -> JSONResponse:
            if not TAUGHT_PATH_AVAILABLE:
                return _taught_unavailable()
            r = DeleteTaughtPath.Request()
            r.path_id = path_id
            resp = await asyncio.to_thread(self._call_service, "delete_taught_path", r)
            if resp is None:
                return JSONResponse(
                    {"success": False, "message": "教導路徑節點沒有回應"}, status_code=503
                )
            return JSONResponse({"success": bool(resp.success), "message": resp.message},
                                status_code=200 if resp.success else 400)

        @app.post("/api/paths/follow", dependencies=admin_only)
        async def api_follow_path(req: FollowPathRequest) -> JSONResponse:
            """重播教導路徑。與導航一樣走作業機制，進度在作業清單裡看。"""
            if not TAUGHT_PATH_AVAILABLE:
                return _taught_unavailable()
            goal = FollowTaughtPath.Goal()
            goal.path_id = req.path_id
            goal.reverse = bool(req.reverse)
            goal.speed_scale = float(req.speed_scale or 0.0)
            label = ("反向重播「%s」" if req.reverse else "重播「%s」") % (req.name or req.path_id)
            job_id = self._start_action("follow_taught_path", goal, label)
            return JSONResponse({"success": True, "message": label + "：已送出", "job_id": job_id})

        @app.post("/api/paths/plan", dependencies=admin_only)
        async def api_plan_path(req: PlanPathRequest) -> JSONResponse:
            """把地圖上點選的一串位置規劃成教導路徑

            逐段呼叫 nav2 的 /compute_path_to_pose。刻意不自己做曲線內插——
            SmacPlannerHybrid + REEDS_SHEPP 本來就會產生符合最小轉彎半徑、
            含折返點的阿克曼可行路徑。規劃器沒問題，出問題的是 MPPI 控制器。
            """
            if not TAUGHT_PATH_AVAILABLE:
                return _taught_unavailable()
            if not (req.name or "").strip():
                return JSONResponse({"success": False, "message": "請先輸入路徑名稱"},
                                    status_code=400)
            if not req.points:
                return JSONResponse({"success": False, "message": "請先在地圖上點選至少一個位置"},
                                    status_code=400)
            r = PlanTaughtPath.Request()
            r.name = req.name.strip()
            r.start_from_robot = bool(req.start_from_robot)
            for pt in req.points:
                pose = Pose()
                pose.position.x = float(pt.get("x", 0.0))
                pose.position.y = float(pt.get("y", 0.0))
                yaw = float(pt.get("yaw", 0.0))
                pose.orientation.z = math.sin(yaw * 0.5)
                pose.orientation.w = math.cos(yaw * 0.5)
                r.waypoints.append(pose)
            # 規劃要逐段呼叫 nav2，段數多時會慢，逾時放寬
            resp = await asyncio.to_thread(self._call_service, "plan_taught_path", r, 60.0)
            if resp is None:
                return JSONResponse(
                    {"success": False, "message": "規劃逾時或節點沒有回應"}, status_code=503
                )
            out = {"success": bool(resp.success), "message": resp.message}
            if resp.success:
                out.update({"path_id": resp.path_id, "num_points": int(resp.num_points),
                            "length_m": round(float(resp.length_m), 2),
                            "num_cusps": int(resp.num_cusps)})
            return JSONResponse(out, status_code=200 if resp.success else 400)

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
            """抓當下這一張影格。前端「從畫面拍照」用這支

            影像訂閱是按需建立的（見 _client_tick），所以這裡要先舉手說
            「我要影格」，再給它一點時間把訂閱建起來、收到第一張。
            正常情況下拍照時 /video 已經開著，這段等待會直接跳過。
            """
            self.note_video_interest()
            deadline = time.monotonic() + 2.0
            while True:
                with self._frame_lock:
                    jpeg = self._latest_jpeg
                if jpeg is not None or time.monotonic() >= deadline:
                    break
                await asyncio.sleep(0.1)
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

        @app.post("/api/tts_state")
        async def api_tts_state(req: Request) -> JSONResponse:
            """平板回報 speechSynthesis 的起訖。★ 蓄意不需要登入：

            它不會讓車子動、不改任何狀態，只是把「我正在唸」告訴 ASR。
            要求登入反而會讓未登入的展示平板變成「機器人聽自己講話」。
            """
            try:
                body = await req.json()
            except Exception:
                body = {}
            active = bool(body.get("active"))
            self.playback_pub.publish(Bool(data=active))
            self.state.set_system(speaking=active)
            return JSONResponse({"ok": True, "active": active})

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

    async def _mjpeg_generator(self, request: Optional[Request] = None):
        """產生 multipart MJPEG 串流

        相機關掉時（迎賓機目前的常態）原本照樣以 video_fps=12 一直重送同一張
        「NO CAMERA SIGNAL」佔位圖——每秒 12 次穿過 starlette 的 chunked 編碼，
        內容還完全一樣。這裡改成沒有真影格時降到 1 fps：畫面上看不出差別
        （本來就是靜態圖），HTTP 執行緒的工作量降到十二分之一。

        ★ 2026-08-17：串流的存在本身現在是一個訊號——它開著，影像訂閱才會
          建立（見 _client_tick）。所以進出都要記帳，而且要用 try/finally，
          不然對方一斷線就永遠漏掉一次減量，計數只增不減。
          乾淨的斷線不必自己偵測：starlette 的 StreamingResponse 內部就有一條
          listen_for_disconnect，收到 http.disconnect 會把產生器取消掉，
          finally 照樣會執行。（自己再輪詢一次 is_disconnected() 反而會跟它
          搶同一個 receive 通道。）
        """
        interval = 1.0 / max(self.video_fps, 1.0)
        placeholder = self._placeholder_jpeg()
        with self._client_lock:
            self._video_viewers += 1
        try:
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
        finally:
            with self._client_lock:
                self._video_viewers = max(0, self._video_viewers - 1)

    @staticmethod
    def _placeholder_jpeg() -> bytes:
        """相機還沒有資料時顯示的佔位圖——比讓 <img> 一直轉圈好判讀"""
        img = np.full((360, 640, 3), 30, dtype=np.uint8)
        cv2.putText(img, "NO CAMERA SIGNAL", (120, 190), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (90, 90, 200), 2)
        ok, buf = cv2.imencode(".jpg", img)
        return buf.tobytes() if ok else b""

    def _run_server(self) -> None:
        """啟動 uvicorn

        ★ ws_ping_interval / ws_ping_timeout（2026-08-17）
          這是「踢掉斷線用戶端」真正的判準。伺服器每 interval 秒送一個
          **WebSocket 協定層** 的 ping，對方要在 timeout 秒內回 pong，
          否則連線關閉。

          為什麼用協定層而不是自己在 JS 裡寫心跳：pong 是瀏覽器網路層自動
          回的，**不經過 JavaScript**。平板螢幕關掉、分頁切到背景、JS 計時器
          被節流甚至凍結，pong 照樣會回 —— 也就是「平板只是放著沒操作」
          絕對不會被誤踢。真正收不到 pong 的只有「連線實際上已經沒了」：
          走出 WiFi 範圍、平板關機、瀏覽器行程被系統回收。
          （這正是舊寫法抓不到的那一類：TCP 送出去不會失敗，只會塞在
          緩衝區裡，所以 send 永遠不丟例外，迴圈就一直空轉。）

          誤判的代價也很低：前端 ws.onclose 會自動重連（退避上限 8 秒），
          畫面上就是右上角那顆點閃一下。

        timeout_keep_alive：閒置的 HTTP keep-alive 連線多久後關掉。
          平板一直開著頁面時 /api/* 是零星打的，連線留著只是佔 socket。
        """
        config = uvicorn.Config(
            self.app, host=self.host, port=self.port, log_level="warning",
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
