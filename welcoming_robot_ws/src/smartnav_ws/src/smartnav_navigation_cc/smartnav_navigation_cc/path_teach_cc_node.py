#!/usr/bin/env python3
"""教導-重現（Teach & Repeat）：錄下人開過的路徑，之後照著重播。

## 為什麼要這個

自動規劃在這台車上有幾個難以繞開的問題：

- 阿克曼底盤最小轉彎半徑 0.8 m，測試環境走廊只有 0.99 m 寬，掉頭需要 1.6 m
- MPPI 的預測視野 = time_steps(20) x model_dt(0.1) = 2.0 秒，
  以 vx_max 0.25 m/s 計算只能看到 0.5 公尺 —— **比完成一次轉彎所需的
  1.26 公尺還短**，控制器看不到「轉完之後會如何」
- 規劃器 tolerance 0.5 m 與 goal_checker 的 xy 0.25 m 是串接疊加的，
  最壞情況終點誤差 0.75 公尺
- Pi 4 的 CPU 吃緊，MPPI 每週期要算 400 條軌跡 x 20 步

而人開過的路徑有一個很強的性質：**它物理上保證可行**。不會出現
「理論上算得出來、實際開不過去」的路徑，也不需要即時最佳化。
對固定場地的展示（大廳 -> 接待點 -> 洗手間）這是更可靠的做法。

## 錄的是位姿不是指令 —— 這個差別很關鍵

「錄下 cmd_vel 再重播」是**開迴路**：輪胎打滑、地板材質、電池電壓下降
都會讓實際軌跡偏離，而且沒有任何機制發現偏了。走 10 公尺累積 1 度的
朝向誤差就是 17 公分側向偏移，在 0.99 m 走廊裡足以撞牆。

這裡錄的是 `map -> base_footprint` 的**位姿序列**，重播時用 AMCL 的當前
位置算橫向偏差，用純追蹤持續修回去。誤差被綁定在 AMCL 的定位精度上
（走廊裡約 5~10 公分），**不會累積**。

## 純追蹤（pure pursuit）與阿克曼

在路徑上取一個前視點，車體座標系下該點為 (x, y)，前視距離 L_d，則

    曲率 k = 2y / L_d^2          角速度 w = v * k

k 會被夾在 1/0.8 = 1.25（最小轉彎半徑的物理極限）以內。

前視距離採「隨速度調整」：L_d = clamp(k_v * |v| + L_min, L_min, L_max)。
太短會蛇行、太長會切彎（轉彎時內切撞牆），窄走廊寧可短一點。

## 折返點（cusp）

錄製時記錄每個點的行進方向（+1 前進 / -1 後退）。方向改變的地方就是
折返點。重播到折返點時**先停穩再換方向** —— 直接反向會讓底盤的
速度指令瞬間變號，實機上會頓一下，而且純追蹤的幾何在換向瞬間不成立。

這也是這套方案能做多段掉頭（K-turn）的原因：你開的時候怎麼折返，
重播就怎麼折返，不需要控制器自己想出來。

## 障礙處理

    前方 slow_distance 內有障礙 -> 減速
    前方 stop_distance 內有障礙 -> 嘗試橫向繞開，繞不過去就停下等待
    等待超過 wait_timeout       -> 放棄，回報失敗

繞開的可行性是**用即時掃描算出來的**，不是寫死的。0.99 m 走廊裡
人站著（約 0.4 m 寬）+ 車身 0.37 m = 0.77 m，只剩 0.22 m 餘裕，
物理上不該硬繞；大廳寬敞處才繞得過去。
"""
import json
import math
import os
import threading
import time
import uuid
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from smartnav_msgs.action import FollowTaughtPath
from smartnav_msgs.msg import TaughtPathInfo
from smartnav_msgs.srv import (
    DeleteTaughtPath,
    ListTaughtPaths,
    PlanTaughtPath,
    RecordPath,
)


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def norm_angle(a: float) -> float:
    """把角度收斂到 (-pi, pi]。差角運算沒有這個一定會在 ±180 度附近出錯"""
    return math.atan2(math.sin(a), math.cos(a))


class PathPoint:
    """路徑上的一個點。direction 是錄製當下的行進方向，重播時要照做"""

    __slots__ = ("x", "y", "yaw", "direction")

    def __init__(self, x: float, y: float, yaw: float, direction: int = 1):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.direction = direction      # +1 前進 / -1 後退

    def as_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "yaw": self.yaw, "d": self.direction}

    @staticmethod
    def from_dict(d: dict) -> "PathPoint":
        return PathPoint(float(d["x"]), float(d["y"]), float(d["yaw"]),
                         int(d.get("d", 1)))


class PathTeachNode(Node):

    def __init__(self):
        super().__init__("path_teach_cc_node")
        cb = ReentrantCallbackGroup()

        # ── 參數 ──────────────────────────────────────────────
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("cmd_topic", "cmd_vel_smoothed")
        self.declare_parameter("scan_topic", "scan")
        # 錄製時要「聽」哪個話題判斷行進方向。
        # 這跟 cmd_topic 是**不同的東西**：cmd_topic 是重播時本節點自己
        # 發布的位置（cmd_vel_smoothed，之後還會經過 steering_trim 與
        # collision_monitor）；錄製時要聽的是**真正驅動底盤的那一個**，
        # 也就是遙控直接發布的 /cmd_vel。
        # 2026-08-03 踩過：訂成 cmd_topic 的話收不到遙控指令，方向偵測全失效。
        self.declare_parameter("record_cmd_topic", "cmd_vel")

        # 錄製：距離或角度變化達到門檻才記一個點。
        # 用「走了多遠」而不是「過了多久」當觸發條件，車停著時才不會
        # 在原地堆出幾百個重複的點。
        self.declare_parameter("record_spacing_m", 0.05)
        self.declare_parameter("record_angle_rad", 0.10)
        # 兩個取樣點之間的位移上限。50 Hz tick、車速最高 0.25 m/s -> 正常 5 mm，
        # 留到 0.30 m 是給漏拍的餘裕；超過就是 AMCL 跳了。
        self.declare_parameter("jump_dist_limit_m", 0.30)
        self.declare_parameter("max_points", 20000)
        # 存檔前把過短的方向段併掉（假折返點）的門檻。
        #
        # ★ 2026-08-03 參數化：原本是 _merge_short_segments 的預設引數、
        # 硬編碼 0.35 且無法在啟動時調整。這會擋掉「在窄轉角錄製三點轉向」
        # 這個計畫——淨寬 0.967 m 的走廊裡，單次前進／後退的位移很可能
        # 不到 0.35 m（對照：脫困 v1 沿路徑退也只退了 0.38 m），
        # 人示範的修正動作會被整段當成雜訊併掉，跟修好方向偵測之前一樣白做。
        #
        # 要錄三點轉向時把它降到 0.12~0.15：
        #   ros2 param set /path_teach_cc_node merge_min_segment_m 0.15
        # 一般錄製維持 0.35（理由見 _merge_short_segments 的 docstring）。
        #
        # ★ 2026-08-04：這個值**不快取**，_merge_short_segments 每次存檔時
        # 現讀。原本跟其他參數一樣在 __init__ 讀進 self.merge_min_seg，
        # 於是上面那行 `ros2 param set` 只改到參數伺服器、不會改變行為——
        # 而它正是本參數存在的唯一理由。實測代價：錄了一趟含三點轉向的
        # 路徑，後退段 0.28 m 仍被 0.35 的舊值併掉，兩個折返點全沒了。
        # `ros2 param get` 讀回 0.15 只證明伺服器存了值，不證明節點會用它。
        self.declare_parameter("merge_min_segment_m", 0.35)

        # 重播
        self.declare_parameter("follow_speed", 0.15)
        self.declare_parameter("reverse_speed", 0.10)
        self.declare_parameter("lookahead_min_m", 0.30)
        self.declare_parameter("lookahead_max_m", 0.80)
        self.declare_parameter("lookahead_k", 1.2)      # L_d = k*|v| + min
        self.declare_parameter("min_turning_radius", 0.80)
        self.declare_parameter("goal_tolerance_m", 0.12)
        self.declare_parameter("control_rate", 20.0)
        # 偏離路徑超過這個距離就中止：代表定位跑掉或被推走了，
        # 硬追回去反而危險（純追蹤在大偏差下會畫出很大的弧）
        self.declare_parameter("max_cross_track_m", 0.60)
        self.declare_parameter("cusp_pause_sec", 0.6)

        # 障礙
        self.declare_parameter("obstacle_slow_m", 1.20)
        # 0.55 -> 0.45（2026-08-03 實機）。這個距離是從**雷達**量的，
        # 而車頭在雷達前方 0.311 m（base_footprint 在後輪軸心、車頭 0.40、
        # 雷達 0.089），所以 0.55 實際代表「車頭離障礙 0.24 m 就停」——
        # 在門口這種夾道太保守。0.45 對應車頭餘裕 0.14 m，仍然安全。
        self.declare_parameter("obstacle_stop_m", 0.45)
        # 倒車的停止距離要另外給。這兩個都是從**雷達**量的，
        # 而車頭與車尾離雷達的距離差很多（base_footprint 在後輪軸心）：
        #     車頭 0.40 - 雷達 0.089 = 比雷達再前面 0.311 m
        #     車尾 0.09 + 雷達 0.089 = 比雷達再後面 0.179 m
        # 用同一個 0.45 的話：前進時保險桿餘裕 0.14 m、倒車時 0.27 m
        # —— 倒車保守了一倍，實測在走廊裡會卡住不動。
        # 0.33 讓倒車的保險桿餘裕也是 0.15 m，跟前進一致。
        self.declare_parameter("obstacle_stop_reverse_m", 0.33)
        # 0.26 -> 0.22（2026-08-03 實機）。車半寬 0.185，0.26 等於左右各留
        # 7.5 cm。實測重播卡在門口：門框邊緣落在側向 -0.26 m，**正好壓在
        # 帶寬邊界上**，量到前方淨空 0.54 m 對上停止門檻 0.55——差 1 公分
        # 被判定阻擋，等 20 秒後放棄。
        #
        # 收到 0.22（左右各 3.5 cm 餘裕）後同一幀量到 0.56 m，通過。
        #
        # 更根本的理由：**教導路徑是人開過的，物理上保證通得過**。
        # 這裡的檢查只該攔「錄製時不存在的東西」（人、臨時障礙），
        # 不該比實際車身寬那麼多而把門框當障礙。
        self.declare_parameter("obstacle_half_width_m", 0.22)
        self.declare_parameter("avoid_max_offset_m", 0.25)
        self.declare_parameter("avoid_step_m", 0.05)
        self.declare_parameter("avoid_clearance_m", 0.10)
        self.declare_parameter("wait_timeout_sec", 20.0)
        # ── 脫困（2026-08-03 加）─────────────────────────────
        #
        # 原本擋住就只會等，等滿 wait_timeout 才放棄。**障礙是牆的時候，
        # 等待永遠不會成功。** 實測倒車過轉角時右後輪頂到牆，車子等了
        # 20 秒然後放棄——而它需要的只是「往前退開一點、換個角度再進」，
        # 那正是人開車過窄轉角的標準做法（三點轉向）。
        #
        # 動態障礙（人擋路）該等、靜態障礙（牆）該脫困，兩者外觀一樣，
        # 沒辦法從單幀掃描分辨。所以策略是「先等一下下，還在就試著脫困」：
        # 人通常幾秒內會讓開，牆不會。
        self.declare_parameter("escape_after_sec", 4.0)     # 等這麼久還沒通就試脫困
        self.declare_parameter("escape_distance_m", 0.35)   # 沿路徑往回退多遠
        # 3 -> 10（2026-08-03 實測）。使用者回報「修正動作是對的，
        # 但三次前後退太少」。窄轉角的三點轉向本來就要來回好幾趟才轉得夠，
        # 每次只轉 20 度，要轉過 60~90 度的彎需要 4~6 次。
        self.declare_parameter("max_escapes", 10)
        # 進度檢查：索引這麼久沒前進就當卡住。
        #
        # 2026-08-03 實測：脫困原本只掛在 waiting 狀態，但車子頂到牆時
        # 淨空還沒低到 stop 門檻，狀態是 **slowing**——它用 30% 速度一直
        # 頂著牆，輪子打滑、位姿慢慢漂，直到偏離超過 60 cm 才被攔下。
        # 從頭到尾索引都停在 65/149。
        #
        # 「有沒有在前進」比「看到什麼」可靠得多：不管是頂到牆、輪子打滑、
        # 還是控制器算不出可行解，索引不動就是卡住。
        self.declare_parameter("no_progress_sec", 5.0)
        # no_progress_sec 內至少要沿路徑推進這麼多才算「有在前進」。
        # 取 8 cm：slowing 時速度 30% ≈ 0.03 m/s，5 秒能走 15 cm，
        # 留一半餘裕。用弧長而不是索引數，才不會被點距影響
        # （教導路徑 5 cm、nav2 規劃路徑 15 cm，差三倍）。
        self.declare_parameter("stuck_min_advance_m", 0.08)
        # base_footprint -> laser 的 x 偏移（查 TF：0.089）。
        # ★ 不要跟 0.311 搞混 —— 那是「雷達到車頭保險桿」的距離。
        #   用錯會讓每個雷射點多推 0.222 m；在走廊裡沿長軸看不出來
        #   （平行牆的縱向歧義），進到房間才會冒出來。
        self.declare_parameter("laser_x_offset_m", 0.089)
        # 單側空隙低於這個值就啟動貼牆排斥，把追蹤目標往外推。
        # 做成「靠太近才推開」而不是「隨時往中間拉」：後者會跟路徑追蹤打架，
        # 有些路徑本來就該貼一邊走。
        self.declare_parameter("wall_keepout_m", 0.15)
        self.declare_parameter("wall_push_max_m", 0.12)

        p = self.get_parameter
        self.map_frame = p("map_frame").value
        self.robot_frame = p("robot_frame").value
        self.record_spacing = float(p("record_spacing_m").value)
        self.record_angle = float(p("record_angle_rad").value)
        self.max_points = int(p("max_points").value)
        self.follow_speed = float(p("follow_speed").value)
        self.reverse_speed = float(p("reverse_speed").value)
        self.la_min = float(p("lookahead_min_m").value)
        self.la_max = float(p("lookahead_max_m").value)
        self.la_k = float(p("lookahead_k").value)
        self.min_radius = float(p("min_turning_radius").value)
        self.jump_dist_limit = float(p("jump_dist_limit_m").value)
        # merge_min_segment_m 刻意**不**在這裡快取（見宣告處）：它要能在
        # 錄製前用 `ros2 param set` 臨時調整，快取了就永遠讀不到新值。
        self.goal_tol = float(p("goal_tolerance_m").value)
        self.control_dt = 1.0 / max(1.0, float(p("control_rate").value))
        self.max_xte = float(p("max_cross_track_m").value)
        self.cusp_pause = float(p("cusp_pause_sec").value)
        self.obs_slow = float(p("obstacle_slow_m").value)
        self.obs_stop = float(p("obstacle_stop_m").value)
        self.obs_stop_rev = float(p("obstacle_stop_reverse_m").value)
        self.obs_half_w = float(p("obstacle_half_width_m").value)
        self.avoid_max = float(p("avoid_max_offset_m").value)
        self.avoid_step = float(p("avoid_step_m").value)
        self.avoid_clear = float(p("avoid_clearance_m").value)
        self.wait_timeout = float(p("wait_timeout_sec").value)
        self.escape_after = float(p("escape_after_sec").value)
        self.escape_dist = float(p("escape_distance_m").value)
        self.max_escapes = int(p("max_escapes").value)
        self.no_progress = float(p("no_progress_sec").value)
        self.stuck_min_advance = float(p("stuck_min_advance_m").value)
        self.laser_x = float(p("laser_x_offset_m").value)
        self.wall_keepout = float(p("wall_keepout_m").value)
        self.wall_push_max = float(p("wall_push_max_m").value)

        # ── 狀態 ──────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=20, static_qos=1)

        self._db_lock = threading.Lock()
        self._paths: dict = {}          # path_id -> {"meta": {...}, "points": [PathPoint]}
        self._db_dir = os.path.join(os.path.expanduser("~"), ".smartnav", "path_database")
        os.makedirs(self._db_dir, exist_ok=True)
        self._db_file = os.path.join(self._db_dir, "paths.json")
        self._load_db()

        self._recording = False
        self._rec_points: List[PathPoint] = []
        self._rec_lock = threading.Lock()

        self._scan: Optional[LaserScan] = None
        self._scan_lock = threading.Lock()

        self.current_map = ""
        self._following = False
        self._active_path_id = ""
        self._rec_jumps = 0
        self._rec_abort_reason = ""

        # ── 介面 ──────────────────────────────────────────────
        cmd_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.cmd_pub = self.create_publisher(Twist, p("cmd_topic").value, cmd_qos)
        # 讓 RViz / HMI 看得到目前在追哪條路徑
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.path_pub = self.create_publisher(Path, "taught_path", latched)

        scan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)
        # 錄製時的行進方向：直接看送給底盤的速度指令，不要用位置反推。
        #
        # 原本從「位移向量與車頭的夾角」推方向。慢速來回修正（門口掉頭）時
        # **AMCL 的位置抖動跟真實位移同一個量級**，夾角落進模糊帶，
        # 程式就沿用上一點的方向 —— 整段 K-turn 被記成單向行駛。
        # 重播時純追蹤把「前後修正」當成一條連續弧線去追，
        # 而那條弧線的曲率遠超物理極限（實測有 25 rad/m，半徑 4 cm），
        # 車子打滿舵也走不出去。
        #
        # 速度指令的正負號是**確定的**：使用者按前進鍵就是正、後退鍵就是負。
        # 拿最不可靠的資料（AMCL 位置）去解一個已知的問題，是本末倒置。
        self._cmd_dir = 1
        # 最後一次收到有效速度指令的時刻。**目前只寫不讀**——本來想做
        # 「指令過期就不信任方向」的 timeout，但沒有實測依據可以決定
        # 多久算過期：設太短，正常的減速停頓會被誤判成方向失效；
        # 設太長就沒有意義。留著時間戳是為了之後要補判斷時不用再動 callback。
        # 在補上之前，_cmd_dir 會無限期沿用最後一次的方向——錄製時操作者
        # 一直在遙控，所以實務上不會遇到，但換控制來源時要記得這件事。
        self._cmd_dir_t = 0.0
        # 一串脫困共用的轉向（見 _do_escape）。每次真的往前推進就清掉，
        # 讓下一次卡住重新判斷。
        self._escape_steer: Optional[float] = None
        from rclpy.qos import ReliabilityPolicy as _RP
        self.create_subscription(
            Twist, p("record_cmd_topic").value, self._cmd_cb,
            QoSProfile(depth=1, reliability=_RP.RELIABLE, history=HistoryPolicy.KEEP_LAST))
        self.create_subscription(LaserScan, p("scan_topic").value,
                                 self._scan_cb, scan_qos, callback_group=cb)
        self.create_subscription(String, "current_map", self._map_cb, latched,
                                 callback_group=cb)

        self.create_service(RecordPath, "record_path", self._record_cb, callback_group=cb)
        self.create_service(ListTaughtPaths, "list_taught_paths", self._list_cb,
                            callback_group=cb)
        self.create_service(DeleteTaughtPath, "delete_taught_path", self._delete_cb,
                            callback_group=cb)
        self.create_service(PlanTaughtPath, "plan_taught_path", self._plan_cb,
                            callback_group=cb)

        # 地圖點選模式借用 nav2 的規劃器。規劃器（SmacPlannerHybrid + REEDS_SHEPP）
        # 本來就會產生符合最小轉彎半徑、含折返點的阿克曼可行路徑 —— 出問題的是
        # MPPI 控制器，不是規劃器。所以「用 nav2 規劃、用純追蹤執行」。
        self._planner_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose", callback_group=cb)

        # 規劃前先清一次全域成本地圖的 obstacle 層（2026-08-05）。
        #
        # 8/04 實測：清空前 13 個取樣點中 11 個 = 254（走廊全段不可通行），
        # 清空後只剩 2 個 —— 同一個規劃請求從失敗變成產出 51 點含 2 折返點
        # 的可行路徑。污染源已在 config 修掉（obstacle_layer 改吃 /scan_slam），
        # 這裡是第二道保險：靜態層有完整地圖，obstacle 層在一次任務開始時
        # 沒有任何值得保留的東西，清掉不會遺失資訊。
        self._clear_costmap_client = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap",
            callback_group=cb)

        self._action = ActionServer(
            self, FollowTaughtPath, "follow_taught_path",
            execute_callback=self._execute_follow,
            goal_callback=lambda _: (GoalResponse.REJECT if self._following
                                     else GoalResponse.ACCEPT),
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb,
        )

        # 錄製取樣。50 Hz 只是查 TF，成本很低；真正決定點密度的是距離門檻。
        self.create_timer(0.02, self._record_tick, callback_group=cb)

        self.get_logger().info(
            f"教導路徑節點啟動：資料庫 {self._db_file}，已載入 {len(self._paths)} 條路徑"
        )

    # ==================================================================
    # 資料庫
    # ==================================================================
    def _load_db(self) -> None:
        if not os.path.exists(self._db_file):
            return
        try:
            with open(self._db_file, "r", encoding="utf-8") as fp:
                raw = json.load(fp).get("paths", {})
            for pid, entry in raw.items():
                self._paths[pid] = {
                    "meta": entry.get("meta", {}),
                    "points": [PathPoint.from_dict(d) for d in entry.get("points", [])],
                }
        except (OSError, ValueError, KeyError) as exc:
            self.get_logger().error(f"讀取教導路徑資料庫失敗：{exc}")

    def _save_db(self) -> bool:
        try:
            payload = {
                "paths": {
                    pid: {"meta": e["meta"], "points": [pt.as_dict() for pt in e["points"]]}
                    for pid, e in self._paths.items()
                }
            }
            tmp = self._db_file + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fp:
                json.dump(payload, fp, ensure_ascii=False, indent=1)
            # 先寫暫存再 rename：中途斷電不會留下半個 JSON 檔把整個資料庫毀掉
            os.replace(tmp, self._db_file)
            return True
        except OSError as exc:
            self.get_logger().error(f"寫入教導路徑資料庫失敗：{exc}")
            return False

    # ==================================================================
    # 訂閱
    # ==================================================================
    def _cmd_cb(self, msg: Twist) -> None:
        """記住最後一次「有意義」的速度指令方向

        門檻 0.02 m/s：低於這個值是停車或雜訊，維持原方向不要翻轉，
        否則減速到零的那一瞬間會被記成折返點。
        """
        if abs(msg.linear.x) > 0.02:
            self._cmd_dir = 1 if msg.linear.x > 0 else -1
            self._cmd_dir_t = time.monotonic()

    def _scan_cb(self, msg: LaserScan) -> None:
        with self._scan_lock:
            self._scan = msg

    def _map_cb(self, msg: String) -> None:
        self.current_map = msg.data

    def _robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
        except Exception:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quat(tf.transform.rotation)

    # ==================================================================
    # 錄製
    # ==================================================================
    def _record_tick(self) -> None:
        """以固定頻率查位姿，但只有「走了夠遠」才真的記點"""
        if not self._recording:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        x, y, yaw = pose

        with self._rec_lock:
            if not self._rec_points:
                self._rec_points.append(PathPoint(x, y, yaw, 1))
                return

            last = self._rec_points[-1]
            dx, dy = x - last.x, y - last.y
            dist = math.hypot(dx, dy)
            dyaw = abs(norm_angle(yaw - last.yaw))

            if dist < self.record_spacing and dyaw < self.record_angle:
                return
            # 角度觸發也要有最小位移。
            #
            # record_angle_rad 是 0.10 rad = 5.7 度，而 AMCL 的 yaw 抖動本來
            # 就有這個量級。車子幾乎靜止時，光靠抖動就會不斷觸發記點，
            # 錄出「位移 4 公釐、轉 5.9 度」這種點——換算曲率 25 rad/m、
            # 轉彎半徑 4 公分，純追蹤根本追不動（實測門口就是卡在這裡）。
            #
            # 1 公分是「真的有在動」的下限：0.06 m/s 下 0.17 秒的行程，
            # 遠大於 AMCL 的位置雜訊。
            if dist < 0.01:
                return
            if len(self._rec_points) >= self.max_points:
                return

            # ── 定位跳位偵測（2026-08-03 加）──────────────────────
            #
            # 錄製的是 AMCL 位姿，而 AMCL 在走廊裡會跳。實測一條 18 m 的路徑
            # 在第 88 點出現 **3.85 公尺**的瞬移（相鄰點本該只差 5 公分），
            # 等於前後半段落在兩個相差近 4 公尺的座標系裡。重播時純追蹤會
            # 朝著跳位後的點直線開過去——穿越一段從沒走過的空間。
            #
            # 這個 tick 是 50 Hz，車子最快 0.25 m/s，兩次之間最多走 5 mm；
            # 就算漏跑幾拍也不該超過 0.3 m。超過就是定位跳了，不是車子動了。
            #
            # 同理，轉角也有物理上限：最小轉彎半徑 0.80 m 之下，
            # 走 dist 公尺最多轉 dist/0.80 弧度。超過就是位姿跳動。
            #
            # 丟掉這種點而不是照收：路徑寧可少幾個點（純追蹤本來就會內插），
            # 也不能有斷點。同時記下來，存檔時一併回報。
            # ★ 跳位必須「中止錄製」而不是「丟棄這一點」★
            #
            # 第一版寫成丟棄，測試才發現那是錯的：跳位之後車子的真實位姿
            # 距離「最後保留的點」永遠是 3.85 m，於是後續每一點都被拒收 ——
            # 錄製在跳位當下實質死掉，而操作者毫無所覺，繼續開完 18 公尺
            # 才發現只錄到 5 公尺。默默失敗比大聲失敗糟得多。
            #
            # 而且定位一旦跳過，跳位前後的點就落在兩個不同的座標系裡，
            # 這條路徑本質上已經廢了，補救沒有意義。直接中止、講清楚原因、
            # 請操作者重錄，才是對的。
            if dist > self.jump_dist_limit:
                self._rec_jumps += 1
                self._rec_abort_reason = (
                    f"定位跳位 {dist:.2f} m（相鄰取樣點正常只差 {self.record_spacing:.2f} m）。"
                    f"跳位前後的點在不同座標系，路徑已無法使用"
                )
                self._recording = False
                self.get_logger().error(f"錄製中止：{self._rec_abort_reason}")
                return
            # ── 轉角檢查已移除（2026-08-03 實機驗證後） ──────────
            #
            # 原本這裡拒收「轉角超過 dist/min_radius」的點，理由是物理上
            # 做不到。**那個檢查在實機上是災難性的**，實測一趟錄製拒收
            # 570 次後中止：
            #
            #   1. 連鎖拒收。點被拒收後「最後保留的點」不動，車子繼續走，
            #      距離累積 0.05 -> 0.10 -> ... -> 0.30，最後觸發距離上限
            #      而中止。那個「0.30 m 跳位」根本不是真的跳位。
            #      這正是我在跳位那條識破的連鎖模式，卻在這裡重犯，
            #      還在註解裡寫「不會連鎖拒收」。
            #
            #   2. 門檻在 5 cm 的粒度上無法執行：上限只有 3.6 度 + 3 度容忍，
            #      而 **AMCL 的 yaw 抖動本身就有好幾度**，正常行駛也會違反。
            #
            # 小幅 yaw 雜訊不會破壞路徑，純追蹤會平滑掉它。真正會毀掉路徑的
            # 是「位置整個跳掉」，由上面的距離檢查負責且是中止而非丟棄。
            # 一道防線就夠，兩道會互相干擾。

            # 行進方向：直接取自速度指令（見 _cmd_cb），不從位置反推。
            #
            # 演進過程（兩次都錯，記下來免得再犯）：
            #   v1  投影量 > 0.01 m -> 轉彎時側向位移大、投影趨近零，
            #       AMCL 抖動就翻轉正負號，18 m 錄出 29 個假折返點。
            #   v2  改看夾角（cos > 0.5 / < -0.5）-> 直線段沒問題，
            #       但門口慢速來回修正時位置抖動與真實位移同量級，
            #       夾角落進模糊帶而沿用舊值，**整段 K-turn 被記成單向**。
            #       重播時純追蹤要追一條 25 rad/m 的弧線（半徑 4 cm），
            #       車子打滿舵也走不出去。
            #   v3  用 cmd_vel 的正負號。使用者按前進鍵就是正、後退就是負，
            #       確定、無雜訊、而且本來就有。
            #
            # 教訓：能直接量到的量，不要從別的量反推。
            direction = self._cmd_dir
            self._rec_points.append(PathPoint(x, y, yaw, direction))

    def _smooth_path(self, pts, window=2):
        """【已停用，實測無效】對 x,y 做移動平均

        2026-08-03 實測結果：超限點數 35->15 有改善，但**最大曲率
        37.50 -> 158.00 反而變成 4 倍**，路徑長度縮了 36 公分（切彎）。
        原因是移動平均會把「車子幾乎沒動」那幾段的點擠得更近，
        外接圓公式 4A/(d1*d2*d3) 的分母趨近零，曲率就爆掉。

        真正該做的是在**錄製時**就不要記那些點（已用 dist < 0.01 擋掉一部分），
        或改用保長度的平滑（例如樣條擬合），不是事後做移動平均。

        原始說明：

        相鄰取樣點只隔 5 公分，而 AMCL 的位置雜訊約 1 公分。三點外接圓
        算出來的曲率是 8*jitter/spacing^2 = 8*0.01/0.0025 ≈ 32 rad/m ——
        看起來像半徑 3 公分的急彎，實際上只是抖動。實測一條 7.45 m 的路徑
        有 35/139 個點的幾何曲率超過物理極限 1.25 rad/m。

        純追蹤的前視距離是 0.30~0.80 m（往前 6~16 個點），本身就會平滑掉
        大部分雜訊，所以未濾波也能跑（實測循跡誤差 3.4 cm）。但濾掉之後
        曲率更接近真實、前視點的選擇也更穩定。

        **只平滑 x,y，不動 yaw 與 direction**：
          - yaw 純追蹤根本不用（轉向是從前視點的位置算的）
          - direction 是折返點的依據，平滑會把它糊掉

        端點不動，避免起終點位置被拉偏。
        """
        if len(pts) < 2 * window + 1:
            return pts
        out = list(pts)
        for i in range(window, len(pts) - window):
            xs = sum(pts[j].x for j in range(i - window, i + window + 1))
            ys = sum(pts[j].y for j in range(i - window, i + window + 1))
            k = 2 * window + 1
            out[i] = PathPoint(xs / k, ys / k, pts[i].yaw, pts[i].direction)
        return out

    def _merge_short_segments(self, pts, min_seg_m=None):
        """把長度不足的方向段併進前一段，消掉假折返點。

        方向現在讀 cmd_vel 的正負號（比舊的位移投影可靠得多），但操作者
        減速、微調時仍可能產生一兩點的極短方向段。真正的折返一定要停車、
        打方向、再走一段，所以「太短的方向段」多半是雜訊。

        預設門檻 0.35 m 的來由：這台車最小轉彎半徑 0.80 m，做一次有意義的
        折返修正至少要走過該圓弧的一小段（0.8 x 0.4 rad 約 0.32 m）。
        取 0.35 m 略高於它，寧可漏掉極短的真折返，也不要留下假的——
        假折返會讓重播時無謂地停車換向。

        ★ 但這個取捨在**窄轉角錄製三點轉向**時會反過來：那裡人做的每一次
        前進／後退本來就短（走廊淨寬 0.967 m），0.35 m 會把真正要保留的
        修正動作整段吃掉。所以門檻已參數化為 `merge_min_segment_m`，
        錄三點轉向前先降到 0.12~0.15。
        """
        if min_seg_m is None:
            # 現讀，不用 __init__ 的快取——否則 `ros2 param set` 無效（見宣告處）。
            min_seg_m = float(self.get_parameter("merge_min_segment_m").value)
        if len(pts) < 3:
            return pts, 0
        # 切成同方向的段
        segs = []
        start = 0
        for i in range(1, len(pts)):
            if pts[i].direction != pts[i - 1].direction:
                segs.append((start, i))
                start = i
        segs.append((start, len(pts)))

        def seg_len(a, b):
            return sum(math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y)
                       for i in range(a + 1, b))

        merged = 0
        for idx, (a, b) in enumerate(segs):
            if idx == 0:
                # 第一段沒有「前一段」可併，但它同樣可能是假的：_cmd_dir 的初值
                # 是 +1，所以「收到第一則 cmd_vel 之前」錄下的點一律被標成前進。
                # 倒車起步的路徑因此必定在索引 0 多出一個極短的前進段。
                # 實測（2026-08-04「大廳到起點」）：長度 0.00 m 的前進段，
                # 讓折返點從 2 個變成 3 個，重播時開頭會多一次無謂的停車換向。
                # 太短的第一段改採**下一段**的方向。
                if len(segs) > 1 and seg_len(a, b) < min_seg_m:
                    next_dir = pts[segs[1][0]].direction
                    for i in range(a, b):
                        pts[i] = PathPoint(pts[i].x, pts[i].y, pts[i].yaw, next_dir)
                    merged += 1
                continue
            if seg_len(a, b) < min_seg_m:
                prev_dir = pts[segs[idx - 1][0]].direction
                for i in range(a, b):
                    pts[i] = PathPoint(pts[i].x, pts[i].y, pts[i].yaw, prev_dir)
                merged += 1
        return pts, merged

    def _record_cb(self, req, resp):
        if req.action == RecordPath.Request.START:
            if self._following:
                resp.success = False
                resp.message = "重播進行中，不能同時錄製"
                return resp
            if self._robot_pose() is None:
                resp.success = False
                resp.message = f"取不到 {self.map_frame} -> {self.robot_frame} 的 TF，請先確認定位已就緒"
                return resp
            with self._rec_lock:
                self._rec_points = []
            self._rec_jumps = 0
            self._rec_abort_reason = ""
            self._recording = True
            self.get_logger().info("開始錄製教導路徑")
            resp.success = True
            resp.message = "開始錄製，請把車開一遍，完成後按結束"
            return resp

        if req.action == RecordPath.Request.CANCEL:
            self._recording = False
            with self._rec_lock:
                n = len(self._rec_points)
                self._rec_points = []
            resp.success = True
            resp.message = f"已取消錄製（丟棄 {n} 個點）"
            return resp

        if req.action != RecordPath.Request.STOP:
            resp.success = False
            resp.message = f"未知的動作代碼 {req.action}"
            return resp

        # ── STOP：存檔 ──
        if not self._recording:
            resp.success = False
            resp.message = (f"錄製已於中途中止：{self._rec_abort_reason}。請重新錄製"
                            if self._rec_abort_reason else "目前沒有在錄製")
            # 報過就清掉。否則之後任何一次「沒在錄製卻按了停止」都會重播
            # 這段舊訊息，操作者會以為剛剛那次也跳位了。
            self._rec_abort_reason = ""
            return resp
        self._recording = False
        with self._rec_lock:
            pts = list(self._rec_points)
            self._rec_points = []

        if len(pts) < 2:
            resp.success = False
            resp.message = f"只錄到 {len(pts)} 個點，路徑太短未存檔"
            return resp

        pts, merged = self._merge_short_segments(pts)
        if merged:
            self.get_logger().info(
                f"存檔前合併掉 {merged} 個短於 "
                f"{float(self.get_parameter('merge_min_segment_m').value):.2f} m "
                f"的方向段（視為假折返點）")

        name = (req.name or "").strip() or f"path_{len(self._paths) + 1}"
        pid = "path_" + uuid.uuid4().hex[:12]
        length = self._path_length(pts)
        cusps = self._count_cusps(pts)
        meta = {
            "name": name,
            "map_id": self.current_map,
            "source": "teach",
            "created_at": self._now_iso(),
            "length_m": length,
            "num_cusps": cusps,
        }
        with self._db_lock:
            self._paths[pid] = {"meta": meta, "points": pts}
            ok = self._save_db()
        if not ok:
            with self._db_lock:
                self._paths.pop(pid, None)
            resp.success = False
            resp.message = "寫入資料庫失敗"
            return resp

        self.get_logger().info(
            f"教導路徑已存檔「{name}」：{len(pts)} 點、{length:.2f} m、{cusps} 個折返點"
        )
        resp.success = True
        # （舊版這裡會附上「錄製中丟棄 N 個跳位點」。跳位改成**中止錄製**
        #   之後，能執行到這一行就代表全程沒跳過位，那段訊息恆為空字串，
        #   留著只會讓人以為還有「丟棄但繼續」的路徑。已移除。）
        resp.message = (f"已存檔「{name}」：{len(pts)} 點、{length:.2f} 公尺、"
                        f"{cusps} 個折返點")
        resp.path_id = pid
        resp.num_points = len(pts)
        resp.length_m = length
        return resp

    @staticmethod
    def _path_length(pts: List[PathPoint]) -> float:
        return sum(math.hypot(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y)
                   for i in range(len(pts) - 1))

    @staticmethod
    def _count_cusps(pts: List[PathPoint]) -> int:
        return sum(1 for i in range(1, len(pts)) if pts[i].direction != pts[i - 1].direction)

    @staticmethod
    def _now_iso() -> str:
        return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())

    # ==================================================================
    # 列表 / 刪除
    # ==================================================================
    def _list_cb(self, req, resp):
        with self._db_lock:
            items = list(self._paths.items())
        out = []
        for pid, entry in items:
            meta = entry["meta"]
            if req.current_map_only and self.current_map and meta.get("map_id") != self.current_map:
                continue
            info = TaughtPathInfo()
            info.path_id = pid
            info.name = meta.get("name", "")
            info.map_id = meta.get("map_id", "")
            info.num_points = len(entry["points"])
            info.length_m = float(meta.get("length_m", 0.0))
            info.num_cusps = int(meta.get("num_cusps", 0))
            info.source = meta.get("source", "teach")
            info.created_at = meta.get("created_at", "")
            out.append(info)
        out.sort(key=lambda i: i.created_at)
        resp.success = True
        resp.paths = out
        resp.message = f"共 {len(out)} 條路徑"
        return resp

    def _delete_cb(self, req, resp):
        if self._following and req.path_id == self._active_path_id:
            resp.success = False
            resp.message = "這條路徑正在重播中，不能刪除"
            return resp
        with self._db_lock:
            if req.path_id not in self._paths:
                resp.success = False
                resp.message = f"找不到路徑 {req.path_id}"
                return resp
            name = self._paths[req.path_id]["meta"].get("name", req.path_id)
            removed = self._paths.pop(req.path_id)
            if not self._save_db():
                self._paths[req.path_id] = removed      # 寫檔失敗就還原
                resp.success = False
                resp.message = "寫入資料庫失敗，未刪除"
                return resp
        resp.success = True
        resp.message = f"已刪除路徑「{name}」"
        return resp

    # ==================================================================
    # 障礙偵測
    # ==================================================================
    def _forward_clearance(self, direction: int, lateral_offset: float = 0.0) -> float:
        """車子沿目前朝向前進（或後退）時，多遠會撞到東西

        把雷射點轉到車體座標系，只看寬度在 ±(半車寬) 內、位於行進方向那一側
        的點，回傳最近的縱向距離。lateral_offset 是「假設車子往側邊平移這麼多」
        時的結果，用來評估繞開可不可行。

        回傳 inf 代表這個方向淨空。
        """
        with self._scan_lock:
            scan = self._scan
        if scan is None:
            # 沒有雷達資料時回 0 而不是 inf：寧可誤停也不要盲衝
            return 0.0

        best = float("inf")
        ang = scan.angle_min
        inc = scan.angle_increment
        half_w = self.obs_half_w
        for r in scan.ranges:
            a = ang
            ang += inc
            if not (scan.range_min <= r <= scan.range_max) or r != r:  # 含 NaN
                continue
            # 車體座標：x 向前、y 向左（雷達與 base_footprint 同向，只差高度）
            x = r * math.cos(a)
            y = r * math.sin(a) - lateral_offset
            if abs(y) > half_w:
                continue
            longitudinal = x if direction >= 0 else -x
            if longitudinal <= 0.0:
                continue                      # 在身後，不管
            if longitudinal < best:
                best = longitudinal
        return best

    def _side_clearance(self) -> Tuple[float, float]:
        """車身**兩側**的橫向空隙 (左,右)，單位公尺。沒看到牆就回 inf。

        跟 `_forward_clearance` 不同：那個量的是「某條縱向帶裡最近的前方障礙」，
        這裡量的是「車身旁邊還剩多少」。走廊裡要保持餘裕靠的是後者。

        只取與車身縱向重疊的掃描點（前後各放寬 5 cm），因為遠處的牆
        不影響現在會不會刮到。
        """
        with self._scan_lock:
            scan = self._scan
        if scan is None:
            return 0.0, 0.0                     # 沒資料時當作貼牆，寧可保守
        front, rear, half_w = 0.40, -0.09, 0.185    # 車身矩形（base_footprint 在後輪軸心）
        left = right = float("inf")
        ang = scan.angle_min
        for r in scan.ranges:
            a = ang
            ang += scan.angle_increment
            if not (scan.range_min <= r <= scan.range_max) or r != r:
                continue
            if r > 3.0:
                continue
            x = self.laser_x + r * math.cos(a)
            y = r * math.sin(a)
            if not (rear - 0.05 <= x <= front + 0.05):
                continue
            if y > 0:
                left = min(left, y - half_w)
            else:
                right = min(right, -y - half_w)
        return left, right

    def _arc_clearance(self, direction: int, curvature: float, max_dist: float = 1.2) -> float:
        """沿著「打舵之後實際會走的弧線」檢查淨空，而不是正前方的直帶。

        `_forward_clearance` 檢查的是以車頭為軸的矩形帶。脫困時車子打滿舵、
        走的是半徑 0.80 m 的弧，兩者差很多：**左前方明明有空間，
        直帶檢查卻只看正前方，於是判定「淨空不足」而放棄**（2026-08-03 實測）。

        做法：沿弧線每 5 cm 取一個車體中心位置，檢查該位置的車身矩形
        有沒有掃描點落在裡面。回傳第一次碰到障礙的弧長；全程淨空回傳 max_dist。

        這比直帶保守得多也精確得多——它問的是「車子照這個方向盤角度開，
        會不會撞到」，正是真正要回答的問題。
        """
        with self._scan_lock:
            scan = self._scan
        if scan is None:
            return 0.0
        pts = []
        ang = scan.angle_min
        for r in scan.ranges:
            a = ang
            ang += scan.angle_increment
            if not (scan.range_min <= r <= scan.range_max) or r != r:
                continue
            if r > max_dist + 1.0:
                continue
            pts.append((r * math.cos(a), r * math.sin(a)))
        if not pts:
            return max_dist

        half_w = self.obs_half_w
        # 車身縱向範圍（相對 base_footprint，雷達在其前方 0.089 m）
        laser_dx = 0.089
        front = 0.40 - laser_dx      # 車頭在雷達前方
        rear = -(0.09 + laser_dx)    # 車尾在雷達後方

        step = 0.05
        n = int(max_dist / step)
        for k in range(1, n + 1):
            arc = k * step
            # 沿弧線走 arc 之後，車體在雷達座標系的位姿
            #
            # ★ 2026-08-03 修正：倒車的符號原本是錯的（未實機驗證，見下）。
            #
            # 本節點所有發布端都是 ω = |v|·curvature：
            #   _do_escape 的 self._publish_cmd(v, abs(v) * curv)
            #   純追蹤的  w = abs(v) * curvature
            # 也就是**不論前進或後退，車頭都往同一邊轉**——阿克曼倒車時
            # 方向盤要打反邊，韌體的 Vz_to_Akm_Angle 會自己從 R = Vx/Vz
            # 算出該打哪邊（見 廠商原始碼分析_阿克曼控制鏈.md）。
            #
            # 所以正確的運動學是：
            #     朝向變化  dθ = +curvature · arc    與 direction 無關
            #     位移方向  才隨 direction 翻號
            #
            # 原本寫成 th = curvature·arc·(±1)、s_ = ±arc，那是
            # 「ω = v·curvature（隨方向翻號）」的有號弧長公式，與發布端不符。
            # 後果：倒車時檢查的是繞**另一側**的鏡像弧線。
            # 以 curv=1.25(R=0.8)、arc=0.4 m 代入，模型算出 cy=+0.098 m
            # 而實際是 −0.098 m，差 0.196 m —— 已經超過 obs_half_w=0.22
            # 的整個帶寬，等於在檢查一塊車子根本不會經過的區域。
            th = curvature * arc
            if abs(curvature) < 1e-6:
                cx, cy = arc * (1.0 if direction >= 0 else -1.0), 0.0
            else:
                R = 1.0 / curvature
                sgn = 1.0 if direction >= 0 else -1.0
                cx = sgn * R * math.sin(arc * curvature)
                cy = sgn * R * (1.0 - math.cos(arc * curvature))
            c, sn = math.cos(-th), math.sin(-th)
            for px, py in pts:
                # 把掃描點轉到「車子開到那裡之後」的車體座標
                dx, dy = px - cx, py - cy
                bx = dx * c - dy * sn
                by = dx * sn + dy * c
                if rear <= bx <= front and abs(by) <= half_w:
                    return arc
        return max_dist

    def _pick_avoid_offset(self, direction: int) -> Optional[float]:
        """找一個能繞過去的橫向偏移量；找不到回 None

        從小到大試，左右交替（同樣大小優先試左邊只是為了行為一致，
        沒有偏好的理由）。可行的定義是「該偏移下前方淨空距離 > 停止距離 + 餘裕」。
        """
        n = int(self.avoid_max / max(0.01, self.avoid_step))
        for i in range(1, n + 1):
            for sign in (1.0, -1.0):
                off = sign * i * self.avoid_step
                stop_d = self.obs_stop if direction > 0 else self.obs_stop_rev
                if self._forward_clearance(direction, off) > stop_d + self.avoid_clear:
                    return off
        return None

    # ==================================================================
    # 純追蹤
    # ==================================================================
    @staticmethod
    def _closest_index(pts: List[PathPoint], x: float, y: float, hint: int) -> int:
        """從 hint 附近往後找最近點

        只往前搜尋不回頭，避免路徑自我交叉（例如折返、繞圈）時
        跳回已經走過的那一段。

        ★ 2026-08-05：搜尋範圍**不可跨越折返點** —— 與 `_lookahead_point`
        同一條規則。

        折返路徑會折回自己：實測 `path_3d85e56c550d`（nav2 規劃的三點轉向）
        的 pts[12] 與 pts[14] 只相距 **3.0 cm**、pts[13] 與 pts[15] 也是 3.0 cm
        —— 車子把同一段 0.15 m 來回走了三趟。而循跡誤差本身就有 3~9 cm，
        純 argmin 會直接跳到折返**之後**那一段，車子於是以為自己已經做完
        折返動作，實際上根本還沒退。索引跳掉之後主迴圈也不會偵測到換向。

        邊界含折返點本身（`limit = i + 1`）：主迴圈靠
        `pts[idx].direction != cur_dir` 偵測折返並停車換向，索引必須
        **到得了**那個點，否則會永遠停在它前面一格。
        """
        limit = min(len(pts), hint + 120)      # 一次最多往前看 120 點（約 6 公尺）
        cur_dir = pts[hint].direction
        for i in range(hint, limit):
            if pts[i].direction != cur_dir:
                limit = i + 1                  # 含折返點，讓主迴圈偵測得到
                break

        best_i, best_d = hint, float("inf")
        for i in range(hint, limit):
            d = (pts[i].x - x) ** 2 + (pts[i].y - y) ** 2
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _lookahead_point(self, pts: List[PathPoint], start: int,
                         x: float, y: float, ld: float) -> Tuple[int, PathPoint]:
        """從 start 往前找第一個距離超過 ld 的點；同時不可跨越折返點

        跨越折返點會讓車子直接朝著「換方向之後的那一段」開過去，
        等於把折返整個跳掉 —— 這在窄走廊裡就是撞牆。
        """
        d0 = pts[start].direction
        i = start
        while i < len(pts) - 1:
            if pts[i].direction != d0:
                return i, pts[i]                       # 停在折返點
            if math.hypot(pts[i].x - x, pts[i].y - y) >= ld:
                return i, pts[i]
            i += 1
        return len(pts) - 1, pts[-1]

    def _pure_pursuit(self, pose, target: PathPoint, direction: int,
                      speed: float, lateral_offset: float) -> Tuple[float, float]:
        """回傳 (linear, angular)"""
        x, y, yaw = pose
        # 目標點轉到車體座標
        dx, dy = target.x - x, target.y - y
        c, s = math.cos(-yaw), math.sin(-yaw)
        bx = dx * c - dy * s
        by = dx * s + dy * c
        # ★ 2026-08-04 修正符號 ★
        # 車體座標 y 向左。`_forward_clearance(dir, +off)` 的檢查帶中心在
        # y = +off，也就是「假設車子往左移 off」；`_do_escape` 也照這個慣例
        # （left = _forward_clearance(d, +0.20)）。
        # 但這裡原本寫 `by -= lateral_offset`，正的偏移會讓目標看起來偏右、
        # 車子往右打 —— 跟量測端**符號相反**。於是 `_pick_avoid_offset`
        # 找到「往左有空間」，控制器卻往右轉，正好轉進障礙物。
        # 統一成「正 = 往左」。
        by += lateral_offset          # 繞障 / 貼牆排斥時把目標往側邊挪

        ld = math.hypot(bx, by)
        if ld < 1e-3:
            return 0.0, 0.0

        # 倒車時把目標鏡射到「車尾朝向」的座標系，公式才成立
        if direction < 0:
            bx, by = -bx, -by

        curvature = 2.0 * by / (ld * ld)
        max_curv = 1.0 / max(0.05, self.min_radius)
        curvature = max(-max_curv, min(max_curv, curvature))

        v = speed if direction >= 0 else -speed
        w = abs(v) * curvature
        return v, w

    def _publish_cmd(self, lin: float, ang: float) -> None:
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)

    def _stop(self, frames: int = 3) -> None:
        for _ in range(frames):
            self._publish_cmd(0.0, 0.0)
            time.sleep(0.02)

    def _publish_path_viz(self, pts: List[PathPoint]) -> None:
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for pt in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = pt.x
            ps.pose.position.y = pt.y
            qx, qy, qz, qw = quat_from_yaw(pt.yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    # ==================================================================
    # 重播主迴圈
    # ==================================================================
    def _execute_follow(self, goal_handle):
        req = goal_handle.request
        result = FollowTaughtPath.Result()

        with self._db_lock:
            entry = self._paths.get(req.path_id)
        if entry is None:
            goal_handle.abort()
            result.success = False
            result.message = f"找不到路徑 {req.path_id}"
            return result

        meta = entry["meta"]
        if self.current_map and meta.get("map_id") and meta["map_id"] != self.current_map:
            goal_handle.abort()
            result.success = False
            result.message = (f"路徑「{meta.get('name')}」屬於地圖 {meta['map_id']}，"
                              f"目前地圖是 {self.current_map}。座標系不同，不能重播")
            return result

        pts = list(entry["points"])
        if req.reverse:
            # 反向走：點序反轉，而且每個點的行進方向也要翻過來
            pts = [PathPoint(q.x, q.y, q.yaw, -q.direction) for q in reversed(pts)]

        scale = req.speed_scale if req.speed_scale > 0.0 else 1.0
        self._following = True
        self._active_path_id = req.path_id
        self._publish_path_viz(pts)
        rev_note = "（反向）" if req.reverse else ""
        self.get_logger().info(
            f"開始重播「{meta.get('name')}」：{len(pts)} 點{rev_note}，速度 x{scale:.2f}"
        )

        try:
            return self._follow_loop(goal_handle, pts, scale, result)
        finally:
            self._following = False
            self._active_path_id = ""
            self._stop(5)

    def _follow_loop(self, goal_handle, pts, scale, result):
        idx = 0
        cur_dir = pts[0].direction
        wait_started = 0.0
        avoid_offset = 0.0
        state = "following"
        last_fb = 0.0
        escapes = 0
        prog_idx = -1
        prog_t = time.monotonic()
        # 每個路徑點的累計弧長，給進度檢查用（見迴圈裡的說明）
        path_s = [0.0] * len(pts)
        for i in range(1, len(pts)):
            path_s[i] = path_s[i - 1] + math.hypot(pts[i].x - pts[i - 1].x,
                                                   pts[i].y - pts[i - 1].y)
        prog_s = -1.0
        self._escape_steer = None

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                result.success = False
                result.message = "已取消"
                return result

            pose = self._robot_pose()
            if pose is None:
                self._stop()
                goal_handle.abort()
                result.success = False
                result.message = "定位遺失（取不到 map -> base_footprint）"
                return result
            x, y, yaw = pose

            # ── 進度 ──
            idx = self._closest_index(pts, x, y, idx)
            goal = pts[-1]
            dist_to_goal = math.hypot(goal.x - x, goal.y - y)
            # 到終點的判定要「索引也走到最後」才算，否則路徑起終點靠近時
            # 一出發就會被判定成已抵達
            if idx >= len(pts) - 3 and dist_to_goal <= self.goal_tol:
                self._stop(5)
                goal_handle.succeed()
                result.success = True
                result.final_error_m = dist_to_goal
                result.final_yaw_error_deg = math.degrees(abs(norm_angle(goal.yaw - yaw)))
                result.message = (f"已抵達終點，位置誤差 {dist_to_goal * 100:.0f} cm、"
                                  f"朝向誤差 {result.final_yaw_error_deg:.0f} 度")
                self.get_logger().info(result.message)
                return result

            # ── 偏離檢查 ──
            xte = math.hypot(pts[idx].x - x, pts[idx].y - y)
            if xte > self.max_xte:
                self._stop()
                goal_handle.abort()
                result.success = False
                result.message = (f"偏離路徑 {xte * 100:.0f} cm 超過上限 "
                                  f"{self.max_xte * 100:.0f} cm，已停車。"
                                  f"可能是定位跑掉或車子被推動")
                self.get_logger().warn(result.message)
                return result

            # ── 進度檢查：索引沒前進就當卡住 ──
            #
            # 不看狀態、只看有沒有在前進。頂到牆的時候淨空可能還沒低到
            # stop 門檻（狀態是 slowing 而不是 waiting），車子會用 30% 速度
            # 一直頂著，輪子打滑、位姿漂移，直到偏離上限才被攔下。
            # ★ 用「沿路徑走了多遠」而不是「索引變了沒」（2026-08-04）★
            #
            # 索引數是壞的進度指標，因為它跟點距綁在一起：
            #   教導路徑點距 5 cm  -> slowing 時 (30% 速度 ≈ 0.03 m/s) 1.7 秒走一格
            #   規劃路徑點距 15 cm -> 同樣情況要 5.0 秒走一格
            # 而 no_progress_sec 就是 5.0。實測 path_3d85e56c550d：
            # 障礙早就過了、車子也在動，卻每 5 秒誤判一次卡住，
            # 10 次脫困全部用在「正常但慢」的路段上，最後放棄在 25/51。
            # 同一個門檻換條路徑就變成永久誤觸發。
            #
            # 改量實際弧長：走不到 stuck_min_advance 才算卡住，與點距無關。
            s_now = path_s[idx]
            if s_now - prog_s > self.stuck_min_advance:
                prog_idx = idx
                prog_s = s_now
                prog_t = time.monotonic()
                self._escape_steer = None      # 真的前進了，下次卡住重新判斷轉向
            elif time.monotonic() - prog_t > self.no_progress:
                if escapes < self.max_escapes:
                    escapes += 1
                    self.get_logger().warn(
                        f"索引停在 {idx}/{len(pts)} 已 {self.no_progress:.0f} 秒沒前進，"
                        f"判定卡住，嘗試脫困（第 {escapes}/{self.max_escapes} 次）")
                    self._stop(3)
                    if self._do_escape(pts, idx, cur_dir, goal_handle, escapes):
                        prog_t = time.monotonic()
                        avoid_offset = 0.0
                        wait_started = 0.0
                        state = "following"
                        continue
                    prog_t = time.monotonic()      # 退不動也重計時，讓它再試
                else:
                    self._stop()
                    goal_handle.abort()
                    result.success = False
                    result.message = (f"卡在第 {idx}/{len(pts)} 點，"
                                      f"脫困 {escapes} 次都無效，已放棄")
                    self.get_logger().warn(result.message)
                    return result

            # ── 折返點：先停穩再換方向 ──
            if pts[idx].direction != cur_dir:
                self._stop(5)
                was = "前進" if cur_dir > 0 else "後退"
                now_dir = "前進" if pts[idx].direction > 0 else "後退"
                self.get_logger().info(f"折返點：{was} -> {now_dir}")
                time.sleep(self.cusp_pause)
                cur_dir = pts[idx].direction
                continue

            # ── 障礙 ──
            #
            # ★ 2026-08-04：改用弧線淨空，不再用「沿車頭的直帶」★
            #
            # 直帶檢查在窄走廊裡會把**側牆**當成正前方障礙：車身一斜，
            # 牆就落進 ±0.22 m 的帶子裡。8/3 算過「車頭偏 35 度以上會中招」，
            # 但那是 0.967 m 的走廊；實測 0.84 m 的走廊**只要歪 19 度就中招**。
            #
            # 實測（path_2fb1081910d7，逐筆 CSV）：車子與走廊軸線差 19 度，
            # 直帶量到「後方 0.60 m 有障礙」，那個點在車體 (-0.60,+0.22)
            # ——正好卡在帶子邊緣，地圖座標 (+2.71,+1.14) 是走廊中段的側牆。
            # 後果：永遠 slowing (30% 速度) -> 走一格 4.7 秒 -> 誤判卡住
            # -> 脫困把車轉得更斜 -> 更容易誤判。78% 的時間在脫困。
            #
            # `_arc_clearance` 沿「接下來實際會走的弧線」取樣車身矩形，
            # 車子斜著走時檢查的就是斜著的那條路。它本來只用在脫困裡。
            #
            # 先算一次純追蹤的曲率當預覽（下面真正下指令時會用當時的速度
            # 重算，兩者的曲率一致——曲率只跟目標點幾何有關，與速度無關）。
            _pv_ld = max(self.la_min, min(self.la_max, self.la_k * abs(
                self.follow_speed if cur_dir > 0 else self.reverse_speed) + self.la_min))
            _pv_i, _pv_tgt = self._lookahead_point(pts, idx, x, y, _pv_ld)
            _pv_v, _pv_w = self._pure_pursuit(pose, _pv_tgt, cur_dir, 1.0, avoid_offset)
            preview_curv = (_pv_w / abs(_pv_v)) if abs(_pv_v) > 1e-6 else 0.0
            # ★ 往前看的距離必須**大於**拿來比的門檻 ★
            # 第一版寫成 max_dist=self.obs_slow（1.20），但門檻是
            # slow_d = obs_slow - bumper = 1.11 —— 回傳值上限 1.20、
            # 「算清空」要 > 1.11，可用區間只有 9 公分，於是幾乎永遠 slowing。
            # 實測（path_07068062ebc0）686 筆裡 following 只有 1 筆。
            # 多看 0.30 m，讓「真的清空」有機會成立。
            clearance = self._arc_clearance(cur_dir, preview_curv,
                                            max_dist=self.obs_slow + 0.30)
            speed = (self.follow_speed if cur_dir > 0 else self.reverse_speed) * scale

            # ★ 換算門檻的單位 ★
            # `_forward_clearance` 回傳「從 base_footprint 原點到障礙的距離」，
            # 車身自己的長度**含在裡面**；`_arc_clearance` 回傳「還能走多遠才撞到」，
            # 車身矩形已經扣掉了。obs_stop / obs_slow 是照前者調出來的，
            # 直接拿來比會保守一倍以上（0.45 會變成「還有 45 cm 就停」）。
            # 前保險桿在 +0.40、後保險桿在 -0.09，扣掉即可。
            bumper = 0.40 if cur_dir > 0 else 0.09
            stop_d = (self.obs_stop if cur_dir > 0 else self.obs_stop_rev) - bumper
            slow_d = self.obs_slow - bumper

            if clearance <= stop_d:
                # 先試著繞開
                off = self._pick_avoid_offset(cur_dir)
                if off is not None and abs(off) <= self.avoid_max:
                    if state != "avoiding":
                        self.get_logger().info(f"前方障礙，橫向偏移 {off * 100:+.0f} cm 繞過")
                    avoid_offset = off
                    state = "avoiding"
                    speed *= 0.5
                    wait_started = 0.0
                else:
                    # 繞不過去 -> 停下等
                    self._stop()
                    if wait_started == 0.0:
                        wait_started = time.monotonic()
                        self.get_logger().info("前方障礙且無法繞過，停車等待")
                    waited = time.monotonic() - wait_started

                    # 等一下下還沒通 -> 試著脫困。
                    # 人擋路通常幾秒內就讓開，牆不會；兩者從單幀掃描分不出來，
                    # 所以用「等了多久」當判準。
                    if waited > self.escape_after and escapes < self.max_escapes:
                        escapes += 1
                        self.get_logger().info(
                            f"等了 {waited:.0f} 秒仍不通，嘗試脫困（第 {escapes}/{self.max_escapes} 次）")
                        if self._do_escape(pts, idx, cur_dir, goal_handle, escapes):
                            wait_started = 0.0
                            avoid_offset = 0.0
                            state = "following"
                            continue
                        # 退不動就繼續等，等滿 wait_timeout 再放棄

                    if waited > self.wait_timeout:
                        goal_handle.abort()
                        result.success = False
                        result.message = (
                            f"障礙持續 {waited:.0f} 秒未移開，已放棄"
                            f"（嘗試脫困 {escapes} 次）")
                        self.get_logger().warn(result.message)
                        return result
                    state = "waiting"
                    self._send_feedback(
                        goal_handle, idx, len(pts), xte, state,
                        f"等待障礙移開（{waited:.0f}/{self.wait_timeout:.0f} 秒，"
                        f"已脫困 {escapes} 次）")
                    time.sleep(self.control_dt)
                    continue
            else:
                wait_started = 0.0
                if clearance <= slow_d:
                    # 線性減速：距離越近越慢，最低到 30%
                    span = max(1e-3, slow_d - stop_d)
                    speed *= max(0.3, (clearance - stop_d) / span)
                    state = "slowing"
                else:
                    state = "following"
                    # 障礙消失就慢慢收回偏移，不要瞬間切回原路徑
                    if avoid_offset != 0.0:
                        avoid_offset *= 0.9
                        if abs(avoid_offset) < 0.01:
                            avoid_offset = 0.0

            # ── 貼牆排斥 ──
            #
            # 純追蹤只管追路徑點，路徑本身歪了就跟著歪。實測在 0.84 m 的
            # 走廊裡，車子右側只剩 0.07 m（車寬 0.37，理想是兩側各 0.235）。
            # 這裡在單側低於 wall_keepout 時把追蹤目標往外推，推的量正比於
            # 侵入深度、上限 wall_push_max。
            #
            # 刻意做成「靠太近才推」而不是「隨時往中線拉」：後者會跟路徑
            # 追蹤持續打架，而有些路徑本來就該貼一邊走。這裡只保餘裕。
            # 符號慣例：正 = 往左（與 _forward_clearance / _do_escape 一致）。
            sl, sr = self._side_clearance()
            wall_push = 0.0
            if sr < self.wall_keepout and sr < sl:
                wall_push = min(self.wall_push_max, self.wall_keepout - sr)      # 往左推
            elif sl < self.wall_keepout and sl < sr:
                wall_push = -min(self.wall_push_max, self.wall_keepout - sl)     # 往右推
            if wall_push != 0.0 and state == "following":
                state = "slowing"
                speed *= 0.6
                if int(time.monotonic() * 2) % 4 == 0:
                    self.get_logger().info(
                        f"貼牆（左 {sl:.2f} m、右 {sr:.2f} m），"
                        f"目標往{'左' if wall_push > 0 else '右'}推 {abs(wall_push)*100:.0f} cm")

            # ── 純追蹤 ──
            ld = max(self.la_min, min(self.la_max, self.la_k * abs(speed) + self.la_min))
            _tgt_i, target = self._lookahead_point(pts, idx, x, y, ld)
            lin, ang = self._pure_pursuit(pose, target, cur_dir, abs(speed),
                                          avoid_offset + wall_push)
            self._publish_cmd(lin, ang)

            now = time.monotonic()
            if now - last_fb > 0.25:
                last_fb = now
                self._send_feedback(goal_handle, idx, len(pts), xte, state, "")
            time.sleep(self.control_dt)

        self._stop()
        goal_handle.abort()
        result.success = False
        result.message = "節點關閉"
        return result

    def _do_escape(self, pts, idx, cur_dir, goal_handle, attempt: int = 1) -> bool:
        """卡住時的脫困：打舵前進改變車頭角度，不是直線退回去。

        === 為什麼要打舵（2026-08-03 實測後重寫）===

        第一版是「沿路徑往回退」——用純追蹤去追路徑上前面幾個點。
        那些點大致就在車子正後方，算出來的曲率接近零，等於**直線退**。
        實測連退三次、每次 0.38 m，車子姿態完全沒變，退完再進還是同樣
        卡在轉角。三次都失敗。

        阿克曼車卡在窄轉角時，問題從來不是「位置不對」而是**車頭角度不對**。
        直線進退不會改變角度，只有「打舵移動」才會。人開車過窄彎時做的
        三點轉向就是這件事：往一邊打舵前進、再往另一邊打舵後退，
        每一次都把車頭轉一點，直到角度足夠通過。

        === 往哪邊打 ===

        比較左右兩側的淨空，往寬的那邊打。用即時掃描判斷而不是寫死，
        因為卡住的方位每次都不一樣（實測是右後輪貼牆，但左後也可能）。

        曲率直接用物理極限 1/min_radius（打滿舵）——脫困要的就是
        最大的角度變化率，沒有理由留餘裕。

        回傳 True 代表車頭角度確實變了、值得重試。
        """
        # === 交替方向（2026-08-03 二次修正）===
        # 原本每次都往同一個方向脫困，等於「退、退、退」——那不是三點轉向。
        # 真正的三點轉向是**前進打左 -> 後退打右 -> 前進打左**：
        # 每一段都把車頭往同一個方向多轉一點，位置卻大致留在原地。
        # 單向重複只會把車子愈推愈遠，車頭角度卻不會累積。
        #
        # attempt 從 1 開始：奇數次往行進的反向、偶數次往行進方向。
        #
        # ★ 打舵方向**不跟著翻**（2026-08-03 修正；舊註解寫「也跟著翻」是錯的，
        #   下面的 steer 只由 yaw_err 決定，與 attempt / back_dir 完全無關）。
        #   阿克曼運動學 dθ/dt = v·tan(δ)/L：前進要 CCW 就 δ 往左、後退要 CCW
        #   就 δ 往右——兩段的方向盤位置相反，但**指令角速度同號**，韌體的
        #   Vz_to_Akm_Angle 會自己從 R = Vx/Vz 算出該打哪邊。
        #   翻了反而是把剛轉過來的角度又轉回去（實測：前進轉 20 度、
        #   後退只轉 17→14→8 度）。
        back_dir = -cur_dir if attempt % 2 == 1 else cur_dir
        start = self._robot_pose()
        if start is None:
            return False
        x0, y0, yaw0 = start

        # === 該往哪邊轉：看路徑要求的朝向，不是看哪邊比較空 ===
        #
        # 第一版用「左右淨空誰大」決定。那是錯的判準——脫困的目的是
        # **把車頭轉到能沿路徑繼續走的角度**，跟哪邊有空間無關。
        # 實測 log「左側淨空 0.91 m、右側 0.00 m」所以選了左邊，
        # 但使用者現場看到的是「該往右打」——選反了，於是愈蹭愈糟。
        #
        # 正確判準：比對車頭與路徑在前方幾個點的朝向差，往差的方向轉。
        # 前視 5 個點（約 25 cm）而不是當前點，因為要對齊的是「接下來
        # 要走的方向」而不是「現在站的位置」。
        look = min(idx + 5, len(pts) - 1)
        yaw_err = norm_angle(pts[look].yaw - yaw0)
        # 倒車路徑上，車頭朝向與行進方向相反，但**路徑點記的就是車頭朝向**，
        # 所以不需要額外處理——直接對齊即可。
        left = self._forward_clearance(back_dir, +0.20)
        right = self._forward_clearance(back_dir, -0.20)
        # ★ 不要翻角速度的符號 ★
        #
        # 阿克曼運動學：dθ/dt = v·tan(δ)/L。要讓車頭持續往同一邊轉：
        #     前進 (v>0) 要 CCW -> δ 往左 -> ω = v·tan(δ)/L > 0
        #     後退 (v<0) 要 CCW -> δ 往右 -> ω = (負)·(負)/L > 0   <- ω 仍為正
        # 兩段的**方向盤位置相反**（一左一右），但**指令角速度同號**。
        # 韌體的 Vz_to_Akm_Angle 會自己從 R = Vx/Vz 算出該打哪邊。
        #
        # 第一版額外翻了 ω 的符號，等於把後退那段轉回去，兩段互相抵消。
        # 實測「前進轉 20 度、後退只轉 17→14→8 度」就是這樣來的：
        # 車子在原地打轉、位置卻一點一點往牆邊漂。
        # 而且我當時記的是 abs(dyaw)，轉回去和轉過去印出來一樣，
        # 儀器本身掩蓋了錯誤——負面結果比正面結果更需要懷疑量測方式。
        # ω 的正負號 = 想要的旋轉方向（CCW 為正）。前進與後退**同號**，
        # 韌體的 Vz_to_Akm_Angle 會自己把它換算成左右相反的方向盤角度。
        steer = 1.0 if yaw_err > 0 else -1.0
        # 若朝向差很小（已經對齊了），那卡住的原因不是角度而是位置，
        # 這時才退而用淨空決定往哪邊挪
        if abs(yaw_err) < math.radians(5.0):
            steer = 1.0 if left >= right else -1.0

        # ★ 整串脫困只決定一次轉向，之後沿用（2026-08-04）★
        #
        # 上面那個判準本身是對的，但把它放在「每次嘗試都重算」的迴圈裡，
        # 它會自我推翻：脫困把車頭轉過去 -> 路徑朝向差變號 -> 下一次就往回轉。
        #
        # 實測（規劃路徑 path_3d85e56c550d，逐筆 CSV）：
        #     前進段  -8, -23, -24, -23, -26 度
        #     後退段 -14, +30, +32, +34, +31 度
        #     10 段脫困總共轉了約 250 度，車頭淨變化 +9 度。
        # 從第 3 段起是完美的極限環，每一對淨賺 7 度，然後永遠重複。
        #
        # 這是 8/3「v3 兩段互相抵消」換了個形式回來：那次是角速度符號寫錯，
        # 這次符號對了，但判準被放在一個會自我推翻的迴圈裡。
        # ★ 判準對不對，跟它被呼叫的時機對不對，是兩件事。
        if attempt > 1 and self._escape_steer is not None:
            if steer != self._escape_steer:
                self.get_logger().info(
                    f"沿用第一次脫困決定的轉向（本次算出的是"
                    f"{'逆時針' if steer > 0 else '順時針'}，已忽略）")
            steer = self._escape_steer
        self._escape_steer = steer
        side = "左" if steer > 0 else "右"
        turn_dir = "逆時針" if steer > 0 else "順時針"
        self.get_logger().info(
            f"脫困：{'前進' if back_dir > 0 else '後退'}、車頭往{turn_dir}轉"
            f"（路徑朝向差 {math.degrees(yaw_err):+.0f} 度；"
            f"左側淨空 {left:.2f} m、右側 {right:.2f} m）"
        )

        speed = (self.follow_speed if back_dir > 0 else self.reverse_speed) * 0.6
        curv = steer / max(0.05, self.min_radius)     # 打滿舵
        # 20 -> 35 度（2026-08-03）。每段轉得多，需要的來回次數就少。
        # 上限來自空間：0.80 m 轉彎半徑下轉 35 度，車子會前進約
        # 0.80 * 0.61 = 0.49 m 的弧長，走廊寬度撐得住。
        # 轉太多的風險是「衝出可用空間」，所以不會一次設到 90 度——
        # 淨空檢查隨時會提前中止，那時已轉的量仍然算數（部分成功）。
        target_dyaw = math.radians(35.0)

        # ★ 貼牆時先直線退出來，退到有空間了才打方向（2026-08-04）★
        #
        # 打滿舵的弧線第一件事是把車尾往外掃。車身側面已經貼著牆的時候，
        # 那一掃就是往牆裡送——所以 8/3 會「愈蹭愈糟」最後楔死。
        # 使用者現場的判斷是對的：「要直線後退才能打方向」，順序不能顛倒。
        #
        # 這裡不打舵、只沿 back_dir 直線走，直到兩側都有 side_min 的空間
        # 或走滿 escape_dist。走直線時車身掃掠的寬度就是車寬本身，
        # 是所有動作裡最小的，貼牆時唯一安全的選擇。
        side_min = 0.15
        if min(left, right) < side_min:
            self.get_logger().info(
                f"側向太窄（左 {left:.2f} m、右 {right:.2f} m），先直線"
                f"{'前進' if back_dir > 0 else '後退'}拉開距離再打方向")
            ts = time.monotonic()
            while time.monotonic() - ts < 5.0 and rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop()
                    return False
                p = self._robot_pose()
                if p is None:
                    self._stop()
                    return False
                if math.hypot(p[0] - x0, p[1] - y0) >= self.escape_dist:
                    break
                l2 = self._forward_clearance(back_dir, +0.20)
                r2 = self._forward_clearance(back_dir, -0.20)
                if min(l2, r2) >= side_min:
                    break
                if self._arc_clearance(back_dir, 0.0) <= 0.12:
                    self.get_logger().warn("直線方向也被擋住，無法拉開距離")
                    break
                v0 = (self.follow_speed if back_dir > 0 else self.reverse_speed) * 0.5
                self._publish_cmd(v0 if back_dir > 0 else -v0, 0.0)
                self._send_feedback(goal_handle, idx, len(pts), 0.0, "escaping",
                                    f"脫困中（直線{'前進' if back_dir > 0 else '後退'}拉開距離）")
                time.sleep(self.control_dt)
            self._stop(3)
            # 拉開之後車子位置變了，轉向的基準要重新取
            p = self._robot_pose()
            if p is not None:
                x0, y0, yaw0 = p

        t0 = time.monotonic()
        while time.monotonic() - t0 < 8.0 and rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                return False
            pose = self._robot_pose()
            if pose is None:
                self._stop()
                return False
            x, y, yaw = pose
            dyaw_signed = norm_angle(yaw - yaw0)     # 有號：看得出是累積還是抵消
            dyaw = abs(dyaw_signed)
            moved = math.hypot(x - x0, y - y0)

            if dyaw >= target_dyaw:
                self._stop(3)
                self.get_logger().info(
                    f"脫困完成：車頭轉了 {math.degrees(dyaw_signed):+.0f} 度、移動 {moved:.2f} m")
                return True

            # 脫困方向也要看淨空，不能往牆裡開
            # 沿實際會走的弧線檢查，不是正前方的直帶（見 _arc_clearance）
            # ★ 2026-08-05：以下兩個失敗出口都必須先發一筆 feedback ★
            #
            # 它們原本直接 return，而第一筆 `escaping` 的 _send_feedback 在
            # 迴圈更下面（見下方）。第一圈就被擋的話 —— 貼牆時正是如此 ——
            # **一筆 escaping 都不會發出去**，HMI 與 CSV 的 state 停在
            # `following`，看起來就像「進度檢查根本沒觸發」。
            #
            # 8/03 記錄的「索引 66 卡 40 秒沒觸發脫困、原因未明」極可能就是
            # 這個觀測缺口：主迴圈拿到 False 之後會重設 prog_t「讓它再試」，
            # 於是每 5 秒無聲重來一次，10 次才 abort（約 50 秒），
            # 而使用者在 40 秒（約 8 次）就人工取消了 —— 連 abort 訊息都沒出現。
            #
            # 這不是脫困的 bug，是**看不見脫困失敗**的 bug。
            # 修好觀測之後，下次同樣的現象才查得下去。
            if self._arc_clearance(back_dir, curv) <= 0.12:
                self._stop(3)
                self.get_logger().warn(
                    f"脫困弧線上有障礙（轉了 {math.degrees(dyaw_signed):+.0f} 度就停）")
                self._send_feedback(
                    goal_handle, idx, len(pts), 0.0, "escaping",
                    f"脫困第 {attempt} 次：弧線被擋，只轉了 "
                    f"{math.degrees(dyaw_signed):+.0f} 度")
                # 有轉到一點就算部分成功，值得重試
                return dyaw > math.radians(5.0)
            if moved > self.escape_dist * 3.0:
                self._stop(3)
                self.get_logger().warn(f"已移動 {moved:.2f} m 但只轉了 {math.degrees(dyaw):.0f} 度")
                self._send_feedback(
                    goal_handle, idx, len(pts), 0.0, "escaping",
                    f"脫困第 {attempt} 次：移動 {moved:.2f} m 但只轉了 "
                    f"{math.degrees(dyaw):.0f} 度，放棄這次")
                return dyaw > math.radians(5.0)

            v = speed if back_dir > 0 else -speed
            self._publish_cmd(v, abs(v) * curv)
            self._send_feedback(goal_handle, idx, len(pts), 0.0, "escaping",
                                f"脫困中（{'前進' if back_dir > 0 else '後退'}），已轉 {math.degrees(dyaw_signed):+.0f}/35 度")
            time.sleep(self.control_dt)

        self._stop(3)
        return False

    def _send_feedback(self, goal_handle, idx, total, xte, state, message) -> None:
        fb = FollowTaughtPath.Feedback()
        fb.point_index = idx
        fb.total_points = total
        fb.progress = idx / max(1, total - 1)
        fb.cross_track_error_m = xte
        fb.state = state
        fb.message = message
        goal_handle.publish_feedback(fb)



    # ==================================================================
    # 地圖點選 -> 用 nav2 規劃器產生教導路徑
    # ==================================================================
    def _plan_cb(self, req, resp):
        """把使用者在地圖上點的一串位置，規劃成一條可存檔的教導路徑

        不自己做曲線內插：SmacPlannerHybrid + REEDS_SHEPP 本來就會產生
        符合最小轉彎半徑、含折返點的阿克曼可行路徑。規劃器本身沒問題，
        出問題的是 MPPI 控制器 —— 所以「用 nav2 規劃、用純追蹤執行」
        是最省事也最可靠的組合。
        """
        if self._following or self._recording:
            resp.success = False
            resp.message = "錄製或重播進行中，請先結束"
            return resp

        targets = list(req.waypoints)
        if not targets:
            resp.success = False
            resp.message = "沒有給任何位置"
            return resp

        # 起點
        if req.start_from_robot:
            pose = self._robot_pose()
            if pose is None:
                resp.success = False
                resp.message = "取不到車子目前位置，無法從車子開始規劃"
                return resp
            start = self._make_pose(pose[0], pose[1], pose[2])
        else:
            if len(targets) < 2:
                resp.success = False
                resp.message = "不從車子開始的話至少要兩個點"
                return resp
            start = targets.pop(0)

        if not self._planner_client.wait_for_server(timeout_sec=5.0):
            resp.success = False
            resp.message = "等不到 /compute_path_to_pose 動作伺服器，nav2 有啟動嗎？"
            return resp

        # ★ 規劃前先清一次全域成本地圖 —— 見 __init__ 裡的說明。
        # 失敗不擋下規劃：清空是最佳化不是前提。
        self._clear_global_costmap()

        all_pts: List[PathPoint] = []
        cur = start
        for leg, tgt in enumerate(targets, 1):
            seg = self._plan_leg(cur, tgt)
            if seg is None:
                resp.success = False
                resp.message = f"第 {leg} 段規劃失敗（{len(targets)} 段中）。目標可能落在障礙物或膨脹區裡"
                return resp
            # 相鄰段的接點會重複，去掉後面那段的第一個點
            all_pts.extend(seg[1:] if all_pts else seg)
            cur = tgt

        if len(all_pts) < 2:
            resp.success = False
            resp.message = "規劃結果太短"
            return resp

        self._annotate_directions(all_pts)

        name = (req.name or "").strip() or f"planned_{len(self._paths) + 1}"
        pid = "path_" + uuid.uuid4().hex[:12]
        length = self._path_length(all_pts)
        cusps = self._count_cusps(all_pts)
        meta = {
            "name": name,
            "map_id": self.current_map,
            "source": "plan",
            "created_at": self._now_iso(),
            "length_m": length,
            "num_cusps": cusps,
        }
        with self._db_lock:
            self._paths[pid] = {"meta": meta, "points": all_pts}
            ok = self._save_db()
        if not ok:
            with self._db_lock:
                self._paths.pop(pid, None)
            resp.success = False
            resp.message = "寫入資料庫失敗"
            return resp

        self._publish_path_viz(all_pts)
        self.get_logger().info(
            f"規劃路徑已存檔「{name}」：{len(all_pts)} 點、{length:.2f} m、{cusps} 個折返點"
        )
        resp.success = True
        resp.message = (f"已存檔「{name}」：{len(all_pts)} 點、{length:.2f} 公尺、"
                        f"{cusps} 個折返點")
        resp.path_id = pid
        resp.num_points = len(all_pts)
        resp.length_m = length
        resp.num_cusps = cusps
        return resp

    def _make_pose(self, x: float, y: float, yaw: float) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        qx, qy, qz, qw = quat_from_yaw(yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def _clear_global_costmap(self, timeout: float = 3.0) -> bool:
        """清空全域成本地圖的 obstacle 層，回傳是否成功。

        **失敗不該擋下規劃。** 清空是最佳化不是前提——服務不在
        （nav2 還沒起來、或改用別的堆疊）時照樣讓規劃跑，
        讓規劃器自己回報真正的錯誤，不要在這裡多一個失敗點。

        用 `_spin_until` 輪詢而不是 `spin_until_future_complete`：
        這是在服務回呼裡呼叫的，後者會在同一個 executor 上死鎖。
        """
        if not self._clear_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "清空成本地圖的服務不在，跳過（規劃仍會進行）。"
                "若規劃失敗且訊息說「目標可能落在障礙物或膨脹區裡」，手動清一次：\n"
                "  ros2 service call /global_costmap/clear_entirely_global_costmap "
                "nav2_msgs/srv/ClearEntireCostmap '{}'")
            return False
        future = self._clear_costmap_client.call_async(ClearEntireCostmap.Request())
        if not self._spin_until(future, timeout):
            self.get_logger().warn("清空成本地圖逾時，仍繼續規劃")
            return False
        self.get_logger().info("已清空全域成本地圖的 obstacle 層")
        return True

    def _plan_leg(self, start: Pose, goal: Pose) -> Optional[List[PathPoint]]:
        """呼叫 nav2 規劃一段，回傳路徑點；失敗回 None

        用同步等待而不是 callback：這是服務回呼，本來就允許阻塞，
        而且節點用的是 MultiThreadedExecutor，不會卡住其他回呼。
        """
        req = ComputePathToPose.Goal()
        req.use_start = True
        req.start = self._stamped(start)
        req.goal = self._stamped(goal)

        send_future = self._planner_client.send_goal_async(req)
        if not self._spin_until(send_future, 10.0):
            self.get_logger().error("送出規劃請求逾時")
            return None
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("規劃請求被拒絕")
            return None

        result_future = handle.get_result_async()
        if not self._spin_until(result_future, 20.0):
            self.get_logger().error("等待規劃結果逾時")
            return None
        wrapped = result_future.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return None

        poses = wrapped.result.path.poses
        if len(poses) < 2:
            return None
        return [PathPoint(ps.pose.position.x, ps.pose.position.y,
                          yaw_from_quat(ps.pose.orientation), 1) for ps in poses]

    def _stamped(self, pose: Pose) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose
        return ps

    @staticmethod
    def _spin_until(future, timeout: float) -> bool:
        """等 future 完成。不呼叫 spin_until_future_complete —— 這裡已經在
        執行器的回呼裡，再要求執行器 spin 會遞迴進去死鎖。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.02)
        return False

    def _annotate_directions(self, pts: List[PathPoint]) -> None:
        """規劃器回傳的路徑沒有標行進方向，這裡從幾何反推

        Reeds-Shepp 會產生含折返的路徑：某一點的位移向量與該點朝向反向，
        就代表那一段是倒車。判定用位移在朝向上的投影，接近 0 時沿用前一點
        （避免原地小幅修正被誤判成折返）。
        """
        for i in range(len(pts) - 1):
            dx = pts[i + 1].x - pts[i].x
            dy = pts[i + 1].y - pts[i].y
            forward = dx * math.cos(pts[i].yaw) + dy * math.sin(pts[i].yaw)
            if abs(forward) > 1e-3:
                pts[i].direction = 1 if forward > 0 else -1
            elif i > 0:
                pts[i].direction = pts[i - 1].direction
        if len(pts) >= 2:
            pts[-1].direction = pts[-2].direction


def main(args=None):
    rclpy.init(args=args)
    node = PathTeachNode()
    # 動作的執行回呼裡有 sleep 迴圈，必須用多執行緒執行器，
    # 否則同一條執行緒被佔住，服務與取消請求都進不來
    executor = MultiThreadedExecutor()
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
