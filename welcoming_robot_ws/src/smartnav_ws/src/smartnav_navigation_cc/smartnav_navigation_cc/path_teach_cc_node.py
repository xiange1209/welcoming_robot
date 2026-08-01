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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
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
from smartnav_msgs.srv import DeleteTaughtPath, ListTaughtPaths, RecordPath


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

        # 錄製：距離或角度變化達到門檻才記一個點。
        # 用「走了多遠」而不是「過了多久」當觸發條件，車停著時才不會
        # 在原地堆出幾百個重複的點。
        self.declare_parameter("record_spacing_m", 0.05)
        self.declare_parameter("record_angle_rad", 0.10)
        self.declare_parameter("max_points", 20000)

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
        self.declare_parameter("obstacle_stop_m", 0.55)
        self.declare_parameter("obstacle_half_width_m", 0.26)   # 車半寬 0.185 + 餘裕
        self.declare_parameter("avoid_max_offset_m", 0.25)
        self.declare_parameter("avoid_step_m", 0.05)
        self.declare_parameter("avoid_clearance_m", 0.10)
        self.declare_parameter("wait_timeout_sec", 20.0)

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
        self.goal_tol = float(p("goal_tolerance_m").value)
        self.control_dt = 1.0 / max(1.0, float(p("control_rate").value))
        self.max_xte = float(p("max_cross_track_m").value)
        self.cusp_pause = float(p("cusp_pause_sec").value)
        self.obs_slow = float(p("obstacle_slow_m").value)
        self.obs_stop = float(p("obstacle_stop_m").value)
        self.obs_half_w = float(p("obstacle_half_width_m").value)
        self.avoid_max = float(p("avoid_max_offset_m").value)
        self.avoid_step = float(p("avoid_step_m").value)
        self.avoid_clear = float(p("avoid_clearance_m").value)
        self.wait_timeout = float(p("wait_timeout_sec").value)

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
        self.create_subscription(LaserScan, p("scan_topic").value,
                                 self._scan_cb, scan_qos, callback_group=cb)
        self.create_subscription(String, "current_map", self._map_cb, latched,
                                 callback_group=cb)

        self.create_service(RecordPath, "record_path", self._record_cb, callback_group=cb)
        self.create_service(ListTaughtPaths, "list_taught_paths", self._list_cb,
                            callback_group=cb)
        self.create_service(DeleteTaughtPath, "delete_taught_path", self._delete_cb,
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
            if len(self._rec_points) >= self.max_points:
                return

            # 行進方向：位移向量投影到車頭方向。正=前進、負=倒車。
            # 這是折返點偵測的依據 —— 重播時必須照著原樣前進/倒車，
            # 否則阿克曼車的轉向方向會整個相反。
            direction = 1
            if dist > 1e-4:
                forward = dx * math.cos(last.yaw) + dy * math.sin(last.yaw)
                # 用一點遲滯：接近 0 的投影量是雜訊，沿用上一點的方向，
                # 避免車子幾乎靜止時錄出一堆假的折返點
                if abs(forward) > 0.01:
                    direction = 1 if forward > 0 else -1
                else:
                    direction = last.direction

            self._rec_points.append(PathPoint(x, y, yaw, direction))

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
            resp.message = "目前沒有在錄製"
            return resp
        self._recording = False
        with self._rec_lock:
            pts = list(self._rec_points)
            self._rec_points = []

        if len(pts) < 2:
            resp.success = False
            resp.message = f"只錄到 {len(pts)} 個點，路徑太短未存檔"
            return resp

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
        resp.message = f"已存檔「{name}」：{len(pts)} 點、{length:.2f} 公尺、{cusps} 個折返點"
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

    def _pick_avoid_offset(self, direction: int) -> Optional[float]:
        """找一個能繞過去的橫向偏移量；找不到回 None

        從小到大試，左右交替（同樣大小優先試左邊只是為了行為一致，
        沒有偏好的理由）。可行的定義是「該偏移下前方淨空距離 > 停止距離 + 餘裕」。
        """
        n = int(self.avoid_max / max(0.01, self.avoid_step))
        for i in range(1, n + 1):
            for sign in (1.0, -1.0):
                off = sign * i * self.avoid_step
                if self._forward_clearance(direction, off) > self.obs_stop + self.avoid_clear:
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
        """
        best_i, best_d = hint, float("inf")
        upper = min(len(pts), hint + 120)      # 一次最多往前看 120 點（約 6 公尺）
        for i in range(hint, upper):
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
        by -= lateral_offset          # 繞障時把目標往側邊挪

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
            clearance = self._forward_clearance(cur_dir, avoid_offset)
            speed = (self.follow_speed if cur_dir > 0 else self.reverse_speed) * scale

            if clearance <= self.obs_stop:
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
                    if waited > self.wait_timeout:
                        goal_handle.abort()
                        result.success = False
                        result.message = f"障礙持續 {waited:.0f} 秒未移開，已放棄"
                        self.get_logger().warn(result.message)
                        return result
                    state = "waiting"
                    self._send_feedback(
                        goal_handle, idx, len(pts), xte, state,
                        f"等待障礙移開（{waited:.0f}/{self.wait_timeout:.0f} 秒）")
                    time.sleep(self.control_dt)
                    continue
            else:
                wait_started = 0.0
                if clearance <= self.obs_slow:
                    # 線性減速：距離越近越慢，最低到 30%
                    span = max(1e-3, self.obs_slow - self.obs_stop)
                    speed *= max(0.3, (clearance - self.obs_stop) / span)
                    state = "slowing"
                else:
                    state = "following"
                    # 障礙消失就慢慢收回偏移，不要瞬間切回原路徑
                    if avoid_offset != 0.0:
                        avoid_offset *= 0.9
                        if abs(avoid_offset) < 0.01:
                            avoid_offset = 0.0

            # ── 純追蹤 ──
            ld = max(self.la_min, min(self.la_max, self.la_k * abs(speed) + self.la_min))
            _tgt_i, target = self._lookahead_point(pts, idx, x, y, ld)
            lin, ang = self._pure_pursuit(pose, target, cur_dir, abs(speed), avoid_offset)
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

    def _send_feedback(self, goal_handle, idx, total, xte, state, message) -> None:
        fb = FollowTaughtPath.Feedback()
        fb.point_index = idx
        fb.total_points = total
        fb.progress = idx / max(1, total - 1)
        fb.cross_track_error_m = xte
        fb.state = state
        fb.message = message
        goal_handle.publish_feedback(fb)


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
