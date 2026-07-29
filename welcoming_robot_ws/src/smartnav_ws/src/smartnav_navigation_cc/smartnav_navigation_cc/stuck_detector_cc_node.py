#!/usr/bin/env python3

"""卡住偵測節點 (_cc)

## 為什麼需要

雷達裝在離地約 0.11 m 的高度，**比它矮的東西完全掃不到**：地上的雜物、
電線、地毯捲邊、椅腳橫桿。這些東西卡得住輪子，但 costmap 上一片空白，
Nav2 會持續下前進指令，馬達空轉、輪子空滑，系統卻以為一切正常。

這台車的超音波話題 (`/ultrasonic_data_A~F`) 雖然存在，但原廠
`ranger_avoid_flag` 與 `ultrasonic_avoid` 都是 false、實測收不到任何資料，
所以沒有辦法從感測器端偵測這類障礙。

## 做法

改成偵測「結果」而不是「原因」：比對**下出去的速度指令**與**實際里程計位移**。

    指令要求在動 (|v| > cmd_threshold)
    但實際幾乎沒動 (實測速度 < motion_threshold)
    持續超過 stuck_time_sec
    => 判定卡住

這種偵測方式不管障礙物是什麼、有沒有被掃到，只要輪子沒有把車子推動就會發現。

判定卡住後：
  1. 發布 /robot_stuck (std_msgs/Bool)
  2. 取消目前的 navigate_to_pose 目標 —— 讓 Nav2 停止硬推，
     並讓 frontier explorer 有機會把這個目標記為失敗而換下一個
  3. 冷卻一段時間避免反覆觸發

## 注意

倒車脫困、窄處來回修正時，車子本來就會短暫停頓，所以 stuck_time_sec
不能設太短，否則會把正常的恢復動作誤判成卡住。
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class StuckDetectorCcNode(Node):
    """指令在動但車子沒動 -> 判定卡住"""

    def __init__(self):
        super().__init__("stuck_detector_cc_node")

        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("odom_topic", "odom_combined")
        # 指令速度超過這個值才視為「要求車子移動」
        self.declare_parameter("cmd_threshold", 0.05)
        # 實際速度低於這個值視為「幾乎沒動」
        self.declare_parameter("motion_threshold", 0.02)
        # 上述狀態持續多久才判定卡住。
        # 不能太短：阿克曼車在窄處來回修正、倒車脫困時本來就會短暫停頓。
        self.declare_parameter("stuck_time_sec", 4.0)
        # 判定後的冷卻時間，避免同一次卡住反覆觸發
        self.declare_parameter("cooldown_sec", 10.0)
        self.declare_parameter("cancel_navigation", True)

        self.cmd_threshold = float(self.get_parameter("cmd_threshold").value)
        self.motion_threshold = float(self.get_parameter("motion_threshold").value)
        self.stuck_time_sec = float(self.get_parameter("stuck_time_sec").value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.cancel_navigation = bool(self.get_parameter("cancel_navigation").value)

        cb = ReentrantCallbackGroup()
        self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self._cmd_cb, 10, callback_group=cb
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._odom_cb, 10, callback_group=cb
        )
        self.stuck_pub = self.create_publisher(Bool, "robot_stuck", 10)
        self.create_service(Trigger, "get_stuck_status", self._status_cb, callback_group=cb)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cb)

        self._lock = threading.Lock()
        self._cmd_speed = 0.0
        self._cmd_time = 0.0
        self._actual_speed = 0.0
        self._suspect_since = None
        self._last_trigger = 0.0
        self._stuck = False

        self.create_timer(0.5, self._check, callback_group=cb)
        self.stuck_pub.publish(Bool(data=False))
        self.get_logger().info(
            f"卡住偵測啟動：指令>{self.cmd_threshold} 而實測<{self.motion_threshold} "
            f"持續 {self.stuck_time_sec}s 即判定"
        )

    # ==================================================================
    def _cmd_cb(self, msg: Twist) -> None:
        with self._lock:
            self._cmd_speed = abs(msg.linear.x)
            self._cmd_time = time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            self._actual_speed = abs(msg.twist.twist.linear.x)

    def _check(self) -> None:
        now = time.monotonic()
        with self._lock:
            cmd = self._cmd_speed
            cmd_age = now - self._cmd_time
            actual = self._actual_speed

        # 指令太舊代表現在根本沒有人在下命令 (例如導航已結束)
        if cmd_age > 1.0:
            self._reset()
            return

        moving_requested = cmd > self.cmd_threshold
        actually_moving = actual > self.motion_threshold

        if not moving_requested or actually_moving:
            self._reset()
            return

        # 要求動、但沒在動
        if self._suspect_since is None:
            self._suspect_since = now
            return

        if now - self._suspect_since < self.stuck_time_sec:
            return

        if now - self._last_trigger < self.cooldown_sec:
            return

        self._last_trigger = now
        self._stuck = True
        self.stuck_pub.publish(Bool(data=True))
        self.get_logger().error(
            f"偵測到卡住：指令 {cmd:.3f} m/s 但實測 {actual:.3f} m/s，"
            f"持續 {now - self._suspect_since:.1f} 秒。"
            "可能是低於雷達高度 (0.11 m) 的雜物卡住輪子 —— costmap 上看不到這種障礙。"
        )
        if self.cancel_navigation:
            threading.Thread(target=self._cancel_nav, daemon=True).start()

    def _reset(self) -> None:
        self._suspect_since = None
        if self._stuck:
            self._stuck = False
            self.stuck_pub.publish(Bool(data=False))

    def _cancel_nav(self) -> None:
        """取消目前的導航目標

        不直接對 cmd_vel 發零速 —— 那會跟 collision_monitor 搶同一個話題。
        取消目標讓 Nav2 自己停下來，順帶讓 frontier explorer 收到失敗、
        把這個到不了的地方記起來而換下一個目標。
        """
        try:
            if not self.nav_client.wait_for_server(timeout_sec=2.0):
                return
            fut = self.nav_client._cancel_goal_async  # noqa: SLF001
        except Exception:  # noqa: BLE001
            pass
        try:
            from action_msgs.srv import CancelGoal

            client = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("找不到 cancel_goal 服務，無法取消導航")
                return
            client.call_async(CancelGoal.Request())  # 空的 goal_info = 取消全部
            self.get_logger().warn("已請求取消目前的導航目標")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"取消導航失敗: {exc}")

    def _status_cb(self, _req, res):
        res.success = True
        res.message = "stuck" if self._stuck else "ok"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = StuckDetectorCcNode()
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
