#!/usr/bin/env python3

"""相機電源管理節點 (_cc)

為什麼需要這個節點：Raspberry Pi 4 只有四核，實測相機相關行程就吃掉約 110% CPU
(astra_camera_node 92% + republish 17%)，導致整台車的 load average 衝到 52
(正常應該 < 4)。在那個狀態下導航必定失敗 —— 不是規劃錯誤，而是 bt_navigator
送出的 action 目標等不到 planner_server 回應就逾時放棄。

但相機又不能永久關掉：迎賓流程要靠它做人臉辨識認出貴賓。
所以做成「分階段供電」：需要辨識時開，建圖與導航時關。

    待機 / 迎賓辨識  -> 相機開   (人臉辨識需要影像)
    建圖中           -> 相機關   (SLAM 只用雷射，相機純粹是負擔)
    導航中           -> 相機關   (辨識已完成，把 CPU 全部讓給 Nav2)
    導航結束         -> 相機開   (回到待機，準備迎接下一位)

控制方式有兩種，可以並用：

  自動：訂閱 /navigation_active (std_msgs/Bool)，由 navigation_action_cc 發布。
        導航開始關相機、結束開相機，不需要任何人介入。

  手動：/camera_standby、/camera_resume (std_srvs/Trigger)
        給 smartnav_brain 或 HMI 在自訂流程裡呼叫，
        例如「已經認出貴賓、開始帶路」時主動讓相機待機。

相機不是 lifecycle node，沒有辦法用生命週期轉換叫它停，
所以這裡直接管理它的 launch 行程 (啟動 / 送 SIGTERM)。
"""

import os
import shlex
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class CameraManagerCcNode(Node):
    """依據機器人目前在做什麼，決定相機要不要開"""

    def __init__(self):
        super().__init__("camera_manager_cc_node")

        self.declare_parameter(
            "camera_launch_cmd",
            "ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py",
        )
        # 開機時要不要先把相機打開 (迎賓待機狀態預設是要辨識人臉的)
        self.declare_parameter("start_with_camera", True)
        # 導航結束後隔多久恢復相機。留一點緩衝，避免車子剛停穩、
        # Nav2 還在收尾時就把 110% CPU 的相機拉起來。
        self.declare_parameter("resume_delay_sec", 3.0)
        # 自動模式：跟著 /navigation_active 走
        self.declare_parameter("auto_follow_navigation", True)

        self.camera_launch_cmd = self.get_parameter("camera_launch_cmd").value
        self.resume_delay_sec = float(self.get_parameter("resume_delay_sec").value)
        self.auto_follow = bool(self.get_parameter("auto_follow_navigation").value)

        self._proc = None
        self._lock = threading.RLock()
        self._resume_timer = None

        self.create_service(Trigger, "camera_standby", self._standby_cb, callback_group=ReentrantCallbackGroup())
        self.create_service(Trigger, "camera_resume", self._resume_cb, callback_group=ReentrantCallbackGroup())
        self.create_service(Trigger, "camera_status", self._status_cb, callback_group=ReentrantCallbackGroup())

        self.state_pub = self.create_publisher(String, "camera_manager_state", 10)

        if self.auto_follow:
            self.create_subscription(Bool, "/navigation_active", self._nav_active_cb, 10)

        if bool(self.get_parameter("start_with_camera").value):
            self.start_camera("開機")
        else:
            self.get_logger().info("相機管理節點啟動 (相機維持關閉)")

    # ==================================================================
    def _running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start_camera(self, reason: str) -> bool:
        with self._lock:
            if self._running():
                return True
            try:
                # start_new_session：讓相機自成一個 process group，
                # 停的時候可以整組收掉 (launch 底下還有 camera node 與 republish)
                self._proc = subprocess.Popen(
                    shlex.split(self.camera_launch_cmd),
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"啟動相機失敗: {exc}")
                return False
            self.get_logger().info(f"相機已啟動 ({reason})")
            self.state_pub.publish(String(data="on"))
            return True

    def stop_camera(self, reason: str) -> bool:
        with self._lock:
            if not self._running():
                return True
            try:
                # 對整個 process group 送訊號，只 kill launch 的話
                # 底下的 astra_camera_node 與 republish 會變成孤兒繼續吃 CPU
                pgid = os.getpgid(self._proc.pid)
                os.killpg(pgid, signal.SIGTERM)
                for _ in range(30):
                    if self._proc.poll() is not None:
                        break
                    time.sleep(0.2)
                if self._proc.poll() is None:
                    os.killpg(pgid, signal.SIGKILL)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"停止相機時發生問題: {exc}")
            self._proc = None
            self.get_logger().info(f"相機已停止 ({reason}) —— CPU 讓給導航")
            self.state_pub.publish(String(data="off"))
            return True

    # ==================================================================
    def _nav_active_cb(self, msg: Bool) -> None:
        """導航狀態變化：開始就關相機，結束就恢復"""
        if msg.data:
            self._cancel_resume_timer()
            self.stop_camera("導航開始")
        else:
            self._schedule_resume()

    def _schedule_resume(self) -> None:
        self._cancel_resume_timer()

        def _fire():
            self._resume_timer = None
            self.start_camera("導航結束")

        self._resume_timer = threading.Timer(self.resume_delay_sec, _fire)
        self._resume_timer.daemon = True
        self._resume_timer.start()

    def _cancel_resume_timer(self) -> None:
        if self._resume_timer is not None:
            self._resume_timer.cancel()
            self._resume_timer = None

    # ==================================================================
    def _standby_cb(self, _req, res):
        self._cancel_resume_timer()
        ok = self.stop_camera("收到 camera_standby")
        res.success = ok
        res.message = "相機已進入待機" if ok else "停止相機失敗"
        return res

    def _resume_cb(self, _req, res):
        self._cancel_resume_timer()
        ok = self.start_camera("收到 camera_resume")
        res.success = ok
        res.message = "相機已恢復" if ok else "啟動相機失敗"
        return res

    def _status_cb(self, _req, res):
        res.success = True
        res.message = "on" if self._running() else "off"
        return res

    def destroy_node(self):
        self._cancel_resume_timer()
        self.stop_camera("節點關閉")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraManagerCcNode()
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
