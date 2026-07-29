#!/usr/bin/env python3

"""smartnav_navigation_cc 診斷工具

為什麼不直接用 ros2 CLI：這個網段上有別台機器在跑同一套 stack，
`ros2 node list` / `ros2 topic info` 常常逾時 (實測 20 秒以上還在 Terminated)。
這裡改用 rclpy 的 graph API 直接問本機的 DDS participant，快很多也穩很多。

所有服務呼叫一律走 MultiThreadedExecutor + 背景執行緒。
用單執行緒 spin 再 call_async 然後在同一執行緒等 future，會直接死鎖。

用法：
    nav_cc_check.py sensors      檢查 /scan、odom TF
    nav_cc_check.py mode         檢查目前模式與生命週期狀態
    nav_cc_check.py map          檢查 /map 發布者數與地圖內容
    nav_cc_check.py nav2         檢查 nav2 節點是否 active
    nav_cc_check.py ghosts       檢查網路上有沒有別台機器的幽靈節點
    nav_cc_check.py all          全部
    nav_cc_check.py plan X Y     測試規劃器能否從目前位置規劃到 (X, Y)

離開碼 0 = 通過，1 = 有問題。
"""

import math
import os
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
import tf2_ros

OK = "\033[32m✓\033[0m"
BAD = "\033[31m✗\033[0m"
WARN = "\033[33m!\033[0m"

# 由 map_service_cc 獨佔管理的生命週期節點
MODE_NODES = ["slam_toolbox", "amcl", "map_server", "map_saver"]
# 由 lifecycle_manager_navigation_cc 管理的 nav2 節點
NAV2_NODES = [
    "controller_server",
    "planner_server",
    "bt_navigator",
    "behavior_server",
    "smoother_server",
    "velocity_smoother",
    "collision_monitor",
    "waypoint_follower",
]


class Checker:
    def __init__(self):
        rclpy.init()
        self.node = Node("nav_cc_check")
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.failures = []
        self._subs = []  # 保留訂閱參照，避免被 GC (拆訂閱會弄死 spin 執行緒，見 _collect)
        time.sleep(2.0)  # 等 discovery

    def shutdown(self):
        """關閉順序很重要，弄錯會 segfault

        背景執行緒還在 spin 時就 destroy_node()，executor 會存取already-freed
        的節點記憶體 -> "Segmentation fault (core dumped)"。結果其實是對的
        (檢查訊息都印完了)，但 exit code 會變成 139，讓呼叫端誤判成失敗。
        必須先讓 executor 停下來、等執行緒收工，才能拆節點。
        """
        try:
            self.executor.shutdown(timeout_sec=2.0)
        except Exception:
            pass
        self._spin_thread.join(timeout=3.0)
        try:
            self.executor.remove_node(self.node)
        except Exception:
            pass
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def fail(self, msg):
        self.failures.append(msg)
        print(f"  {BAD} {msg}")

    def ok(self, msg):
        print(f"  {OK} {msg}")

    def warn(self, msg):
        print(f"  {WARN} {msg}")

    # ------------------------------------------------------------------
    def _call(self, client, request, timeout=8.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(request)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return fut.result() if fut.done() else None

    def _can_tf(self, a, b, timeout=8.0):
        """TF 檢查一律要重試

        TransformListener 剛建立時 buffer 是空的，尤其 map -> odom_combined
        要等 amcl 發第一則才會出現。查太早會誤報「TF 不存在」，
        但下一秒 tf2_echo 卻查得到 —— 這種假警報會讓人去追不存在的問題。
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.tf_buffer.can_transform(a, b, rclpy.time.Time()):
                return True
            time.sleep(0.3)
        return False

    def _lifecycle_state(self, node_name):
        c = self.node.create_client(GetState, f"/{node_name}/get_state")
        res = self._call(c, GetState.Request(), timeout=5.0)
        return res.current_state.label if res else "無回應"

    def _collect(self, topic, msg_type, qos, secs=5.0, want=1):
        """訂閱一段時間收訊息。

        **不要 destroy_subscription。** MultiThreadedExecutor 正在背景 spin 時
        拆訂閱，executor 重建 wait set 會踩到已標記銷毀的物件，spin 執行緒直接
        噴 InvalidHandle 死掉：
            rclpy._rclpy_pybind11.InvalidHandle:
                cannot use Destroyable because destruction was requested
        spin 一死，TransformListener 就再也收不到 TF，後面所有 can_transform
        全部回 false —— 於是診斷工具報「TF map -> base_footprint 不存在」，
        但同一時間 tf2_echo 明明查得到。追這個假故障浪費很多時間。

        這是短命的 CLI 工具，訂閱留著到行程結束完全沒差。
        """
        got = []
        self._subs.append(
            self.node.create_subscription(msg_type, topic, lambda m: got.append(m), qos)
        )
        t0 = time.time()
        while time.time() - t0 < secs and len(got) < want:
            time.sleep(0.1)
        return got

    # ------------------------------------------------------------------
    def check_sensors(self):
        print("\n--- 感測器 ---")
        n_pub = self.node.count_publishers("/scan")
        if n_pub == 0:
            self.fail("/scan 沒有發布者 —— 底盤/雷達沒起來，slam 會 active 但永遠不發 map->odom")
        else:
            self.ok(f"/scan 發布者 {n_pub} 個")

        scans = self._collect("/scan", LaserScan, qos_profile_sensor_data, secs=6.0, want=5)
        if not scans:
            self.fail("/scan 收不到資料")
        else:
            m = scans[-1]
            valid = [r for r in m.ranges if r not in (float("inf"), float("nan")) and r > 0]
            now = self.node.get_clock().now().nanoseconds / 1e9
            st = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
            lag = now - st
            self.ok(f"/scan {len(scans)} 則, {len(m.ranges)} 點, 有效 {len(valid)}, 延遲 {lag:+.3f}s")
            if lag > 0.5:
                self.fail(f"雷射時間戳延遲 {lag:.2f}s 過大 —— costmap 會一直 dropping message")
            if not valid:
                self.fail("雷射所有點都無效")

        for a, b in [("odom_combined", "base_footprint"), ("base_footprint", "laser")]:
            if self._can_tf(a, b):
                self.ok(f"TF {a} -> {b}")
            else:
                self.fail(f"TF {a} -> {b} 不存在")

    def check_mode(self):
        print("\n--- 模式與生命週期 ---")
        c = self.node.create_client(Trigger, "/get_nav_mode")
        res = self._call(c, Trigger.Request())
        mode = res.message if res else None
        if mode is None:
            self.fail("/get_nav_mode 無回應 —— map_service_cc 沒起來?")
        else:
            self.ok(f"目前模式: {mode}")

        states = {n: self._lifecycle_state(n) for n in MODE_NODES}
        for n, s in states.items():
            print(f"      {n}: {s}")

        # 核心不變量：建圖與定位的地圖來源必須互斥
        if mode == "mapping":
            if states["slam_toolbox"] != "active":
                self.fail(f"建圖模式但 slam_toolbox={states['slam_toolbox']} (應為 active)")
            if states["amcl"] != "unconfigured":
                self.fail(f"建圖模式但 amcl={states['amcl']} (應為 unconfigured)")
            if states["map_server"] != "unconfigured":
                self.fail(f"建圖模式但 map_server={states['map_server']} (應為 unconfigured)")
            if not self.failures:
                self.ok("建圖模式的生命週期組合正確")
        elif mode == "localization":
            if states["slam_toolbox"] != "unconfigured":
                self.fail(
                    f"定位模式但 slam_toolbox={states['slam_toolbox']} (應為 unconfigured) "
                    "—— 這就是舊版 /map 兩個發布者的根因"
                )
            if states["amcl"] != "active":
                self.fail(f"定位模式但 amcl={states['amcl']} (應為 active)")
            if states["map_server"] != "active":
                self.fail(f"定位模式但 map_server={states['map_server']} (應為 active)")
            if not self.failures:
                self.ok("定位模式的生命週期組合正確")

    def check_map(self):
        print("\n--- 地圖 ---")
        # map_server 剛 activate 時 publisher 可能還沒註冊完，
        # 太早查會誤報「沒有發布者」但下一秒又讀得到地圖。重試幾次再判定。
        n_pub = self.node.count_publishers("/map")
        for _ in range(10):
            if n_pub > 0:
                break
            time.sleep(0.5)
            n_pub = self.node.count_publishers("/map")

        if n_pub == 1:
            self.ok("/map 發布者 1 個 (正確)")
        elif n_pub == 0:
            self.fail("/map 沒有發布者")
        else:
            self.fail(f"/map 有 {n_pub} 個發布者 —— static_layer 會反覆 resize，規劃器會回 Start occupied")

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        maps = self._collect("/map", OccupancyGrid, qos, secs=6.0)
        if not maps:
            self.fail("/map 收不到資料")
        else:
            m = maps[-1]
            known = sum(1 for v in m.data if v >= 0)
            total = len(m.data)
            self.ok(
                f"地圖 {m.info.width}x{m.info.height} @ {m.info.resolution:.3f}m, "
                f"origin ({m.info.origin.position.x:.2f}, {m.info.origin.position.y:.2f}), "
                f"已知 {known}/{total} ({100*known/max(total,1):.0f}%)"
            )
            if known < 100:
                self.warn("已知格數很少 —— 才剛開始建圖?")

        if self._can_tf("map", "base_footprint", timeout=15.0):
            tf = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.ok(
                f"map 位姿 ({tf.transform.translation.x:.2f}, "
                f"{tf.transform.translation.y:.2f}, {math.degrees(yaw):.0f}°)"
            )
        else:
            self.fail("TF map -> base_footprint 不存在")

    def check_nav2(self):
        print("\n--- Nav2 ---")
        inactive = []
        for n in NAV2_NODES:
            s = self._lifecycle_state(n)
            if s != "active":
                inactive.append(f"{n}={s}")
        if inactive:
            self.fail(f"Nav2 節點未 active: {', '.join(inactive)}")
        else:
            self.ok(f"Nav2 {len(NAV2_NODES)} 個節點全部 active")

    def check_ghosts(self):
        """檢查網路上有沒有別台機器的節點

        這個網段實測有 3 個 /navigate action server (節點名 /navigation_service_node)，
        本機 ps 卻查無此行程。送 action 可能被別台接走並回覆它們舊程式碼的訊息。
        """
        print("\n--- 幽靈節點 (其他機器) ---")
        names = [f"{ns}{nm}" for nm, ns in self.node.get_node_names_and_namespaces()]

        old_pkg_nodes = [n for n in names if n in ("/navigation_service_node", "/waypoint_service_node", "/map_service_node")]
        if old_pkg_nodes:
            from collections import Counter
            cnt = Counter(old_pkg_nodes)
            self.warn(f"看到舊套件節點 (可能是別台機器): {dict(cnt)}")
            self.warn("  本機若沒跑舊 stack，這些就是網路上其他機器的，送 action 可能被它們接走")
        else:
            self.ok("沒有看到舊套件的節點")

        for action in ["/navigate", "/create_map"]:
            info = self.node.get_publishers_info_by_topic(f"{action}/_action/status")
            n = len(info)
            if n <= 1:
                self.ok(f"{action} action server {n} 個")
            else:
                self.warn(f"{action} 有 {n} 個 action server —— 目標可能被別台機器接走")
                for i in info:
                    print(f"      {i.node_namespace}{i.node_name}")

    def check_plan(self, gx, gy):
        """測試規劃器 (只規劃，車子不會動)"""
        print(f"\n--- 規劃測試 -> ({gx}, {gy}) ---")
        from rclpy.action import ActionClient
        from nav2_msgs.action import ComputePathToPose
        from geometry_msgs.msg import PoseStamped

        ac = ActionClient(self.node, ComputePathToPose, "compute_path_to_pose")
        if not ac.wait_for_server(timeout_sec=15.0):
            self.fail("compute_path_to_pose server 不在線 —— planner_server 沒 active?")
            return

        goal = ComputePathToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = "map"
        p.pose.position.x = float(gx)
        p.pose.position.y = float(gy)
        p.pose.orientation.w = 1.0
        goal.goal = p
        goal.use_start = False
        goal.planner_id = "GridBased"

        fut = ac.send_goal_async(goal)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 15:
            time.sleep(0.1)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted:
            self.fail("規劃目標未被接受")
            return

        rf = gh.get_result_async()
        t0 = time.time()
        while not rf.done() and time.time() - t0 < 20:
            time.sleep(0.1)
        if not rf.done():
            self.fail("規劃逾時")
            return

        res = rf.result()
        n_poses = len(res.result.path.poses)
        if res.status == 4 and n_poses > 0:
            self.ok(f"規劃成功，路徑 {n_poses} 點 (error_code={res.result.error_code})")
        else:
            self.fail(
                f"規劃失敗 status={res.status} error_code={res.result.error_code} "
                "—— error_code=101 通常是 Start occupied (機器人腳下被判定為障礙)"
            )


def main():
    args = sys.argv[1:] or ["all"]
    cmd = args[0]

    c = Checker()
    try:
        if cmd == "plan":
            if len(args) < 3:
                print("用法: nav_cc_check.py plan <X> <Y>")
                return 1
            c.check_plan(float(args[1]), float(args[2]))
        elif cmd == "sensors":
            c.check_sensors()
        elif cmd == "mode":
            c.check_mode()
        elif cmd == "map":
            c.check_map()
        elif cmd == "nav2":
            c.check_nav2()
        elif cmd == "ghosts":
            c.check_ghosts()
        elif cmd == "all":
            c.check_sensors()
            c.check_mode()
            c.check_map()
            c.check_nav2()
            c.check_ghosts()
        else:
            print(f"未知指令: {cmd}")
            return 1

        print()
        if c.failures:
            print(f"\033[31m{len(c.failures)} 項未通過:\033[0m")
            for f in c.failures:
                print(f"  - {f}")
            return 1
        print("\033[32m全部通過\033[0m")
        return 0
    finally:
        c.shutdown()


if __name__ == "__main__":
    code = main()
    # 直接 _exit，不走 Python/rclpy 的正常關閉流程。
    #
    # 理由：rclpy 的關閉在這台機器上不可靠。TransformListener 與
    # MultiThreadedExecutor 在拆除時會互相踩到對方的 guard condition，實測會出現
    #   InvalidHandle: cannot use Destroyable because destruction was requested
    #   'NoneType' object has no attribute 'trigger'
    # 更糟的是接上實機後它會直接卡住不返回 (90 秒還沒結束)，
    # 呼叫端的測試腳本就整個掛在那裡。
    #
    # 這是個一次性的 CLI 診斷工具，行程結束時 OS 會回收所有資源，
    # 沒有需要優雅關閉的東西。結果印完就走人最可靠。
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
