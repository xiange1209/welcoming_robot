#!/usr/bin/env python3
"""直接叫 MPPI 跑一趟 —— 繞過「nav2 規劃 + 純追蹤執行」那條主線。

★ 為什麼需要單獨一支
navigation_action_cc_node 從 2026-08-10 起預設走「nav2 規劃 + 純追蹤執行」，
MPPI 只剩下「教導-重現路線根本起不了跑」時的退路（見該檔 84~104 行）。
所以送 /navigate 動作**測不到 MPPI**，log 會寫「純追蹤執行」。
要測 MPPI 必須直接送 nav2 的 navigate_to_pose。

★ 這一趟要回答兩個問題
    1. MPPI 現在能不能走完（它是退路，退路壞掉等於沒有退路）
    2. MPPI 會不會前後打舵 —— 看 /cmd_vel 的 linear.x 有沒有負值
       ★ 純追蹤那條在 8/17 的大廳實測已經證實會倒車（指令 335 筆、實測 336 筆），
         但那是**另一個控制器**，不能拿來替 MPPI 背書。

★ 安全
    前方扇區最小淨空低於 STOP_CLEAR 就送取消 + 零速。
    真的失控跑 `~/maprun/estop_cc.sh`，或按平板紅鈕
    （紅鈕對 action 送「取消全部」，8/17 已實機驗證 0.768 秒生效）。

用法：
    python3 mppi_direct.py 門口
    python3 mppi_direct.py 門口 180      # 最多跑 180 秒
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from smartnav_msgs.srv import ListWaypoints

GOAL = sys.argv[1] if len(sys.argv) > 1 else "門口"
MAX_SEC = float(sys.argv[2]) if len(sys.argv) > 2 else 180.0
STOP_CLEAR = 0.15        # 前方淨空門檻（m）
FAN = math.radians(25)   # 前方扇區半角


class MppiRun(Node):
    def __init__(self):
        super().__init__("mppi_direct")
        self.cli = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.wp = self.create_client(ListWaypoints, "/list_waypoints")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._cmd, 10)
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.front = None
        self.cmds = []       # (t, vx, wz)
        self.odom_vx = []
        self.t0 = time.time()

    def _scan(self, m):
        best = None
        for i, r in enumerate(m.ranges):
            a = m.angle_min + i * m.angle_increment
            if -FAN <= a <= FAN and r > 0.05 and not math.isinf(r) and not math.isnan(r):
                best = r if best is None else min(best, r)
        self.front = best

    def _cmd(self, m):
        self.cmds.append((time.time() - self.t0, m.linear.x, m.angular.z))

    def _odom(self, m):
        self.odom_vx.append(m.twist.twist.linear.x)

    def zero(self):
        z = Twist()
        for _ in range(10):
            self.pub.publish(z)
            time.sleep(0.02)


def main():
    rclpy.init()
    n = MppiRun()

    if not n.wp.wait_for_service(timeout_sec=8.0):
        print("✗ /list_waypoints 不在"); return 1
    f = n.wp.call_async(ListWaypoints.Request())
    rclpy.spin_until_future_complete(n, f, timeout_sec=15.0)
    if not f.done():
        print("✗ 查不到地點"); return 1
    byname = {w.waypoint_name: w for w in f.result().waypoints_info}
    if GOAL not in byname:
        print(f"✗ 沒有地點「{GOAL}」，有的是：{list(byname)}"); return 1

    # 等雷射，沒有就不動車
    t = time.time()
    while rclpy.ok() and time.time() - t < 6.0 and n.front is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.front is None:
        print("✗ 收不到 /scan，不動車。"); n.zero(); return 1
    print(f"  出發前正前方淨空 {n.front:.2f} m")

    if not n.cli.wait_for_server(timeout_sec=15.0):
        print("✗ navigate_to_pose 動作伺服器不在（nav2 沒起來？）"); return 1

    goal = NavigateToPose.Goal()
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = n.get_clock().now().to_msg()
    ps.pose = byname[GOAL].pose
    goal.pose = ps

    print(f"  → 送 navigate_to_pose 到「{GOAL}」（MPPI 控制器）")
    n.t0 = time.time()
    n.cmds.clear()
    fut = n.cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=20.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print("✗ 目標被拒絕"); n.zero(); return 1

    res_fut = gh.get_result_async()
    reason = "動作自己結束"
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if res_fut.done():
            break
        if time.time() - n.t0 > MAX_SEC:
            reason = f"逾時 {MAX_SEC:.0f} 秒，主動取消"
            gh.cancel_goal_async(); break
        if n.front is not None and n.front < STOP_CLEAR:
            reason = f"★看門狗：前方只剩 {n.front:.2f} m，主動取消"
            gh.cancel_goal_async(); n.zero(); break
    time.sleep(1.0)
    n.zero()

    dur = time.time() - n.t0
    vx = [c[1] for c in n.cmds]
    neg = [v for v in vx if v < -0.01]
    negod = [v for v in n.odom_vx if v < -0.01]
    print("\n" + "=" * 62)
    print(f"  結束原因：{reason}　歷時 {dur:.1f} 秒")
    if res_fut.done():
        try:
            print(f"  動作結果狀態：{res_fut.result().status}")
        except Exception:
            pass
    print("-" * 62)
    if vx:
        print(f"  /cmd_vel {len(vx)} 筆　{len(vx)/max(dur,0.1):.1f} Hz")
        print(f"  linear.x 範圍 {min(vx):+.3f} ~ {max(vx):+.3f}")
        print(f"  ★ 倒車指令 {len(neg)} 筆　實際倒車 {len(negod)} 筆")
        if neg:
            print("    -> MPPI **會**前後打舵")
        else:
            print("    -> MPPI 全程只前進，**沒有**倒車")
    else:
        print("  /cmd_vel 一筆都沒有 —— MPPI 根本沒輸出")
    print(f"  結束時前方淨空 {n.front:.2f} m" if n.front else "")
    print("=" * 62)
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
