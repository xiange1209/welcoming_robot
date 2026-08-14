#!/usr/bin/env python3
"""段五 5-2：MPPI 新基準 —— 跑一趟，什麼都別再改。

★ 這一趟要回答的問題（作戰文件 5-2 的驗收表）
    過得了窄轉角        -> MPPI 救回來了，二值成本地圖是主因
    走得比以前直        -> 梯度生效但視野仍不夠
    /cmd_vel 掉到 10 Hz 以下 -> 膨脹計算吃 CPU，先回 0.22/3.5
    完全沒變            -> 瓶頸不在成本地圖

★ 已套用但未實測的改動（2026-08-14）
    R1  PathAlignCritic.use_path_orientations: true -> false
        理由：規劃路徑的朝向是 11.25 度一階的階梯（angle_quantization_bins: 32），
        而這個 critic 權重 25 全場最高，正在追那個階梯。純追蹤只讀位置所以免疫。
    8/12 已在檔案裡：inflation 0.35/4.0、wz_max 0.35、wz_std 0.20

★ 安全：使用者不在場。看門狗量前方扇區最小淨空，低於門檻就自動取消。
  真的失控要跑 `~/maprun/estop_cc.sh`（它會直接砍控制節點再補零速）。
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from smartnav_msgs.action import Navigate
from smartnav_msgs.srv import ListWaypoints

GOAL = sys.argv[1] if len(sys.argv) > 1 else "大廳"
STOP_CLEAR = 0.12        # 前方淨空低於這個就取消（雷達到車頭保險桿還有 0.311-0.089）
FAN = math.radians(25)   # 前方扇區半角
MAX_SEC = 240.0


class Base(Node):
    def __init__(self):
        super().__init__("mppi_baseline")
        self.cmd_t = []
        self.speed = 0.0
        self.clear = None
        self.min_clear = 9.9
        self.zero_cmd = 0
        self.create_subscription(Twist, "cmd_vel", self._cmd, 10)
        self.create_subscription(Odometry, "odom", self._odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan", self._scan, qos_profile_sensor_data)
        self.wp = self.create_client(ListWaypoints, "list_waypoints")
        self.nav = ActionClient(self, Navigate, "navigate")

    def _cmd(self, m):
        self.cmd_t.append(time.time())
        if abs(m.linear.x) < 1e-3 and abs(m.angular.z) < 1e-3:
            self.zero_cmd += 1

    def _odom(self, m):
        self.speed = abs(m.twist.twist.linear.x)

    def _scan(self, m):
        best = None
        for i, r in enumerate(m.ranges):
            if not (m.range_min < r < m.range_max):
                continue
            a = m.angle_min + i * m.angle_increment
            if abs(a) <= FAN:
                best = r if best is None else min(best, r)
        if best is not None:
            self.clear = best
            self.min_clear = min(self.min_clear, best)

    def rate(self, win=5.0):
        now = time.time()
        n = sum(1 for t in self.cmd_t if now - t <= win)
        return n / win


def main():
    rclpy.init(); b = Base()
    if not b.wp.wait_for_service(timeout_sec=12.0):
        print("  ✗ list_waypoints 沒回應"); return 1
    f = b.wp.call_async(ListWaypoints.Request())
    rclpy.spin_until_future_complete(b, f, timeout_sec=15.0)
    tgt = next((w for w in f.result().waypoints_info if w.waypoint_name == GOAL), None)
    if tgt is None:
        print(f"  ✗ 找不到 {GOAL}"); return 1
    if not b.nav.wait_for_server(timeout_sec=15.0):
        print("  ✗ /navigate 沒有 action server"); return 1

    print(f"  目標 {GOAL}　看門狗：前方 ±{math.degrees(FAN):.0f}° 淨空 < {STOP_CLEAR:.2f} m 就取消\n")
    g = Navigate.Goal(); g.waypoint_id = tgt.waypoint_id
    t0 = time.time()
    sf = b.nav.send_goal_async(g)
    rclpy.spin_until_future_complete(b, sf, timeout_sec=20.0)
    gh = sf.result()
    if gh is None or not gh.accepted:
        print("  ✗ 目標被拒絕"); return 1
    rf = gh.get_result_async()

    last = 0.0
    aborted = None
    while time.time() - t0 < MAX_SEC:
        rclpy.spin_once(b, timeout_sec=0.05)
        if rf.done():
            break
        now = time.time() - t0
        if b.clear is not None and b.clear < STOP_CLEAR and b.speed > 0.02:
            aborted = f"淨空 {b.clear:.3f} m 低於門檻"
            print(f"\n  ★ 看門狗觸發：{aborted}，送取消")
            gh.cancel_goal_async()
            break
        if now - last >= 5.0:
            last = now
            print(f"  [{now:6.1f}s] cmd_vel {b.rate():5.2f} Hz　速度 {b.speed:+.3f} m/s　"
                  f"前方淨空 {b.clear if b.clear is not None else float('nan'):.2f} m")

    # 收尾
    t1 = time.time()
    while time.time() - t1 < 15.0 and not rf.done():
        rclpy.spin_once(b, timeout_sec=0.05)

    print()
    total = len(b.cmd_t)
    dur = time.time() - t0
    print(f"  /cmd_vel 總計 {total} 則 / {dur:.0f} 秒 = {total/dur:.2f} Hz"
          f"　（目標 10 Hz，{'✓ 正常' if total/dur >= 8 else '✗ 掉速 —— 膨脹計算吃 CPU，考慮回 0.22/3.5'}）")
    print(f"  其中零速指令 {b.zero_cmd} 則"
          f"（{100*b.zero_cmd/total if total else 0:.0f}%）"
          f"{'　★ 大量零速 = MPPI 收斂到不動，成本地圖仍無梯度' if total and b.zero_cmd > 0.5*total else ''}")
    print(f"  全程最小前方淨空 {b.min_clear:.3f} m")
    if aborted:
        print(f"  ✗ 看門狗中止：{aborted}")
    elif rf.done() and rf.result() is not None:
        r = rf.result().result
        print(f"  結果 success={r.success}　{r.message[:60]}")
    else:
        print(f"  ⚠ {MAX_SEC:.0f} 秒內沒有結果")
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
