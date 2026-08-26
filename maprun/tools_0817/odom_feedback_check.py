#!/usr/bin/env python3
"""量「指令速度 → 實際輪速 → EKF 融合速度」這條回授鏈到底斷在哪。

★ 為什麼要量
2026-08-17 路線 B 導航 4.7 秒就被 stuck_detector_cc 判定卡住：
    偵測到卡住：指令 0.130 m/s 但實測 0.000 m/s，持續 4.0 秒
但**同一段時間雷射量到的牆距一直在變**（左0.35/右0.21 → 左0.17/右0.34），
使用者也親眼看到車子在動。雷射說有動、里程計說沒動 —— 兩者必有一個在說謊。

stuck_detector_cc 讀的是 `/odom_combined` 的 twist.twist.linear.x（EKF 輸出）。
這支同時訂 `/odom`（輪速原始值）與 `/odom_combined`（EKF），
再自己發一段小小的前進指令，就能分辨三種情況：

    指令有、/odom 有、/odom_combined 有   -> 回授正常，卡住判定是真的
    指令有、/odom 有、/odom_combined 0    -> EKF 沒把速度傳出來（設定或時間戳問題）
    指令有、/odom 0                       -> 底盤韌體/驅動沒回報輪速

★ 安全
  - 只走 DIST 公尺（預設 0.20），到了就停
  - 每個週期檢查正前方雷射，< SAFE_M 立刻停
  - 不論怎麼結束（例外、Ctrl-C）都會補送零速

用法：
    python3 odom_feedback_check.py            # 前進 0.20 m
    python3 odom_feedback_check.py 0.30       # 前進 0.30 m
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

DIST = float(sys.argv[1]) if len(sys.argv) > 1 else 0.20
# ★ 2026-08-17：第二個參數是指令速度。導航失敗時的指令是 0.050 m/s，
#   而 0.10 m/s 實測走得動 —— 懷疑底盤在低速有死區，要逐段掃出臨界值。
SPEED = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10
SAFE_M = 0.50         # 正前方低於這個距離就停
FRONT_HALF_DEG = 15.0


class Check(Node):
    def __init__(self):
        super().__init__("odom_feedback_check")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._raw, 20)
        self.create_subscription(Odometry, "/odom_combined", self._ekf, 20)
        # ★ 2026-08-25：改用 qos_profile_sensor_data。
        #   /scan 本身是 RELIABLE（lslidar_x10_driver.cpp:171 用
        #   create_publisher(..., 10)），所以原本的整數 depth 其實收得到 ——
        #   這一改是為了統一：BEST_EFFORT 訂閱端對 RELIABLE 與 BEST_EFFORT
        #   兩種發布端都通，換成 /scan_slam 之類的話題也不用再改一次。
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.raw = []      # (t, vx)
        self.ekf = []      # (t, vx)
        self.raw_pose = None
        self.ekf_pose = None
        self.raw_start = None
        self.ekf_start = None
        self.front = None

    def _raw(self, m):
        self.raw.append((time.time(), m.twist.twist.linear.x))
        p = m.pose.pose.position
        self.raw_pose = (p.x, p.y)
        if self.raw_start is None:
            self.raw_start = (p.x, p.y)

    def _ekf(self, m):
        self.ekf.append((time.time(), m.twist.twist.linear.x))
        p = m.pose.pose.position
        self.ekf_pose = (p.x, p.y)
        if self.ekf_start is None:
            self.ekf_start = (p.x, p.y)

    def _scan(self, m):
        best = None
        n = len(m.ranges)
        for i in range(n):
            a = math.degrees(m.angle_min + i * m.angle_increment)
            if -FRONT_HALF_DEG <= a <= FRONT_HALF_DEG:
                r = m.ranges[i]
                if r > 0.05 and not math.isinf(r) and not math.isnan(r):
                    best = r if best is None else min(best, r)
        self.front = best

    def stop(self):
        z = Twist()
        for _ in range(10):
            self.pub.publish(z)
            time.sleep(0.02)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]) if a and b else 0.0


def main():
    rclpy.init()
    n = Check()

    # 先等感測器
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.front is not None and n.raw_pose is not None:
            break
    if n.front is None:
        print("✗ 收不到 /scan，不動車。")
        n.stop(); return 1
    print(f"  正前方淨空 {n.front:.2f} m")
    if n.front < SAFE_M + DIST:
        print(f"✗ 前方只有 {n.front:.2f} m，不夠走 {DIST:.2f} m + 安全裕度 {SAFE_M:.2f} m。不動車。")
        n.stop(); return 1

    # 基準：靜止 1.5 秒
    n.raw.clear(); n.ekf.clear()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 1.5:
        rclpy.spin_once(n, timeout_sec=0.05)
    base_raw = [v for _, v in n.raw]
    base_ekf = [v for _, v in n.ekf]

    # 走
    n.raw.clear(); n.ekf.clear()
    n.raw_start = n.raw_pose
    n.ekf_start = n.ekf_pose
    cmd = Twist(); cmd.linear.x = SPEED
    t0 = time.time()
    timeout = DIST / SPEED + 3.0
    reason = "走完預定距離"
    try:
        while rclpy.ok():
            el = time.time() - t0
            if el > timeout:
                reason = f"逾時 {timeout:.1f} 秒"
                break
            if n.front is not None and n.front < SAFE_M:
                reason = f"前方 {n.front:.2f} m 低於安全距離"
                break
            if dist(n.raw_start, n.raw_pose) >= DIST:
                break
            n.pub.publish(cmd)
            rclpy.spin_once(n, timeout_sec=0.05)
    finally:
        n.stop()
    moved_raw = dist(n.raw_start, n.raw_pose)
    moved_ekf = dist(n.ekf_start, n.ekf_pose)

    def stat(xs):
        if not xs:
            return "（沒收到訊息）"
        a = [abs(x) for x in xs]
        return f"平均 {sum(a)/len(a):+.4f}　最大 {max(a):+.4f}　筆數 {len(a)}"

    run_raw = [v for _, v in n.raw]
    run_ekf = [v for _, v in n.ekf]

    print("\n" + "=" * 64)
    print(f"  指令 {SPEED:+.3f} m/s，實走 {time.time()-t0:.1f} 秒（{reason}）")
    print("-" * 64)
    print(f"  /odom          靜止時 {stat(base_raw)}")
    print(f"                 行進時 {stat(run_raw)}")
    print(f"  /odom_combined 靜止時 {stat(base_ekf)}")
    print(f"                 行進時 {stat(run_ekf)}")
    print("-" * 64)
    print(f"  位置變化   /odom {moved_raw:.3f} m　/odom_combined {moved_ekf:.3f} m")
    print("-" * 64)
    mr = max([abs(v) for v in run_raw], default=0.0)
    me = max([abs(v) for v in run_ekf], default=0.0)
    TH = 0.02   # stuck_detector 的 motion_threshold
    if mr < TH and moved_raw < 0.05:
        print("  判定：**輪速回報是 0，而且位置也沒變** -> 車子真的沒動（機械/韌體）")
    elif mr < TH <= me:
        print("  判定：**/odom 沒有速度但 EKF 有** -> 底盤沒回報輪速，EKF 靠 IMU 補")
    elif me < TH <= mr:
        print("  判定：★ **/odom 有速度但 /odom_combined 是 0**")
        print("        stuck_detector_cc 讀的正是 /odom_combined —— 這就是誤判卡住的原因")
    else:
        print("  判定：兩條都有速度，回授正常 -> 導航時的『實測 0.000』另有原因")
    print("=" * 64)
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
