#!/usr/bin/env python3
"""驗證：底盤回報的里程計角速度在倒車時是否不翻號。

## 為什麼要查這個（2026-08-04 的量測結果）

    靜止 60 秒        EKF yaw 漂移 +0.03 度/分   （陀螺零偏已被補償）
    前進 2.570 m      −13.5 度/分
    倒車 1.949 m      −12.4 度/分

**前進與倒車同號**，而且換算成「每分鐘」兩者幾乎相同。

這兩件事各自排除一個嫌疑：

- 同號 → 不是轉向角偏移。阿克曼運動學 `ω = v·tan(δ)/L`，`v` 變號則 `ω`
  必然變號；一個固定的舵角偏差在倒車時會讓車往**反方向**偏。
- 時間比例而非距離比例 → 不是「每公尺」的效應，所以 `trim_rad_per_m`
  這個參數從單位上就不對，速度一變就對不上。這解釋了為什麼歷來量三次
  得到三個不同的值。

剩下的嫌疑落在「一動起來才進來的東西」。靜止時輪速里程計沒有貢獻，
一動 EKF 就開始吃它 —— 所以懷疑落在里程計本身。

## 具體假設

本專案所有速度指令的發布端都用 `ω = |v|·curvature`（不隨方向翻號），
韌體的 `Vz_to_Akm_Angle` 自己從 `R = Vx/Vz` 推出方向盤該打哪邊。

**如果底盤「回報」的里程計角速度也用同一個慣例**，那倒車時 `/odom` 的
`angular.z` 符號就是錯的，EKF 會吃到反向的 yaw 速率。而教導路徑有很大
比例是倒車，所以誤差會持續累積。

## 怎麼驗

IMU 的陀螺儀直接量車體的角速度，**不受行進方向影響**，可以當基準。
同時記錄兩者，分前進段與倒車段比對符號關係：

    前進段 odom 與 imu 同號、倒車段也同號  -> 底盤正常，另尋原因
    前進段同號、倒車段**反號**             -> 命中，就是這個問題

## 用法

    # 1. 車子放空曠處（大廳），前後各留 1.5 m 以上
    # 2. 先啟動感測器
    ~/maprun/run_sensors_cc.sh false

    # 3. 跑這個工具（會自己開車，不需要遙控）
    python3 ~/maprun/odom_gyro_check_cc.py

    # 或只被動記錄、自己用遙控開：
    python3 ~/maprun/odom_gyro_check_cc.py --passive

自動模式會依序做四段：前進直行 / 前進左彎 / 倒車直行 / 倒車左彎，
每段 4 秒，段間停 2 秒。**轉彎段才是關鍵** —— 直行時 ω 接近零，
噪聲會主導符號，看不出東西。

結果印在畫面上，同時存成 `~/odom_gyro_check_<時間>.csv`。
"""
import argparse
import csv
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# 低於這個角速度就不列入符號比對 —— 直行時兩邊都在噪聲裡打轉，
# 符號一致率會退化成擲硬幣，反而稀釋掉轉彎段的真實訊號。
OMEGA_DEADBAND = 0.05      # rad/s
# 低於這個線速度視為靜止，不分類到前進或倒車
SPEED_DEADBAND = 0.02      # m/s


class OdomGyroCheck(Node):
    def __init__(self, passive: bool):
        super().__init__("odom_gyro_check_cc")
        self.passive = passive
        self._lock = threading.Lock()
        self.rows = []

        self.odom_wz = 0.0
        self.odom_vx = 0.0
        self.imu_wz = 0.0
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self._got_odom = False
        self._got_imu = False

        # /odom 通常是 RELIABLE，/imu/data_raw 是 sensor data（BEST_EFFORT）。
        # 訂閱端的要求不能高於發布端的保證，否則連線根本不會建立。
        self.create_subscription(Odometry, "odom", self._odom_cb, 10)
        self.create_subscription(
            Imu, "imu/data_raw", self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel", self._cmd_cb, 10)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.t0 = time.monotonic()
        self.create_timer(0.05, self._sample)     # 20 Hz 取樣

    # ── 訂閱 ──────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        self.odom_wz = msg.twist.twist.angular.z
        self.odom_vx = msg.twist.twist.linear.x
        self._got_odom = True

    def _imu_cb(self, msg: Imu) -> None:
        self.imu_wz = msg.angular_velocity.z
        self._got_imu = True

    def _cmd_cb(self, msg: Twist) -> None:
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z

    def _sample(self) -> None:
        with self._lock:
            self.rows.append({
                "t": round(time.monotonic() - self.t0, 3),
                "cmd_vx": round(self.cmd_vx, 4),
                "cmd_wz": round(self.cmd_wz, 4),
                "odom_vx": round(self.odom_vx, 4),
                "odom_wz": round(self.odom_wz, 5),
                "imu_wz": round(self.imu_wz, 5),
            })

    # ── 自動測試動作 ──────────────────────────────────────
    def _drive(self, vx: float, wz: float, sec: float, label: str) -> None:
        self.get_logger().info(f"▶ {label}：vx={vx:+.2f} wz={wz:+.2f}，{sec:.0f} 秒")
        t_end = time.monotonic() + sec
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        while time.monotonic() < t_end and rclpy.ok():
            self.pub.publish(msg)      # 韌體指令逾時 1.0 秒，必須持續送
            time.sleep(0.05)
        self._stop(2.0)

    def _stop(self, sec: float) -> None:
        t_end = time.monotonic() + sec
        while time.monotonic() < t_end and rclpy.ok():
            self.pub.publish(Twist())
            time.sleep(0.05)

    def run_sequence(self) -> None:
        # 等感測器都有資料再開始，否則前幾秒的紀錄是空的
        for _ in range(100):
            if self._got_odom and self._got_imu:
                break
            time.sleep(0.1)
        if not (self._got_odom and self._got_imu):
            self.get_logger().error(
                f"感測器沒資料（odom={self._got_odom} imu={self._got_imu}），"
                "先確認 run_sensors_cc.sh 起來了")
            return

        self.get_logger().info("開始 —— 車子會自己動，確認前後各有 1.5 m 淨空")
        self._stop(2.0)
        # 直行段留著當對照組，轉彎段才是判斷依據
        self._drive(+0.12, 0.0, 4.0, "① 前進直行")
        self._drive(+0.12, +0.25, 4.0, "② 前進左彎  ← 關鍵")
        self._drive(-0.10, 0.0, 4.0, "③ 倒車直行")
        self._drive(-0.10, +0.25, 4.0, "④ 倒車左彎  ← 關鍵")
        self._stop(1.0)
        self.get_logger().info("動作完成")


def analyse(rows) -> None:
    """分前進段與倒車段，比對 odom 與 imu 的角速度符號關係。"""
    fwd = [r for r in rows if r["cmd_vx"] > SPEED_DEADBAND]
    rev = [r for r in rows if r["cmd_vx"] < -SPEED_DEADBAND]

    print()
    print("=" * 68)
    print("結果")
    print("=" * 68)

    verdicts = {}
    for label, seg in (("前進", fwd), ("倒車", rev)):
        # 只看兩邊都明確在轉的取樣，直行段的噪聲會稀釋訊號
        turning = [r for r in seg
                   if abs(r["imu_wz"]) > OMEGA_DEADBAND
                   and abs(r["odom_wz"]) > OMEGA_DEADBAND]
        print(f"\n【{label}】取樣 {len(seg)} 筆，其中明確轉彎 {len(turning)} 筆")
        if len(turning) < 20:
            print("   ⚠ 轉彎取樣太少，無法判斷。請確認轉彎段車子真的有轉。")
            verdicts[label] = None
            continue

        same = sum(1 for r in turning
                   if (r["odom_wz"] > 0) == (r["imu_wz"] > 0))
        ratio = same / len(turning)
        mean_odom = sum(r["odom_wz"] for r in turning) / len(turning)
        mean_imu = sum(r["imu_wz"] for r in turning) / len(turning)
        print(f"   odom 平均 ω = {mean_odom:+.4f} rad/s")
        print(f"   imu  平均 ω = {mean_imu:+.4f} rad/s")
        print(f"   符號一致率 = {ratio * 100:.1f}%")
        verdicts[label] = ratio

    print()
    print("-" * 68)
    f_r, r_r = verdicts.get("前進"), verdicts.get("倒車")
    if f_r is None or r_r is None:
        print("判定：資料不足。轉彎段要確實有轉起來才判斷得出。")
    elif f_r > 0.8 and r_r < 0.2:
        print("★★ 命中：前進時 odom 與 imu 同號，倒車時反號。")
        print("   底盤回報的里程計角速度在倒車時沒有翻號，EKF 吃到的 yaw")
        print("   速率在倒車段是反的。教導路徑有大量倒車，誤差會持續累積。")
        print()
        print("   修法：在 odom 進 EKF 之前，依 linear.x 的符號修正 angular.z，")
        print("   或直接讓 EKF 的 odom 設定不吃 vyaw、改由 IMU 提供。")
        print("   ★ 在這之前不要再調 trim_rad_per_m —— 那是錯的旋鈕。")
    elif f_r > 0.8 and r_r > 0.8:
        print("判定：兩段都同號，底盤回報正常。這個假設**不成立**，")
        print("   左偏的原因要往別處找（機械對中、輪徑不一致、EKF 參數）。")
    else:
        print(f"判定：不明確（前進 {f_r * 100:.0f}% / 倒車 {r_r * 100:.0f}%）。")
        print("   可能是轉彎幅度太小或有其他雜訊。加大 wz 再測一次。")
    print("-" * 68)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--passive", action="store_true",
                    help="只記錄不開車，由你自己遙控（前進轉一段、倒車轉一段）")
    ap.add_argument("--seconds", type=float, default=60.0,
                    help="passive 模式的記錄秒數（預設 60）")
    args = ap.parse_args()

    rclpy.init()
    node = OdomGyroCheck(args.passive)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    try:
        if args.passive:
            node.get_logger().info(
                f"被動記錄 {args.seconds:.0f} 秒 —— 請遙控車子："
                "前進並持續左轉一段，然後倒車並持續左轉一段")
            time.sleep(args.seconds)
        else:
            node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop(0.5)
        with node._lock:
            rows = list(node.rows)

        if rows:
            path = os.path.expanduser(
                f"~/odom_gyro_check_{int(rows[-1]['t'])}s.csv")
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
            print(f"\n逐筆資料已存：{path}（{len(rows)} 筆）")
            analyse(rows)
        else:
            print("沒有取樣到任何資料")

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
