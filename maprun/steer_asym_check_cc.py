#!/usr/bin/env python3
"""轉向不對稱：直接量三個實際值，不要從比例反推。

## 為什麼要重量（2026-08-06 的教訓）

那天用兩點打滿舵的資料反推中位：

    R_left  = 0.944 m （舵角 18.83 度）
    R_right = 0.751 m （舵角 23.21 度，正好是韌體硬限）
    用 δ_right = X + δ₀ 反推 -> δ₀ = −2.19 度（偏右）

但**直接下 ω = 0 走 1.398 m，量到的是 +17.75 度（偏左）** —— 方向相反。

★ **兩點打滿舵的資料不足以推出中位。** 打滿舵時舵角被韌體的
`Servo_min/max` 削掉（右邊正好落在硬限 0.751），削掉多少不知道，
反推出來的偏移就是錯的。

## 這支工具量什麼

三段各自獨立，互相不依賴：

    ① 直走（ω = 0）      -> 零位偏移 δ₀      這是「中位在哪」
    ② 左打滿             -> R_left           這是「往左能轉多急」
    ③ 右打滿             -> R_right          這是「往右能轉多急」

零位偏移只由 ① 決定 —— 不從 ②③ 反推，因為那兩個受硬限截斷污染。
②③ 之間的差異才是真正的左右增益不對稱。

## 用法

    # 空曠處（大廳），前後左右各留 1.5 m 以上
    ~/maprun/run_sensors_cc.sh false
    python3 ~/maprun/steer_asym_check_cc.py

    python3 ~/maprun/steer_asym_check_cc.py --dry   # 只印計畫不開車

★ 車子會自己動。開始前會檢查前方淨空，但**側向要你自己看**。
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

WHEELBASE = 0.322          # 軸距（廠商韌體值）
MIN_RADIUS = 0.80          # 控制器箝制值
SPEED = 0.08               # 低速：舵機動態的影響最小
SEG_SEC = 6.0              # 每段時間
NEED_FRONT = 1.2           # 開始前要求的前方淨空



def radius_of(chord: float, dyaw: float) -> float:
    """從**弦長**與轉角求轉彎半徑。

    ★ 2026-08-10：原本直接寫 `chord / abs(dyaw)`，但那是把弦長當弧長。
      正確關係是 c = 2R sin(theta/2)，所以 chord/theta = R * sinc(theta/2)，
      **系統性地把半徑報得比實際小**（左 theta=0.51 rad 低估 1.1%、
      右 theta=0.64 rad 低估 1.7%）。

      偏誤方向剛好是最糟的那一邊：讓車看起來比實際**更能轉**。
      而這支工具的用途正是去量「左右極限到底是多少」，
      而 path_teach 的餘裕只留 0.6%~6.5% —— 同一個量級，會影響結論。
    """
    a = abs(dyaw)
    if a < 1e-3:
        return chord / max(a, 1e-9)
    return chord / (2.0 * math.sin(a / 2.0))

def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class SteerAsym(Node):
    def __init__(self):
        super().__init__("steer_asym_check_cc")
        self._lock = threading.Lock()
        self.odom = None
        self.scan = None
        # ★★ 2026-08-10 修正：改直接發 /cmd_vel ★★
        #
        # 原本發 `cmd_vel_smoothed`（而上面的舊註解寫的是 cmd_vel_trimmed，
        # 兩者還互相矛盾）。那條話題**唯一的訂閱者是 steering_trim_cc**，
        # 而它只在 nav_bringup_cc.launch.py 裡啟動 —— 但本檔用法段寫的是
        # 「只跑 run_sensors_cc.sh」。
        #
        # 後果是**完全靜默的**：發布成功、節點正常、四段照樣印「▶ ① 前進直走」、
        # 等滿秒數、最後印出完整結果表，只是每段都「走 0.000 m、轉 +0.00 度」，
        # 三個判定區塊因為 arc > 0.05 的門檻沒過而全部跳過。
        # 現場會先去查電量、舵機、串口——查錯方向。
        #
        # 而且**這支工具本來就不該經過 steering_trim**：它的目的是量
        # 硬體原始的左右不對稱，補償器會把要量的東西改掉。
        # 對照組：odom_gyro_check_cc.py:101 就是直接發 cmd_vel，
        # 同樣只跑 sensors 卻會動。
        #
        # ⚠️ 直接發 cmd_vel 會繞過 collision_monitor，所以本檔自己的
        #    掃描淨空檢查是唯一的防線 —— 執行前務必確認前後左右各 1.5 m。
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom_combined", self._odom_cb, 10)
        self.create_subscription(LaserScan, "scan", self._scan_cb,
                                 qos_profile_sensor_data)

    def _odom_cb(self, msg):
        with self._lock:
            self.odom = msg

    def _scan_cb(self, msg):
        with self._lock:
            self.scan = msg

    def front_clearance(self) -> float:
        with self._lock:
            scan = self.scan
        if scan is None:
            return 0.0
        best = float("inf")
        ang = scan.angle_min
        for r in scan.ranges:
            a = ang
            ang += scan.angle_increment
            if not (scan.range_min <= r <= scan.range_max) or r != r:
                continue
            if abs(norm(a)) <= math.radians(15.0):
                best = min(best, r)
        return best if best < float("inf") else 0.0

    def pose(self):
        with self._lock:
            o = self.odom
        if o is None:
            return None
        return o.pose.pose.position.x, o.pose.pose.position.y, yaw_of(o)

    def stop(self, sec: float = 1.5) -> None:
        end = time.monotonic() + sec
        while time.monotonic() < end and rclpy.ok():
            self.pub.publish(Twist())
            time.sleep(0.05)

    def run_segment(self, label: str, curvature: float, direction: int = 1):
        """走一段，回傳 (弧長, 朝向變化)。curvature 為 0 就是直走。

        direction: +1 前進、−1 倒車。
        ★ 角速度用 `abs(v) * curvature` —— 前進與後退**同號**，
          與本專案其他發布端一致（韌體的 Vz_to_Akm_Angle 會自己從
          R = Vx/Vz 算出方向盤該打哪邊）。
        """
        p0 = self.pose()
        if p0 is None:
            self.get_logger().error("拿不到 odom")
            return None
        x0, y0, yaw0 = p0
        v = SPEED * (1 if direction > 0 else -1)
        self.get_logger().info(
            f"▶ {label}：v={v:+.2f} m/s，κ={curvature:+.3f} /m，{SEG_SEC:.0f} 秒")

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = abs(v) * curvature      # 見 docstring：不隨方向翻號
        end = time.monotonic() + SEG_SEC
        while time.monotonic() < end and rclpy.ok():
            self.pub.publish(msg)               # 韌體逾時 1.0 秒，必須持續送
            time.sleep(0.05)
        self.stop()

        p1 = self.pose()
        if p1 is None:
            return None
        # ★ 這是**弦長**（起訖兩點的直線距離），不是弧長。
        #   換算半徑要用 c = 2R sin(theta/2)，不能直接除以轉角——見 radius_of()。
        chord = math.hypot(p1[0] - x0, p1[1] - y0)
        dyaw = norm(p1[2] - yaw0)
        return chord, dyaw


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true", help="只印計畫，不開車")
    args = ap.parse_args()

    kappa = 1.0 / MIN_RADIUS
    if args.dry:
        print("計畫（每段 %.0f 秒 @ %.2f m/s，約走 %.2f m）：" % (SEG_SEC, SPEED, SEG_SEC * SPEED))
        print("  ① 直走     κ = 0")
        print("  ② 左打滿   κ = %+.3f /m" % +kappa)
        print("  ③ 右打滿   κ = %+.3f /m" % -kappa)
        print("需要前後 1.5 m、左右各 1.0 m 以上的淨空")
        return

    rclpy.init()
    node = SteerAsym()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    for _ in range(100):
        if node.pose() is not None and node.scan is not None:
            break
        time.sleep(0.1)
    if node.pose() is None:
        print("等不到 odom_combined，先確認 run_sensors_cc.sh 起來了")
        sys.exit(1)

    front = node.front_clearance()
    if front < NEED_FRONT:
        print("前方淨空只有 %.2f m（需要 %.2f m），換個空曠的地方" % (front, NEED_FRONT))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    print("前方淨空 %.2f m，開始。側向請自己確認有 1 m 以上。" % front)

    results = {}
    try:
        node.stop(2.0)
        # ★ ①② 必須連續、中間不可搬動車子 —— 它們是決定性的對照組。
        #   8/06 量到「前進 −13.5、倒車 −12.4 度/分（同號）」，但那兩次
        #   中間車子被搬動、楔住、人工救車過，而搬動會改變轉向零位
        #   （停車時 hold_steer_on_stop 不扳正前輪，搬動時輪子是自由的）。
        #   所以那不是乾淨對照，不能用來下結論。這裡連著做才算數。
        SEGMENTS = (
            ("① 前進直走", 0.0, +1),
            ("② 倒車直走", 0.0, -1),
            ("③ 左打滿", +kappa, +1),
            ("④ 右打滿", -kappa, +1),
        )
        for label, k, d in SEGMENTS:
            r = node.run_segment(label, k, d)
            if r is None:
                print("量測中斷")
                break
            results[label] = r
            time.sleep(1.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop(0.5)

    print()
    print("=" * 68)
    print("結果")
    print("=" * 68)
    for label, (arc, dyaw) in results.items():
        deg = math.degrees(dyaw)
        r_txt = "—（直走）"
        if abs(dyaw) > math.radians(1.0):
            r_txt = "%.3f m" % radius_of(arc, dyaw)
        print("  %-10s 走 %.3f m，轉 %+.2f 度   實際半徑 %s" % (label, arc, deg, r_txt))

    print()
    if "① 前進直走" in results:
        arc, dyaw = results["① 前進直走"]
        if arc > 0.05:
            per_m = math.degrees(dyaw) / arc
            delta0 = math.degrees(math.atan(WHEELBASE * dyaw / arc))
            print("零位偏移（只由①決定，不從③④反推）：")
            print("    %+.2f 度/m  ->  等效舵角偏移 %+.2f 度（%s）" % (
                per_m, delta0, "偏左" if dyaw > 0 else "偏右"))
            print("    ★ 若偏轉是舵角造成的，trim_rad_per_m 設 %+.4f 可抵消" % (-dyaw / arc))

    # ═══ 決定性判定：偏轉隨不隨行進方向翻號 ═══
    if "① 前進直走" in results and "② 倒車直走" in results:
        af, df = results["① 前進直走"]
        ab, db = results["② 倒車直走"]
        if af > 0.05 and ab > 0.05:
            pf = math.degrees(df) / af
            pb = math.degrees(db) / ab
            print()
            print("★★ 前進 vs 倒車 —— 這一項決定 trim 的公式對不對")
            print("    前進  %+.2f 度/m   （走 %.2f m 轉 %+.2f 度）" % (pf, af, math.degrees(df)))
            print("    倒車  %+.2f 度/m   （走 %.2f m 轉 %+.2f 度）" % (pb, ab, math.degrees(db)))
            print()
            if pf * pb > 0:
                print("    -> **同號**：偏轉與行進方向無關。")
                print("       但補償公式是  angular.z += trim_rad_per_m * linear.x")
                print("       倒車時 linear.x 變號 -> **補償方向相反、誤差加倍**。")
                print("       ⇒ 這解釋了「前進沒問題、後退有問題」。")
                print()
                print("       兩個修法：")
                print("         (a) 補償改成 trim * abs(linear.x)  —— 治標，但立刻有效")
                print("         (b) 找出真正的成因 —— 時間比例、與方向無關的偏轉")
                print("             不是舵角偏移造成的（舵角偏移必定隨方向翻號）。")
                print("             候選：EKF 融合、陀螺零偏殘留、輪徑不一致。")
            else:
                print("    -> **反號**：符合舵角偏移的模型")
                print("       （阿克曼 ω = v·tan(δ)/L，v 變號則 ω 必然變號）。")
                print("       現行的 trim * linear.x 公式**是對的**，")
                print("       後退的問題要往別處找（例如倒車時的純追蹤前視、或脫困邏輯）。")

    if "③ 左打滿" in results and "④ 右打滿" in results:
        al, dl = results["③ 左打滿"]
        ar, dr = results["④ 右打滿"]
        if abs(dl) > math.radians(1.0) and abs(dr) > math.radians(1.0):
            rl, rr = radius_of(al, dl), radius_of(ar, dr)
            asym = abs(rl - rr) / max(rl, rr) * 100.0
            print()
            print("左右不對稱：R_left %.3f m  vs  R_right %.3f m  ->  %.1f%%" % (rl, rr, asym))
            print("    參考：廠商韌體的角度→PWM 是二次映射，增益差 28%")
            print("          （右滿舵 1052、中位 916、左滿舵 824 PWM/rad）")
            print("          -> 左轉的實際反應比右轉弱約 22%。**這是韌體特性，不是故障。**")
            if min(rl, rr) < 0.76:
                print("    ⚠ 其中一邊 < 0.76 m —— 那一邊的舵角被 Servo_min/max 削掉了")
                print("      （右滿舵的 1014.10 PWM 被 Servo_min=1020 截斷，實際只到 0.7584 m）。")
                print("      這個值代表『硬體極限』而不是『指令對應的角度』。")
            print("    ★ 不要用這兩個值反推零位 —— 被硬限截斷的資料推不出中位（8/06 就錯在這）。")
    print("=" * 68)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
