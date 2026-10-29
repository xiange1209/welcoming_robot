#!/usr/bin/env python3
"""「往前開時平板上車子的位置跟實際不吻合、倒退是準的」——用捲尺把它量成數字（唯讀：不發任何速度指令）。

★ 2026-10-28 新寫（10/29 從 0104 增補包補回 repo）。使用者（10/28 凌晨）：「不吻合」＝平板上車子的位置＋車子沒照畫的路線走。
  10/27 輕載 A/B 的逐拍紀錄：前進時位姿被定位「拉回」>3 cm 有 9 次（最大 14.4 cm），倒車 0 次；
  前進指令 0.15 m/s、倒車 0.10 m/s。但那天用的是 9/29 的舊地圖（掃描吻合度只剩 14~49%）。
  這支把「人用捲尺量到的」跟「三個估計各說走了多少」並排：
      定位（/amcl_pose＝平板顯示的來源）、EKF（/odom_combined＝導航用的里程計）、輪速里程計（/odom）
  每一段：停好按 Enter 記起點 → 你遙控開 → 停好按 Enter 記終點 → 輸入捲尺量到的距離／大約轉的角度。
  每次記錄前會請 AMCL 就地更新一次（/request_nomotion_update），拿到的是「停下來之後」的定位。

用法（導航單元要在跑、已經定位；平板「遙控」頁或鍵盤遙控開車）：
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/pose_drift_test.py
    結果：~/maprun/verify_out/day_MMDD/pose_drift_HHMM.tsv
判讀（印在每段後面）：
    直線：定位與捲尺差 ≤ 5 cm＝準；EKF 準、定位不準＝地圖／雷射匹配問題；EKF 也不準＝里程計問題
    彎道：定位轉角與 EKF 轉角差 > 5 度＝兩者不一致（看哪個接近你目測的角度）
★ 車子是你開的：這支不會動車。鍵盤遙控直接發 /cmd_vel，平板急停停不了，要在鍵盤那個視窗 Ctrl-C。
"""
import math
import os
import sys
import threading
import time

SEGMENTS = [
    ("前進直線", "車頭對準前方的膠帶線，**前進直線**約 2 m（不要轉方向），停好"),
    ("倒車直線", "從現在的位置**倒車直線**約 2 m（不要轉方向），停好"),
    ("前進右彎打滿", "**前進、方向打滿往右**，大約轉 90 度就停（要約 1.5×1.5 m 空地）"),
    ("倒車右彎打滿", "**倒車、方向打滿**，大約轉 90 度就停"),
    ("前進左彎打滿", "**前進、方向打滿往左**，大約轉 90 度就停"),
]
# repo 的 maprun/env.sh（這支在 maprun/tools_1020/ 底下）；資料照舊寫在 ~/maprun/verify_out（不進 git）
ENV_SH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "env.sh")


def ensure_ros_env():
    """DDS 設定要跟車上節點一樣（env.sh）。沒 source 過就自己重新以 env.sh 執行一次。"""
    if os.environ.get("CYCLONEDDS_URI") or os.environ.get("_POSE_DRIFT_REEXEC"):
        return
    os.environ["_POSE_DRIFT_REEXEC"] = "1"
    os.execvp("bash", ["bash", "-c", f'source "{ENV_SH}" >/dev/null 2>&1; exec python3 "$0" "$@"',
                       os.path.abspath(__file__)] + sys.argv[1:])


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class Watcher:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from std_srvs.srv import Empty
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("pose_drift_test")
        self.lock = threading.Lock()
        self.last = {"amcl": None, "ekf": None, "wheel": None}
        self.amcl_t = 0.0
        amcl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl, amcl_qos)
        self.node.create_subscription(Odometry, "/odom_combined", lambda m: self._odom("ekf", m), qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "/odom", lambda m: self._odom("wheel", m), qos_profile_sensor_data)
        self.nomotion = self.node.create_client(Empty, "/request_nomotion_update")
        self.Empty = Empty
        # 自己管 executor：結束時先停 executor、等執行緒收掉，再拆節點（直接 shutdown 會讓 C++ 端 terminate）
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._th = threading.Thread(target=self._spin, daemon=True)
        self._th.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:  # noqa: BLE001 —— 結束時 executor.shutdown 會讓 spin 返回／丟例外
            pass

    def _amcl(self, m):
        p = m.pose.pose
        with self.lock:
            self.last["amcl"] = (p.position.x, p.position.y, math.degrees(yaw_of(p.orientation)))
            self.amcl_t = time.monotonic()

    def _odom(self, key, m):
        p = m.pose.pose
        with self.lock:
            self.last[key] = (p.position.x, p.position.y, math.degrees(yaw_of(p.orientation)))

    def snapshot(self):
        """請 AMCL 就地更新一次，等新的 /amcl_pose（最多 4 秒），回三個估計。"""
        t_req = time.monotonic()
        if self.nomotion.wait_for_service(timeout_sec=2.0):
            self.nomotion.call_async(self.Empty.Request())
        t_end = time.monotonic() + 4.0
        while time.monotonic() < t_end:
            with self.lock:
                if self.amcl_t > t_req:
                    break
            time.sleep(0.1)
        time.sleep(0.3)
        with self.lock:
            fresh = self.amcl_t > t_req
            return dict(self.last), fresh

    def close(self):
        try:
            self.executor.shutdown(timeout_sec=2.0)
            self._th.join(timeout=3.0)
            self.node.destroy_node()
            self.rclpy.try_shutdown()
        except Exception:  # noqa: BLE001
            pass


def delta(a, b):
    if not a or not b:
        return None
    return math.hypot(b[0] - a[0], b[1] - a[1]), wrap_deg(b[2] - a[2])


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return "q"


def main():
    ensure_ros_env()
    w = Watcher()
    day = time.strftime("%m%d")
    out_dir = os.path.expanduser(f"~/maprun/verify_out/day_{day}")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f"pose_drift_{time.strftime('%H%M')}.tsv")
    print(__doc__.split("用法")[0].strip())
    print(f"\n結果會寫到 {out}\n")
    time.sleep(2.0)
    with w.lock:
        have = {k: v is not None for k, v in w.last.items()}
    if not have["ekf"] or not have["wheel"]:
        print(f"⚠ 還沒收到里程計（EKF {'有' if have['ekf'] else '沒有'}、輪速 {'有' if have['wheel'] else '沒有'}）——底盤／感測器單元有開嗎？")
    rows = []
    try:
        for name, how in SEGMENTS:
            print(f"\n════ {name} ════")
            a = ask("車子停穩、在膠帶起點了嗎？按 Enter 記起點（s＝跳過這段、q＝結束）：")
            if a == "q":
                break
            if a == "s":
                continue
            s0, f0 = w.snapshot()
            if not s0["amcl"]:
                print("✗ 收不到 /amcl_pose——導航單元（定位）有開、而且定位過了嗎？這段跳過。")
                continue
            print(f"  起點已記（定位{'已就地更新' if f0 else '沒有更新到，用上一筆'}）。現在：{how}。")
            a = ask("停好之後按 Enter 記終點（q＝結束）：")
            if a == "q":
                break
            s1, f1 = w.snapshot()
            tape = ask("捲尺量到的直線距離幾 cm？（起點膠帶→車子同一個點；不量就 Enter）：")
            ang = ask("實際大約轉了幾度？（直線段打 0；不知道就 Enter）：")
            tape_m = float(tape) / 100.0 if tape.replace(".", "", 1).isdigit() else None
            ang_d = float(ang) if ang.replace(".", "", 1).replace("-", "", 1).isdigit() else None
            res = {k: delta(s0[k], s1[k]) for k in ("amcl", "ekf", "wheel")}
            print(f"  {'':8}{'位移 cm':>9}{'轉角 度':>9}{'與捲尺差 cm':>12}")
            for k, label in (("amcl", "定位(平板)"), ("ekf", "EKF"), ("wheel", "輪速")):
                r = res[k]
                if r is None:
                    print(f"  {label:8}  （沒有資料）")
                    continue
                diff = f"{(r[0] - tape_m) * 100:+.1f}" if tape_m is not None else "-"
                print(f"  {label:8}{r[0] * 100:9.1f}{r[1]:9.1f}{diff:>12}")
            verdict = []
            ra, re_ = res["amcl"], res["ekf"]
            if tape_m is not None and ra:
                verdict.append(f"定位 vs 捲尺 {(ra[0] - tape_m) * 100:+.1f} cm" + ("（準）" if abs(ra[0] - tape_m) <= 0.05 else "（★ 偏差大）"))
            if tape_m is not None and re_:
                verdict.append(f"EKF vs 捲尺 {(re_[0] - tape_m) * 100:+.1f} cm")
            if ra and re_:
                dyaw = wrap_deg(ra[1] - re_[1])
                verdict.append(f"定位與 EKF 轉角差 {dyaw:+.1f} 度" + ("（★ 不一致）" if abs(dyaw) > 5 else ""))
            if ang_d is not None and ra:
                verdict.append(f"定位轉角 vs 目測 {ra[1] - ang_d:+.1f} 度")
            if ra and abs(ra[1]) > 20:
                chord = ra[0]
                radius = chord / (2 * math.sin(math.radians(abs(ra[1])) / 2))
                verdict.append(f"由定位反推轉彎半徑 ≈ {radius:.2f} m")
            print("  → " + "；".join(verdict))
            rows.append([time.strftime("%H:%M:%S"), name, tape or "", ang or ""]
                        + [f"{res[k][0] * 100:.1f}\t{res[k][1]:.1f}" if res[k] else "\t" for k in ("amcl", "ekf", "wheel")]
                        + ["是" if (f0 and f1) else "否", "；".join(verdict)])
    except KeyboardInterrupt:
        print("\n（中斷）")
    finally:
        if rows:
            new = not os.path.exists(out)
            with open(out, "a", encoding="utf-8") as fh:
                if new:
                    fh.write("時間\t段\t捲尺cm\t目測度\t定位位移cm\t定位轉角\tEKF位移cm\tEKF轉角\t輪速位移cm\t輪速轉角\t定位有就地更新\t判讀\n")
                for r in rows:
                    fh.write("\t".join(str(x) for x in r) + "\n")
            print(f"\n已存：{out}（{len(rows)} 段）")
        w.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
