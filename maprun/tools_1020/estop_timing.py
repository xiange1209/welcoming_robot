#!/usr/bin/env python3
"""平板急停計時（交接文件 §5-2：重播中按平板急停，車要在 ≤1.3 s 內停下）。

用法（車上；腳本自己會 source 不到環境，請先 source ~/welcoming_robot_ws/src/smartnav_ws/maprun/env.sh）：
    source ~/welcoming_robot_ws/src/smartnav_ws/maprun/env.sh
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/estop_timing.py            # 一直等；Ctrl-C 結束並印總表
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/estop_timing.py --limit 1.3
然後：讓車在重播（或脫困）中，去平板按「急停」。每按一次印一行結果。

t0 是什麼：HMI 處理 /api/estop 時，會先送零速、接著發一則 /unwedge_abort（std_msgs/Empty）。
我們收到這則的時刻當 t0——它發生在「HMI 伺服器收到請求之後」數毫秒內，所以
★ 量到的是「HMI 收到請求 → 車停下」，**不含**平板按鈕到 Pi 的 Wi-Fi 延遲（通常 < 50 ms）。

每次急停量三個時間（都相對 t0）：
    最後非零指令   t0 之後最後一則 |linear.x|>1e-4 或 |angular.z|>1e-4 的 /cmd_vel
                   （None＝按下之後沒有再看到非零指令）
    第一則零速     t0 之後第一則全零的 /cmd_vel
    車速<0.02 穩定 /odom 線速度連續 0.5 s 低於 0.02 m/s 的起點  ← ★ 過關看這個
前提：t0 之前 3 秒內車速曾超過 0.05 m/s（確實在動），否則標「無效：按下時車沒在動」。
過關：車速<0.02 穩定 ≤ --limit（預設 1.3 s；舊基準 0.768 s 收到、1.303 s 完成）。

★ 韌體的指令逾時是 1.0 s：如果急停只是「讓指令停止」而沒有送零速，車會照最後一則指令再滑 1 秒——
  所以「最後非零指令」與「車速穩定」之間差多少，就是在看這件事。
★ 這支是 2026-10-20 車上新寫的量測腳本，只用合成資料（隔離的 ROS_DOMAIN_ID）驗證過邏輯，沒有在實車上驗證過。
"""
import argparse
import collections
import os
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty

V_STOP = 0.02        # m/s，視為停住
HOLD_S = 0.5         # 低於 V_STOP 要維持多久才算穩定
MOVING_V = 0.05      # t0 前要超過它才算「確實在動」
LOOKBACK_S = 3.0
GIVE_UP_S = 8.0


class EstopTiming(Node):
    def __init__(self, limit: float, out_tsv: str):
        super().__init__("estop_timing_cc")
        self.limit = limit
        self.out_tsv = out_tsv
        self.odom = collections.deque()    # (t, |v|)
        self.cmd = collections.deque()     # (t, vx, wz)
        self.pending = []
        self.done = []
        self.create_subscription(Empty, "/unwedge_abort", self._on_abort, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self.create_timer(0.05, self._tick)
        if not os.path.exists(out_tsv):
            with open(out_tsv, "w", encoding="utf-8") as fh:
                fh.write("時間\t序號\t按下前最大車速\t最後非零指令\t第一則零速\t車速穩定<0.02\t判定\n")
        print(f"等待急停……（/unwedge_abort、/cmd_vel、/odom 都已訂閱；過關門檻 ≤{limit:.2f} s）", flush=True)
        print("現在讓車在重播中，去平板按「急停」。Ctrl-C 結束。", flush=True)

    # ── 收訊 ────────────────────────────────────────────────
    def _prune(self, now):
        while self.odom and now - self.odom[0][0] > 20:
            self.odom.popleft()
        while self.cmd and now - self.cmd[0][0] > 20:
            self.cmd.popleft()

    def _on_odom(self, m: Odometry):
        now = time.time()
        self.odom.append((now, abs(float(m.twist.twist.linear.x))))
        self._prune(now)

    def _on_cmd(self, m: Twist):
        now = time.time()
        self.cmd.append((now, float(m.linear.x), float(m.angular.z)))

    def _on_abort(self, _m):
        t0 = time.time()
        before = [v for t, v in self.odom if t0 - LOOKBACK_S <= t <= t0]
        vmax = max(before) if before else None
        n = len(self.done) + len(self.pending) + 1
        # ★ 2026-10-27：記下按下當下的負載。上機第一天 load 25+ 時量到 4.55 s（零速指令 +0.10 s 就送出）——
        #   滿載時底盤驅動解析串口回報會落後，/odom 的「車速」可能比實際慢好幾秒，判讀時要一起看。
        self.pending.append({"n": n, "t0": t0, "vmax": vmax, "load1": os.getloadavg()[0]})
        print(f"[#{n}] 收到急停（t0）；按下前 {LOOKBACK_S:.0f} 秒內最大車速 "
              f"{'無 /odom 資料' if vmax is None else f'{vmax:.3f} m/s'}。量測中……", flush=True)

    # ── 量測 ────────────────────────────────────────────────
    @staticmethod
    def _is_zero(vx, wz):
        return abs(vx) < 1e-4 and abs(wz) < 1e-4

    def _stable_start(self, t0):
        """t0 之後第一個「之後 HOLD_S 秒內 /odom 速度全部 < V_STOP」的起點；還沒有就回 (None, None)。"""
        pts = [(t, v) for t, v in self.odom if t >= t0]
        for i, (t, v) in enumerate(pts):
            if v >= V_STOP:
                continue
            window = [(tt, vv) for tt, vv in pts[i:] if tt - t <= HOLD_S]
            if len(window) >= 3 and window[-1][0] - t >= HOLD_S - 0.15 and all(vv < V_STOP for _, vv in window):
                return t - t0, window[-1][0]
        return None, None

    def _tick(self):
        now = time.time()
        for p in list(self.pending):
            t0 = p["t0"]
            stable, _ = self._stable_start(t0)
            timed_out = now - t0 > GIVE_UP_S
            if stable is None and not timed_out:
                continue
            after = [(t, vx, wz) for t, vx, wz in self.cmd if t >= t0]
            nz = [t for t, vx, wz in after if not self._is_zero(vx, wz)]
            z = [t for t, vx, wz in after if self._is_zero(vx, wz)]
            last_nz = (max(nz) - t0) if nz else None
            first_z = (min(z) - t0) if z else None
            vmax = p["vmax"]
            if vmax is None or vmax < MOVING_V:
                verdict = "無效：按下時車沒在動"
            elif stable is None:
                verdict = f"FAIL：{GIVE_UP_S:.0f} 秒內車速沒穩定降到 {V_STOP} 以下"
            elif stable <= self.limit:
                verdict = f"PASS（≤{self.limit:.2f} s）"
            else:
                verdict = f"FAIL（> {self.limit:.2f} s）"
            if p.get("load1", 0.0) >= 8.0:   # 四核：load 8 以上＝嚴重滿載
                verdict += f"｜⚠ 按下時 load {p['load1']:.1f}（滿載，/odom 可能落後，數字偏大）"
            f = lambda x: "無" if x is None else f"+{x:.2f}s"
            line = (f"[#{p['n']}] 按下前最大車速 {('?' if vmax is None else f'{vmax:.3f}')} m/s | "
                    f"最後非零指令 {f(last_nz)} | 第一則零速 {f(first_z)} | "
                    f"車速<{V_STOP} 穩定 {f(stable)}  →  {verdict}")
            print(line, flush=True)
            with open(self.out_tsv, "a", encoding="utf-8") as fh:
                fh.write("\t".join([time.strftime("%Y-%m-%dT%H:%M:%S"), str(p["n"]),
                                    "" if vmax is None else f"{vmax:.3f}", f(last_nz), f(first_z), f(stable), verdict]) + "\n")
            trip = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "trip_mark.sh")
            if os.path.exists(trip):
                try:
                    subprocess.run([trip, f"T4 急停計時 #{p['n']}", line], capture_output=True, timeout=5)
                except Exception:  # noqa: BLE001
                    pass
            self.pending.remove(p)
            self.done.append((p, verdict))


def main():
    ap = argparse.ArgumentParser(description="平板急停計時")
    ap.add_argument("--limit", type=float, default=1.3, help="過關門檻（秒），預設 1.3")
    ap.add_argument("--out", default=os.path.expanduser(
        f"~/welcoming_robot_ws/src/smartnav_ws/maprun/verify_out/estop_timing_{time.strftime('%Y%m%d')}.tsv"))
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    rclpy.init()
    node = EstopTiming(args.limit, args.out)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl-C 或被終止都是正常結束
        pass
    finally:
        print("\n── 總表 ──", flush=True)
        if not node.done:
            print("（沒有完成任何一次量測）", flush=True)
        for p, verdict in node.done:
            print(f"  #{p['n']}  {verdict}", flush=True)
        print(f"逐筆記錄：{args.out}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
