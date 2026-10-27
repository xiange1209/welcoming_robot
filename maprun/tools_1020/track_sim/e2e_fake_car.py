#!/usr/bin/env python3
"""端到端循跡測試（★ 2026-10-24 車上 agent）：**真的 path_teach_cc 節點** ＋ 離線 planner_server ＋ 假車。

假車：
  - 車子模型＝track_sim.Vehicle（8/29 實測：左 1.030／右 1.183 m、底盤偏移 +0.235 rad/m、trim 殘差、舵機延遲、
    死區 0.085 m/s），指令 1 秒沒更新就停（韌體 command_lost_count 行為）
  - 訂 cmd_vel_smoothed（正常路線：實車會再經過 steering_trim，模型裡含 trim）與 cmd_vel（脫困旁路：實車不經過 trim，
    這裡把 trim 扣回去）
  - /scan：從真地圖（~/.smartnav/map_database）光線投射，雷射在 base_footprint 前 0.089 m（path_teach 的 laser_x_offset_m）
  - TF odom_combined -> base_footprint（可加 AMCL 式誤差：每走 10 cm 換一次）＋ base_footprint -> laser；current_map
流程：每一對地點 → 假車瞬移到起點 → /plan_taught_path（從車子目前位置）→ /follow_taught_path → 記錄。
path_teach 由這支用暫存 HOME 開（路徑庫、逐拍 CSV 都在暫存 HOME 底下），**不碰真的路徑庫、不動車**。

離線 planner_server 由這支自己開（plan_bench.launch.py base_tf:=false）：一定要在假車開始發 TF **之後**才開，
不然全域成本地圖啟動時等不到 base_footprint，之後一直「Costmap timed out waiting for update」（10/24 踩過）。
用法（先 source ~/maprun/env.sh）：
  python3 e2e_fake_car.py --config new|old [--pairs 起點:門口,廁所:門口] [--bias 0.03] [--amcl 0.02] [--out DIR]
    new＝今天的改動全開（規劃半徑 1.40＋多候選擇優＋前視內插＋線段偏離＋積分）
    old＝全關（規劃半徑 0.95、只規劃一次、舊純追蹤）
"""
import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose, TransformStamped, Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from smartnav_msgs.action import FollowTaughtPath
from smartnav_msgs.srv import PlanTaughtPath

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import track_sim as ts  # noqa: E402

LASER_X = 0.089
NODE_EXE = "/home/user/welcoming_robot_ws/install/smartnav_navigation_cc/lib/smartnav_navigation_cc/path_teach_cc"
BENCH_LAUNCH = "/home/user/maprun/tools_1020/plan_bench/plan_bench.launch.py"
OLD_PARAMS = ["lookahead_interpolate:=false", "xte_segment:=false", "xte_integral:=false",
              "plan_candidates:=false", "plan_fallback_radii:=[0.0]"]


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    assert tokens[0] == b"P5", "只支援 P5 PGM"
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data[i + 1:i + 1 + w * h], dtype=np.uint8).reshape(h, w)


class Raycaster:
    def __init__(self, yaml_path):
        meta = yaml.safe_load(open(yaml_path))
        img = read_pgm(os.path.join(os.path.dirname(yaml_path), meta["image"])).astype(float)
        p = img / 255.0 if meta.get("negate", 0) else (255.0 - img) / 255.0
        self.occ = p > float(meta["occupied_thresh"])
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        self.h, self.w = self.occ.shape
        self.r = np.arange(0.05, 8.0, self.res * 0.5)
        self.n = 360
        self.angles = -math.pi + np.arange(self.n) * (2 * math.pi / self.n)

    def ranges(self, lx, ly, lyaw):
        a = lyaw + self.angles[:, None]
        cx = ((lx + np.cos(a) * self.r[None, :] - self.ox) / self.res).astype(int)
        cy = ((ly + np.sin(a) * self.r[None, :] - self.oy) / self.res).astype(int)
        ok = (cx >= 0) & (cx < self.w) & (cy >= 0) & (cy < self.h)
        hit = np.zeros(cx.shape, dtype=bool)
        hit[ok] = self.occ[self.h - 1 - cy[ok], cx[ok]]
        return np.where(hit.any(axis=1), self.r[hit.argmax(axis=1)], np.inf)


class FakeCar(Node):
    def __init__(self, map_yaml, map_id, bias, amcl_xy, amcl_yaw):
        super().__init__("e2e_fake_car")
        self.rc = Raycaster(map_yaml)
        self.lock = threading.Lock()
        self.bias, self.amcl = bias, (amcl_xy, amcl_yaw)
        self.reset(0.0, 0.0, 0.0)
        self.tfb = TransformBroadcaster(self)
        st = TransformStamped()
        st.header.frame_id, st.child_frame_id = "base_footprint", "laser"
        st.transform.translation.x = LASER_X
        st.transform.rotation.w = 1.0
        self.stb = StaticTransformBroadcaster(self)
        self.stb.sendTransform(st)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(String, "current_map", latched)
        self.map_pub.publish(String(data=map_id))
        self.scan_pub = self.create_publisher(LaserScan, "scan", 5)
        self.create_subscription(Twist, "cmd_vel_smoothed", lambda m: self._cmd(m, True), 10)
        self.create_subscription(Twist, "cmd_vel", lambda m: self._cmd(m, False), 10)
        self.create_timer(0.01, self._physics)
        self.create_timer(0.02, self._tf)
        self.create_timer(0.1, self._scan)
        self.cmd_n = 0

    def reset(self, x, y, yaw):
        with self.lock:
            self.veh = ts.Vehicle(x, y, yaw, bias_res=self.bias)
            self.loc = ts.Localizer(__import__("random").Random(7), amcl_xy=self.amcl[0], amcl_yaw_deg=self.amcl[1])
            self.last_cmd_t = time.monotonic()
            self.seen = (x, y, yaw)

    def _cmd(self, m, trimmed):
        v, w = m.linear.x, m.angular.z
        if not trimmed:
            w -= self.veh.trim * v        # 旁路不經過 steering_trim：把模型裡內建的 trim 扣回去
        with self.lock:
            self.veh.set_cmd(v, w)
            self.last_cmd_t = time.monotonic()
            self.cmd_n += 1

    def _physics(self):
        with self.lock:
            if time.monotonic() - self.last_cmd_t > 1.0 and self.veh.applied != (0.0, 0.0):
                self.veh.set_cmd(0.0, 0.0)     # 韌體：1 秒沒指令就停
                self.veh.set_cmd(0.0, 0.0)
            self.veh.integrate(0.01)

    def _tf(self):
        with self.lock:
            sx, sy, syaw = self.loc.see(self.veh)
            self.seen = (sx, sy, syaw)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "odom_combined", "base_footprint"
        t.transform.translation.x, t.transform.translation.y = sx, sy
        t.transform.rotation.z, t.transform.rotation.w = math.sin(syaw / 2), math.cos(syaw / 2)
        self.tfb.sendTransform(t)

    def _scan(self):
        with self.lock:
            x, y, yaw = self.veh.x, self.veh.y, self.veh.yaw
        rng = self.rc.ranges(x + LASER_X * math.cos(yaw), y + LASER_X * math.sin(yaw), yaw)
        s = LaserScan()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = "laser"
        s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / self.rc.n
        s.angle_max = s.angle_min + s.angle_increment * (self.rc.n - 1)
        s.time_increment, s.scan_time = 0.0, 0.1
        s.range_min, s.range_max = 0.05, 8.0
        s.ranges = [float(r) for r in rng]
        self.scan_pub.publish(s)

    def true_pose(self):
        with self.lock:
            return self.veh.x, self.veh.y, self.veh.yaw


def polyline_dist(pts, x, y):
    best = float("inf")
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / L2))
        best = min(best, (x - x0 - t * dx) ** 2 + (y - y0 - t * dy) ** 2)
    return math.sqrt(best)


def wait(fut, timeout):
    t_end = time.monotonic() + timeout
    while not fut.done() and time.monotonic() < t_end:
        time.sleep(0.05)
    return fut.result() if fut.done() else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", choices=("new", "old"), default="new")
    ap.add_argument("--pairs", default="起點:門口,廁所:門口,廁所:貴賓室,門口:櫃台,大廳:起點,櫃台:廁所,大廳:貴賓室,廁所:起點")
    ap.add_argument("--bias", type=float, default=0.03)
    ap.add_argument("--amcl", type=float, default=0.02)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    out = a.out or os.path.expanduser(f"~/maprun/verify_out/e2e_{a.config}_{time.strftime('%H%M')}")
    os.makedirs(out, exist_ok=True)
    real_home = Path.home()
    map_id = json.loads((real_home / ".smartnav" / "nav_state.json").read_text())["last_active_map"]
    db = json.loads((real_home / ".smartnav" / "waypoint_database" / "waypoints.json").read_text())["waypoints_db"]
    wps = {v["name"]: v["pose"] for v in db.values() if v.get("map_id") == map_id}
    fake_home = os.path.join(out, "fake_home")
    os.makedirs(os.path.join(fake_home, ".smartnav", "path_database"), exist_ok=True)

    rclpy.init()
    car = FakeCar(str(real_home / ".smartnav" / "map_database" / "data" / f"{map_id}.yaml"), map_id,
                  a.bias, a.amcl, a.amcl * 50.0)
    drv = Node("e2e_driver")
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(car)
    ex.add_node(drv)
    threading.Thread(target=ex.spin, daemon=True).start()

    # 假車已經在發 TF → 這時才開離線 planner_server（見檔頭）
    time.sleep(2.0)
    bench_log = open(os.path.join(out, "plan_bench.log"), "w")
    bench = subprocess.Popen(["ros2", "launch", BENCH_LAUNCH, "base_tf:=false"], stdout=bench_log,
                             stderr=subprocess.STDOUT, start_new_session=True)
    setp = drv.create_client(SetParameters, "/planner_server/set_parameters")
    assert setp.wait_for_service(timeout_sec=90), "planner_server 沒起來（看 plan_bench.log）"
    time.sleep(8.0)                                     # 等 lifecycle 啟動完、成本地圖收到地圖
    req = SetParameters.Request()
    req.parameters = [Parameter(name="GridBased.minimum_turning_radius", value=ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE, double_value=1.40 if a.config == "new" else 0.95))]
    r = wait(setp.call_async(req), 20)
    print(f"規劃半徑 → {req.parameters[0].value.double_value}：{'OK' if r and r.results[0].successful else '失敗'}", flush=True)

    # 開 path_teach（暫存 HOME；old 帶全關參數）
    env = dict(os.environ, HOME=fake_home, ROS_LOG_DIR=os.path.join(fake_home, "ros_log"))
    cmd = [NODE_EXE, "--ros-args", "-r", "__node:=path_teach_cc_node", "-p", "cmd_topic:=cmd_vel_smoothed",
           "-p", "scan_topic:=scan"]
    if a.config == "old":
        for kv in OLD_PARAMS:
            cmd += ["-p", kv]
    logf = open(os.path.join(out, "path_teach.log"), "w")
    proc = subprocess.Popen(cmd, env=env, stdout=logf, stderr=subprocess.STDOUT)
    plan_cli = drv.create_client(PlanTaughtPath, "plan_taught_path")
    follow_cli = ActionClient(drv, FollowTaughtPath, "follow_taught_path")
    assert plan_cli.wait_for_service(timeout_sec=60) and follow_cli.wait_for_server(timeout_sec=60), "path_teach 沒起來"
    time.sleep(3.0)

    results = []
    try:
        for pair in a.pairs.split(","):
            sn, gn = pair.split(":")
            sp, gp = wps[sn], wps[gn]
            syaw = 2 * math.atan2(sp["orientation"]["z"], sp["orientation"]["w"])
            car.reset(sp["position"]["x"], sp["position"]["y"], syaw)
            time.sleep(2.5)
            g = Pose()
            g.position.x, g.position.y = gp["position"]["x"], gp["position"]["y"]
            g.orientation.z, g.orientation.w = gp["orientation"]["z"], gp["orientation"]["w"]
            preq = PlanTaughtPath.Request(name=f"_e2e_{sn}_{gn}", start_from_robot=True, waypoints=[g])
            t0 = time.monotonic()
            pres = wait(plan_cli.call_async(preq), 90)
            t_plan = time.monotonic() - t0
            row = dict(起點=sn, 終點=gn, 規劃秒=round(t_plan, 1))
            if not pres or not pres.success:
                row.update(成功=False, 訊息=f"規劃失敗：{pres.message if pres else '逾時'}")
                results.append(row)
                print(row, flush=True)
                continue
            dbp = json.load(open(os.path.join(fake_home, ".smartnav", "path_database", "paths.json")))["paths"]
            ppts = [(q["x"], q["y"]) for q in dbp[pres.path_id]["points"]]
            row.update(點數=pres.num_points, 長=round(pres.length_m, 2), 折返=pres.num_cusps)
            fb = {"xte": [], "states": {}}

            def on_fb(m, fb=fb):
                f = m.feedback
                fb["xte"].append(f.cross_track_error_m)
                fb["states"][f.state] = fb["states"].get(f.state, 0) + 1

            gfut = follow_cli.send_goal_async(FollowTaughtPath.Goal(path_id=pres.path_id, reverse=False, speed_scale=0.0),
                                              feedback_callback=on_fb)
            gh = wait(gfut, 20)
            if gh is None or not gh.accepted:
                row.update(成功=False, 訊息="重播請求被拒")
                results.append(row)
                continue
            rfut = gh.get_result_async()
            t0 = time.monotonic()
            t_max = 60.0 + pres.length_m / 0.04
            true_e = []
            while not rfut.done() and time.monotonic() - t0 < t_max:
                x, y, _ = car.true_pose()
                true_e.append(polyline_dist(ppts, x, y))
                time.sleep(0.1)
            if not rfut.done():
                gh.cancel_goal_async()
                time.sleep(2.0)
            res = rfut.result().result if rfut.done() else None
            e = sorted(true_e) or [0.0]
            row.update(成功=bool(res and res.success), 訊息=(res.message if res else f"逾時 {t_max:.0f} s（已取消）")[:120],
                       秒=round(time.monotonic() - t0, 1), 終點誤差=round(res.final_error_m, 3) if res else None,
                       真實偏離中位cm=round(100 * e[len(e) // 2], 1), 真實偏離p95cm=round(100 * e[int(0.95 * (len(e) - 1))], 1),
                       真實偏離最大cm=round(100 * e[-1], 1),
                       節點顯示xte最大cm=round(100 * max(fb["xte"]), 1) if fb["xte"] else None, 狀態=fb["states"])
            results.append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
            car.reset(*car.true_pose())     # 停車
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.terminate()
        logf.close()
        # 收掉自己開的 planner_server：對它的行程群組（start_new_session 開的）送 SIGINT，ros2 launch 會收子節點
        try:
            os.killpg(bench.pid, signal.SIGINT)
            bench.wait(timeout=25)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(bench.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        bench_log.close()
        ok = sum(r.get("成功", False) for r in results)
        summ = dict(組態=a.config, 成功=f"{ok}/{len(results)}", bias=a.bias, amcl=a.amcl, 結果=results)
        json.dump(summ, open(os.path.join(out, "e2e.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        print(f"{a.config}：成功 {ok}/{len(results)}；輸出 {out}", flush=True)
        ex.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
