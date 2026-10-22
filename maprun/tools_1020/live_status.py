#!/usr/bin/env python3
"""實時狀態一頁看完（唯讀；不發任何指令、不改任何東西）。

用法：
    source ~/maprun/env.sh
    python3 ~/maprun/tools_1020/live_status.py            # 預設量 6 秒
    python3 ~/maprun/tools_1020/live_status.py --secs 10

看什麼：電池／溫度／降頻旗標／CPU／記憶體｜平板「系統開關」各單元｜ROS 節點與重複節點｜
nav2 與地圖來源的生命週期狀態｜關鍵話題實際頻率｜TF（map／odom→base_footprint 的新舊）｜
ASR 的 input overflow（近 1／5 分鐘）與最近辨識｜LLM 最近幾輪（來源、工具、耗時）與閒置時間｜導航 log 近 60 秒的警告。

★ 2026-10-20 車上新寫，只在實車的唯讀情境跑過；話題頻率是「這幾秒內收到幾則」，不是長期平均。
★ 它登入 HMI 管理端 API 只為了讀 /api/system/status；密碼從 systemd 服務環境變數取、不印出。
"""
import argparse
import collections
import datetime as dt
import glob
import json
import os
import re
import subprocess
import sys
import time
import urllib.error
import urllib.request

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener

LOGS = os.path.expanduser("~/.smartnav/logs/system_control")
EPOCH = re.compile(r"\[(\d{10}\.\d+)\]")
HM = lambda t: dt.datetime.fromtimestamp(t).strftime("%H:%M:%S") if t else "?"


def hdr(s):
    print(f"\n── {s} " + "─" * max(0, 62 - len(s)))


def newest(pattern):
    fs = glob.glob(os.path.join(LOGS, pattern))
    return max(fs, key=os.path.getmtime) if fs else None


def ts_of(line):
    m = EPOCH.search(line)
    return float(m.group(1)) if m else None


# ── /proc 取樣 ────────────────────────────────────────────────
def proc_snapshot():
    d = {}
    for e in os.scandir("/proc"):
        if not e.name.isdigit():
            continue
        try:
            s = open(f"/proc/{e.name}/stat").read()
            rp = s.rindex(")")
            comm = s[s.index("(") + 1:rp]
            f = s[rp + 2:].split()
            d[int(e.name)] = (comm, int(f[11]) + int(f[12]))
        except Exception:  # noqa: BLE001
            pass
    return d


def cpu_total():
    v = list(map(int, open("/proc/stat").readline().split()[1:9]))
    return sum(v), v[3] + v[4]


class Probe(Node):
    """同時訂一批話題、數這幾秒收到幾則；順便拿 TF。"""

    def __init__(self):
        super().__init__("live_status_cc")
        self.n = collections.Counter()
        self.last = {}
        self.volt = None
        self.amcl = None
        self.amcl_rx = None
        sensor = qos_profile_sensor_data
        for name, typ in (("/scan", LaserScan), ("/scan_slam", LaserScan), ("/odom", Odometry),
                          ("/odom_combined", Odometry), ("/imu/data_unbiased", Imu)):
            self.create_subscription(typ, name, self._mk(name), sensor)
        for name in ("/cmd_vel_smoothed", "/cmd_vel_trimmed", "/cmd_vel_prefloor", "/cmd_vel"):
            self.create_subscription(Twist, name, self._mk_twist(name), sensor)
        self.create_subscription(Float32, "/PowerVoltage", self._volt, sensor)
        self.create_subscription(String, "/user_text", self._mk("/user_text"), 10)
        self.create_subscription(String, "/llm_response", self._mk("/llm_response"), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl, latched)
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)

    def _mk(self, name):
        def cb(_m):
            self.n[name] += 1
        return cb

    def _mk_twist(self, name):
        def cb(m):
            self.n[name] += 1
            self.last[name] = (m.linear.x, m.angular.z)
            if abs(m.linear.x) > 1e-4 or abs(m.angular.z) > 1e-4:
                self.n[name + ":nonzero"] += 1
        return cb

    def _volt(self, m):
        self.volt = float(m.data)

    def _amcl(self, m):
        self.amcl = m
        self.amcl_rx = time.time()
        self.n["/amcl_pose"] += 1


def get_units():
    try:
        env = subprocess.run(["systemctl", "show", "smartnav-hmi", "-p", "Environment", "--value"],
                             capture_output=True, text=True, timeout=5).stdout.split()
        kv = dict(x.split("=", 1) for x in env if "=" in x)
        pw = kv.get("SMARTNAV_ADMIN_PASSWORD", "")

        def call(path, body=None, token=None, method="GET"):
            h = {"Content-Type": "application/json"}
            if token:
                h["Authorization"] = "Bearer " + token
            req = urllib.request.Request("http://127.0.0.1:8080" + path, method=method,
                                         data=json.dumps(body).encode() if body is not None else None, headers=h)
            with urllib.request.urlopen(req, timeout=8) as r:
                return json.loads(r.read().decode() or "{}")
        tok = call("/api/login", {"username": "admin", "password": pw}, method="POST")["token"]
        units = call("/api/system/status", token=tok).get("units", [])
        try:
            call("/api/logout", {}, token=tok, method="POST")
        except Exception:  # noqa: BLE001
            pass
        return units
    except Exception as e:  # noqa: BLE001
        return f"讀不到（{e.__class__.__name__}）"


def lifecycle(node, names, timeout=1.2):
    out = {}
    for nm in names:
        cli = node.create_client(GetState, f"/{nm}/get_state")
        if not cli.wait_for_service(timeout_sec=0.25):
            out[nm] = None
            continue
        fut = cli.call_async(GetState.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
        out[nm] = fut.result().current_state.label if fut.done() and fut.result() else "逾時"
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=6.0)
    a = ap.parse_args()
    now0 = time.time()
    print(f"實時狀態   {dt.datetime.fromtimestamp(now0).strftime('%Y-%m-%d %H:%M:%S')}   "
          f"開機 {subprocess.run(['uptime', '-s'], capture_output=True, text=True).stdout.strip()}   "
          f"NTP 同步 {subprocess.run(['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'], capture_output=True, text=True).stdout.strip()}")

    rclpy.init()
    node = Probe()
    p0, (t0c, i0c) = proc_snapshot(), cpu_total()
    t_start = time.time()
    while time.time() - t_start < a.secs:
        rclpy.spin_once(node, timeout_sec=0.05)
    W = time.time() - t_start
    p1, (t1c, i1c) = proc_snapshot(), cpu_total()
    clk = os.sysconf("SC_CLK_TCK")

    # ── 1 硬體與資源 ──
    hdr("1 電池／資源")
    names = {n for n, _ in node.get_node_names_and_namespaces()}
    print(f"  電池 {'讀不到（底盤沒開？）' if node.volt is None else f'{node.volt:.2f} V'}"
          + ("" if node.volt is None else ("   ✓ ≥23.5" if node.volt >= 23.5 else "   ⚠ <23.5 V，不要做會動車的實驗")))
    thr = subprocess.run(["sudo", "-n", "vcgencmd", "get_throttled"], capture_output=True, text=True).stdout.strip() or \
        ("throttled=0x" + open("/sys/devices/platform/soc/soc:firmware/get_throttled").read().strip())
    try:
        temp = int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
    except Exception:  # noqa: BLE001
        temp = float("nan")
    mem = {l.split(":")[0]: int(l.split()[1]) // 1024 for l in open("/proc/meminfo") if l.startswith(("MemAvailable", "MemTotal"))}
    dtot, didle = t1c - t0c, i1c - i0c
    idle = 100.0 * didle / dtot if dtot else 0.0
    print(f"  {thr}（0x0＝沒有欠壓／降頻；高位元是開機以來累積）   溫度 {temp:.1f} °C")
    print(f"  CPU 閒置 {idle:.0f}%（{W:.0f} 秒平均；4 核）   load1 {open('/proc/loadavg').read().split()[0]}   "
          f"可用記憶體 {mem.get('MemAvailable', -1)}/{mem.get('MemTotal', -1)} MB")
    agg = collections.Counter()
    for pid, (comm, tk) in p1.items():
        if pid in p0:
            agg[comm] += (tk - p0[pid][1]) / clk / W * 100
    print("  前幾名行程（% of 1 核）：" + "  ".join(f"{c}={v:.0f}" for c, v in agg.most_common(8) if v >= 3))

    # ── 2 單元 ──
    hdr("2 平板「系統開關」單元")
    units = get_units()
    running = set()
    if isinstance(units, str):
        print("  " + units)
    else:
        for u in units:
            k = u.get("key") or u.get("unit") or u.get("name")
            if u.get("running"):
                running.add(k)
        print("  開著：" + "、".join(sorted(running)) if running else "  開著：（無）")
        print("  關著：" + "、".join(sorted((u.get("key") or u.get("unit") or u.get("name")) for u in units
                                         if not u.get("running"))))

    # ── 3 節點 ──
    hdr("3 ROS 節點")
    allnodes = [f"{n}" for n, _ in node.get_node_names_and_namespaces()]
    dup = [k for k, v in collections.Counter(allnodes).items() if v > 1]
    groups = {
        "底盤／感測": ["wheeltec_robot", "ekf_filter_node", "lslidar_driver_node", "imu_bias_corrector_cc_node"],
        "導航": ["controller_server", "planner_server", "bt_navigator", "behavior_server", "velocity_smoother",
               "collision_monitor", "amcl", "map_server", "slam_toolbox", "steering_trim_cc_node", "cmd_vel_floor_cc_node",
               "path_teach_cc_node", "map_service_cc_node", "scan_filter_cc_node", "stuck_detector_cc_node"],
        "語音": ["speech_recognizer_node", "voice_trigger_node"],
        "LLM": ["llm_node", "llm_service_node"],
        "人臉": ["face_embedding_node", "user_auth_node"],
    }
    have = set(allnodes)
    for g, lst in groups.items():
        present = [x for x in lst if x in have]
        print(f"  {g:<8} {len(present)}/{len(lst)}  " + ("、".join(present) if present else "（沒有）"))
    nav_up = "controller_server" in have or "path_teach_cc_node" in have
    print(f"  重複節點：{'、'.join(dup) if dup else '無'}")

    # ── 4 生命週期 ──
    if nav_up:
        hdr("4 導航生命週期狀態")
        lc = lifecycle(node, ["controller_server", "planner_server", "bt_navigator", "behavior_server",
                              "velocity_smoother", "collision_monitor", "amcl", "map_server", "slam_toolbox"])
        print("  " + "  ".join(f"{k}={v}" for k, v in lc.items() if v is not None))
        miss = [k for k, v in lc.items() if v is None]
        if miss:
            print("  （沒回應／不存在：" + "、".join(miss) + "——建圖模式沒有 amcl／map_server，定位模式沒有 slam_toolbox 是正常的）")

    # ── 5 話題頻率 ──
    hdr(f"5 話題實際頻率（{W:.0f} 秒內收到幾則）")
    def rate(t):
        return f"{node.n[t]:>4}則 = {node.n[t] / W:5.1f} Hz"
    for t in ("/scan", "/scan_slam", "/odom", "/odom_combined", "/imu/data_unbiased"):
        print(f"  {t:<20} {rate(t)}")
    for t in ("/cmd_vel_smoothed", "/cmd_vel_trimmed", "/cmd_vel_prefloor", "/cmd_vel"):
        nz = node.n[t + ":nonzero"]
        print(f"  {t:<20} {rate(t)}   非零 {nz}   最後 {('(%.2f, %.2f)' % node.last[t]) if t in node.last else '—'}")
    if node.n["/cmd_vel:nonzero"] > 0:
        print("  ⚠ /cmd_vel 這幾秒有非零速度指令——車子可能正在動")
    print(f"  /user_text {node.n['/user_text']} 則   /llm_response {node.n['/llm_response']} 則（這幾秒內）")

    # ── 6 TF 與定位 ──
    hdr("6 TF 與定位")
    nowt = node.get_clock().now()
    for a_, b_ in (("map", "base_footprint"), ("odom_combined", "base_footprint")):
        try:
            tf = node.tfb.lookup_transform(a_, b_, rclpy.time.Time())
            age = (nowt - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
            t_ = tf.transform.translation
            q = tf.transform.rotation
            import math
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            print(f"  {a_}→{b_}：({t_.x:+.2f}, {t_.y:+.2f}) yaw {yaw:+.0f}°   新舊 {age:.2f} s" + ("   ⚠ 過期" if age > 1.0 else ""))
        except Exception as e:  # noqa: BLE001
            print(f"  {a_}→{b_}：查不到（{str(e)[:50]}）")
    if node.amcl is not None:
        c = node.amcl.pose.covariance
        age = time.time() - (node.amcl.header.stamp.sec + node.amcl.header.stamp.nanosec * 1e-9)
        print(f"  amcl_pose：最後一則 {age:.1f} s 前；位置共變異 x={c[0]:.3f} y={c[7]:.3f} yaw={c[35]:.3f}（越小越確定）")
    else:
        print("  amcl_pose：沒收到（建圖模式不跑 amcl，或定位還沒發布）")

    # ── 7 ASR ──
    hdr("7 語音輸入（ASR）")
    alog = newest("asr_chain_*.log")
    asr_final = []
    if alog:
        ovf = []
        for ln in open(alog, encoding="utf-8", errors="replace"):
            t = ts_of(ln)
            if "input overflow" in ln and t:
                ovf.append(t)
            m = re.search(r"最終結果（(.*?)）: '(.*)'", ln)
            if m and t:
                asr_final.append((t, m.group(1), m.group(2)))
        nowt_ = time.time()
        print(f"  log {os.path.basename(alog)}   input overflow 累計 {len(ovf)}   近 1 分鐘 {sum(1 for t in ovf if nowt_ - t <= 60)}   近 5 分鐘 {sum(1 for t in ovf if nowt_ - t <= 300)}")
        print(f"  最終辨識累計 {len(asr_final)} 筆；最近 3 筆：")
        for t, how, txt in asr_final[-3:]:
            print(f"    {HM(t)} [{how[:6]}] {txt[:44]}")
    else:
        print("  找不到 asr_chain log（語音輸入這次開機沒開過）")

    # ── 8 LLM ──
    hdr("8 LLM")
    llog = newest("llm_*.log")
    if llog:
        rounds, cur = [], None
        for ln in open(llog, encoding="utf-8", errors="replace"):
            t = ts_of(ln)
            if "📥 收到使用者輸入:" in ln:
                cur = {"t_in": t, "in": ln.split("收到使用者輸入:", 1)[1].strip(), "tools": [], "out": None, "t_out": None}
                rounds.append(cur)
            elif cur is not None:
                if "🛠️ LLM 決定呼叫服務:" in ln:
                    cur["tools"].append(ln.split("決定呼叫服務:", 1)[1].strip().split(",")[0])
                elif "🤖 Agent 最終決策回應:" in ln:
                    cur["out"], cur["t_out"] = ln.split("最終決策回應:", 1)[1].strip(), t
        typed = [r for r in rounds if not any(r["in"][:8] in x[2] or x[2][:8] in r["in"] for x in asr_final)]
        called = sum(1 for r in typed if r["tools"])
        print(f"  log {os.path.basename(llog)}   共 {len(rounds)} 輪（文字 {len(typed)}、語音 {len(rounds) - len(typed)}）；文字輸入中有呼叫工具的 {called} 輪")
        for r in rounds[-4:]:
            d = f"{r['t_out'] - r['t_in']:.1f}s" if r["t_in"] and r["t_out"] else "—"
            o = "語音" if r not in typed else "文字"
            print(f"    {HM(r['t_in'])} {o} {d:>6} 工具={'/'.join(r['tools']) or '無'}  {r['in'][:16]} → {(r['out'] or '（無回應）')[:34]}")
        last = max((r["t_out"] or r["t_in"] or 0) for r in rounds) if rounds else 0
        if last:
            idle_min = (time.time() - last) / 60
            print(f"  距離上一輪結束 {idle_min:.1f} 分鐘" + ("   ⚠ >5 分鐘：Ollama 可能已卸載模型，下一題約慢 5~6 秒（推論）" if idle_min > 5 else ""))
        warn = subprocess.run(["grep", "-c", "撞到 num_predict", llog], capture_output=True, text=True).stdout.strip()
        print(f"  「撞到 num_predict 上限」警告 {warn} 次")
    else:
        print("  找不到 llm log（LLM 這次開機沒開過）")

    # ── 9 導航 log ──
    hdr("9 導航 log（近 60 秒的警告／錯誤）")
    nlog = newest("nav_*.log")
    if nlog:
        mode, mode_t, nwarn, group = None, None, 0, collections.Counter()
        for ln in open(nlog, encoding="utf-8", errors="replace"):
            t = ts_of(ln)
            m = re.search(r"已進入(建圖|定位)模式(?: \(地圖 (\S+)\))?", ln)
            if m:
                mode, mode_t = f"{m.group(1)}模式" + (f"（{m.group(2)}）" if m.group(2) else ""), t
            if t and time.time() - t <= 60 and re.search(r"\[(WARN|ERROR)\]", ln):
                nwarn += 1
                group[re.sub(r"[0-9]+(\.[0-9]+)?", "N", EPOCH.sub("", ln)).strip()[:130]] += 1
        print(f"  log {os.path.basename(nlog)}（最後寫入 {HM(os.path.getmtime(nlog))}）   目前模式：{mode or '還沒進入任何模式'}"
              + (f"（{HM(mode_t)} 起）" if mode_t else ""))
        print(f"  近 60 秒 WARN／ERROR 共 {nwarn} 則" + ("：" if nwarn else "（沒有）"))
        for k, v in group.most_common(5):
            print(f"    ×{v}  {k}")
    else:
        print("  找不到 nav log（導航堆疊這次開機沒開過）")

    # ── 10 風險提示（由上面的事實推出，不是額外量測）──
    hdr("10 組合風險")
    notes = []
    asr_on = "speech_recognizer_node" in have or "voice_trigger_node" in have
    llm_on = "llm_node" in have or "llm_service_node" in have
    if nav_up and asr_on and llm_on:
        notes.append("導航、語音輸入、LLM 同時開著：麥克風聽到的話會直接餵 LLM，LLM 有導航工具，**亂碼或旁人說話可能觸發導航、車子會動**。要做動車實驗前先關語音輸入。")
    if nav_up and asr_on and idle < 30:
        notes.append(f"CPU 閒置只剩 {idle:.0f}%，語音輸入容易 input overflow。")
    if "face_embedding_node" in have and (nav_up or asr_on):
        notes.append("人臉辨識與導航／語音同開：9/17 實測人臉推論會把音訊執行緒餓死。")
    if node.n["/scan_slam"] == 0 and nav_up:
        notes.append("導航在線但 /scan_slam 這幾秒沒有資料——scan_filter_cc 沒在發，SLAM／AMCL／全域成本地圖會停更。")
    print("  " + ("\n  ".join("⚠ " + n for n in notes) if notes else "沒有觸發任何已知的組合風險"))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
