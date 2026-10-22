#!/usr/bin/env python3
"""多工負載時間序列（唯讀；不發任何指令、不改任何東西）。

用法：
    source ~/maprun/env.sh
    python3 ~/maprun/tools_1020/load_monitor.py --label 人臉+語音 --secs 300            # 每 10 秒一列
    python3 ~/maprun/tools_1020/load_monitor.py --label 全開 --secs 600 --interval 5

每一列記：CPU 閒置%、load1、可用記憶體、溫度、降頻旗標（sysfs）、這段時間內 ASR log 新增的 input overflow 次數、
各話題收到幾則（/user_identity、/face_embedding、/camera/color/camera_info、/scan、/user_text、/llm_response、/cmd_vel），
以及吃 CPU 最多的 6 個行程（依行程名稱加總，% of 1 核）。
★ 影像只訂 camera_info（與影像同頻、幾十 bytes），不訂原始影像——訂 640x480 影像本身就會吃掉一大塊 CPU，量測會干擾被量的東西。
★ 話題數字是「這段時間收到幾則」，訂閱端自己漏收也會算少；要的是相對變化，不是絕對頻率。
輸出：~/maprun/verify_out/load_<標籤>_<HHMM>.csv；結束時印平均／最差。
★ 2026-10-20 車上新寫。
"""
import argparse
import collections
import csv
import datetime as dt
import glob
import os
import re
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import String

LOGS = os.path.expanduser("~/.smartnav/logs/system_control")
EPOCH = re.compile(r"\[(\d{10}\.\d+)\]")


def proc_ticks():
    d = {}
    for e in os.scandir("/proc"):
        if not e.name.isdigit():
            continue
        try:
            s = open(f"/proc/{e.name}/stat").read()
            rp = s.rindex(")")
            f = s[rp + 2:].split()
            d[int(e.name)] = (s[s.index("(") + 1:rp], int(f[11]) + int(f[12]))
        except Exception:  # noqa: BLE001
            pass
    return d


def cpu_total():
    v = list(map(int, open("/proc/stat").readline().split()[1:9]))
    return sum(v), v[3] + v[4]


def asr_overflows_since(t_from, t_to):
    fs = glob.glob(os.path.join(LOGS, "asr_chain_*.log"))
    if not fs:
        return ""
    n = 0
    with open(max(fs, key=os.path.getmtime), encoding="utf-8", errors="replace") as fh:
        for ln in fh:
            if "input overflow" in ln:
                m = EPOCH.search(ln)
                if m and t_from < float(m.group(1)) <= t_to:
                    n += 1
    return n


def read(path, conv=str):
    try:
        return conv(open(path).read().strip())
    except Exception:  # noqa: BLE001
        return ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="未標記")
    ap.add_argument("--secs", type=float, default=300)
    ap.add_argument("--interval", type=float, default=10)
    a = ap.parse_args()
    out = os.path.expanduser(f"~/maprun/verify_out/load_{re.sub(r'[^0-9A-Za-z一-龥+_-]', '_', a.label)}_{time.strftime('%H%M')}.csv")

    rclpy.init()
    n = Node("load_monitor_cc")
    cnt = collections.Counter()
    topics = {"/user_identity": None, "/face_embedding": None}
    # 自訂訊息型別：只為了計數，用執行期查到的型別訂閱
    from rosidl_runtime_py.utilities import get_message
    for name, types in n.get_topic_names_and_types():
        if name in topics and types:
            try:
                n.create_subscription(get_message(types[0]), name, lambda _m, k=name: cnt.update([k]), 10)
                topics[name] = types[0]
            except Exception:  # noqa: BLE001
                pass
    n.create_subscription(CameraInfo, "/camera/color/camera_info", lambda _m: cnt.update(["camera_info"]), qos_profile_sensor_data)
    n.create_subscription(LaserScan, "/scan", lambda _m: cnt.update(["/scan"]), qos_profile_sensor_data)
    n.create_subscription(String, "/user_text", lambda _m: cnt.update(["/user_text"]), 10)
    n.create_subscription(String, "/llm_response", lambda _m: cnt.update(["/llm_response"]), 10)
    n.create_subscription(Twist, "/cmd_vel", lambda _m: cnt.update(["/cmd_vel"]), qos_profile_sensor_data)
    # 底盤里程計：全開時曾從 24 Hz 掉到 6 Hz（底盤驅動也被餓到），要一起看
    n.create_subscription(Odometry, "/odom", lambda _m: cnt.update(["/odom"]), qos_profile_sensor_data)
    clk = os.sysconf("SC_CLK_TCK")
    cols = ["時間", "標籤", "CPU閒置%", "load1", "可用MB", "溫度C", "throttled", "ASR_overflow",
            "user_identity", "face_embedding", "camera_info", "scan", "user_text", "llm_response", "cmd_vel", "odom", "前6行程(%1核)"]
    rows = []
    print(f"負載記錄 {a.label}：{a.secs:.0f} 秒、每 {a.interval:.0f} 秒一列 → {out}")
    print("  " + "  ".join(cols[2:16]))
    t_end = time.time() + a.secs
    p0, (t0c, i0c), w0 = proc_ticks(), cpu_total(), time.time()
    try:
        while time.time() < t_end:
            t_next = w0 + a.interval
            while time.time() < t_next:
                rclpy.spin_once(n, timeout_sec=0.05)
            p1, (t1c, i1c), w1 = proc_ticks(), cpu_total(), time.time()
            W = w1 - w0
            idle = 100.0 * (i1c - i0c) / max(1, t1c - t0c)
            agg = collections.Counter()
            for pid, (comm, tk) in p1.items():
                if pid in p0:
                    agg[comm] += (tk - p0[pid][1]) / clk / W * 100
            top = " ".join(f"{c}={v:.0f}" for c, v in agg.most_common(6))
            mem = next((int(l.split()[1]) // 1024 for l in open("/proc/meminfo") if l.startswith("MemAvailable")), "")
            row = [dt.datetime.fromtimestamp(w1).strftime("%H:%M:%S"), a.label, f"{idle:.0f}", open("/proc/loadavg").read().split()[0],
                   mem, read("/sys/class/thermal/thermal_zone0/temp", lambda s: f"{int(s) / 1000:.1f}"),
                   read("/sys/devices/platform/soc/soc:firmware/get_throttled"), asr_overflows_since(w0, w1),
                   cnt["/user_identity"], cnt["/face_embedding"], cnt["camera_info"], cnt["/scan"],
                   cnt["/user_text"], cnt["/llm_response"], cnt["/cmd_vel"], cnt["/odom"], top]
            rows.append(row)
            print("  " + "  ".join(str(x) for x in row[:16]) + "  | " + top)
            cnt.clear()
            p0, t0c, i0c, w0 = p1, t1c, i1c, w1
    except KeyboardInterrupt:
        pass
    finally:
        with open(out, "w", newline="", encoding="utf-8-sig") as fh:
            w = csv.writer(fh)
            w.writerow(cols)
            w.writerows(rows)
        if rows:
            idles = [float(r[2]) for r in rows]
            ovf = [r[7] for r in rows if r[7] != ""]
            print(f"\n══ {a.label}：{len(rows)} 列；CPU 閒置 平均 {sum(idles) / len(idles):.0f}%、最低 {min(idles):.0f}%"
                  + (f"；ASR overflow 共 {sum(ovf)} 次（{sum(ovf) / (len(rows) * a.interval / 60):.1f} 次/分）" if ovf else "；（沒有 ASR log）")
                  + f"；溫度最高 {max(float(r[5]) for r in rows if r[5]):.1f} °C ══\n存檔：{out}")
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
