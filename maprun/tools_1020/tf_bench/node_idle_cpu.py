#!/usr/bin/env python3
"""實際節點的閒置 CPU：tf_idle_sec 預設（需要時才掛 TF）vs 0（一直掛著＝10/23 以前的行為）。

只跑不會寫資料庫的兩支：path_teach_cc（錄製／重播才寫）、waypoint_service_cc（建點／刪點才寫）。
map_service_cc 開機流程會切導航模式、寫 nav_state.json，不在這裡單獨跑。
發布端：tf_bench.py 的假 /tf（odom_combined→base_footprint 50 Hz ＋ map→odom_combined 20 Hz）。
用法：python3 node_idle_cpu.py [--settle 40] [--secs 20]
"""
import argparse
import os
import signal
import subprocess
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("--settle", type=float, default=40.0, help="啟動後等多久才開始量（要大於 tf_idle_sec 才看得到拆掉的效果）")
ap.add_argument("--secs", type=float, default=20.0)
a = ap.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__))
hz = os.sysconf("SC_CLK_TCK")


def child_pid(wrapper_pid, name):
    """ros2 run 會再起一個 python 行程；找名字對得上的那個子行程"""
    deadline = time.time() + 20
    while time.time() < deadline:
        for d in os.listdir("/proc"):
            if not d.isdigit():
                continue
            try:
                stat = open(f"/proc/{d}/stat").read()
                cmd = open(f"/proc/{d}/cmdline", "rb").read().split(b"\0")
            except OSError:
                continue
            ppid = int(stat.rsplit(")", 1)[1].split()[1])
            if ppid == wrapper_pid and any(c.endswith(name.encode()) for c in cmd[:2]):
                return int(d)
        time.sleep(0.5)
    return None


def ticks(pid):
    f = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
    return int(f[11]) + int(f[12])


pub = subprocess.Popen([sys.executable, os.path.join(HERE, "tf_bench.py"), "--role", "pub", "--secs", "600"])
rows = []
for exe in ("path_teach_cc", "waypoint_service_cc"):
    for idle in ("30.0", "0.0"):
        w = subprocess.Popen(["ros2", "run", "smartnav_navigation_cc", exe, "--ros-args", "-r", f"__node:={exe}_bench",
                              "-p", f"tf_idle_sec:={idle}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        pid = child_pid(w.pid, exe)
        if pid is None:
            print(f"✗ {exe} 沒起來")
            w.send_signal(signal.SIGINT)
            continue
        time.sleep(a.settle)
        c0, t0 = ticks(pid), time.time()
        time.sleep(a.secs)
        cpu = 100.0 * (ticks(pid) - c0) / hz / (time.time() - t0)
        os.kill(pid, signal.SIGINT)   # ros2 run 不轉送 SIGINT，直接對子行程送
        try:
            w.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.kill(pid, signal.SIGTERM)
            w.wait(timeout=10)
        label = "需要時才掛（新）" if idle != "0.0" else "一直掛著（舊）"
        rows.append((exe, label, cpu))
        print(f"{exe:<22} {label}：閒置 CPU {cpu:5.1f}%（單核）", flush=True)
pub.terminate()
pub.wait()
temp = int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
print(f"（啟動後 {a.settle:.0f} 秒開始量 {a.secs:.0f} 秒；/tf 約 70 則/秒；CPU {temp:.0f}°C）")
