#!/usr/bin/env python3
"""path_teach_cc 的按需 TF＋錄製計時器實測：閒置 CPU → 開始錄製（要接上 TF、計時器要跑）→ 取消（不存檔）→ 閒置 CPU。
用 tf_bench.py 的假 /tf（base_footprint 以 0.05 m/s 往前走），只呼叫 START 與 CANCEL，**不會寫路徑資料庫**。
用法：python3 path_teach_record_check.py"""
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from node_idle_cpu import child_pid, ticks, hz  # noqa: E402  （沿用同一套量法）

pub = subprocess.Popen([sys.executable, os.path.join(HERE, "tf_bench.py"), "--role", "pub", "--secs", "300"])
log = open(os.path.expanduser("~/maprun/verify_out/path_teach_record_check.log"), "w")
w = subprocess.Popen(["ros2", "run", "smartnav_navigation_cc", "path_teach_cc", "--ros-args", "-r",
                      "__node:=path_teach_cc_bench"], stdout=log, stderr=subprocess.STDOUT)
pid = child_pid(w.pid, "path_teach_cc")


def cpu(secs):
    c0, t0 = ticks(pid), time.time()
    time.sleep(secs)
    return 100.0 * (ticks(pid) - c0) / hz / (time.time() - t0)


def call(action):
    r = subprocess.run(["ros2", "service", "call", "/record_path", "smartnav_msgs/srv/RecordPath",
                        f"{{action: {action}}}"], capture_output=True, text=True, timeout=30)
    return r.stdout.strip().splitlines()[-1] if r.stdout.strip() else r.stderr.strip()[-120:]


time.sleep(40)
idle1 = cpu(15)
print(f"閒置（TF 拆掉、計時器停）：{idle1:.1f}%")
print("開始錄製 →", call(0))
rec = cpu(10)
print(f"錄製中（TF 接著、計時器 50 Hz）：{rec:.1f}%")
print("取消錄製 →", call(2))
time.sleep(40)
idle2 = cpu(15)
print(f"取消後再閒置：{idle2:.1f}%")
os.kill(pid, signal.SIGINT)
w.wait(timeout=20)
pub.terminate()
pub.wait()
log.close()
txt = open(os.path.expanduser("~/maprun/verify_out/path_teach_record_check.log"), encoding="utf-8").read()
for key in ("TF 監聽接上", "TF 監聽閒置", "開始錄製教導路徑"):
    print(f"log「{key}」出現 {txt.count(key)} 次")
