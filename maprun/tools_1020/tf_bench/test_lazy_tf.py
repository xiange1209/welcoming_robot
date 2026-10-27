#!/usr/bin/env python3
"""LazyTf（smartnav_navigation_cc/lazy_tf.py）的行為測試：接上→查得到、閒置→自動拆、peek 不叫醒、可再接上、
拆掉時 CPU 真的下來。發布端用 tf_bench.py 的假 /tf（70 則/秒）。用法：python3 test_lazy_tf.py"""
import os
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

sys.path.insert(0, os.path.expanduser("~/welcoming_robot_ws/src/smartnav_navigation_cc"))
from smartnav_navigation_cc.lazy_tf import LazyTf  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
pub = subprocess.Popen([sys.executable, os.path.join(HERE, "tf_bench.py"), "--role", "pub", "--secs", "60"])
rclpy.init()
n = rclpy.create_node("lazy_tf_test_cc")
tf = LazyTf(n, idle_sec=3.0, warmup_sec=1.0)
ex = MultiThreadedExecutor(num_threads=4)
ex.add_node(n)
threading.Thread(target=ex.spin, daemon=True).start()
time.sleep(2.0)
ok = fail = 0


def check(cond, label):
    global ok, fail
    ok, fail = ok + bool(cond), fail + (not cond)
    print(f"  {'✓' if cond else '✗'} {label}")


def cpu_pct(secs, keep_using):
    f = lambda: sum(int(x) for x in open(f"/proc/{os.getpid()}/stat").read().rsplit(")", 1)[1].split()[11:13])  # noqa: E731
    c0, t0 = f(), time.time()
    while time.time() - t0 < secs:
        if keep_using:
            tf.buffer()
        time.sleep(0.5)
    return 100.0 * (f() - c0) / os.sysconf("SC_CLK_TCK") / (time.time() - t0)


check(tf.peek() is None and not tf.active(), "一開始沒有掛監聽（peek 回 None）")
t0 = time.monotonic()
buf = tf.buffer(wait_for=("map", "base_footprint"))
dt = time.monotonic() - t0
check(buf.can_transform("map", "base_footprint", Time()), f"接上後 {dt:.2f} 秒內查得到 map→base_footprint（上限 1.0）")
check(tf.peek() is buf, "接上之後 peek 拿到同一個 Buffer")
active_cpu = cpu_pct(8.0, keep_using=True)
check(tf.active(), "一直在用 → 不會被拆")
for _ in range(3):  # peek 不延長壽命
    tf.peek()
    time.sleep(1.5)
time.sleep(4.0)
check(not tf.active() and tf.peek() is None, "閒置 3 秒後自動拆掉（peek 不會讓它續命）")
idle_cpu = cpu_pct(8.0, keep_using=False)
check(idle_cpu < active_cpu * 0.5, f"拆掉時 CPU 降下來：用的時候 {active_cpu:.1f}% → 拆掉 {idle_cpu:.1f}%")
buf2 = tf.buffer(wait_for=("map", "base_footprint"))
check(buf2 is not buf and buf2.can_transform("map", "base_footprint", Time()), "拆掉後可以再接上（新的 Buffer）")
check(tf.activations == 2, f"一共接上 {tf.activations} 次（預期 2）")
pub.terminate()
pub.wait()
print(f"\n{ok}/{ok + fail} 通過")
ex.shutdown()
sys.exit(0 if fail == 0 else 1)
