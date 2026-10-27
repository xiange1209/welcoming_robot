#!/usr/bin/env python3
"""量「Python 節點掛著 TransformListener」本身吃多少 CPU（★ 2026-10-23，單板優化評估用）。

背景：10/20 全功能同開時 CPU 閒置 0%；map_service_cc、waypoint_service_cc、path_teach_cc、HMI 都是
Python 節點且掛著 TF 監聽，閒置時 path_teach_cc 也吃 25~38%、map_service_cc 18%。
TF 監聽要在 Python 裡反序列化每一則 /tf，懷疑這是閒置成本的大宗。

做法：一個行程發假 /tf（odom_combined→base_footprint 50 Hz ＋ map→odom_combined 20 Hz，跟實車差不多），
另一個行程用跟 _cc 節點一樣的 MultiThreadedExecutor，分「有掛 TransformListener」與「沒掛」各量 N 秒。
用法：python3 tf_bench.py [--secs 30]
"""
import argparse
import os
import subprocess
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("--secs", type=float, default=30.0)
ap.add_argument("--role", default="", help=argparse.SUPPRESS)
a = ap.parse_args()

if a.role == "pub":
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    n = rclpy.create_node("tf_bench_pub_cc")
    pub = n.create_publisher(TFMessage, "/tf", 100)

    def tf(parent, child, x):
        t = TransformStamped()
        t.header.stamp = n.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x = x
        t.transform.rotation.w = 1.0
        return t

    k, end = 0, time.time() + a.secs + 8.0
    while time.time() < end:
        pub.publish(TFMessage(transforms=[tf("odom_combined", "base_footprint", k * 1e-3)]))
        if k % 5 == 0:
            pub.publish(TFMessage(transforms=[tf("map", "odom_combined", 0.0)]))
        k += 1
        time.sleep(0.02)
    sys.exit(0)

if a.role in ("listener", "none"):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    n = rclpy.create_node("tf_bench_listen_cc")
    if a.role == "listener":
        from tf2_ros import Buffer, TransformListener

        buf = Buffer()
        lis = TransformListener(buf, n)  # noqa: F841 — 掛著就好，什麼都不查（等同節點閒置）
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    end = time.time() + a.secs + 4.0
    while time.time() < end:
        ex.spin_once(timeout_sec=0.1)
    sys.exit(0)


def ticks(pid):
    f = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
    return int(f[11]) + int(f[12])


hz = os.sysconf("SC_CLK_TCK")
me = os.path.abspath(__file__)
pub = subprocess.Popen([sys.executable, me, "--role", "pub", "--secs", str(a.secs * 2 + 10)])
time.sleep(2.0)
out = {}
for role in ("none", "listener"):
    p = subprocess.Popen([sys.executable, me, "--role", role, "--secs", str(a.secs)])
    time.sleep(3.0)  # 等 import、探索完成
    c0, t0 = ticks(p.pid), time.time()
    time.sleep(a.secs)
    out[role] = 100.0 * (ticks(p.pid) - c0) / hz / (time.time() - t0)
    p.wait()
pub.terminate()
pub.wait()
temp = int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
print(f"沒掛 TF 監聽：{out['none']:.1f}%（單核）")
print(f"掛著 TF 監聽：{out['listener']:.1f}%（單核）")
print(f"→ 每個 Python 節點光是掛著 TF 監聽就多 {out['listener'] - out['none']:.1f} 個百分點（/tf 約 70 則/秒，CPU {temp:.0f}°C）")
