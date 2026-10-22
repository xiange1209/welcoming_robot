#!/usr/bin/env python3
"""帶位途中說「停下來」的測試（配合 mock_nav_cc.py --secs 8，車不會動）。

情境 A：「帶我去大廳」→ 2 秒後「停下來」→ 應該在 1~2 秒內收到「車子已經停下來了…」，假後端記一筆（取消）
情境 B：「帶我去大廳」→ 2 秒後「請問幾點營業」（不是停止）→ 應該照舊回「正在處理上一個指令」，車繼續到達
用法：python3 voice_stop_test.py [--out ~/maprun/verify_out/voice_stop_<HHMM>.txt]
"""
import argparse
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

ap = argparse.ArgumentParser()
ap.add_argument("--out", default=os.path.expanduser(f"~/maprun/verify_out/voice_stop_{time.strftime('%H%M')}.txt"))
a = ap.parse_args()

rclpy.init()
n = Node("voice_stop_test_cc")
pub = n.create_publisher(String, "/user_text", 10)
clr = n.create_publisher(Empty, "/clear_conversation", 10)
got = []
n.create_subscription(String, "/llm_response", lambda m: got.append((time.time(), m.data)), 10)


def spin(secs):
    t = time.time()
    while time.time() - t < secs:
        rclpy.spin_once(n, timeout_sec=0.05)


spin(4.0)
lines = []


def case(label, second, wait_after):
    clr.publish(Empty())
    spin(1.0)
    got.clear()
    t0 = time.time()
    pub.publish(String(data="帶我去大廳"))
    spin(2.0)
    t1 = time.time()
    pub.publish(String(data=second))
    spin(wait_after)
    lines.append(f"── {label}：「帶我去大廳」→ +{t1 - t0:.1f} s「{second}」")
    for ts, txt in got:
        lines.append(f"   +{ts - t0:5.2f} s（第二句後 +{ts - t1:5.2f} s）{txt}")


case("A 帶位途中喊停", "停下來", 6.0)
case("B 帶位途中問別的", "請問幾點營業", 9.0)
open(a.out, "w", encoding="utf-8").write("\n".join(lines) + "\n")
print("\n".join(lines))
print("輸出：", a.out)
n.destroy_node()
rclpy.shutdown()
