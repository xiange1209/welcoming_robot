#!/usr/bin/env python3
"""發假的人臉身分（/user_identity，2 Hz），給 HMI／LLM 的靜態測試用。姓名請用虛構的，不要用人臉庫裡的人。
★ 迎賓劇本（bank_reception）開著時不要發 BLACKLIST——會真的觸發通報。

用法：python3 fake_identity_cc.py --type VIP|GUEST|BLACKLIST|ADMIN [--name 測試貴賓] [--secs 30]
"""
import argparse
import time

import rclpy
from smartnav_msgs.msg import UserIdentity, UserType

ap = argparse.ArgumentParser()
ap.add_argument("--type", required=True, choices=["GUEST", "VIP", "ADMIN", "BLACKLIST"])
ap.add_argument("--name", default="")
ap.add_argument("--secs", type=float, default=30.0)
a = ap.parse_args()

rclpy.init()
n = rclpy.create_node("fake_identity_cc")
time.sleep(1.5)
if any("bank_reception" in nm for nm, _ in n.get_node_names_and_namespaces()) and a.type == "BLACKLIST":
    raise SystemExit("✗ 迎賓劇本開著，不發假黑名單")
pub = n.create_publisher(UserIdentity, "user_identity", 10)
m = UserIdentity()
m.user_name = a.name
m.user_type = UserType(type={"GUEST": 0, "VIP": 1, "ADMIN": 2, "BLACKLIST": 3}[a.type])
m.recognized = a.type != "GUEST" or bool(a.name)
m.similarity = 0.86 if m.recognized else 0.55
m.description = "測試用虛構資料" if a.type == "VIP" else ""
m.bbox = [250.0, 140.0, 330.0, 240.0]
end = time.time() + a.secs
while time.time() < end:
    m.header.stamp = n.get_clock().now().to_msg()
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.5)
n.destroy_node()
rclpy.shutdown()
