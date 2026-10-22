#!/usr/bin/env python3
"""身分權限的整合測試（★ 2026-10-22）：發假的 /user_identity（虛構姓名，不碰人臉庫），輪流扮演
「沒有人／訪客／VIP／黑名單」，同時測平板帶位 API（HMI）與 LLM 的回答。

需要：HMI 服務、LLM 單元、mock_nav_cc.py（假導航，不動車）。迎賓劇本（bank_reception）要關著——
這裡發的是假身分，開著會真的對黑名單發通報。
用法：python3 identity_access_test.py [--out ~/maprun/verify_out/identity_access_<HHMM>.txt]
"""
import argparse
import json
import os
import time
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from smartnav_msgs.msg import UserIdentity, UserType

ap = argparse.ArgumentParser()
ap.add_argument("--out", default=os.path.expanduser(f"~/maprun/verify_out/identity_access_{time.strftime('%H%M')}.txt"))
a = ap.parse_args()
HMI = "http://127.0.0.1:8080"
_home = os.path.expanduser("~/.smartnav")
_map = json.load(open(f"{_home}/nav_state.json"))["last_active_map"]
ALL_IDS = {v["name"]: k for k, v in json.load(open(f"{_home}/waypoint_database/waypoints.json"))["waypoints_db"].items()
           if v.get("map_id") == _map}

rclpy.init()
n = Node("identity_access_test_cc")
nodes = {nm for nm, _ in (time.sleep(2.0) or n.get_node_names_and_namespaces())}
if any("bank_reception" in x for x in nodes):
    raise SystemExit("✗ 迎賓劇本（bank_reception）開著——假黑名單會觸發真的通報，先在平板關掉")
id_pub = n.create_publisher(UserIdentity, "user_identity", 10)
txt_pub = n.create_publisher(String, "/user_text", 10)
clr_pub = n.create_publisher(Empty, "/clear_conversation", 10)
got = []
n.create_subscription(String, "/llm_response", lambda m: got.append((time.time(), m.data)), 10)
current = {"msg": None}


def set_identity(kind, name=""):
    if kind is None:
        current["msg"] = None
        return
    m = UserIdentity()
    m.user_name = name
    m.user_type = UserType(type={"GUEST": 0, "VIP": 1, "ADMIN": 2, "BLACKLIST": 3}[kind])
    m.recognized = kind != "GUEST" or bool(name)
    m.similarity = 0.86 if m.recognized else 0.55
    m.description = "測試用虛構資料：偏好中文服務、常辦外匯" if kind == "VIP" else ""
    m.bbox = [250.0, 140.0, 330.0, 240.0]
    current["msg"] = m


def spin(secs):
    t, last = time.time(), 0.0
    while time.time() - t < secs:
        if current["msg"] is not None and time.time() - last > 0.5:
            current["msg"].header.stamp = n.get_clock().now().to_msg()
            id_pub.publish(current["msg"])
            last = time.time()
        rclpy.spin_once(n, timeout_sec=0.05)


def http(method, path, body=None):
    req = urllib.request.Request(HMI + path, method=method, data=json.dumps(body).encode() if body is not None else None,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        return {"http": e.code, **json.loads(e.read() or b"{}")}


def ask(q, wait=8.0):
    clr_pub.publish(Empty())
    spin(0.8)
    got.clear()
    t0 = time.time()
    txt_pub.publish(String(data=q))
    end = t0 + wait
    while time.time() < end:
        spin(0.2)
        if got and time.time() - got[-1][0] > 2.5 and not got[-1][1].startswith("好的，請跟我來"):
            break
    return " ｜ ".join(x for _, x in got) or "（沒有回應）"


out = []
spin(3.0)
for label, kind, name in (("沒有人", None, ""), ("一般訪客（沒認出）", "GUEST", ""), ("VIP", "VIP", "測試貴賓"),
                          ("黑名單", "BLACKLIST", "測試黑名單")):
    set_identity(kind, name)
    spin(2.0)
    d = http("GET", "/api/guide/destinations")
    names = [x["name"] + ("★" if x.get("vip_only") else "") for x in d.get("destinations", [])]
    wid = {x["name"]: x["waypoint_id"] for x in d.get("destinations", [])}
    out.append(f"══ {label}")
    out.append(f"  平板帶位清單：can_navigate={d.get('can_navigate')} tier={d.get('tier')} → {names or '（不顯示）'}"
               + (f"；訊息「{d.get('message')}」" if d.get("message") else ""))
    for nm in ("貴賓室", "櫃台"):
        # 清單上沒有的也照樣送 ID 過去——測的是後端會不會擋（不能只靠平板不顯示）
        r = http("POST", "/api/guide", {"waypoint_id": wid.get(nm) or ALL_IDS.get(nm, "")})
        out.append(f"  平板點「{nm}」→ success={r.get('success')}：{r.get('message')}")
        spin(3.0)
    for q in ("帶我去貴賓室", "帶我去大廳", "你知道我是誰嗎？", "請問銀行幾點營業？"):
        out.append(f"  說「{q}」→ {ask(q)}")
    print("\n".join(out[-8:]), flush=True)

set_identity(None)
open(a.out, "w", encoding="utf-8").write("\n".join(out) + "\n")
print("輸出：", a.out)
n.destroy_node()
rclpy.shutdown()
