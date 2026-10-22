#!/usr/bin/env python3
"""靜態測試用的「假導航後端」：沒有雷達／底盤的板子上，讓 LLM 與 HMI 走到跟實車一樣的帶位路徑。

提供（介面與實車相同，只是不動車）：
  - 服務 list_waypoints（smartnav_msgs/ListWaypoints）：回目前地圖的真實地點（讀 ~/.smartnav 的資料庫，唯讀）
  - 服務 list_maps（smartnav_msgs/ListMaps）＋ latched 話題 current_map：HMI 地圖頁的下拉與頂列地圖名
  - 動作 navigate（smartnav_msgs/Navigate）：收到目標記一筆，等 --secs 秒後回「成功（模擬）」
★ 不發 /cmd_vel、不寫任何資料庫。節點名 mock_nav_cc 不在 llm10.py 的 NAV_NODES 裡，
  所以 llm10 --nav-down-ok 會直接送題（車不會動，本來就沒有底盤）。
★ 只能用在沒有導航堆疊的板子：真的導航開著時同名服務會打架，啟動時偵測到就拒絕。

用法：python3 mock_nav_cc.py [--secs 2] [--map map_xxx] [--log ~/maprun/verify_out/mock_nav_<HHMM>.csv]
停止：Ctrl-C，或 kill -INT <PID>
"""
import argparse
import csv
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from geometry_msgs.msg import Pose
from smartnav_msgs.action import Navigate
from smartnav_msgs.msg import MapInfo, WaypointInfo
from smartnav_msgs.srv import ListMaps, ListWaypoints

ap = argparse.ArgumentParser()
ap.add_argument("--secs", type=float, default=2.0, help="模擬到達要幾秒")
ap.add_argument("--map", default="", help="地圖 ID（預設讀 nav_state.json 的 last_active_map）")
ap.add_argument("--log", default=os.path.expanduser(f"~/maprun/verify_out/mock_nav_{time.strftime('%H%M')}.csv"))
a = ap.parse_args()

home = Path.home() / ".smartnav"
map_id = a.map or json.loads((home / "nav_state.json").read_text()).get("last_active_map", "")
db = json.loads((home / "waypoint_database" / "waypoints.json").read_text())["waypoints_db"]
maps_db = json.loads((home / "map_database" / "maps_db.json").read_text())["maps_db"]


def to_pose(p) -> Pose:
    pose = Pose()
    pos, ori = p.get("position", {}), p.get("orientation", {})
    pose.position.x, pose.position.y = float(pos.get("x", 0.0)), float(pos.get("y", 0.0))
    pose.orientation.z, pose.orientation.w = float(ori.get("z", 0.0)), float(ori.get("w", 1.0))
    return pose


class MockNav(Node):
    def __init__(self):
        super().__init__("mock_nav_cc")
        self.create_service(ListWaypoints, "list_waypoints", self.on_list)
        self.create_service(ListMaps, "list_maps", self.on_list_maps)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(String, "current_map", latched)
        self.map_pub.publish(String(data=map_id))
        # Reentrant：執行中的目標要能同時收到取消（預設的互斥 callback group 會讓取消排在執行完之後）
        ActionServer(self, Navigate, "navigate", self.on_navigate, callback_group=ReentrantCallbackGroup(),
                     cancel_callback=lambda _gh: CancelResponse.ACCEPT)
        self.fh = open(a.log, "w", newline="", encoding="utf-8-sig")
        self.w = csv.writer(self.fh)
        self.w.writerow(["時間", "epoch", "waypoint_id", "地點", "target_name", "use_target_pose"])
        names = [v["name"] for v in db.values() if v.get("map_id") == map_id]
        self.get_logger().info(f"假導航後端就緒：地圖 {map_id}，地點 {names}；到達要 {a.secs:.1f} 秒；紀錄 {a.log}")

    def on_list(self, req, res):
        want = req.map_id or map_id
        for wid, v in db.items():
            if v.get("map_id") == want:
                res.waypoints_info.append(
                    WaypointInfo(waypoint_id=wid, waypoint_name=v["name"], map_id=want, pose=to_pose(v.get("pose", {})))
                )
        res.success, res.message = True, f"（模擬）{len(res.waypoints_info)} 個地點"
        return res

    def on_list_maps(self, req, res):
        for mid, v in maps_db.items():
            res.maps_info.append(MapInfo(map_id=mid, map_name=v.get("map_name") or v.get("name") or mid))
        res.success, res.message = True, f"（模擬）{len(res.maps_info)} 張地圖"
        return res

    def on_navigate(self, goal_handle):
        g = goal_handle.request
        name = db.get(g.waypoint_id, {}).get("name", "（不存在的 ID）")
        now = time.time()
        self.w.writerow([time.strftime("%H:%M:%S", time.localtime(now)), f"{now:.3f}", g.waypoint_id, name,
                         g.target_name, g.use_target_pose])
        self.fh.flush()
        self.get_logger().info(f"收到導航目標 {g.waypoint_id}（{name}）→ {a.secs:.1f} 秒後回成功（車沒動）")
        result = Navigate.Result()
        if g.waypoint_id and g.waypoint_id not in db and not g.use_target_pose:
            goal_handle.abort()
            result.success, result.message = False, f"地點 {g.waypoint_id} 不存在（模擬）"
            return result
        t0 = time.time()
        while time.time() - t0 < a.secs:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.w.writerow([time.strftime("%H:%M:%S"), f"{time.time():.3f}", g.waypoint_id, name, "（取消）",
                                 f"開始後 {time.time() - t0:.2f} s"])
                self.fh.flush()
                self.get_logger().info(f"收到取消 → {name} 在 {time.time() - t0:.2f} s 停下（模擬）")
                result.success, result.message = False, f"導航已取消（模擬，{name}）"
                return result
            time.sleep(0.05)
        goal_handle.succeed()
        result.success, result.message = True, f"已到達{name}（模擬，車沒動）"
        return result


rclpy.init()
probe = rclpy.create_node("mock_nav_probe_cc")
time.sleep(2.0)
# 只看「提供」服務的節點：LLM／HMI 的 client 也會讓服務名出現在 get_service_names_and_types() 裡
clash = [f"{ns.rstrip('/')}/{nm}" for nm, ns in probe.get_node_names_and_namespaces()
         if any(s.endswith(("/list_waypoints", "/list_maps")) for s, _ in probe.get_service_names_and_types_by_node(nm, ns))]
probe.destroy_node()
if clash:
    raise SystemExit(f"✗ 已經有節點在提供 list_waypoints／list_maps（{clash}）——真的導航開著，不要跑假後端")
node = MockNav()
ex = MultiThreadedExecutor(num_threads=4)
ex.add_node(node)
try:
    ex.spin()
except KeyboardInterrupt:
    pass
finally:
    node.fh.close()
    node.destroy_node()
    rclpy.try_shutdown()
