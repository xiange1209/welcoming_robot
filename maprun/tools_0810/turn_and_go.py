#!/usr/bin/env python3
"""大廳掉頭 -> 純前進開回起點。

★ 為什麼要掉頭（2026-08-10 實測結論）★
阿克曼車輛的倒車循跡是**不穩定平衡**：朝向誤差會指數成長。實測對照：

    起點->大廳  折返 0 次、6.51 m 一整段        success，誤差 11 cm
    起點->大廳  折返 2 次（4.70/0.15/2.16 m）   偏離 76 cm 中止
    大廳->起點  折返 2 次（2.34/0.16/4.97 m）   偏離 282 cm 中止

長倒車段必失敗。正解不是把倒車練好，是**讓倒車段短到不穩定性來不及長大**
—— 三點轉向裡的倒車是 0.5 m 級的，安全。

★ 不自己寫迴轉動作：給規劃器「同位置、朝向 +180 度」當目標，
  SmacPlannerHybrid 會自己產生符合最小轉彎半徑的折返路徑，
  而且它本來就吃成本地圖 —— **避障是內建的，不用另外寫**。
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from smartnav_msgs.srv import PlanTaughtPath, ListWaypoints
from smartnav_msgs.action import FollowTaughtPath
from std_srvs.srv import Empty
import tf2_ros

MIN_RADIUS_GATE = 0.76
MAX_REVERSE_SEG = 1.5      # 倒車段超過這個就拒絕（發散會長大）
MIN_SEG = 0.30             # 段長低於這個就拒絕（執行不了的抖動）


class TurnGo(Node):
    def __init__(self):
        super().__init__("turn_and_go")
        self.wp = self.create_client(ListWaypoints, "list_waypoints")
        self.plan = self.create_client(PlanTaughtPath, "plan_taught_path")
        self.clear = self.create_client(Empty, "/global_costmap/clear_entirely_global_costmap")
        self.follow = ActionClient(self, FollowTaughtPath, "follow_taught_path")
        self.buf = tf2_ros.Buffer(); self.tl = tf2_ros.TransformListener(self.buf, self)
        self.fb_last = ""

    def call(self, cli, req, name, t=90.0):
        if not cli.wait_for_service(timeout_sec=12.0):
            print(f"  ✗ {name} 沒回應"); return None
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=t)
        return f.result()

    def pose(self):
        t0 = time.time()
        while time.time() - t0 < 8.0:
            try:
                tr = self.buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
                q = tr.transform.rotation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                return tr.transform.translation.x, tr.transform.translation.y, yaw
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.2)
        return None

    @staticmethod
    def _segments(pts):
        co = []
        for q in pts:
            p = q.position if hasattr(q, "position") else None
            if p: co.append((p.x, p.y))
        segs = []; cur = 0.0; last = None
        for a, b in zip(co, co[1:]):
            v = (b[0]-a[0], b[1]-a[1]); d = math.hypot(*v)
            if d < 1e-6: continue
            if last is not None:
                dot = (v[0]*last[0]+v[1]*last[1])/(d*math.hypot(*last))
                if dot < -0.3:
                    segs.append(cur); cur = 0.0
            cur += d; last = v
        segs.append(cur)
        return segs

    def do_plan(self, name, goal_pose, from_robot=True):
        self.call(self.clear, Empty.Request(), "clear_costmap", t=20.0)
        r = PlanTaughtPath.Request()
        r.name = name; r.start_from_robot = from_robot; r.waypoints = [goal_pose]
        res = self.call(self.plan, r, "plan_taught_path", t=120.0)
        if res is None or not res.success:
            print(f"  ✗ 規劃失敗：{res.message if res else '無回應'}"); return None
        print(f"  ✓ {res.num_points} 點、{res.length_m:.2f} m、折返 {res.num_cusps} 次")
        return res

    def execute(self, path_id, label):
        if not self.follow.wait_for_server(timeout_sec=15.0):
            print("  ✗ follow 伺服器沒回應"); return False
        g = FollowTaughtPath.Goal(); g.path_id = path_id; g.reverse = False; g.speed_scale = 0.0
        self.t0 = time.time()
        sf = self.follow.send_goal_async(g, feedback_callback=self._fb)
        rclpy.spin_until_future_complete(self, sf, timeout_sec=20.0)
        gh = sf.result()
        if gh is None or not gh.accepted:
            print("  ✗ 目標被拒絕"); return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=420.0)
        r = rf.result()
        if r is None:
            print("  ✗ 逾時"); return False
        print(f"  {label}：success={r.result.success}  {r.result.message}")
        print(f"  終點誤差 {r.result.final_error_m:.3f} m  朝向誤差 {r.result.final_yaw_error_deg:.1f} 度")
        return r.result.success

    def _fb(self, fb):
        f = fb.feedback
        line = f"{time.time()-self.t0:6.1f}s {f.point_index:3d}/{f.total_points:3d} 偏離{f.cross_track_error_m*100:5.1f}cm [{f.state}]"
        if line[:40] != self.fb_last[:40]:
            print("   " + line + ("  " + f.message[:44] if f.message else ""))
            self.fb_last = line


def main():
    rclpy.init(); n = TurnGo()

    p = n.pose()
    if p is None:
        print("  ✗ 取不到位姿"); return 1
    x, y, yaw = p
    print(f"  目前位姿 x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):+.1f}度\n")

    # ── 第一步：原地掉頭 ──
    print("── 第一步：叫規劃器規劃「原地掉頭」（避障由成本地圖負責）──")
    g = Pose()
    g.position.x, g.position.y = x, y
    ny = yaw + math.pi
    g.orientation.z = math.sin(ny/2.0); g.orientation.w = math.cos(ny/2.0)
    res = n.do_plan(f"大廳掉頭_{time.strftime('%H%M')}", g)
    if res is None:
        print("  ★ 掉頭規劃不出來 —— 大廳空間可能不足，或成本地圖有殘影")
        return 1
    if not n.execute(res.path_id, "掉頭"):
        print("  ★ 掉頭執行失敗，停在這裡不繼續"); return 1

    p2 = n.pose()
    if p2:
        d = abs((math.degrees(p2[2]-yaw)+540) % 360 - 180)
        print(f"  掉頭後朝向變化 {d:.0f} 度（目標 180）\n")

    # ── 第二步：規劃回起點，要求 0 折返 ──
    print("── 第二步：規劃回起點，檢查倒車段 ──")
    wr = n.call(n.wp, ListWaypoints.Request(), "list_waypoints")
    if wr is None: return 1
    tgt = next((w for w in wr.waypoints_info if w.waypoint_name == "起點"), None)
    if tgt is None:
        print("  ✗ 找不到起點"); return 1
    res2 = n.do_plan(f"掉頭後回起點_{time.strftime('%H%M')}", tgt.pose)
    if res2 is None: return 1
    print(f"  折返 {res2.num_cusps} 次 "
          f"{'★ 0 折返，純前進 —— 這正是我們要的' if res2.num_cusps == 0 else '⚠ 仍有折返'}")
    if res2.num_cusps > 0:
        print(f"  ⚠ 有折返代表規劃器仍認為需要倒車。先執行看看，"
              f"但依 2026-08-10 的實測，長倒車段有很高機率發散。")
    if not n.execute(res2.path_id, "回起點"):
        return 1
    print("\n  ★★ 大廳 -> 起點 成功（掉頭 + 純前進）★★")
    return 0


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        rclpy.try_shutdown()
    sys.exit(rc)
