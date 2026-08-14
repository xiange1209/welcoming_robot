#!/usr/bin/env python3
"""走廊實跑：規劃 起點→大廳，做可行性檢查，然後重播並逐筆存 CSV。

★ 這一趟的目的是撈出 8/06 缺的那一行診斷：
      側 左0.51/右0.04  排斥+0.11  繞障-0.10  合計+0.01  曲率★飽和
  它會告訴我們車子貼右牆到底是「繞障把排斥抵消掉」、「曲率箝制吃掉」還是別的。
"""
import csv, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from smartnav_msgs.srv import PlanTaughtPath, ListWaypoints
from smartnav_msgs.action import FollowTaughtPath
from std_srvs.srv import Empty

MIN_RADIUS_GATE = 0.76      # ★ 用 0.76 不是 0.80——規劃器就是貼著 0.80 規劃的
CSV = f"/home/user/replay_corridor_{time.strftime('%m%d_%H%M')}.csv"


class Runner(Node):
    def __init__(self):
        super().__init__("corridor_run")
        self.wp_cli = self.create_client(ListWaypoints, "list_waypoints")
        self.plan_cli = self.create_client(PlanTaughtPath, "plan_taught_path")
        self.clear_cli = self.create_client(Empty, "/global_costmap/clear_entirely_global_costmap")
        self.follow = ActionClient(self, FollowTaughtPath, "follow_taught_path")
        self.rows = []
        self.diag = []          # 帶診斷字串的 feedback
        self.states = []

    def call(self, cli, req, name, t=30.0):
        if not cli.wait_for_service(timeout_sec=10.0):
            print(f"  ✗ {name} 沒回應"); return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=t)
        return fut.result()

    def run(self, target_name):
        # ── 清成本地圖（操作者的身體會被記進去）──
        self.call(self.clear_cli, Empty.Request(), "clear_global_costmap", t=15.0)
        print("  ✓ 已清全域成本地圖")

        res = self.call(self.wp_cli, ListWaypoints.Request(), "list_waypoints")
        if res is None:
            return False
        wp = next((w for w in res.waypoints_info if w.waypoint_name == target_name), None)
        if wp is None:
            print(f"  ✗ 找不到地點「{target_name}」"); return False

        req = PlanTaughtPath.Request()
        req.name = f"走廊診斷_{time.strftime('%m%d_%H%M')}"
        req.start_from_robot = True
        req.waypoints = [wp.pose]
        print(f"  規劃 目前位置 → {target_name} …")
        pr = self.call(self.plan_cli, req, "plan_taught_path", t=90.0)
        if pr is None or not pr.success:
            print(f"  ✗ 規劃失敗：{pr.message if pr else '無回應'}"); return False
        print(f"  ✓ {pr.message}")
        print(f"    path_id={pr.path_id}  {pr.num_points} 點  {pr.length_m:.2f} m  折返 {pr.num_cusps} 次")

        # ── 執行 ──
        if not self.follow.wait_for_server(timeout_sec=15.0):
            print("  ✗ follow_taught_path 動作伺服器沒回應"); return False
        g = FollowTaughtPath.Goal()
        g.path_id = pr.path_id
        g.reverse = False
        g.speed_scale = 0.0
        print(f"  ── 開始重播，逐筆存 {CSV} ──")
        self.t0 = time.time()
        sfut = self.follow.send_goal_async(g, feedback_callback=self._fb)
        rclpy.spin_until_future_complete(self, sfut, timeout_sec=20.0)
        gh = sfut.result()
        if gh is None or not gh.accepted:
            print("  ✗ 目標被拒絕"); return False
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=420.0)
        r = rfut.result()
        self._save()
        if r is None:
            print("  ✗ 逾時，沒收到結果"); return False
        res = r.result
        print()
        print(f"  結果 success={res.success}  {res.message}")
        print(f"  終點誤差 {res.final_error_m:.3f} m   朝向誤差 {res.final_yaw_error_deg:.1f} 度")
        return res.success

    def _fb(self, fb):
        f = fb.feedback
        t = time.time() - self.t0
        self.rows.append([f"{t:.2f}", f.point_index, f.total_points,
                          f"{f.progress:.3f}", f"{f.cross_track_error_m:.4f}",
                          f.state, f.message])
        self.states.append(f.state)
        if "側" in f.message or "排斥" in f.message:
            self.diag.append((t, f.cross_track_error_m, f.message))
        if len(self.rows) % 8 == 0:
            print(f"   {t:6.1f}s {f.point_index:3d}/{f.total_points:3d} "
                  f"偏離 {f.cross_track_error_m*100:5.1f}cm [{f.state}] {f.message[:52]}")

    def _save(self):
        with open(CSV, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "idx", "total", "progress", "xte_m", "state", "message"])
            w.writerows(self.rows)
        print(f"  ✓ CSV 已存：{CSV}（{len(self.rows)} 筆）")
        if self.states:
            from collections import Counter
            c = Counter(self.states); n = len(self.states)
            print("  狀態佔比：" + "  ".join(f"{k} {v*100/n:.1f}%" for k, v in c.most_common()))
        print()
        print("  ★★ 診斷行（8/06 缺的就是這個）★★")
        if not self.diag:
            print("     （整趟都沒有觸發診斷——代表沒有側向偏移，也就是沒有貼牆）")
        else:
            for t, xte, m in self.diag[:25]:
                print(f"     {t:6.1f}s  偏離{xte*100:5.1f}cm  {m}")
            if len(self.diag) > 25:
                print(f"     …共 {len(self.diag)} 行，其餘見 CSV")


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else "大廳"
    rclpy.init()
    n = Runner()
    ok = False
    try:
        ok = n.run(target)
    finally:
        n.destroy_node(); rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
