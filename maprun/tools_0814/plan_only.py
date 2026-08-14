#!/usr/bin/env python3
"""只規劃、不執行：確認路線 B（教導-重現）在目前設定下規得出路徑。

★ 2026-08-14 為什麼要跑這支
我把 inflation_radius 從 0.35/0.22 提高到 0.55（實測讓 MPPI 的控制率
6.14 -> 9.01 Hz、速度從爬行變滿速）。但**規劃器用的是同一張成本地圖**，
而 0.55 m 的膨脹在 0.99~1.41 m 的走廊裡等於整條走廊都有非零成本。

路線 B 是主力，不能為了救備援的 MPPI 而把主力弄壞。**車子不會動。**
"""
import sys, time
import rclpy
from rclpy.node import Node
from smartnav_msgs.srv import PlanTaughtPath, ListWaypoints

GOALS = sys.argv[1:] or ["大廳", "起點"]


class Plan(Node):
    def __init__(self):
        super().__init__("plan_only")
        self.wp = self.create_client(ListWaypoints, "list_waypoints")
        self.plan = self.create_client(PlanTaughtPath, "plan_taught_path")

    def call(self, cli, req, name, t=150.0):
        if not cli.wait_for_service(timeout_sec=12.0):
            print(f"  ✗ {name} 沒回應"); return None
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=t)
        return f.result()


def main():
    rclpy.init(); n = Plan()
    wr = n.call(n.wp, ListWaypoints.Request(), "list_waypoints")
    if wr is None:
        return 1
    byname = {w.waypoint_name: w for w in wr.waypoints_info}

    print(f"  {'目標':>6} {'點數':>5} {'長度':>8} {'折返':>5}  結果")
    print("  " + "-" * 56)
    rc = 0
    for g in GOALS:
        if g not in byname:
            print(f"  {g:>6}   —      —      —   ✗ 地圖裡沒有這個地點")
            rc = 1
            continue
        r = PlanTaughtPath.Request()
        r.name = f"_planonly_{g}_{time.strftime('%H%M%S')}"
        r.start_from_robot = True
        r.waypoints = [byname[g].pose]
        res = n.call(n.plan, r, "plan_taught_path")
        if res is None or not res.success:
            print(f"  {g:>6}   —      —      —   ✗ {(res.message if res else '無回應')[:34]}")
            rc = 1
            continue
        print(f"  {g:>6} {res.num_points:5d} {res.length_m:7.2f}m {res.num_cusps:5d}  ✓")
    print("\n  ★ 只做規劃，車子沒有動過。")
    print("  ★ 可行性警告（「有 N 處車子做不到」）要看 ~/maprun/logs/nav_cc.log")
    rclpy.try_shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
