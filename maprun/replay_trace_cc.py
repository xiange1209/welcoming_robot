#!/usr/bin/env python3
"""重播教導路徑並逐筆記錄 feedback —— 給 A-2「為什麼會偏」用。

## 這支解決的問題

8/27 兩次「大廳→起點」失敗、一次「起點→大廳」失敗，訊號都是
`側淨空 0.00 m`，但當下**分不出兩件事**：

    ① 路徑本身就經過過不去的地方（那要重錄路徑）
    ② 車偏離了路徑（那是循跡問題，換場地照樣會發生）

action 的 feedback 裡有 `cross_track_error_m`（橫向偏離），
把它跟 `point_index` 一起存下來就能直接分辨：
索引 60 附近若 cross_track 很小 -> 是①；很大 -> 是②。

## 用法

    python3 ~/maprun/replay_trace_cc.py path_75c58190328b
    python3 ~/maprun/replay_trace_cc.py path_75c58190328b --reverse
    python3 ~/maprun/replay_trace_cc.py path_75c58190328b --speed 0.7

CSV 存到 ~/maprun/logs/replay_<path_id>_<時間>.csv
"""
import argparse
import csv
import math
import pathlib
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from smartnav_msgs.action import FollowTaughtPath


class ReplayTracer(Node):
    def __init__(self, path_id: str, reverse: bool, speed: float, watch_index=None):
        super().__init__("replay_tracer_cc")
        self.path_id = path_id
        # ★ 2026-08-30（CC-14）：要盯的索引改成可選，不再寫死家裡那條路徑的 60
        self.watch_index = watch_index
        self.reverse = reverse
        self.speed = speed
        self.rows: list[dict] = []
        self.t0 = time.monotonic()
        self.done = False
        self.result = None

        # AMCL 位置：用來回答「車實際在哪」，不只是「偏離多少」
        self.pose = (float("nan"), float("nan"), float("nan"))
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._pose_cb, 10)

        self.client = ActionClient(self, FollowTaughtPath, "/follow_taught_path")

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.position.x, p.position.y, math.degrees(yaw))

    def _feedback_cb(self, fb) -> None:
        f = fb.feedback
        x, y, yaw = self.pose
        self.rows.append({
            "t": round(time.monotonic() - self.t0, 2),
            "index": f.point_index,
            "total": f.total_points,
            "progress": round(f.progress, 4),
            "cross_track_m": round(f.cross_track_error_m, 4),
            "state": f.state,
            "amcl_x": round(x, 3),
            "amcl_y": round(y, 3),
            "amcl_yaw_deg": round(yaw, 1),
            "message": f.message,
        })
        # 只在索引變動或狀態改變時印，免得洗版
        if len(self.rows) == 1 or self.rows[-1]["index"] != self.rows[-2]["index"] \
                or self.rows[-1]["state"] != self.rows[-2]["state"]:
            print(f"  [{f.point_index:3d}/{f.total_points}] "
                  f"偏離 {f.cross_track_error_m:+.3f} m  {f.state:<10s} "
                  f"({x:.2f}, {y:.2f}) {yaw:+.0f}°  {f.message}", flush=True)

    def run(self) -> int:
        print(f"等待 /follow_taught_path …", flush=True)
        if not self.client.wait_for_server(timeout_sec=20.0):
            print("✗ action server 沒出現", file=sys.stderr)
            return 1

        goal = FollowTaughtPath.Goal()
        goal.path_id = self.path_id
        goal.reverse = self.reverse
        goal.speed_scale = self.speed
        print(f"送出：{self.path_id} reverse={self.reverse} speed_scale={self.speed}",
              flush=True)

        fut = self.client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print("✗ goal 被拒絕", file=sys.stderr)
            return 1
        # ★ 2026-08-30 修正（CC-14）：舊版無條件印「盯著索引 60 附近」——
        #   那是家裡那條路徑的門口位置，換場地後毫無意義而且會誤導判讀。
        print("goal 已接受，開始走。", flush=True)
        if self.watch_index is not None:
            print(f"★ 盯著索引 {self.watch_index} 附近，看車實際在哪。", flush=True)
        print(flush=True)

        rfut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)
        self.result = rfut.result().result
        return 0

    def save(self) -> pathlib.Path:
        d = pathlib.Path.home() / "maprun" / "logs"
        d.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        tag = "rev" if self.reverse else "fwd"
        p = d / f"replay_{self.path_id}_{tag}_{stamp}.csv"
        if self.rows:
            with p.open("w", newline="", encoding="utf-8-sig") as fh:
                w = csv.DictWriter(fh, fieldnames=list(self.rows[0].keys()))
                w.writeheader()
                w.writerows(self.rows)
        return p


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("path_id")
    ap.add_argument("--reverse", action="store_true")
    ap.add_argument("--speed", type=float, default=0.0,
                    help="speed_scale，0 = 用預設速度")
    a = ap.parse_args()

    rclpy.init()
    n = ReplayTracer(a.path_id, a.reverse, a.speed)
    rc = 0
    try:
        rc = n.run()
    except KeyboardInterrupt:
        print("\n(中斷)")
    finally:
        p = n.save()
        r = n.result
        print()
        if r is not None:
            print(f"結果：success={r.success}  {r.message}")
            print(f"      終點位置誤差 {r.final_error_m:.3f} m、"
                  f"朝向誤差 {r.final_yaw_error_deg:+.1f}°")
        if n.rows:
            # ★ 2026-08-30 修正（CC-04）：cross_track_m 這一欄在來源端混了三種物理量——
            #   following/slowing/avoiding 才是真的橫向偏離；
            #   escaping 七個呼叫點全部硬傳 0.0（會把平均往下拉）；
            #   blocked 傳的是 dir_margin＝車身外緣淨空（公尺），根本不是偏離。
            #   舊版沒有任何 state 過濾就三者混算，而報告的 0.175 m 就是這樣算出來的。
            #   脫困佔全程 22~62%，污染量很大。以下只用真正的循跡列。
            TRACK_STATES = ("following", "slowing", "avoiding")
            track = [x for x in n.rows if x["state"] in TRACK_STATES]
            ct = sorted(abs(x["cross_track_m"]) for x in track)
            n_skip = len(n.rows) - len(ct)
            if ct:
                print(f"      橫向偏離（只計 {'/'.join(TRACK_STATES)}，{len(ct)} 筆）："
                      f"平均 {sum(ct)/len(ct):.3f} m、"
                      f"最大 {ct[-1]:.3f} m、95% {ct[int(len(ct)*0.95)]:.3f} m")
                # ★ 2026-08-30 新增（CC-12）：上面那組是**時間加權**——每個控制
                #   週期一列，所以車子慢下來或停住的地方會被重複取樣。而慢下來
                #   正好就是循跡吃力的地方，分佈會被那幾個點主導。
                #   報告要的是「沿路徑的偏離分佈」＝**距離加權**。路徑點大致等距，
                #   所以先把同一索引的樣本平均掉再對索引統計，就是乾淨的距離加權。
                per_idx = {}
                for _x in track:
                    per_idx.setdefault(_x["index"], []).append(abs(_x["cross_track_m"]))
                di = sorted(sum(v) / len(v) for v in per_idx.values())
                print(f"      ★ 距離加權（{len(di)} 個路徑索引）："
                      f"平均 {sum(di)/len(di):.3f} m、"
                      f"最大 {di[-1]:.3f} m、95% {di[int(len(di)*0.95)]:.3f} m")
                print(f"        ★ 寫報告用這一組。時間加權那組每索引平均"
                      f" {len(ct)/max(1,len(di)):.1f} 筆，重複取樣愈多偏差愈大")
            else:
                print("      橫向偏離：無有效取樣（全程都在 escaping/blocked）")
            if n_skip:
                print(f"      ★ 已排除 {n_skip} 筆非循跡列"
                      f"（escaping 硬傳 0.0、blocked 傳的是車身淨空）——"
                      f"舊版把它們一起算進去，數字會偏低")
            print(f"      最遠索引 {max(x['index'] for x in n.rows)}"
                  f" / {n.rows[-1]['total']}")
        print(f"CSV -> {p}  （{len(n.rows)} 筆）")
        n.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
