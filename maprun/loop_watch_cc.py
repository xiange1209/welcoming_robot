#!/usr/bin/env python3
"""建圖時數迴路閉合次數。

用法（建圖開始後另開一個終端）：
    nice -n 15 python3 ~/maprun/loop_watch_cc.py [分鐘數]

為什麼看 map -> odom 而不是看 log：
    slam_toolbox 沒有「閉合次數」的話題，各版本的 log 字串也不一樣。
    但**迴路閉合的定義**就是後端重新求解位姿圖、把 map -> odom 修正掉一段。
    所以「這條 TF 出現階躍」就是閉合發生了，這個訊號跟版本無關。
    slam_mapping_cc.yaml 裡「全程最大修正 0.198 m」就是同一個量。

★ 為什麼要數：2026-07-31 把門檻收到 chain 15 / fine 0.65，結果**一次閉合
  都沒觸發** —— 去程與回程變成兩段互不相干的軌跡，疊不起來，地圖出現「鬼牆」
  畫在空地上，離車子 5 cm、落在 inscribed_radius 內，global_costmap 把車子
  那格標成 253，規劃器回 "Start occupied"，四個導航目標全失敗。
  目前值 chain 14 / coarse 0.450 / fine 0.580 距離那組失敗值只差一點，
  而且是三項同時收緊 —— 沒有經過建圖驗證。

  **跑完一趟如果閉合 0 次，立刻停下來退回 chain 12 / fine 0.52，一項一項調。**
  不要在沒觀察的情況下做長距離建圖。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

# 小於這個量的變化當成求解器的連續微調，不算一次閉合。
# 取 2 cm：AMCL/slam 的逐次修正通常在毫米級，閉合則是公分到公尺級。
JUMP_M = 0.02
JUMP_DEG = 1.0


def main() -> None:
    minutes = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = rclpy.create_node("loop_watch_cc")
    buf = Buffer()
    TransformListener(buf, node)

    print(f"監看 map -> odom_combined 的階躍（門檻 {JUMP_M * 100:.0f} cm / {JUMP_DEG:.0f} 度），"
          f"共 {minutes:.0f} 分鐘")
    print("時刻       位移修正   角度修正   累計閉合   備註", flush=True)

    last = None
    count = 0
    biggest = 0.0
    t_end = time.time() + minutes * 60.0
    t_report = time.time() + 60.0

    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tr = buf.lookup_transform("map", "odom_combined", rclpy.time.Time())
        except Exception:
            continue
        t = tr.transform.translation
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)
        cur = (t.x, t.y, yaw)

        if last is not None:
            d = math.hypot(cur[0] - last[0], cur[1] - last[1])
            dth = abs((cur[2] - last[2] + math.pi) % (2 * math.pi) - math.pi)
            if d > JUMP_M or math.degrees(dth) > JUMP_DEG:
                count += 1
                biggest = max(biggest, d)
                print(f"{time.strftime('%H:%M:%S')}  {d * 100:7.1f} cm  "
                      f"{math.degrees(dth):7.1f}°   {count:6d}     ← 迴路閉合", flush=True)
        last = cur

        if time.time() >= t_report:
            t_report += 60.0
            note = "★ 一次都沒閉合 —— 門檻可能太嚴，見檔頭說明" if count == 0 else ""
            print(f"{time.strftime('%H:%M:%S')}  --- 每分鐘回報：累計 {count} 次閉合、"
                  f"最大修正 {biggest * 100:.1f} cm {note}", flush=True)

    print(f"\n=== 結束：共 {count} 次閉合，最大單次修正 {biggest * 100:.1f} cm ===")
    if count == 0:
        print("★ 閉合 0 次。退回 loop_match_minimum_chain_size: 12、")
        print("  loop_match_minimum_response_fine: 0.52，一次只改一項再測。")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
