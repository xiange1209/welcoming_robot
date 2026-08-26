#!/usr/bin/env python3
"""雷達盲區檢查 —— 分辨「後方空曠」與「後方看不見」

## 為什麼需要這支（2026-08-25）

使用者回報「**倒車一直撞牆然後還繼續移動沒有停**」。

查程式碼發現一個 fail-open：`_arc_clearance()` 對無效回波是 `continue`，
掃完沒有點擋住就 `return max_dist`（＝淨空）。`_body_margin()` 同理，
沒有點就回 `inf`。也就是說——

    **「沒有資料」與「沒有障礙」在避障邏輯裡是同一個結果。**

若雷達後方扇區被車身、電子盒或線材遮住，倒車時三個保護
（`_body_margin` 硬停、`_forward_clearance`、`_arc_clearance` 弧線預測）
全部認為安全，車子就一路倒到撞牆為止。

★ 而既有的量測**無法區分這兩件事**：8/24 記錄的
「122 幀、0 筆 < 0.36 m」對「空曠」與「盲區」會給出一模一樣的數字。

## 這支怎麼分辨

看的不是距離，是**每個扇區有幾筆有效回波**。

    後方扇區有效回波 ≈ 0，而其他扇區正常   -> **盲區**（危險）
    每個扇區都有回波，只是後方比較遠       -> 空曠（正常）

★ 走廊裡任何一個方向都不該是 0 —— 四面都有牆。
   在空曠大廳量的話，請把車子開到牆邊再量。

## 用法

    python3 ~/maprun/tools_0825/rear_blind_check.py            # 量 8 秒
    python3 ~/maprun/tools_0825/rear_blind_check.py 15         # 量 15 秒

★ 量的時候**車子靜止、人離開車子兩側各 1 公尺**（人會被算成障礙）。
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0

# 扇區以雷達為原點、車頭為 0 度、逆時針為正
SECTORS = [
    ("正前", -20, 20),
    ("左前", 20, 70),
    ("左側", 70, 110),
    ("左後", 110, 160),
    ("正後", 160, 180),      # 與下面那段合起來是 ±180
    ("正後2", -180, -160),
    ("右後", -160, -110),
    ("右側", -110, -70),
    ("右前", -70, -20),
]


class BlindCheck(Node):
    def __init__(self):
        super().__init__("rear_blind_check")
        self.valid = {name: 0 for name, _, _ in SECTORS}
        self.total = {name: 0 for name, _, _ in SECTORS}
        self.nearest = {name: float("inf") for name, _, _ in SECTORS}
        self.frames = 0
        self.range_min = None
        self.range_max = None
        # ★ 用 BEST_EFFORT 訂閱：它對 RELIABLE 與 BEST_EFFORT 兩種發布端都通。
        #
        # ★★ 2026-08-25 更正：本檔原本寫「雷達是 BEST_EFFORT，用 RELIABLE
        #    會完全收不到」——**那句話是錯的**。查證過兩個發布端不一樣：
        #
        #      /scan       lslidar_x10_driver.cpp:171
        #                  create_publisher<LaserScan>(san_topic_, 10)  -> RELIABLE
        #      /scan_slam  scan_filter_cc_node.py:71
        #                  qos_profile_sensor_data                      -> BEST_EFFORT
        #
        #    DDS 的規則是「發布端提供的必須 ≥ 訂閱端要求的」，所以
        #    RELIABLE 訂閱 /scan 沒問題（8/24 的 tools_0824/sectors.py 就是
        #    用預設 RELIABLE 收到 121 幀的，那是這件事的直接反證），
        #    但 RELIABLE 訂閱 /scan_slam 會靜靜地一則都收不到。
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(LaserScan, "/scan", self._cb, qos)
        self.get_logger().info(
            f"量 {SECS:.0f} 秒。★ 車子靜止、人離開車子兩側各 1 公尺")

    def _cb(self, m: LaserScan) -> None:
        self.frames += 1
        self.range_min, self.range_max = m.range_min, m.range_max
        ang = m.angle_min
        for r in m.ranges:
            deg = math.degrees(ang)
            ang += m.angle_increment
            # 收斂到 (-180, 180]
            while deg > 180.0:
                deg -= 360.0
            while deg <= -180.0:
                deg += 360.0
            for name, lo, hi in SECTORS:
                if lo <= deg < hi:
                    self.total[name] += 1
                    # ★ 這一行必須與 _arc_clearance / _body_margin 的判斷**完全一致**，
                    #   否則量到的不是它們看到的世界。
                    if (m.range_min <= r <= m.range_max) and r == r:
                        self.valid[name] += 1
                        if r < self.nearest[name]:
                            self.nearest[name] = r
                    break

    def report(self) -> None:
        if not self.frames:
            print("★ 一幀都沒收到。檢查：雷達有沒有開、話題是不是 /scan、"
                  "QoS 是不是 BEST_EFFORT")
            return
        # 正後兩段合併
        for k in ("valid", "total"):
            d = getattr(self, k)
            d["正後"] += d.pop("正後2")
        if self.nearest["正後2"] < self.nearest["正後"]:
            self.nearest["正後"] = self.nearest["正後2"]
        self.nearest.pop("正後2")
        order = ["正前", "左前", "左側", "左後", "正後", "右後", "右側", "右前"]

        print()
        print(f"=== {self.frames} 幀　range_min={self.range_min:.3f} "
              f"range_max={self.range_max:.1f} ===")
        print()
        print("| 扇區 | 有效回波 | 總束數 | 有效率 | 最近 |")
        print("|---|---|---|---|---|")
        rates = {}
        for name in order:
            v, t = self.valid[name], self.total[name]
            rate = (100.0 * v / t) if t else 0.0
            rates[name] = rate
            near = self.nearest[name]
            print("| %s | %d | %d | %.1f%% | %s |" % (
                name, v, t, rate,
                ("%.2f m" % near) if near != float("inf") else "—"))

        print()
        others = [rates[n] for n in order if n != "正後"]
        med_other = sorted(others)[len(others) // 2]
        rear = rates["正後"]
        print("正後有效率 %.1f%%　其他扇區中位數 %.1f%%" % (rear, med_other))
        print()
        if rear < 5.0 and med_other > 30.0:
            print("★★ 判定：**後方是盲區**。")
            print("   倒車時 _arc_clearance 會回傳「淨空」而其實看不見，")
            print("   三個保護全部失效 —— 這就是「倒車撞牆不停」的原因。")
            print("   → 檢查雷達後方有沒有被車身/電子盒/線材遮住，")
            print("     或 scan_filter 是不是把遮罩套在 /scan 上（應該只套 /scan_slam）。")
        elif rear < 30.0 and med_other > 60.0:
            print("★ 判定：後方**回波偏少**，不一定是全盲但值得檢查遮擋。")
        elif all(r < 5.0 for r in rates.values()):
            print("★★ 判定：**所有扇區都沒有有效回波**。雷達沒在轉、或 range 設定不對。")
        else:
            print("✓ 判定：後方看得見。倒車撞牆的原因**不在感測**，")
            print("  往控制端查（脫困裸奔、速度下限、死區）。")


def main():
    rclpy.init()
    node = BlindCheck()
    import time
    t0 = time.time()
    try:
        while rclpy.ok() and time.time() - t0 < SECS:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
