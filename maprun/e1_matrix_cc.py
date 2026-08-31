#!/usr/bin/env python3
"""E1 人臉辨識矩陣：距離 × 角度，每格收 N 幀。

## 為什麼要重收（2026-08-29）

現有 `e1_results.csv` 只有 **26 筆**、只有距離、而且門檻在實驗中途從
0.8 改成 0.70 —— 前後不是同一個實驗。用它算出來的
「2 m 辨識率 0%」只有 6 筆，站不住。

## 這支跟舊做法的差別

1. **每格固定 N 幀**（預設 20），不是「量到哪算哪」
2. **同時記客觀距離**：臉寬反推 + 正前方雷射，兩個獨立來源
   —— 不靠人回報「我站在 1.5 m」
3. **未認出的也記 similarity**：否則事後回答不了「門檻該設多少」
   （舊檔未認出時 confidence 欄仍有值，但沒有記「最像的是誰」）
4. 一格收完會**等你按 Enter** 才收下一格，不會漏拍

## 用法

    # 需要：相機 + face_embedding + user_auth 都在跑
    python3 ~/maprun/e1_matrix_cc.py
    python3 ~/maprun/e1_matrix_cc.py --frames 30
    python3 ~/maprun/e1_matrix_cc.py --conditions "1m_正面,1m_30度"

CSV -> ~/maprun/logs/e1_matrix_<時間>.csv
"""
import argparse
import csv
import math
import pathlib
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from smartnav_msgs.msg import UserIdentity

# 臉寬校正：2026-08-19 實測，使用者站 50 cm 時臉寬 66~74 px（取 69）
# ★ 綁定這顆相機與這個人的臉，是粗估不是量測儀器
CAL_D, CAL_W = 0.50, 69.0
FAN = math.radians(12)          # 正前方 ±12 度扇區

DEFAULT_CONDITIONS = [
    "50cm_正面", "50cm_30度", "50cm_45度",
    "1m_正面", "1m_30度", "1m_45度",
    "1.5m_正面", "1.5m_30度", "1.5m_45度",
    "2m_正面", "2m_30度", "2m_45度",
]


class E1Matrix(Node):
    def __init__(self, frames: int):
        super().__init__("e1_matrix_cc")
        self.frames = frames
        self.rows: list[dict] = []
        self.collecting = False
        self.condition = ""
        self.count = 0
        self.laser_front = float("nan")

        self.create_subscription(UserIdentity, "/user_identity",
                                 self._id_cb, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb,
                                 qos_profile_sensor_data)

    def _scan_cb(self, m: LaserScan) -> None:
        best = float("inf")
        n = len(m.ranges)
        for i in range(n):
            a = m.angle_min + i * m.angle_increment
            if abs(a) > FAN:
                continue
            r = m.ranges[i]
            if m.range_min < r < m.range_max and r < best:
                best = r
        self.laser_front = best if best != float("inf") else float("nan")

    def _id_cb(self, m: UserIdentity) -> None:
        if not self.collecting or self.count >= self.frames:
            return
        w = float(m.bbox[2] - m.bbox[0])
        h = float(m.bbox[3] - m.bbox[1])
        # 沒有臉的訊息（bbox 全 0）不算一幀 —— 否則「沒人」會灌滿樣本
        if w <= 1.0:
            return
        d_face = CAL_D * CAL_W / w if w > 0 else float("nan")
        self.count += 1
        self.rows.append({
            "wall_time": time.strftime("%Y-%m-%d %H:%M:%S"),
            # ★ 2026-08-30 新增（CC-05）：秒級的 wall_time 分不出取樣間隔。
            #   發布端是「同一個 key 節流 2 秒、key 一翻轉立刻發」，所以
            #   間隔約 2.0 s 的是穩定樣本、遠小於 2 s 的是**翻轉爆發樣本**。
            "t_mono": round(time.monotonic(), 3),
            "condition": self.condition,
            "frame": self.count,
            "user_name": m.user_name,
            "user_type": int(m.user_type.type) if hasattr(m.user_type, "type") else -1,
            "similarity": round(float(m.similarity), 4),
            "recognized": int(bool(m.recognized)),
            "face_w_px": round(w, 1),
            "face_h_px": round(h, 1),
            "dist_by_face_m": round(d_face, 3),
            "dist_by_laser_m": round(self.laser_front, 3)
                if self.laser_front == self.laser_front else "",
        })
        rec = "✓" if m.recognized else "✗"
        print(f"    {self.count:2d}/{self.frames}  {rec} {m.user_name:<10s} "
              f"相似度 {m.similarity:.3f}  臉寬 {w:.0f}px "
              f"(≈{d_face:.2f} m)  雷射 {self.laser_front:.2f} m", flush=True)

    def collect(self, cond: str) -> int:
        # ★ 2026-08-30 修正（CC-06）：main() 用 input() 等人站位，那段時間節點
        #   完全不 spin，/user_identity 的訊息在 DDS 收端佇列累積（KEEP_LAST
        #   depth 10）。舊版一進來就 collecting=True，於是**上一格站位或走位
        #   途中的最多 10 則舊訊息會被記成新情境**（20 幀裡佔一半），
        #   剛好把「距離 x 辨識率」的曲線抹平 —— 而那正是 E1 要量的東西。
        #   （bbox 全 0 的「無人」訊息會被 _id_cb 濾掉，但走位途中臉還在畫面裡。）
        #   先關著閘門空轉把佇列排乾，再開始收。
        self.condition = cond
        self.count = 0
        self.collecting = False
        _t_drain = time.time()
        while time.time() - _t_drain < 0.4:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.collecting = True
        t0 = time.time()
        while self.count < self.frames and time.time() - t0 < 90.0:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.collecting = False
        return self.count


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--conditions", default="")
    a = ap.parse_args()
    conds = ([c.strip() for c in a.conditions.split(",") if c.strip()]
             if a.conditions else DEFAULT_CONDITIONS)

    rclpy.init()
    n = E1Matrix(a.frames)
    print("等 /user_identity …（需要 相機 + face_embedding + user_auth）")
    t0 = time.time()
    while time.time() - t0 < 15.0 and n.count_publishers("/user_identity") == 0:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.count_publishers("/user_identity") == 0:
        print("✗ /user_identity 沒有發布端 —— user_auth_node 沒在跑", file=sys.stderr)
        rclpy.shutdown()
        return 1

    print(f"共 {len(conds)} 格，每格 {a.frames} 幀。\n")
    # ★ 2026-08-30（CC-05）：這裡的「幀」是 /user_identity 的**訊息**，不是影像幀。
    print("  ⚠ 取樣偏差警告：/user_identity 的發布端有節流"
          "（user_auth_node.py:694，identity_publish_interval 預設 2.0 s）——")
    print("    同一個判定結果 2 秒才發一則，但**判定一翻轉就立刻發**（相機 15 FPS）。")
    print("    於是「翻轉事件」被過度取樣，而翻轉依定義認出/未認出各半，")
    print("    **每一格的認出率都會被系統性拉向 50%**。")
    print("    要拿到逐幀樣本，重收時必須用 launch 參數覆寫：")
    print("      identity_publish_interval:=0.0   ← ★ 在 __init__ 就讀進去了，")
    print("                                          ros2 param set 不會生效（鐵則 2）")
    print("    不重收的話，事後用 CSV 的 t_mono 欄把間隔遠小於 2 s 的列標成")
    print("    「翻轉樣本」分開統計，才知道真實的穩定辨識率。")
    try:
        for i, c in enumerate(conds, 1):
            ans = input(f"[{i}/{len(conds)}] 請站到 **{c}**，"
                        f"站好按 Enter（跳過打 s、結束打 q）> ").strip().lower()
            if ans == "q":
                print("  （提前結束，已收的資料照樣存檔）")
                break
            if ans == "s":
                print(f"  -> 跳過 {c}\n")
                continue
            got = n.collect(c)
            print(f"  -> {c} 收到 {got}/{a.frames} 幀"
                  f"{'  ⚠ 逾時，可能沒偵測到臉' if got < a.frames else ''}\n")
    except (KeyboardInterrupt, EOFError):
        print("\n(中斷)")
    finally:
        d = pathlib.Path.home() / "maprun" / "logs"
        d.mkdir(parents=True, exist_ok=True)
        p = d / f"e1_matrix_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        if n.rows:
            with p.open("w", newline="", encoding="utf-8-sig") as fh:
                w = csv.DictWriter(fh, fieldnames=list(n.rows[0].keys()))
                w.writeheader()
                w.writerows(n.rows)
            print(f"\nCSV -> {p}  （{len(n.rows)} 筆）")
            # 即時摘要
            from collections import defaultdict
            g = defaultdict(list)
            for r in n.rows:
                g[r["condition"]].append(r)
            print(f"\n{'情境':<14}{'n':>4}{'平均相似度':>10}{'認出率':>8}{'臉寬px':>8}{'雷射m':>8}")
            for c, rs in g.items():
                sims = [r["similarity"] for r in rs]
                rec = sum(r["recognized"] for r in rs)
                ws = [r["face_w_px"] for r in rs]
                ls = [r["dist_by_laser_m"] for r in rs if r["dist_by_laser_m"] != ""]
                print(f"{c:<14}{len(rs):>4}{sum(sims)/len(sims):>10.3f}"
                      f"{rec/len(rs)*100:>7.0f}%{sum(ws)/len(ws):>8.0f}"
                      f"{(sum(ls)/len(ls) if ls else float('nan')):>8.2f}")
        else:
            print("（沒有收到任何資料）")
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
