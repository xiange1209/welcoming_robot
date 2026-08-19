#!/usr/bin/env python3
"""同時用「臉寬」與「雷射」估訪客距離，兩個獨立來源互相對照。

★ 為什麼要兩個來源
深度相機關掉了（只開彩色，開深度要多吃 100% CPU），所以沒有直接的深度。
單靠臉寬反推會被「臉大小因人而異」影響；單靠雷射會把椅子、牆當成人。
兩個一起看就分得出來：**數字接近 = 可信；差很多 = 其中一個看到的不是人**。

臉寬法：針孔模型下 距離 ∝ 1/臉寬。校正點是 2026-08-19 實測
（使用者站 50 cm 時臉寬 66~74 px，取平均 69）：

    d ≈ 0.50 m × 69 / 臉寬px

★ 這個校正**綁定這顆相機與這個人的臉**。換人臉寬會變（成人臉寬差異約 ±10%），
  所以它是「粗估」不是量測儀器。真正的用途是**判斷實驗當下的距離對不對**，
  以及在 E1 記錄裡附上一個客觀欄位，而不是靠人回報。

雷射法：取正前方 ±12 度扇區的最小距離。★ 雷射在 base_footprint 前方 0.089 m，
而人站在車頭前方，所以回傳值已經是「離車頭保險桿」的距離再加 0.311 m 的車身
—— 這裡直接印原始雷射距離，不做車身補償，避免又introduce一個沒驗證過的常數。

用法：
    python3 face_distance.py          # 量 10 秒
    python3 face_distance.py 20       # 量 20 秒
"""
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
CAL_D, CAL_W = 0.50, 69.0        # 校正點：50 cm 時臉寬 69 px
FAN = math.radians(12)


class Dist(Node):
    def __init__(self, det):
        super().__init__("face_distance")
        self.det = det
        self.front = None
        self.rows = []
        self.last = 0.0
        self.create_subscription(CompressedImage, "/camera/color/image_raw/compressed",
                                 self._img, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)

    def _scan(self, m):
        best = None
        for i, r in enumerate(m.ranges):
            a = m.angle_min + i * m.angle_increment
            if -FAN <= a <= FAN and r > 0.05 and not math.isinf(r) and not math.isnan(r):
                best = r if best is None else min(best, r)
        self.front = best

    def _img(self, msg):
        if time.time() - self.last < 1.0:
            return
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        faces = self.det.get(img)
        self.last = time.time()
        if not faces:
            print(f"  臉：偵測不到          雷射前方 "
                  f"{'—' if self.front is None else f'{self.front:.2f} m'}", flush=True)
            return
        f = max(faces, key=lambda a: (a.bbox[2]-a.bbox[0]) * (a.bbox[3]-a.bbox[1]))
        w = f.bbox[2] - f.bbox[0]
        d_face = CAL_D * CAL_W / w
        self.rows.append((d_face, self.front, w, float(f.det_score)))
        lz = "—" if self.front is None else f"{self.front:.2f} m"
        diff = "" if self.front is None else f"　差 {abs(d_face - self.front):+.2f} m"
        print(f"  臉寬 {w:5.1f} px -> {d_face:.2f} m　信心 {f.det_score:.2f}"
              f"　雷射前方 {lz}{diff}", flush=True)


def main():
    from insightface.app import FaceAnalysis
    det = FaceAnalysis(name="buffalo_sc", providers=["CPUExecutionProvider"])
    det.prepare(ctx_id=-1)
    rclpy.init()
    n = Dist(det)
    print(f"  量 {DUR:.0f} 秒　（臉寬校正：50 cm 時 69 px）", flush=True)
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.rows:
        fd = [r[0] for r in n.rows]
        lz = [r[1] for r in n.rows if r[1] is not None]
        print(f"\n  臉寬估距　{len(fd)} 筆　中位數 {sorted(fd)[len(fd)//2]:.2f} m"
              f"　範圍 {min(fd):.2f}~{max(fd):.2f} m")
        if lz:
            print(f"  雷射前方　{len(lz)} 筆　中位數 {sorted(lz)[len(lz)//2]:.2f} m"
                  f"　範圍 {min(lz):.2f}~{max(lz):.2f} m")
    else:
        print("\n  ✗ 全程沒偵測到臉")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
