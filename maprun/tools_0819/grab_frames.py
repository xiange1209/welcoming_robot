#!/usr/bin/env python3
"""從相機抓幾張有臉的影格存成 JPG，給 register_from_photo.py 用。

★ 為什麼不直接接相機串流註冊
`register_from_photo.py` 吃的是檔案，而且 E1 需要**同一批影像**可以重跑
（換模型、換門檻都要用同一組輸入才有可比性）。存檔比即時擷取可重現。

★ 只存「偵測得到臉」的影格。沒有臉的影格拿去註冊會直接失敗，
  而失敗訊息長得像模型壞掉，很容易誤導。

用法：
    python3 grab_frames.py 王小明 5        # 抓 5 張存到 ~/e1_photos/王小明_*.jpg
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

NAME = sys.argv[1] if len(sys.argv) > 1 else "test"
WANT = int(sys.argv[2]) if len(sys.argv) > 2 else 5
OUT = os.path.expanduser("~/e1_photos")


class Grab(Node):
    def __init__(self, det):
        super().__init__("grab_frames")
        self.det = det
        self.saved = 0
        self.seen = 0
        self.last = 0.0
        os.makedirs(OUT, exist_ok=True)
        self.create_subscription(CompressedImage, "/camera/color/image_raw/compressed",
                                 self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        self.seen += 1
        if self.saved >= WANT or time.time() - self.last < 1.0:
            return
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        faces = self.det.get(img)
        if not faces:
            return
        f = max(faces, key=lambda a: (a.bbox[2]-a.bbox[0]) * (a.bbox[3]-a.bbox[1]))
        w = int(f.bbox[2] - f.bbox[0])
        p = os.path.join(OUT, f"{NAME}_{self.saved+1}.jpg")
        cv2.imwrite(p, img)
        self.saved += 1
        self.last = time.time()
        print(f"  ✓ 第 {self.saved}/{WANT} 張　臉寬 {w} px　det_score {f.det_score:.2f}　-> {p}",
              flush=True)


def main():
    from insightface.app import FaceAnalysis
    print("  載入模型…", flush=True)
    det = FaceAnalysis(name="buffalo_sc", providers=["CPUExecutionProvider"])
    det.prepare(ctx_id=-1)
    rclpy.init()
    n = Grab(det)
    print(f"  請站在相機前，正在抓 {WANT} 張…（最多等 60 秒）", flush=True)
    t0 = time.time()
    while rclpy.ok() and n.saved < WANT and time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.1)
    print(f"\n  收到影格 {n.seen} 張，存下 {n.saved} 張有臉的。")
    if n.saved == 0:
        print("  ✗ 一張都沒抓到臉 —— 站近一點、正對鏡頭、確認光線。")
    n.destroy_node()
    rclpy.shutdown()
    return 0 if n.saved else 1


if __name__ == "__main__":
    sys.exit(main())
