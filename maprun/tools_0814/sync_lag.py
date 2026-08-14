#!/usr/bin/env python3
"""量 face_embedding 的時間戳落後最新影像多久，並模擬 message_filters 的固定佇列。

★ 2026-08-14 這支是為了驗證一個「優化造成迴歸」的假設：

  `user_auth_node.py:143` 的 ApproximateTimeSynchronizer 是 `queue_size=10`、`slop=0.1`。
  佇列是**固定格數**，所以它涵蓋的時間長度 = 10 / 影像速率：

      影像 6.4 Hz（深度開著時） -> 10 格 = 1.56 秒   > 推論延遲 -> 配得到
      影像  25 Hz（只開彩色）   -> 10 格 = 0.40 秒   < 推論延遲 -> 永遠配不到

  也就是說**把相機加速反而讓人臉辨識完全失效**，而且不報錯：
  `/user_identity` 靜默、註冊收 0/10 張、20 秒後把剛建好的使用者刪掉。

★ 我第一次量「同步命中 97%」是錯的：我拿每則向量去比對整個視窗裡**所有**影像，
  沒有模擬固定格數的滑動佇列。要驗佇列問題就必須模擬佇列。
"""
import sys, time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from smartnav_msgs.msg import FaceEmbedding

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
QSIZE = 10          # 跟 user_auth_node.py:145 一樣
SLOP = 0.1          # 跟 :146 一樣


def stamp(m):
    h = m.header
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Lag(Node):
    def __init__(self):
        super().__init__("sync_lag")
        self.q = deque(maxlen=QSIZE)      # 模擬 message_filters 的固定佇列
        self.img_n = 0
        self.lags = []
        self.hit = 0
        self.miss = 0
        self.create_subscription(CompressedImage, "/camera/color/image_raw/compressed",
                                 self._img, qos_profile_sensor_data)
        self.create_subscription(FaceEmbedding, "face_embedding", self._emb, 10)

    def _img(self, m):
        self.img_n += 1
        self.q.append(stamp(m))

    def _emb(self, m):
        e = stamp(m)
        if not self.q:
            return
        self.lags.append(self.q[-1] - e)          # 最新影像領先向量多久
        if any(abs(t - e) <= SLOP for t in self.q):
            self.hit += 1
        else:
            self.miss += 1


def main():
    rclpy.init(); n = Lag()
    print(f"  量 {DUR:.0f} 秒（佇列 {QSIZE} 格、slop {SLOP}s，與程式相同）—— 請站在鏡頭前\n")
    t0 = time.time()
    while time.time() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.05)
    dt = time.time() - t0

    rate = n.img_n / dt
    print(f"  影像速率        {rate:6.2f} Hz")
    print(f"  佇列涵蓋時間    {QSIZE/rate if rate else 0:6.2f} 秒  （{QSIZE} 格 ÷ 速率）")
    if n.lags:
        n.lags.sort()
        med = n.lags[len(n.lags)//2]
        print(f"  向量落後最新影像 {med:6.2f} 秒  （中位數，{len(n.lags)} 則）")
        print(f"  模擬同步        命中 {n.hit} / 錯過 {n.miss}")
        print()
        if rate and med > QSIZE / rate:
            print(f"  ★★ 落後 {med:.2f}s > 佇列涵蓋 {QSIZE/rate:.2f}s —— 配不到對是必然的")
            print(f"     解法：把影像降到 {QSIZE/med:.1f} Hz 以下，或把 queue_size 加大到 "
                  f"{int(med*rate)+5} 以上")
        else:
            print("  ★ 佇列涵蓋得住延遲，同步失敗要往別的方向查")
    else:
        print("  ✗ 一則向量都沒有 —— 鏡頭前沒人，或 face_embedding 沒在跑")
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
