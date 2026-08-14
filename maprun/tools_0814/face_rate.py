#!/usr/bin/env python3
"""量人臉管線每一段的速率，找出註冊採樣的真正瓶頸。

★ 2026-08-14 背景
`user_auth_node.py:214` 是寫死的 `threading.Timer(20.0, ...)`：
20 秒內收不滿 `num_samples` 張就**把剛註冊好的使用者刪掉**。
實測連續三次都在註冊後正好 20.0 秒被回滾。

樣本是在**同步回呼**裡收的（FaceEmbedding + CompressedImage 時間戳要對得上），
所以真正的上限是三者的最小值：

    相機壓縮影像速率   （實測 29 Hz，不是瓶頸）
    face_embedding 速率（InsightFace CPU 推論，← 最可能的瓶頸）
    同步成功率         （時間戳對不上就整批丟掉）

這支同時量前兩項，並用「同步窗」估第三項。
"""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from smartnav_msgs.msg import FaceEmbedding

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
WIN = 0.10          # 同步窗（秒），估計用


class Rate(Node):
    def __init__(self):
        super().__init__("face_rate")
        self.img_t = []
        self.emb_t = []
        self.create_subscription(CompressedImage, "/camera/color/image_raw/compressed",
                                 self._img, qos_profile_sensor_data)
        self.create_subscription(FaceEmbedding, "face_embedding", self._emb, 10)

    @staticmethod
    def _stamp(m):
        h = getattr(m, "header", None)
        if h is None:
            return None
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    def _img(self, m):
        self.img_t.append(self._stamp(m))

    def _emb(self, m):
        self.emb_t.append(self._stamp(m))


def main():
    rclpy.init(); r = Rate()
    print(f"  量 {DUR:.0f} 秒 —— 請站在鏡頭前\n")
    t0 = time.time()
    while time.time() - t0 < DUR:
        rclpy.spin_once(r, timeout_sec=0.05)
    dt = time.time() - t0

    ni, ne = len(r.img_t), len(r.emb_t)
    print(f"  壓縮影像      {ni:4d} 則  {ni/dt:6.2f} Hz")
    print(f"  face_embedding {ne:4d} 則  {ne/dt:6.2f} Hz"
          f"{'   ★ 一則都沒有 = 沒偵測到人臉' if ne == 0 else ''}")

    if ne:
        # 每則 embedding 找得到時間戳夠近的影像嗎
        imgs = sorted(t for t in r.img_t if t is not None)
        embs = sorted(t for t in r.emb_t if t is not None)
        if imgs and embs:
            import bisect
            hit = 0
            for e in embs:
                i = bisect.bisect_left(imgs, e)
                for j in (i - 1, i):
                    if 0 <= j < len(imgs) and abs(imgs[j] - e) <= WIN:
                        hit += 1
                        break
            print(f"  同步命中      {hit}/{len(embs)}  （時間戳差 ≤ {WIN*1000:.0f} ms）")
            eff = hit / dt
            print(f"\n  → 有效採樣速率 {eff:.2f} 張/秒")
            print(f"  → 20 秒內收得到 {eff*20:.1f} 張"
                  f"{'　✓ 夠 10 張' if eff*20 >= 10 else '　✗ 收不滿 10 張，註冊必被回滾'}")
        else:
            print("  （訊息沒有 header 時間戳，估不了同步率）")
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
