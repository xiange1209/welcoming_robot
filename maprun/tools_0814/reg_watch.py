#!/usr/bin/env python3
"""一邊註冊、一邊盯 registration_progress，看實際收到幾張樣本。

★ 2026-08-14：註冊連續五次都在 20 秒後被回滾（`user_auth_node.py:214` 寫死
  `threading.Timer(20.0, ...)`，收不滿 num_samples 就刪掉剛建好的使用者）。
  已排除的原因：相機速率（22.66 Hz）、face_embedding 速率（0.90 Hz）、
  同步命中率（17/18 = 94%）、使用者離開鏡頭（使用者說全程都在）。
  所以要直接看「採樣回呼到底有沒有被呼叫到」，不要再從外圍推。
"""
import sys, threading, time
import rclpy
from rclpy.node import Node
from smartnav_msgs.msg import RegistrationProgress
from smartnav_msgs.srv import RegisterFace
from smartnav_msgs.msg import UserType

NAME = sys.argv[1] if len(sys.argv) > 1 else "陳佳憲"
N = int(sys.argv[2]) if len(sys.argv) > 2 else 10


class Watch(Node):
    def __init__(self):
        super().__init__("reg_watch")
        self.t0 = time.time()
        self.create_subscription(RegistrationProgress, "registration_progress", self._p, 10)
        self.cli = self.create_client(RegisterFace, "register_face")

    def _p(self, m):
        print(f"  [{time.time()-self.t0:5.1f}s] status={m.status!r:12s} "
              f"{getattr(m,'collected_samples',0)}/{getattr(m,'target_samples',0)}  {m.message[:48]}")


def main():
    rclpy.init(); w = Watch()
    if not w.cli.wait_for_service(timeout_sec=12.0):
        print("  ✗ register_face 服務沒回應"); return 1
    req = RegisterFace.Request()
    req.user_name = NAME
    req.user_type = UserType(type=1)
    req.description = "VIP客戶"
    req.num_samples = N
    print(f"  送出註冊：{NAME}，目標 {N} 張\n")
    fut = w.cli.call_async(req)
    t0 = time.time()
    while time.time() - t0 < 35.0:
        rclpy.spin_once(w, timeout_sec=0.05)
        if fut.done() and fut.result() is not None and time.time() - t0 < 1.0:
            print(f"  服務回應：{fut.result().message}\n")
    print("\n  （35 秒觀察結束）")
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
