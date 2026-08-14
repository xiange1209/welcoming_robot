#!/usr/bin/env python3
"""量 LLM 端到端延遲：發 /user_text，等 /llm_response，逐題計時。"""
import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PROMPTS = [
    ("純閒聊", "你好"),
    ("純閒聊", "謝謝你"),
    ("該叫工具", "現在幾點"),
    ("該查知識庫", "你們有什麼服務"),
]


class Bench(Node):
    def __init__(self):
        super().__init__("llm_bench")
        self.pub = self.create_publisher(String, "user_text", 10)
        self.resp = None
        self.create_subscription(String, "llm_response", self._cb, 10)

    def _cb(self, msg):
        self.resp = msg.data

    def ask(self, text, timeout=240.0):
        self.resp = None
        # 等訂閱端接上再發，否則第一題會掉
        deadline = time.monotonic() + 5.0
        while self.pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        m = String(); m.data = text
        t0 = time.monotonic()
        self.pub.publish(m)
        while self.resp is None and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        return (time.monotonic() - t0, self.resp)


def main():
    rclpy.init()
    n = Bench()
    time.sleep(2.0)
    print(f"{'類型':<12}{'題目':<12}{'秒':>8}   回覆")
    print("-" * 78)
    times = []
    for kind, q in PROMPTS:
        dt, r = n.ask(q)
        if r is None:
            print(f"{kind:<12}{q:<12}{dt:>8.1f}   ✗ 逾時無回覆")
        else:
            times.append(dt)
            one = r.replace("\n", " ")[:46]
            print(f"{kind:<12}{q:<12}{dt:>8.1f}   {one}")
    if times:
        print("-" * 78)
        print(f"  平均 {sum(times)/len(times):.1f} 秒   最快 {min(times):.1f}   最慢 {max(times):.1f}")
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
