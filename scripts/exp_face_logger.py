#!/usr/bin/env python3

"""E1/E2 人臉辨識實驗記錄器

訂閱 /face_recognition/result，把每筆辨識結果連同實驗條件寫入 CSV，
供 docs/實驗設計_報告數據.md 的 E1（準確率矩陣）與 E2（樣本數比較）使用。

用法（Pi 上，另開視窗跑著就會一直記）：
  python3 scripts/exp_face_logger.py --condition "1m_正面_亮" --out e1_results.csv
  # 換條件時 Ctrl-C 停掉，改 --condition 再跑；同一個 --out 檔會續寫（append）

輸出欄位：wall_time, condition, person_uuid, person_name, person_type, confidence
分析時以 condition 分組統計辨識率與 confidence 分布。
"""

import argparse
import csv
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from smartnav_msgs.msg import RecognitionResult


class FaceLogger(Node):
    def __init__(self, condition: str, out_path: str):
        super().__init__("exp_face_logger")
        self.condition = condition
        self.out_path = Path(out_path)
        self.count = 0

        new_file = not self.out_path.exists()
        self.fh = open(self.out_path, "a", newline="", encoding="utf-8-sig")
        self.writer = csv.writer(self.fh)
        if new_file:
            self.writer.writerow(
                ["wall_time", "condition", "person_uuid", "person_name", "person_type", "confidence"]
            )

        self.sub = self.create_subscription(RecognitionResult, "/face_recognition/result", self.cb, 10)
        self.get_logger().info(f"✓ 記錄中：condition={condition} → {self.out_path}（Ctrl-C 結束）")

    def cb(self, msg: RecognitionResult) -> None:
        self.writer.writerow(
            [
                time.strftime("%Y-%m-%d %H:%M:%S"),
                self.condition,
                msg.person_uuid,
                msg.person_name,
                msg.person_type,
                f"{msg.confidence:.4f}",
            ]
        )
        self.fh.flush()
        self.count += 1
        if self.count % 10 == 0:
            self.get_logger().info(f"已記錄 {self.count} 筆")


def main():
    parser = argparse.ArgumentParser(description="E1/E2 人臉辨識實驗記錄器")
    parser.add_argument("--condition", required=True, help="實驗條件標籤，例如 1m_正面_亮 或 samples5_2m_正面_亮")
    parser.add_argument("--out", default="face_exp_results.csv", help="輸出 CSV 路徑（append 模式）")
    args = parser.parse_args()

    rclpy.init()
    node = FaceLogger(args.condition, args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"結束：本次共記錄 {node.count} 筆 → {node.out_path}")
        node.fh.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
