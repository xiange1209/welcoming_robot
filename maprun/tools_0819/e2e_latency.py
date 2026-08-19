#!/usr/bin/env python3
"""E6 端到端延遲量測 —— 「訪客走到面前到聽到迎賓語音要幾秒」

用法（Pi 上，另開視窗）：
    python3 ~/maprun/tools_0819/e2e_latency.py --out e6.csv --rounds 10

然後讓受試者**走進鏡頭 -> 停在 1 公尺 -> 等機器人講話**，重複 N 次。
每一輪自動抓一組時間戳，Ctrl-C 結束時印出統計。

## 量的是哪幾段（★ 這條鏈的每一段都可能是瓶頸，要分開量）

    /image_raw ──> /face_embedding ──> /user_identity ──> /speech_text
       相機幀        抽 512D 向量        比對資料庫         迎賓詞產生
                  ↑ InsightFace       ↑ 餘弦相似度      ↑ bank_reception 劇本

    另一條（對話）：
    /user_text ──> /llm_stream(首 token) ──> /llm_response ──> /speech_text
                 ↑ 網路 + 筆電 GPU 推論

★ 為什麼要分段而不是只量頭尾：8/19 量到 CER 25.6%，但不知道是 VAD 沒觸發
  還是辨識錯 —— 同樣的錯誤在延遲上也會發生。只量頭尾的話，
  「慢」會被歸咎到最顯眼的那一段（通常是 LLM），而不是真正慢的那一段。

## 誠實分級

這支量的是**話題之間的時間差**，不含：
  - 相機曝光到 /image_raw 發布的驅動延遲（要用外部碼錶才量得到）
  - 平板收到 /speech_text 到真正出聲的延遲（那在瀏覽器裡）
所以結果是**下界**。報告要寫「ROS 內部端到端」，不要寫成「使用者感受」。
"""
import argparse
import csv
import statistics
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from smartnav_msgs.msg import FaceEmbedding, UserIdentity


class E2ELatency(Node):
    def __init__(self, out_path: str):
        super().__init__("e2e_latency")
        self.out_path = out_path
        self.rows = []
        self.t = {}          # 這一輪各站的時刻
        self.round = 0

        # 相機話題可能是壓縮或未壓縮，兩個都訂，先到先算
        self.create_subscription(CompressedImage, "/image_raw/compressed",
                                 lambda m: self._mark("image"), 1)
        self.create_subscription(Image, "/image_raw", lambda m: self._mark("image"), 1)
        self.create_subscription(FaceEmbedding, "/face_embedding",
                                 lambda m: self._mark("embedding"), 10)
        self.create_subscription(UserIdentity, "/user_identity", self._identity_cb, 10)
        self.create_subscription(String, "/user_text", lambda m: self._mark("user_text"), 10)
        self.create_subscription(String, "/llm_stream", self._first_token_cb, 50)
        self.create_subscription(String, "/llm_response", lambda m: self._mark("llm_done"), 10)
        self.create_subscription(String, "/speech_text", self._speech_cb, 10)

        self.get_logger().info("✓ 量測中。讓受試者走進鏡頭 -> 停 1 公尺 -> 等機器人講話（Ctrl-C 結束）")

    # ── 各站 ──
    def _mark(self, key):
        # ★ 只記**這一輪的第一次**：相機 15 FPS，不擋掉的話 image 會一直被覆蓋成最新值，
        #   算出來的延遲會趨近 0（這是這類量測最常見的錯）。
        if key not in self.t:
            self.t[key] = time.time()

    def _identity_cb(self, msg: UserIdentity):
        if not msg.recognized:
            return                      # 認不出來的不算一輪
        self._mark("identity")

    def _first_token_cb(self, msg: String):
        self._mark("llm_first_token")

    def _speech_cb(self, msg: String):
        self._mark("speech")
        self._finish()

    def _finish(self):
        if "speech" not in self.t:
            return
        self.round += 1
        base = self.t.get("image") or self.t.get("user_text") or self.t["speech"]
        row = {"round": self.round,
               "wall_time": time.strftime("%Y-%m-%d %H:%M:%S")}
        for k in ("image", "embedding", "identity", "user_text",
                  "llm_first_token", "llm_done", "speech"):
            row[k] = round(self.t[k] - base, 3) if k in self.t else ""
        row["total"] = round(self.t["speech"] - base, 3)
        self.rows.append(row)
        self.get_logger().info(
            f"第 {self.round} 輪：總計 {row['total']} 秒"
            + (f"（辨識 {row['identity']}）" if row["identity"] != "" else "")
            + (f"（LLM 首 token {row['llm_first_token']}）" if row["llm_first_token"] != "" else ""))
        self.t.clear()                  # 下一輪重新開始

    def dump(self):
        if not self.rows:
            self.get_logger().warning("沒有收到任何完整的一輪")
            return
        cols = ["round", "wall_time", "image", "embedding", "identity",
                "user_text", "llm_first_token", "llm_done", "speech", "total"]
        with open(self.out_path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(self.rows)
        totals = [r["total"] for r in self.rows]
        print()
        print(f"共 {len(totals)} 輪 -> {self.out_path}")
        print(f"  總延遲 中位數 {statistics.median(totals):.2f} s"
              f"  最小 {min(totals):.2f}  最大 {max(totals):.2f}")
        # 分段：只用有值的輪次算，並印出樣本數（不足的段落不要當結論用）
        for a, b, name in (("image", "embedding", "相機->抽向量"),
                           ("embedding", "identity", "抽向量->比對"),
                           ("identity", "speech", "比對->迎賓詞"),
                           ("user_text", "llm_first_token", "問句->LLM 首 token"),
                           ("llm_first_token", "llm_done", "LLM 生成")):
            d = [r[b] - r[a] for r in self.rows
                 if isinstance(r.get(a), float) and isinstance(r.get(b), float)]
            if d:
                print(f"  {name:22s} 中位數 {statistics.median(d):6.2f} s（n={len(d)}）")


def main():
    ap = argparse.ArgumentParser(description="E6 端到端延遲量測")
    ap.add_argument("--out", default="e6_latency.csv")
    ap.add_argument("--rounds", type=int, default=0, help="達到這個輪數自動結束；0 = 手動 Ctrl-C")
    args = ap.parse_args()

    rclpy.init()
    node = E2ELatency(args.out)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.rounds and node.round >= args.rounds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.dump()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
