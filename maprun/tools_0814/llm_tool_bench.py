#!/usr/bin/env python3
"""LLM 工具選擇準確率量測器（只發 user_text，不碰任何動作）

作法：
  1. 每題前先發 clear_conversation 清記憶（避免上一題污染）
  2. 記下節點 log 的 byte offset
  3. 發 user_text
  4. 等 llm_response（最終回覆，一次一則）當作結束訊號；
     另外用「收到第一段 speech_text 後 2.5 秒靜默」當備援
  5. 從 log 的 delta 抓 `🛠️ LLM 決定呼叫服務: <name>`

用法： python3 llm_tool_bench.py <輸出json> <重複次數> [log路徑]
"""

import json
import re
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty

LOG_DEFAULT = "/home/user/maprun/logs/smartnav_llm_llm_service.log"
TOOL_RE = re.compile(r"🛠️ LLM 決定呼叫服務: (\S+?),")

CORE = [
    ("Q1_開戶證件", "請問開戶要帶什麼證件", "query_bank_faq_tool"),
    ("Q2_幾點關門", "你們幾點關門", "query_bank_faq_tool"),
    ("Q3_找真人", "我要找真人服務", "notify_staff_tool"),
]

# 回歸題組：確認修改沒有把別的題目弄壞
#
# ★ 「帶我去貴賓室」故意**不放進實機題組**。17:40 之後主 agent 把導航堆疊
#   起來了（/navigate 從 0 個 server 變成 1 個），那題一旦命中就會真的送
#   Navigate goal 讓車子開走。帶位那題改用離線探針（offline_probe.py）驗，
#   探針的工具是 stub，不連 ROS，動不了車。
REGRESSION = [
    ("R1_現在幾點", "現在幾點", "query_datetime_tool"),
    ("R2_打招呼", "你好", None),                       # None = 不該呼叫任何工具
    ("R4_匯率", "美金匯率多少", "query_exchange_rate_tool"),
    ("R5_營業時間", "你們營業時間是幾點到幾點", "query_bank_faq_tool"),
]

SETS = {"core": CORE, "reg": REGRESSION, "all": CORE + REGRESSION}

# 這些工具會讓車子動。測試題目不該選到它們；選到就是出事了，立刻停。
DANGEROUS = ("navigate_tool", "guide_to_vip_room_tool", "create_map_tool",
             "global_localization_tool")

SILENCE = 2.5
HARD_TIMEOUT = 120.0


class Bench(Node):
    def __init__(self):
        super().__init__("llm_tool_bench")
        self.user_text_pub = self.create_publisher(String, "user_text", 10)
        self.clear_pub = self.create_publisher(Empty, "clear_conversation", 10)
        self.speech = []
        self.speech_t = []
        self.response = []
        self.create_subscription(String, "speech_text", self._on_speech, 10)
        self.create_subscription(String, "llm_response", self._on_resp, 10)

    def _on_speech(self, msg):
        self.speech.append(msg.data)
        self.speech_t.append(time.time())

    def _on_resp(self, msg):
        self.response.append(msg.data)


def spin_for(node, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    out_path = Path(sys.argv[1])
    repeats = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    qset = sys.argv[3] if len(sys.argv) > 3 else "core"
    do_clear = (sys.argv[4] if len(sys.argv) > 4 else "clear") == "clear"
    log_path = Path(sys.argv[5]) if len(sys.argv) > 5 else Path(LOG_DEFAULT)
    questions = SETS[qset]
    print(f"# 題組={qset} 重複={repeats} 每題前清記憶={do_clear}")

    rclpy.init()
    node = Bench()

    # 等訂閱端連上
    t0 = time.time()
    while node.user_text_pub.get_subscription_count() == 0 and time.time() - t0 < 30:
        spin_for(node, 0.2)
    if node.user_text_pub.get_subscription_count() == 0:
        print("!! user_text 沒有訂閱者，llm_service 沒起來？")
        return 2
    spin_for(node, 1.0)

    results = []
    for rep in range(repeats):
        if not do_clear:
            # 不逐題清，只在每一輪開頭清一次（重現「連問五題」的情境）
            node.clear_pub.publish(Empty())
            spin_for(node, 1.0)
        for qid, text, expect in questions:
            # 1. 清記憶
            if do_clear:
                node.clear_pub.publish(Empty())
                spin_for(node, 1.0)

            # 2. log offset
            offset = log_path.stat().st_size if log_path.exists() else 0

            # 3. drain
            node.speech.clear()
            node.speech_t.clear()
            node.response.clear()
            spin_for(node, 0.3)
            node.speech.clear()
            node.speech_t.clear()
            node.response.clear()

            # 4. 發問
            t_ask = time.time()
            node.user_text_pub.publish(String(data=text))

            first_latency = None
            deadline = t_ask + HARD_TIMEOUT
            while time.time() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if first_latency is None and node.speech:
                    first_latency = node.speech_t[0] - t_ask
                if node.response:
                    break
            # llm_response 之後可能還有殘餘 speech 段落，收乾淨
            last = node.speech_t[-1] if node.speech_t else time.time()
            while time.time() - last < SILENCE and time.time() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                last = node.speech_t[-1] if node.speech_t else last
            total = time.time() - t_ask

            # 5. 讀 log delta
            tools = []
            if log_path.exists():
                with open(log_path, "r", encoding="utf-8", errors="replace") as f:
                    f.seek(offset)
                    delta = f.read()
                tools = TOOL_RE.findall(delta)

            rec = {
                "rep": rep,
                "qid": qid,
                "question": text,
                "expect": expect,
                "tools": tools,
                "first_tool": tools[0] if tools else None,
                "hit": (tools[0] == expect) if tools else (expect is None),
                "first_latency_s": round(first_latency, 2) if first_latency else None,
                "total_s": round(total, 2),
                "speech": node.speech[:],
                "llm_response": node.response[:],
            }
            results.append(rec)
            print(f"[{rep+1}/{repeats}] {qid:14s} -> {tools if tools else '(沒呼叫)'} "
                  f"{'✓' if rec['hit'] else '✗'} {total:.1f}s")
            sys.stdout.flush()

            hit_danger = [t for t in tools if t in DANGEROUS]
            if hit_danger:
                print(f"\n!!!! 中止：題目「{text}」選到了會讓車子動的工具 {hit_danger}")
                out_path.write_text(json.dumps(results, ensure_ascii=False, indent=1),
                                    encoding="utf-8")
                node.destroy_node()
                rclpy.shutdown()
                return 3

    out_path.write_text(json.dumps(results, ensure_ascii=False, indent=1), encoding="utf-8")

    print("\n===== 統計 =====")
    for qid, text, expect in questions:
        rs = [r for r in results if r["qid"] == qid]
        hits = sum(1 for r in rs if r["hit"])
        from collections import Counter
        dist = Counter(str(r["first_tool"]) for r in rs)
        print(f"{qid:14s} 期望 {str(expect):24s} 命中 {hits}/{len(rs)}  分佈 {dict(dist)}")
    overall = sum(1 for r in results if r["hit"])
    print(f"總計 {overall}/{len(results)}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
