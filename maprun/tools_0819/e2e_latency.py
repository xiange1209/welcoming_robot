#!/usr/bin/env python3
"""E6 端到端延遲量測 —— 「訪客走到面前到聽到迎賓語音要幾秒」

用法（Pi 上，另開視窗）：
    python3 ~/maprun/tools_0819/e2e_latency.py --rounds 10

然後讓受試者**走進鏡頭 -> 停在 1 公尺 -> 等機器人講話**，重複 N 次。

★★ 2026-08-20 大改：原版量到的不是延遲 ★★

原版有三個錯疊在一起，任何一個都足以讓數字不能寫進報告：

  1. **話題名與 QoS 都錯**：訂 `/image_raw`（不存在，實際是
     `/camera/color/image_raw/compressed`）而且用預設 RELIABLE
     （相機那條是 BEST_EFFORT，不相容）-> image 這站永遠沒有時刻。

  2. **相機那站與該輪沒有因果關係**：相機 15~30 FPS 一直在發，
     `t.clear()` 之後幾十毫秒內就被下一張**無關的幀**填上。
     算出來的 `total` 是「這則 /speech_text 與上一則之間的牆鐘間隔」，
     搭配迎賓劇本 60/300 秒的冷卻，會印出「端到端中位數 60 秒」。

  3. **一輪的界定錯**：`/speech_text` 有五個發布者，而 LLM 一次回答
     由 `flush_held()` 逐句發出（實測一題發過 10 段）。
     原版一收到就 `_finish()` + `clear()` -> 一次回答變成 10 輪，
     第 2 輪起只有 speech 一站、`total = 0.0`。
     `--rounds 10` 會在一次回答之後就結束，CSV 只有 1 筆真資料 + 9 筆 0。
     而 `/llm_response` 排在 `/speech_text` **之後**才發
     （llm_service_node.py:807 -> :815），所以 `llm_done` 永遠落到下一輪。

現在改用 **header.stamp 對位**：

    相機幀 header.stamp
      └─> face_embedding_node.py:138  face_msg.header = msg.header
            └─> user_auth_node.py:667  msg.header = face_msg.header

  FaceEmbedding 與 UserIdentity **帶著同一幀的 stamp**，所以可以確定
  「這個向量」與「這個身份」出自同一張影像 —— 不必訂相機，也不會對錯幀。

## 量的是哪幾段

    [相機幀 stamp] ──> /face_embedding ──> /user_identity ──> /speech_text
                      抽 512D 向量          比對資料庫         迎賓詞
                      ↑ InsightFace        ↑ 餘弦相似度      ↑ bank_reception

    另一條（對話）：
    /user_text ──> /llm_stream(首 token) ──> /llm_response
                 ↑ 網路 + 筆電 GPU 推論

## 誠實分級

  - `相機->抽向量` 用的是 header.stamp 到收到向量的時間差。
    若驅動是在**發布時**打戳（不是取像時），驅動延遲就沒被算進去 -> 下界。
  - 平板收到 /speech_text 到真正出聲的延遲在瀏覽器裡，量不到 -> 下界。
  - LLM 那段是「本工具收到 /user_text」到「收到首 token」的區間長度。
    LLM 節點跑在 Pi 上（只是用 HTTP 打筆電的 Ollama），**所有時戳同一顆時鐘**，
    網路 RTT 與 GPU 推論都正確地算在裡面，不需要跨機器時間同步。

報告要寫「ROS 內部端到端」，不要寫成「使用者感受」。

★ 量測前務必把迎賓冷卻降下來，否則輪與輪之間要等 60~300 秒：
    ros2 run smartnav_brain bank_reception --ros-args \\
        -p cooldown_sec:=5.0 -p visitor_cooldown_sec:=5.0
★ 受試者要用**已註冊**的臉，否則 recognized 為 False，identity 那欄整份空白。
"""
import argparse
import csv
import statistics
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from smartnav_msgs.msg import FaceEmbedding, UserIdentity


def _stamp_key(header) -> str:
    """把 header.stamp 變成可比對的字串鍵。"""
    return "%d.%09d" % (header.stamp.sec, header.stamp.nanosec)


def _stamp_sec(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class E2ELatency(Node):
    def __init__(self, out_path: str, quiet: float):
        super().__init__("e2e_latency")
        self.out_path = out_path
        self.quiet = quiet
        self.rows = []
        self.round = 0

        # stamp -> 收到 FaceEmbedding 的時刻。只留最近的，避免無上限成長。
        self.embed_at = {}
        self.cur = None          # 這一輪累積中的資料
        self.last_event = 0.0

        # ★ 完全不訂相機。
        #   (a) 拍攝時刻已經在 FaceEmbedding/UserIdentity 的 header 裡
        #   (b) 原始幀 640x480x3 = 900 KB，訂了等於讓量測工具自己拉高延遲
        #       （觀察者效應），而它跟 InsightFace 搶同一顆 Pi 的 CPU
        self.create_subscription(FaceEmbedding, "/face_embedding", self._embed_cb, 20)
        self.create_subscription(UserIdentity, "/user_identity", self._identity_cb, 10)
        self.create_subscription(String, "/user_text", self._user_text_cb, 10)
        self.create_subscription(String, "/llm_stream", self._first_token_cb, 50)
        self.create_subscription(String, "/llm_response", self._llm_done_cb, 10)
        self.create_subscription(String, "/speech_text", self._speech_cb, 10)

        self.get_logger().info(
            "✓ 量測中。讓受試者走進鏡頭 -> 停 1 公尺 -> 等機器人講話（Ctrl-C 結束）")

    # ── 輪次管理 ────────────────────────────────────────────
    def _open(self, kind: str):
        if self.cur is None:
            self.cur = {"kind": kind}
        self.last_event = time.time()

    def _set(self, key, value):
        """只記這一輪的第一次。第二次以後忽略（同一輪內 /speech_text 會有多則）。"""
        if self.cur is not None and key not in self.cur:
            self.cur[key] = value
        self.last_event = time.time()

    def tick(self):
        """靜默夠久就收掉這一輪。★ 不能一收到 /speech_text 就收 ——
        `/llm_response` 排在 `/speech_text` 之後，那樣 llm_done 永遠落到下一輪。"""
        if self.cur and self.last_event and time.time() - self.last_event > self.quiet:
            self._close()

    def _close(self):
        c, self.cur = self.cur, None
        if not c:
            return
        # 迎賓鏈至少要有 identity；對話鏈至少要有 user_text
        if "ident_recv" not in c and "user_text_recv" not in c:
            return
        self.round += 1
        row = {"round": self.round, "kind": c.get("kind", ""),
               "wall_time": time.strftime("%Y-%m-%d %H:%M:%S")}
        # ★ 每一站都存**絕對時刻**（epoch，微秒）。
        #   原版只存相對 base 的偏移，base 一選錯就沒有任何原始資料可以重算。
        for k in ("capture", "embed_recv", "ident_recv", "user_text_recv",
                  "llm_first_recv", "llm_done_recv", "speech_recv"):
            row[k] = ("%.6f" % c[k]) if k in c else ""

        def seg(a, b):
            if a in c and b in c:
                return round(c[b] - c[a], 3)
            return ""

        row["相機到向量"] = seg("capture", "embed_recv")
        row["向量到比對"] = seg("embed_recv", "ident_recv")
        row["比對到迎賓詞"] = seg("ident_recv", "speech_recv")
        row["問句到首token"] = seg("user_text_recv", "llm_first_recv")
        row["LLM生成"] = seg("llm_first_recv", "llm_done_recv")
        # 端到端：迎賓鏈是「拍到臉 -> 迎賓詞」，對話鏈是「問句 -> 回答完成」
        if "capture" in c and "speech_recv" in c:
            row["total"] = round(c["speech_recv"] - c["capture"], 3)
        elif "user_text_recv" in c and "llm_done_recv" in c:
            row["total"] = round(c["llm_done_recv"] - c["user_text_recv"], 3)
        else:
            row["total"] = ""
        row["stamp_matched"] = int("capture" in c and "embed_recv" in c)
        self.rows.append(row)
        self._append_csv(row)          # ★ 每輪立刻寫檔，不要等結束才一次寫
        self.get_logger().info(
            "第 %d 輪（%s）total=%s　比對到迎賓詞=%s　問句到首token=%s"
            % (self.round, row["kind"], row["total"],
               row["比對到迎賓詞"], row["問句到首token"]))

    # ── 各站 ────────────────────────────────────────────────
    def _embed_cb(self, msg: FaceEmbedding):
        k = _stamp_key(msg.header)
        self.embed_at[k] = time.time()
        if len(self.embed_at) > 200:            # 只留最近的
            for old in list(self.embed_at)[:100]:
                self.embed_at.pop(old, None)

    def _identity_cb(self, msg: UserIdentity):
        if not msg.recognized:
            return                              # 認不出來／沒人 的都不算一輪
        self._open("迎賓")
        if "ident_recv" in self.cur:
            return                              # 同一輪只取第一次
        k = _stamp_key(msg.header)
        # ★★ header.stamp 是 ROS 時鐘，time.time() 是牆鐘 ★★
        #   use_sim_time=false 時兩者相同，但驅動若用別的時鐘打戳（或忘了打），
        #   `相機到向量` 會算出天文數字或負數而**看起來仍像一個數字**。
        #   量測工具最該做的事就是在自己不可信時說出來。
        _cap = _stamp_sec(msg.header)
        if abs(time.time() - _cap) > 30.0:
            if not getattr(self, "_warned_clock", False):
                self._warned_clock = True
                self.get_logger().error(
                    "⚠ header.stamp 與系統時鐘差了 %.0f 秒 —— "
                    "『相機到向量』『向量到比對』兩欄不可用，不要寫進報告。"
                    "（檢查相機驅動有沒有打時間戳、use_sim_time 是不是誤開）"
                    % (time.time() - _cap))
        else:
            self._set("capture", _cap)
        if k in self.embed_at:
            # ★ 同一個 stamp 才對得起來 —— 保證是同一張影像，不會對錯幀
            self._set("embed_recv", self.embed_at[k])
        self._set("ident_recv", time.time())

    def _user_text_cb(self, msg: String):
        self._open("對話")
        self._set("user_text_recv", time.time())

    def _first_token_cb(self, msg: String):
        if self.cur is not None:
            self._set("llm_first_recv", time.time())

    def _llm_done_cb(self, msg: String):
        if self.cur is not None:
            self._set("llm_done_recv", time.time())

    def _speech_cb(self, msg: String):
        # ★ 不在這裡收輪 —— /llm_response 排在 /speech_text 之後才發
        if self.cur is not None:
            self._set("speech_recv", time.time())

    # ── 輸出 ────────────────────────────────────────────────
    COLS = ["round", "kind", "wall_time",
            "capture", "embed_recv", "ident_recv", "user_text_recv",
            "llm_first_recv", "llm_done_recv", "speech_recv",
            "相機到向量", "向量到比對", "比對到迎賓詞",
            "問句到首token", "LLM生成", "total", "stamp_matched"]

    def _append_csv(self, row):
        new = not self.rows[:-1]
        with open(self.out_path, "a", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=self.COLS)
            if new:
                w.writeheader()
            w.writerow(row)

    def dump(self):
        if not self.rows:
            self.get_logger().warning("沒有收到任何完整的一輪")
            return
        print()
        print("共 %d 輪 -> %s" % (len(self.rows), self.out_path))
        n_matched = sum(r["stamp_matched"] for r in self.rows)
        if n_matched < len(self.rows):
            print("  ⚠ 只有 %d/%d 輪對得上 FaceEmbedding 的 stamp。"
                  % (n_matched, len(self.rows)))
            print("    對不上代表向量在身份之前就被丟掉了（佇列滿）或節點沒同時開，")
            print("    那幾輪的『相機到向量』『向量到比對』不可用。")

        rows_md = []
        for name in ("相機到向量", "向量到比對", "比對到迎賓詞",
                     "問句到首token", "LLM生成", "total"):
            d = [r[name] for r in self.rows if isinstance(r.get(name), float)]
            if d:
                rows_md.append((name, statistics.median(d), min(d), max(d), len(d)))
                print("  %-14s 中位數 %6.2f s（最小 %.2f 最大 %.2f，n=%d）"
                      % (name, statistics.median(d), min(d), max(d), len(d)))
        print()
        print("=== 可直接貼進報告的表格 ===")
        print()
        print("| 分段 | 中位數 (s) | 最小 | 最大 | n |")
        print("|---|---|---|---|---|")
        for name, med, lo, hi, n in rows_md:
            print("| %s | %.2f | %.2f | %.2f | %d |" % (name, med, lo, hi, n))
        print()
        print("★ 這是 ROS 內部端到端，不含相機驅動與平板出聲，是**下界**。")


def main():
    ap = argparse.ArgumentParser(description="E6 端到端延遲量測")
    ap.add_argument("--out", default="",
                    help="預設帶時間戳，避免多人量測時互相覆蓋")
    ap.add_argument("--rounds", type=int, default=0,
                    help="達到這個輪數自動結束；0 = 手動 Ctrl-C")
    ap.add_argument("--quiet", type=float, default=2.0,
                    help="靜默幾秒算一輪結束（要大於 LLM 逐句 flush 的間隔）")
    args = ap.parse_args()

    if not args.out:
        args.out = "e6_latency_%s.csv" % time.strftime("%Y%m%d_%H%M%S")

    rclpy.init()
    node = E2ELatency(args.out, args.quiet)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            node.tick()
            if args.rounds and node.round >= args.rounds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node._close()
        node.dump()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
