#!/usr/bin/env python3
"""整體延遲記錄（唯讀；不發任何指令、不改任何東西）——報告第 14 節「整體延遲」用。

只訂閱話題、記下每個事件「收到的時間」，結束時算出每一輪各段花多久：

    說完話 ─(VAD 靜音門檻)→ 最後一塊音訊(is_final) → /user_text（辨識完成）
           → /llm_stream 第一個字 → /llm_response（整句）→ /speech_text（送去朗讀）
           → /cmd_vel 第一次非零（帶位時：車子開始動）
    /user_identity 認出人 → /speech_text 迎賓句

用法（車上；跟故事線、平板操作一起開著就好）：
    source ~/welcoming_robot_ws/src/smartnav_ws/maprun/env.sh
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/latency_logger.py              # 一直記，Ctrl-C 結束並印統計
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/latency_logger.py --secs 1800  # 30 分鐘自動結束

輸出：~/welcoming_robot_ws/src/smartnav_ws/maprun/verify_out/latency_<MMDD_HHMM>/events.csv（每個事件）、turns.csv（每一輪）、summary.txt
★ 「說完話」＝這一段音訊裡**最後一塊有聲音**的收到時間（每塊 32 ms 算音量，門檻＝這段最大音量的 12%
  與底噪 ×2 取大者）。不用「VAD 結束 − 靜音門檻」推，因為 VAD 觸發後前 3 秒連靜音也照送，短句會推錯。
  VAD 的 is_final 時間另外記（「最後音訊→文字」），靜音門檻從 asr log 讀「靜音結束檢測: N ms」。
★ 平板打字送出的 /user_text 前面沒有音訊段，那一輪標「打字」，只算 LLM 與車子的部分。
★ 身分只記類型與相似度，不記名字（不碰人臉資料庫）；對話文字只留前 30 字。
★ 收到時間＝本機時鐘；所有節點都在同一台，DDS 傳遞只差幾 ms。
★ 2026-10-26 車上新寫；用假音訊段＋真 LLM 單元驗過流程，還沒在實車故事線跑過。
"""
import argparse
import csv
import glob
import os
import re
import statistics
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String
from geometry_msgs.msg import Twist
from smartnav_msgs.msg import AudioData, UserIdentity

LOG_DIR = os.path.expanduser("~/.smartnav/logs/system_control")
TYPE_NAME = {0: "訪客", 1: "VIP", 2: "管理者", 3: "黑名單"}
VOICE_WINDOW_S = 8.0      # 音訊段結束後這麼久內出現的 /user_text 才算「語音」
GREET_WINDOW_S = 10.0     # 認出人之後這麼久內的 /speech_text 才算迎賓句
MOVE_WINDOW_S = 60.0      # 指令後這麼久內車子開始動，才算這一輪造成的
STOP_HOLD_S = 1.0         # 指令歸零維持這麼久才算停下


def silence_ms_from_logs(default=500):
    """從最新一份有 VAD 啟動訊息的 log 讀靜音門檻"""
    best = None
    for f in sorted(glob.glob(os.path.join(LOG_DIR, "*.log")), key=os.path.getmtime):
        try:
            with open(f, encoding="utf-8", errors="replace") as fh:
                for ln in fh:
                    m = re.search(r"靜音結束檢測: (\d+) ms", ln)
                    if m:
                        best = int(m.group(1))
        except OSError:
            continue
    return best if best is not None else default


def short(text, n=30):
    text = " ".join(str(text).split())
    return text if len(text) <= n else text[:n] + "…"


def pct(vals, q):
    vals = sorted(vals)
    if not vals:
        return None
    return vals[min(len(vals) - 1, int(round(q * (len(vals) - 1))))]


class LatencyLogger(Node):
    def __init__(self, args):
        super().__init__("latency_logger_cc")
        self.silence_s = args.silence_ms / 1000.0
        self.t0 = time.time()
        self.events = []          # (wall, event, detail)
        self.turns = []           # 每一輪的 dict
        self.greets = []          # 認出人 → 迎賓句
        self.seg = None           # 目前／最近一段音訊：start、chunks[(t, rms)]、final、partial、used
        self.moving = False
        self.zero_since = None
        self.pending_ident = None
        self.last_ident_key = None

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(AudioData, args.audio_topic, self._on_audio, qos)
        self.create_subscription(String, "/partial_text", self._on_partial, qos)
        self.create_subscription(String, "/user_text", self._on_user_text, qos)
        self.create_subscription(Empty, "/llm_stream_reset", self._on_reset, qos)
        self.create_subscription(String, "/llm_stream", self._on_stream, qos)
        self.create_subscription(String, "/llm_response", self._on_response, qos)
        self.create_subscription(String, "/speech_text", self._on_speech_text, qos)
        self.create_subscription(Bool, "/playback_status", self._on_playback, qos)
        self.create_subscription(UserIdentity, args.identity_topic, self._on_identity, qos)
        self.create_subscription(Twist, args.cmd_vel_topic, self._on_cmd_vel, qos)
        self.create_timer(0.2, self._tick)

    # ── 工具 ─────────────────────────────────────────────────────────
    def _ev(self, name, detail=""):
        now = time.time()
        self.events.append((now, name, detail))
        return now

    def _turn(self):
        return self.turns[-1] if self.turns else None

    # ── 回呼 ─────────────────────────────────────────────────────────
    @staticmethod
    def _rms(msg):
        raw = bytes(msg.data)
        if msg.format == "pcm_f32le":
            x = np.frombuffer(raw, dtype="<f4")
        elif msg.format == "pcm_s32le":
            x = np.frombuffer(raw, dtype="<i4").astype(np.float32) / 2147483648.0
        else:                                   # pcm_s16le（預設）
            x = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
        return float(np.sqrt(np.mean(x * x))) if x.size else 0.0

    def _on_audio(self, msg):
        now = time.time()
        if self.seg is None or self.seg["final"] is not None:
            self.seg = {"start": self._ev("音訊段開始"), "chunks": [], "final": None, "partial": None, "used": False}
        self.seg["chunks"].append((now, self._rms(msg)))
        if msg.is_final:
            self.seg["final"] = now
            self._ev("音訊段結束(is_final)", f"長 {now - self.seg['start']:.2f} s")

    @staticmethod
    def _speech_end(chunks, before):
        """最後一塊有聲音的時間（只看 before 之前收到的）"""
        pts = [(t, r) for t, r in chunks if t <= before]
        if not pts:
            return None
        rms = sorted(r for _, r in pts)
        thr = max(0.12 * rms[-1], 2.0 * rms[len(rms) // 10], 1e-4)
        loud = [t for t, r in pts if r >= thr]
        return loud[-1] if loud else None

    def _on_partial(self, msg):
        seg = self.seg
        if seg is not None and seg["final"] is None and seg["partial"] is None and msg.data.strip():
            seg["partial"] = self._ev("第一個部分文字", short(msg.data))

    def _on_user_text(self, msg):
        now = self._ev("辨識完成(/user_text)", short(msg.data))
        t = {"t_text": now, "text": short(msg.data), "source": "打字"}
        seg = self.seg
        if seg is not None and not seg["used"] and (seg["final"] is None or now - seg["final"] <= VOICE_WINDOW_S):
            seg["used"] = True
            t.update(source="語音", seg=seg, speech_end=self._speech_end(seg["chunks"], now))
        self.turns.append(t)

    def _on_reset(self, _msg):
        t = self._turn()
        if t is not None and "t_reset" not in t:
            t["t_reset"] = self._ev("LLM 開始回答")

    def _on_stream(self, msg):
        t = self._turn()
        if t is not None and "t_first" not in t and msg.data:
            t["t_first"] = self._ev("LLM 第一個字", short(msg.data, 10))

    def _on_response(self, msg):
        t = self._turn()
        if t is not None and "t_resp" not in t:
            t["t_resp"] = self._ev("LLM 整句", short(msg.data))
            t["resp"] = short(msg.data)
            self._print_turn(t)
        else:
            self._ev("LLM 其他回覆", short(msg.data))     # 例如「大廳到了」

    def _on_speech_text(self, msg):
        now = self._ev("送去朗讀(/speech_text)", short(msg.data))
        if self.pending_ident and now - self.pending_ident["t_ident"] <= GREET_WINDOW_S:
            g = self.pending_ident
            g["t_greet"] = now
            g["greet"] = short(msg.data)
            self.greets.append(g)
            self.pending_ident = None
            print(f"👤 認出{g['kind']}（相似度 {g['sim']:.2f}）→ 迎賓句 {now - g['t_ident']:.2f} s：{g['greet']}", flush=True)
            return
        t = self._turn()
        if t is not None and "t_tts" not in t and now - t["t_text"] <= MOVE_WINDOW_S:
            t["t_tts"] = now

    def _on_playback(self, msg):
        now = self._ev("播放中" if msg.data else "播放結束")
        t = self._turn()
        if msg.data and t is not None and "t_tts" in t and "t_play" not in t:
            t["t_play"] = now

    def _on_identity(self, msg):
        uid = str(msg.user_uuid or "")
        kind = TYPE_NAME.get(int(msg.user_type.type), str(msg.user_type.type)) if uid else "未註冊"
        key = (bool(uid), kind, uid[-4:])
        if key == self.last_ident_key:
            return                                  # 同一個人持續被認出，只記變化
        self.last_ident_key = key
        now = self._ev("身分變化", f"{kind} 相似度 {msg.similarity:.2f}")
        if uid:
            self.pending_ident = {"t_ident": now, "kind": kind, "sim": float(msg.similarity)}

    def _on_cmd_vel(self, msg):
        now = time.time()
        nonzero = abs(msg.linear.x) > 0.005 or abs(msg.angular.z) > 0.005
        if nonzero:
            self.zero_since = None
            if not self.moving:
                self.moving = True
                self._ev("車子開始動", f"v={msg.linear.x:+.2f} w={msg.angular.z:+.2f}")
                t = self._turn()
                if t is not None and "t_move" not in t and now - t["t_text"] <= MOVE_WINDOW_S:
                    t["t_move"] = now
                    print(f"   🚗 指令後 {now - t['t_text']:.2f} s 車子開始動", flush=True)
        elif self.moving and self.zero_since is None:
            self.zero_since = now

    def _tick(self):
        if self.moving and self.zero_since is not None and time.time() - self.zero_since >= STOP_HOLD_S:
            self.moving = False
            self.events.append((self.zero_since, "車子停下", ""))
            self.zero_since = None

    # ── 輸出 ─────────────────────────────────────────────────────────
    def metrics(self, t):
        m = {}
        seg = t.get("seg")
        if seg is not None:
            if t.get("speech_end") is not None:
                m["說完話→文字"] = t["t_text"] - t["speech_end"]
            if seg["final"] is not None:
                m["最後音訊→文字"] = t["t_text"] - seg["final"]     # 負值＝ASR 自己先斷句，比 VAD 早
            if seg["partial"] is not None:
                m["開口→第一個部分文字"] = seg["partial"] - seg["start"]
        for key, name in (("t_first", "文字→第一個字"), ("t_tts", "文字→送去朗讀"), ("t_move", "文字→車子動")):
            if key in t:
                m[name] = t[key] - t["t_text"]
        if "t_resp" in t:
            # 沒有串流第一個字＝程式直接回答（帶位、固定答案），跟模型生成分開統計
            m["文字→整句（模型）" if "t_first" in t else "文字→整句（程式直接答）"] = t["t_resp"] - t["t_text"]
        if t.get("speech_end") is not None and "t_resp" in t:
            m["說完話→整句"] = t["t_resp"] - t["speech_end"]
        if "t_play" in t and "t_tts" in t:
            m["送去朗讀→開始播放"] = t["t_play"] - t["t_tts"]
        return m

    def _print_turn(self, t):
        m = self.metrics(t)
        parts = "｜".join(f"{k} {v:.2f} s" for k, v in m.items())
        print(f"🗣 {t['source']}「{t['text']}」：{parts}\n   回：{t.get('resp', '')}", flush=True)

    def save(self, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, "events.csv"), "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["時間", "相對秒", "事件", "細節"])
            for wall, name, detail in sorted(self.events):
                w.writerow([time.strftime("%H:%M:%S", time.localtime(wall)) + f".{int(wall % 1 * 1000):03d}",
                            f"{wall - self.t0:.3f}", name, detail])
        names = ["說完話→文字", "最後音訊→文字", "開口→第一個部分文字", "文字→第一個字", "文字→整句（模型）",
                 "文字→整句（程式直接答）", "說完話→整句", "文字→送去朗讀", "送去朗讀→開始播放", "文字→車子動"]
        with open(os.path.join(out_dir, "turns.csv"), "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["時間", "來源", "文字"] + names + ["回答"])
            for t in self.turns:
                m = self.metrics(t)
                w.writerow([time.strftime("%H:%M:%S", time.localtime(t["t_text"])), t["source"], t["text"]]
                           + [f"{m[n]:.3f}" if n in m else "" for n in names] + [t.get("resp", "")])
        lines = [f"整體延遲 {time.strftime('%Y-%m-%d %H:%M', time.localtime(self.t0))}～"
                 f"{time.strftime('%H:%M', time.localtime())}：{len(self.turns)} 輪"
                 f"（語音 {sum(t['source'] == '語音' for t in self.turns)}、打字 {sum(t['source'] == '打字' for t in self.turns)}），"
                 f"靜音門檻 {self.silence_s * 1000:.0f} ms"]
        for n in names:
            vals = [self.metrics(t)[n] for t in self.turns if n in self.metrics(t)]
            if vals:
                lines.append(f"  {n}：中位 {statistics.median(vals):.2f} s、p90 {pct(vals, 0.9):.2f} s、"
                             f"最小 {min(vals):.2f}／最大 {max(vals):.2f}（n={len(vals)}）")
        gv = [g["t_greet"] - g["t_ident"] for g in self.greets]
        if gv:
            lines.append(f"  認出人→迎賓句：中位 {statistics.median(gv):.2f} s（n={len(gv)}）")
        miss = [t["text"] for t in self.turns if "t_resp" not in t]
        if miss:
            lines.append(f"  ⚠ 沒等到 LLM 回答的輪：{len(miss)}（{'、'.join(miss[:5])}）")
        text = "\n".join(lines)
        with open(os.path.join(out_dir, "summary.txt"), "w", encoding="utf-8") as fh:
            fh.write(text + "\n")
        return text


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--secs", type=float, default=0.0, help="記這麼久自動結束（0＝直到 Ctrl-C）")
    ap.add_argument("--silence-ms", type=int, default=None, help="VAD 靜音門檻（預設從 asr log 讀）")
    ap.add_argument("--out", default=os.path.expanduser(f"~/welcoming_robot_ws/src/smartnav_ws/maprun/verify_out/latency_{time.strftime('%m%d_%H%M')}"))
    # 測試用：把音訊／身分／速度改聽別的話題（驗證這支工具時不必碰真的話題）
    ap.add_argument("--audio-topic", default="/audio_in")
    ap.add_argument("--identity-topic", default="/user_identity")
    ap.add_argument("--cmd-vel-topic", default="/cmd_vel")
    args = ap.parse_args()
    if args.silence_ms is None:
        args.silence_ms = silence_ms_from_logs()

    rclpy.init()
    node = LatencyLogger(args)
    print(f"開始記錄（唯讀）。靜音門檻 {args.silence_ms} ms；"
          f"{'Ctrl-C 結束' if args.secs <= 0 else f'{args.secs:.0f} 秒後自動結束（Ctrl-C 可提早）'}。", flush=True)
    end = time.time() + args.secs if args.secs > 0 else float("inf")
    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print("\n" + node.save(args.out), flush=True)
        print(f"存在：{args.out}", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
