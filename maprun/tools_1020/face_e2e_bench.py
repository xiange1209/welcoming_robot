#!/usr/bin/env python3
"""人臉 → 身分 → 迎賓 的端對端延遲（不用相機）：發假相機影像（公開樣本臉），量每一段花多久。

先開三個節點（★ 不用 HMI 單元：要關掉到訪紀錄、不讀 Telegram 密鑰、迎賓語音改送測試話題）：
  ros2 run smartnav_vision face_embedding --ros-args -r __node:=face_node -p enable_gpu:=false
  ros2 run smartnav_brain user_auth
  ros2 run smartnav_brain bank_reception --ros-args -p notify_backend:=dryrun_cc -p enable_visit_log:=false \\
      -p visitor_cooldown_sec:=5.0 -r speech_text:=speech_text_e2e_cc
再跑：python3 face_e2e_bench.py [--trials 5] [--on 10] [--off 8]

  - 影像：640×480 JPEG、5 Hz，發到 /camera/color/image_raw/compressed（相機沒開時這個話題沒有別人發）
  - 臉：insightface 內附 t1.jpg 最大那張臉，縮到臉寬 65 px（約 1.3 m）。不會對到資料庫裡的人 → 一般訪客迎賓
  - 量：臉出現 → 第一則 /face_embedding → 第一則 /user_identity → 迎賓語音；另記 face_node 在「沒臉／有臉」時的 CPU
"""
import argparse
import csv
import os
import statistics
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from smartnav_msgs.msg import FaceEmbedding, UserIdentity
from std_msgs.msg import String

ap = argparse.ArgumentParser()
ap.add_argument("--trials", type=int, default=5)
ap.add_argument("--on", type=float, default=10.0, help="每次臉出現幾秒")
ap.add_argument("--off", type=float, default=8.0, help="每次臉消失幾秒")
ap.add_argument("--fps", type=float, default=5.0)
ap.add_argument("--out", default=os.path.expanduser(f"~/maprun/verify_out/face_e2e_{time.strftime('%H%M')}"))
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)


def make_frames():
    from insightface.app import FaceAnalysis
    from insightface.data import get_image

    app = FaceAnalysis(name="buffalo_sc", providers=["CPUExecutionProvider"])
    app.prepare(ctx_id=-1, det_size=(640, 640))
    t1 = get_image("t1")
    f = max(app.get(t1), key=lambda x: x.bbox[2] - x.bbox[0])
    x1, y1, x2, y2 = f.bbox.astype(int)
    bw, bh = x2 - x1, y2 - y1
    crop = t1[max(0, y1 - bh): y2 + bh, max(0, x1 - bw): x2 + bw]
    s = 65.0 / bw
    face = cv2.resize(crop, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    empty = np.full((480, 640, 3), 110, np.uint8)
    withface = empty.copy()
    h, w = face.shape[:2]
    withface[(480 - h) // 2:(480 - h) // 2 + h, (640 - w) // 2:(640 - w) // 2 + w] = face
    enc = lambda im: cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()  # noqa: E731
    return enc(empty), enc(withface)


def face_pid():
    for d in os.listdir("/proc"):
        if d.isdigit():
            try:
                cmd = open(f"/proc/{d}/cmdline", "rb").read().split(b"\0")
            except OSError:
                continue
            if any(c.endswith(b"face_embedding") for c in cmd[:2]) and b"__node:=face_node" in b" ".join(cmd):
                return int(d)
    return None


def cpu_ticks(pid):
    f = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
    return int(f[11]) + int(f[12])  # utime + stime


EMPTY, FACE = make_frames()
rclpy.init()
n = Node("face_e2e_bench_cc")
# ★ RELIABLE：user_auth 用 message_filters 的預設 QoS（RELIABLE）訂影像，BEST_EFFORT 的發布端它收不到
#   （log：offering incompatible QoS），同步器湊不成對就一則身分都不會發。真相機的 republish 也是 RELIABLE。
pub = n.create_publisher(CompressedImage, "/camera/color/image_raw/compressed", 10)
ev = []  # (時間, 種類, 內容)
n.create_subscription(FaceEmbedding, "face_embedding", lambda m: ev.append((time.time(), "emb", "")), 10)
n.create_subscription(UserIdentity, "user_identity",
                      lambda m: ev.append((time.time(), "id", f"{m.user_name or '-'}/{m.recognized}")), 10)
n.create_subscription(String, "speech_text_e2e_cc", lambda m: ev.append((time.time(), "speech", m.data)), 10)

pid = face_pid()
if pid is None:
    raise SystemExit("✗ 找不到 face_node（__node:=face_node 的 face_embedding）——先照檔頭開三個節點")
hz = os.sysconf("SC_CLK_TCK")
print(f"face_node PID {pid}；每 {1 / a.fps:.1f} s 發一幀")


def run_phase(frame, secs):
    t_end = time.time() + secs
    nxt = time.time()
    c0, w0 = cpu_ticks(pid), time.time()
    while time.time() < t_end:
        if time.time() >= nxt:
            msg = CompressedImage()
            msg.header.stamp = n.get_clock().now().to_msg()
            msg.header.frame_id = "camera_color_optical_frame"
            msg.format, msg.data = "jpeg", frame
            pub.publish(msg)
            nxt += 1.0 / a.fps
        rclpy.spin_once(n, timeout_sec=0.01)
    return 100.0 * (cpu_ticks(pid) - c0) / hz / (time.time() - w0)


time.sleep(1.0)
cpu_idle = [run_phase(EMPTY, 8.0)]
rows, cpu_face = [], []
for k in range(1, a.trials + 1):
    ev.clear()
    t_on = time.time()
    cpu_face.append(run_phase(FACE, a.on))
    first = {kind: next((t - t_on for t, kk, _ in ev if kk == kind), None) for kind in ("emb", "id", "speech")}
    said = next((x for _, kk, x in ev if kk == "speech"), "")
    ident = next((x for _, kk, x in ev if kk == "id"), "")
    n_emb = sum(1 for _, kk, _ in ev if kk == "emb")
    rows.append({"次": k, "臉→向量s": first["emb"], "臉→身分s": first["id"], "臉→迎賓s": first["speech"],
                 f"{a.on:.0f}秒內向量則數": n_emb, "身分": ident, "迎賓詞": said})
    print(rows[-1])
    cpu_idle.append(run_phase(EMPTY, a.off))

fmt = lambda v: "" if v is None else round(v, 2)  # noqa: E731
with open(os.path.join(a.out, "trials.csv"), "w", newline="", encoding="utf-8-sig") as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows([{k: (fmt(v) if isinstance(v, float) or v is None else v) for k, v in r.items()} for r in rows])


def med(key):
    xs = [r[key] for r in rows if r[key] is not None]
    return f"中位 {statistics.median(xs):.2f} s（{min(xs):.2f}~{max(xs):.2f}，{len(xs)}/{len(rows)} 次有）" if xs else "沒有收到"


temp = int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
lines = [
    f"人臉端對端 {time.strftime('%Y-%m-%d %H:%M')}：{a.trials} 次（臉出現 {a.on:.0f} s／消失 {a.off:.0f} s，{a.fps:.0f} Hz），結束時 CPU {temp:.0f}°C",
    f"臉出現 → 第一則人臉向量：{med('臉→向量s')}",
    f"臉出現 → 第一則身分：{med('臉→身分s')}",
    f"臉出現 → 迎賓語音：{med('臉→迎賓s')}",
    f"face_node CPU（% of 1 核）：沒臉 {statistics.mean(cpu_idle):.0f}%、有臉 {statistics.mean(cpu_face):.0f}%",
]
open(os.path.join(a.out, "summary.txt"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
print("\n".join(lines))
print("輸出：", a.out)
n.destroy_node()
rclpy.shutdown()
