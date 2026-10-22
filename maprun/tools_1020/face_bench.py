#!/usr/bin/env python3
"""人臉辨識離線基準（不用相機）：在 Pi 上量 buffalo_sc 每幀延遲，並示範特徵向量比對。

設定照 face_engine.py：buffalo_sc、偵測輸入 640×640、CPU、限 2 顆核心（OMP 執行緒＋CPU 親和性）。
影像用 insightface 套件內附的公開樣本（Tom_Hanks_54745.png、t1.jpg），**不讀人臉資料庫**。

量的東西：
  1. 模型載入時間
  2. 640×480 畫面、臉寬 40/65/100/160 px（約 2/1.3/0.8/0.5 m）與「沒有臉」時，
     偵測、特徵擷取、整幀（= 節點每幀做的 FaceAnalysis.get）各自的延遲
  3. 相似度：同一張臉經過亮度／模糊／縮小／旋轉／翻轉後 vs 原圖；t1.jpg 裡 6 個不同的人兩兩比較

用法：python3 face_bench.py [--iters 30] [--out ~/maprun/verify_out/face_bench_<HHMM>]
"""
import argparse
import csv
import os
import statistics
import time

ap = argparse.ArgumentParser()
ap.add_argument("--iters", type=int, default=30)
ap.add_argument("--cores", type=int, default=2)
ap.add_argument("--out", default=os.path.expanduser(f"~/maprun/verify_out/face_bench_{time.strftime('%H%M')}"))
a = ap.parse_args()

# 跟 face_engine._init_model 一樣：建 session 之前限制執行緒、綁核心
if a.cores > 0:
    for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[var] = str(a.cores)
    total = os.cpu_count() or 4
    os.sched_setaffinity(0, set(range(total - a.cores, total)))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import insightface  # noqa: E402
from insightface.app import FaceAnalysis  # noqa: E402
from insightface.data import get_image  # noqa: E402

os.makedirs(a.out, exist_ok=True)


def stats(xs):
    xs = sorted(xs)
    return {
        "平均ms": round(statistics.mean(xs) * 1000, 1),
        "中位ms": round(xs[len(xs) // 2] * 1000, 1),
        "p90ms": round(xs[min(len(xs) - 1, int(len(xs) * 0.9))] * 1000, 1),
        "最小ms": round(xs[0] * 1000, 1),
    }


t0 = time.time()
app = FaceAnalysis(name="buffalo_sc", providers=["CPUExecutionProvider"])
app.prepare(ctx_id=-1, det_thresh=0.5, det_size=(640, 640))
load_s = time.time() - t0
rec = app.models["recognition"]
print(f"模型載入 {load_s:.2f} s；模組：{sorted(app.models)}；核心 {sorted(os.sched_getaffinity(0))}")

# ── 準備畫面：t1.jpg（多人合照）裡最大的那張臉當受測者，裁出臉＋一圈背景，
#    縮放到指定臉寬後貼進 640×480 的灰底畫面（相機解析度）
#    （Tom_Hanks_54745.png 是 112×112 已對齊的臉，偵測器抓不到，只拿來當「另一個人」）
t1 = get_image("t1")
group = sorted(app.get(t1), key=lambda f: f.bbox[0])
f_big = max(group, key=lambda f: f.bbox[2] - f.bbox[0])
x1, y1, x2, y2 = f_big.bbox.astype(int)
bw, bh = x2 - x1, y2 - y1
src = t1[max(0, y1 - bh): y2 + bh, max(0, x1 - bw): x2 + bw].copy()
f0 = app.get(src, max_num=1)[0]
src_face_w = int(f0.bbox[2] - f0.bbox[0])
print(f"受測臉：t1 第 {group.index(f_big) + 1} 人，原始臉寬 {src_face_w}px；t1 共 {len(group)} 張臉")


def frame_with_face(face_w):
    s = face_w / src_face_w
    img = cv2.resize(src, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    canvas = np.full((480, 640, 3), 110, np.uint8)
    h, w = img.shape[:2]
    oy, ox = max(0, (480 - h) // 2), max(0, (640 - w) // 2)
    crop = img[: min(h, 480), : min(w, 640)]
    canvas[oy:oy + crop.shape[0], ox:ox + crop.shape[1]] = crop
    return canvas


cases = [("沒有臉", np.full((480, 640, 3), 110, np.uint8))]
cases += [(f"臉寬{w}px", frame_with_face(w)) for w in (40, 65, 100, 160)]

rows = []
for name, img in cases:
    for _ in range(3):                           # 暖機
        app.get(img, max_num=1)
    det_t, rec_t, full_t, found = [], [], [], 0
    for _ in range(a.iters):
        t = time.perf_counter()
        bboxes, kpss = app.det_model.detect(img, max_num=1, metric="default")
        det_t.append(time.perf_counter() - t)
        if len(bboxes):
            face = insightface.app.common.Face(bbox=bboxes[0, :4], kps=kpss[0], det_score=bboxes[0, 4])
            t = time.perf_counter()
            rec.get(img, face)
            rec_t.append(time.perf_counter() - t)
        t = time.perf_counter()
        n = len(app.get(img, max_num=1))
        full_t.append(time.perf_counter() - t)
        found += n > 0
    r = {"情境": name, "次數": a.iters, "偵測到臉%": round(100 * found / a.iters)}
    r.update({f"偵測_{k}": v for k, v in stats(det_t).items()})
    if rec_t:
        r.update({f"特徵_{k}": v for k, v in stats(rec_t).items()})
    r.update({f"整幀_{k}": v for k, v in stats(full_t).items()})
    rows.append(r)
    print(r)

keys = []
for r in rows:
    keys += [k for k in r if k not in keys]
with open(os.path.join(a.out, "latency.csv"), "w", newline="", encoding="utf-8-sig") as fh:
    w = csv.DictWriter(fh, fieldnames=keys)
    w.writeheader()
    w.writerows(rows)

# ── 相似度示範：特徵向量（512 維、L2 正規化）的餘弦相似度＝內積
emb0 = f0.normed_embedding


def emb_of(img):
    fs = app.get(img, max_num=1)
    return fs[0].normed_embedding if fs else None


aug = {
    "原圖再算一次": src,
    "左右翻轉": cv2.flip(src, 1),
    "變暗 40%": cv2.convertScaleAbs(src, alpha=0.6, beta=0),
    "變亮 +40": cv2.convertScaleAbs(src, alpha=1.0, beta=40),
    "高斯模糊 σ=2": cv2.GaussianBlur(src, (0, 0), 2),
    "旋轉 15°": cv2.warpAffine(src, cv2.getRotationMatrix2D((src.shape[1] / 2, src.shape[0] / 2), 15, 1.0),
                               (src.shape[1], src.shape[0])),
    "縮到臉寬 65px（約 1.3 m）": frame_with_face(65),
    "縮到臉寬 40px（約 2 m）": frame_with_face(40),
}
sim_rows = []
for k, img in aug.items():
    e = emb_of(img)
    sim_rows.append(["同一人", k, "" if e is None else round(float(np.dot(emb0, e)), 3)])

ge = [f.normed_embedding for f in group]
diff = [float(np.dot(ge[i], ge[j])) for i in range(len(ge)) for j in range(i + 1, len(ge))]
for i in range(len(ge)):
    for j in range(i + 1, len(ge)):
        sim_rows.append(["不同人", f"t1 第{i + 1}人 vs 第{j + 1}人", round(float(np.dot(ge[i], ge[j])), 3)])
# Tom Hanks：已對齊的 112×112，直接送特徵模型
th = rec.get_feat(get_image("Tom_Hanks_54745")).flatten()
th /= np.linalg.norm(th)
cross = [float(np.dot(th, e)) for e in ge]
for i, s in enumerate(cross):
    sim_rows.append(["不同人", f"Tom Hanks vs t1 第{i + 1}人", round(s, 3)])

with open(os.path.join(a.out, "similarity.csv"), "w", newline="", encoding="utf-8-sig") as fh:
    w = csv.writer(fh)
    w.writerow(["類別", "比較", "餘弦相似度"])
    w.writerows(sim_rows)

same = [r[2] for r in sim_rows if r[0] == "同一人" and r[2] != ""]
other = diff + cross
summary = (
    f"模型載入 {load_s:.2f} s（buffalo_sc、偵測 640、CPU {a.cores} 核心）\n"
    + "\n".join(
        f"{r['情境']}: 整幀中位 {r['整幀_中位ms']} ms（偵測 {r['偵測_中位ms']} ms"
        + (f" + 特徵 {r['特徵_中位ms']} ms" if "特徵_中位ms" in r else "")
        + f"），偵測到臉 {r['偵測到臉%']}%"
        for r in rows
    )
    + f"\n相似度：同一人變化 {min(same):.3f}~{max(same):.3f}（{len(same)} 組）；"
    f"不同人 {min(other):.3f}~{max(other):.3f}（{len(other)} 組，t1 偵測到 {len(ge)} 張臉）\n"
)
open(os.path.join(a.out, "summary.txt"), "w", encoding="utf-8").write(summary)
print(summary)
print("輸出：", a.out)
