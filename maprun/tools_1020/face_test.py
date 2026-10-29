#!/usr/bin/env python3
"""人臉辨識 × 距離（交接文件第 9 項人臉部分：新場地重新註冊使用者；記辨識結果與距離）。

用法（車上。平板「測試情境」選「測人臉辨識 / 註冊」＝相機人臉模式＋人臉辨識＋使用者認證；
導航一定要關——導航中人臉節點完全不推論）：
    source ~/welcoming_robot_ws/src/smartnav_ws/maprun/env.sh
    # ① 註冊時開著（建議）：記下「這次新註冊的是哪一筆」的雜湊，之後才分得出本人／別的已註冊者（例如舊場地的舊檔）
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/face_test.py --watch-registration          # 先開這個，再去平板按「開始註冊」
    # ② 每個距離一輪（人先站好、正對鏡頭，再開始）
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/face_test.py --label 1.0m --secs 20 --expect-registered
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/face_test.py --label stranger --secs 20    # 不加 --expect-registered＝這個人「應該認不出來」
    # ③ 今天所有輪的總表（--date 20261020 指定日期、--date all 全部）
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/face_test.py --summary
    # ④ 某個名字對應哪個名字雜湊（名字用不回顯的方式輸入，不進 shell 歷史、不上螢幕）
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/face_test.py --hash-name

讀什麼（全部唯讀：只訂閱，不發話題、不呼叫服務、不碰 HMI API、不讀人臉資料庫）：
    /user_identity   smartnav_msgs/UserIdentity，user_auth_node 發。★ 只有偵測到臉才發（face_embedding 每 2 秒一幀 → 最多約
                     0.5 Hz）；人走掉 10 秒後另發一則「無人」（bbox 全 0、similarity 0）。用到 recognized、similarity
                     （(cos+1)/2，門檻 recognition_threshold 預設 0.70）、bbox [x1,y1,x2,y2]（640x480 彩色影像像素）、
                     header.stamp（＝影像時間，算延遲）、user_uuid（只算雜湊來比對，不寫檔、不上螢幕）。
    /camera/color/camera_info    取 fx（K[0]）；收到一則就退訂。
    /registration_progress       只有 --watch-registration 用（collected/target/status；message 欄含名字，不讀）。
    ~/.smartnav/logs/system_control/user_auth_*.log   同時段「身份驗證成功／未達門檻」筆數與相似度，對照話題有沒有漏收（不讀名字）。

距離怎麼估：人臉模式只開彩色（camera_face_cc.launch.py：enable_depth=false），沒有深度可查 →
    距離 ≈ fx × 臉寬 ÷ bbox 寬(px)，臉寬預設 0.15 m（--face-width-m）。這是**估計**；捲尺量的距離寫在 --label。
    標籤是距離（1.0m、50cm…）的輪次，總表另算「推算臉寬」＝標稱距離 × bbox 寬 ÷ fx：各輪一致（差 10% 內）表示估計可信，
    也可以拿 1 m 那輪的值重設 --face-width-m。

輸出：每則一列  ~/welcoming_robot_ws/src/smartnav_ws/maprun/verify_out/face_test_<YYYYMMDD>/<HHMMSS>_<標籤>.csv
      每輪一列  ~/welcoming_robot_ws/src/smartnav_ws/maprun/verify_out/face_test_<YYYYMMDD>.tsv（註冊另記 face_test_<YYYYMMDD>_reg.tsv）
分類（只記這幾種，**不記名字**）：
    registered_match  認出（recognized=True）且是 ① 記下的那筆；沒有 ① 的雜湊時分不出是誰，認出就記這個
    other_registered  認出，但不是 ① 記下的那筆（例如舊場地的舊檔比新檔還像）
    unknown           有臉、未過門檻
    no_face           「無人」訊息，不算進辨識率與相似度
    辨識率＝認出／有臉；判對＝預期已註冊時 registered_match／有臉，預期未註冊時 unknown／有臉。
    名字雜湊：認出時另記 sha256("smartnav-1020:"+名字) 前 8 碼（每則一欄、每輪記各雜湊次數）——同一人的新檔與舊檔
    是兩個雜湊，被認成別的已註冊者則是第三個；哪個雜湊是誰用 ④ 對。名字本身不寫檔、不上螢幕。

★ 註冊的坑（讀 user_auth_node.py／UsersPage.tsx 得知）：同名會被拒（「使用者名稱已存在」，而且**不發進度**，① 會一直等不到）
  ——要嘛先在平板刪掉舊檔，要嘛換名字。平板提示寫「20 秒上限」是舊字，實際逾時 45 秒（逾時會把這筆使用者自動刪掉）。
★ 只讓受測者一個人入鏡：face_embedding 每幀只取一張臉（max_num=1，最大、最靠中間的那張）。
★ 2026-10-20 車上新寫，只用假資料驗證過（隔離的 ROS_DOMAIN_ID=77 假發布端＋假 log），沒有在實車上跑過。
"""
import argparse
import collections
import csv
import datetime as dt
import glob
import hashlib
import json
import os
import re
import statistics
import sys
import time
import unicodedata

OUT_DIR = os.path.expanduser("~/welcoming_robot_ws/src/smartnav_ws/maprun/verify_out")
LOG_DIR = os.path.expanduser("~/.smartnav/logs/system_control")
# 只在收不到 camera_info 又沒給 --fx 時用：Astra S 彩色規格 HFOV 60°、寬 640 px → 320 / tan 30°
DEFAULT_FX = 554.3
EPOCH = re.compile(r"\[(\d{10}\.\d+)\]")
LOG_OK = re.compile(r"相似度: ([0-9]+\.[0-9]+), 描述")
LOG_REJ = re.compile(r"」相似度 ([0-9]+\.[0-9]+) < ")
STATUS_ZH = {"running": "進行中", "succeeded": "完成", "timeout": "逾時", "cancelled": "取消", "failed": "失敗"}
CSV_COLS = ["t_wall", "t_rel_s", "result", "name_hash", "recognized", "similarity", "bbox_x1", "bbox_y1", "bbox_x2",
            "bbox_y2", "bbox_w_px", "bbox_h_px", "dist_est_m", "dist_method", "stamp_age_s"]
TSV_COLS = ["時間", "標籤", "預期", "預期身份", "秒數", "則數", "Hz", "有臉則", "registered_match", "other_registered",
            "unknown", "no_face", "名字雜湊次數", "辨識率%", "判對%", "sim平均", "sim最小", "sim最大", "距離中位m",
            "bbox寬中位px", "fx", "fx來源", "臉寬假設m", "推算臉寬m", "延遲中位s", "log成功/未達", "csv"]
REG_COLS = ["時間", "狀態", "樣本", "目標", "耗時s", "身份雜湊", "名字雜湊"]


# ── 小工具 ──────────────────────────────────────────────────
def uid_hash(uuid: str) -> str:
    """UUID → 16 碼雜湊。檔案裡只存這個，不存 UUID 本身，更不存名字。"""
    return hashlib.sha256(("face_test_cc|" + uuid).encode()).hexdigest()[:16]


def name_hash(name: str) -> str:
    """名字 → sha256("smartnav-1020:"+名字) 前 8 碼。名字只在這裡經手，不落地、不上螢幕。"""
    return hashlib.sha256(("smartnav-1020:" + name.strip()).encode("utf-8")).hexdigest()[:8]


def hash_counts(rows) -> str:
    c = collections.Counter(r["nh"] for r in rows if r.get("nh"))
    return " ".join(f"{h}:{n}" for h, n in c.most_common())


def classify(recognized: bool, uuid: str, bbox, expected_sha):
    if not recognized and all(abs(v) < 1e-6 for v in bbox):
        return "no_face"            # user_auth 在 identity_timeout_sec 沒看到臉後發的「無人」
    if not recognized:
        return "unknown"
    if expected_sha and uid_hash(uuid) != expected_sha:
        return "other_registered"
    return "registered_match"


def nominal_m(label: str):
    """標籤開頭是距離（1.0m、50cm、1.5、2m_側臉）就回公尺，否則 None。"""
    m = re.match(r"\s*(\d+(?:\.\d+)?)\s*(cm|m|公尺|米)?(?![0-9A-Za-z])", label or "")
    if not m:
        return None
    v = float(m.group(1))
    return v / 100.0 if m.group(2) == "cm" else v


def med(xs):
    return statistics.median(xs) if xs else None


def cell(x, nd):
    return "" if x is None else f"{x:.{nd}f}"


def fmt(x, nd=3):
    return "—" if x is None else f"{x:.{nd}f}"


def pct(num, den):
    return 100.0 * num / den if den else None


def safe(label: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", label).strip("_") or "run"


def append_tsv(path, cols, row):
    new = not os.path.exists(path)
    with open(path, "a", encoding="utf-8") as fh:
        if new:
            fh.write("\t".join(cols) + "\n")
        fh.write("\t".join(str(row.get(c, "")).replace("\t", " ") for c in cols) + "\n")


def expected_path(out_dir):
    return os.path.join(out_dir, "face_test_expected_id.json")


def load_expected(out_dir):
    try:
        with open(expected_path(out_dir), encoding="utf-8") as fh:
            d = json.load(fh)
        return d if d.get("sha") else None
    except (OSError, ValueError):
        return None


def log_crosscheck(log_dir, t0, t1):
    """同時段 user_auth log 的筆數與相似度。★ 只抓時間戳與數字，整行（含名字）從不印出、不存。"""
    out = {"files": 0, "ok": 0, "rej": 0, "cleared": 0, "ok_sims": [], "rej_sims": []}
    for path in glob.glob(os.path.join(log_dir, "user_auth_*.log")):
        try:
            if os.path.getmtime(path) < t0 - 1.0:
                continue
            out["files"] += 1
            with open(path, encoding="utf-8", errors="replace") as fh:
                for ln in fh:
                    m = EPOCH.search(ln)
                    if not m or not (t0 - 0.5 <= float(m.group(1)) <= t1 + 0.5):
                        continue
                    if "身份驗證成功" in ln:
                        out["ok"] += 1
                        s = LOG_OK.search(ln)
                        if s:
                            out["ok_sims"].append(float(s.group(1)))
                    elif "未達門檻" in ln:
                        out["rej"] += 1
                        s = LOG_REJ.search(ln)
                        if s:
                            out["rej_sims"].append(float(s.group(1)))
                    elif "身份清除為『無人』" in ln:
                        out["cleared"] += 1
        except OSError:
            continue
    return out


def dwidth(s: str) -> int:
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in s)


def print_table(header, rows):
    widths = [max([dwidth(h)] + [dwidth(r[i]) for r in rows if i < len(r)]) for i, h in enumerate(header)]
    pad = lambda s, w: s + " " * (w - dwidth(s))
    print("  ".join(pad(h, w) for h, w in zip(header, widths)))
    for r in rows:
        print("  ".join(pad(r[i] if i < len(r) else "", w) for i, w in enumerate(widths)))


# ── --summary ───────────────────────────────────────────────
SUMMARY = [("時間", "時間"), ("標籤", "標籤"), ("預期", "預期"), ("秒數", "秒"), ("則數", "則"), ("Hz", "Hz"),
           ("registered_match", "match"), ("other_registered", "other"), ("unknown", "unk"), ("no_face", "無人"),
           ("辨識率%", "辨識%"), ("判對%", "判對%"), ("sim平均", "sim平均"), ("sim最小", "最小"), ("sim最大", "最大"),
           ("距離中位m", "距離m"), ("bbox寬中位px", "寬px"), ("推算臉寬m", "推算臉寬"), ("log成功/未達", "log"),
           ("名字雜湊次數", "名字雜湊:次數")]


def do_summary(a):
    if a.date == "all":
        paths = sorted(p for p in glob.glob(os.path.join(a.out_dir, "face_test_2*.tsv")) if not p.endswith("_reg.tsv"))
        regs = sorted(glob.glob(os.path.join(a.out_dir, "face_test_2*_reg.tsv")))
    else:
        paths = [os.path.join(a.out_dir, f"face_test_{a.date}.tsv")]
        regs = [os.path.join(a.out_dir, f"face_test_{a.date}_reg.tsv")]
    rows = []
    for p in paths:
        if os.path.exists(p):
            with open(p, encoding="utf-8") as fh:
                rows += list(csv.DictReader(fh, delimiter="\t"))
    print(f"── 人臉辨識 × 距離（{a.date}）共 {len(rows)} 輪 ──")
    if rows:
        cut = (lambda s: s[11:]) if a.date != "all" else (lambda s: s)
        print_table([h for _, h in SUMMARY],
                    [[cut(r.get(k, "")) if k == "時間" else r.get(k, "") for k, _ in SUMMARY] for r in rows])
        print("（match/other/unk/無人＝registered_match/other_registered/unknown/no_face 則數；"
              "距離是 bbox 寬估的，推算臉寬＝標稱距離 × bbox 寬 ÷ fx）")
    else:
        print("（還沒有任何一輪）")
    reg_rows = []
    for p in regs:
        if os.path.exists(p):
            with open(p, encoding="utf-8") as fh:
                reg_rows += list(csv.DictReader(fh, delimiter="\t"))
    if reg_rows:
        print(f"\n── 註冊 {len(reg_rows)} 次 ──")
        print_table(REG_COLS, [[r.get(c, "") for c in REG_COLS] for r in reg_rows])
    exp = load_expected(a.out_dir)
    if exp:
        print(f"\n目前的預期身份：{exp.get('recorded', '?')} 註冊時記下（雜湊 {exp['sha'][:6]}…）")
    return 0


# ── 量測 ────────────────────────────────────────────────────
def do_measure(a):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo
    from smartnav_msgs.msg import UserIdentity

    exp = load_expected(a.out_dir) if a.expect_registered and not a.no_expected_id else None
    exp_sha = exp["sha"] if exp else None
    zh = {"registered_match": "認出本人" if exp_sha else "認出", "other_registered": "認成別人", "unknown": "認不出"}

    class Rec(Node):
        def __init__(self):
            # 不開 rosout、不開參數服務：除了 rclpy 一定會建的 /parameter_events，不在 graph 上多留任何東西
            super().__init__("face_test_cc", enable_rosout=False, start_parameter_services=False)
            self.rows = []
            self.fx = None
            self.ci_wh = None
            self.t0 = time.time()
            self.ci_sub = self.create_subscription(CameraInfo, a.camera_info_topic, self._on_ci, qos_profile_sensor_data)
            self.create_subscription(UserIdentity, a.topic, self._on_id, 10)

        def _on_ci(self, m):
            if self.fx is None:
                f = float(m.k[0]) if m.k[0] > 0 else float(m.p[0])
                if f > 0:
                    self.fx, self.ci_wh = f, (int(m.width), int(m.height))

        def _on_id(self, m):
            now = time.time()
            bbox = [float(v) for v in m.bbox]
            cat = classify(bool(m.recognized), m.user_uuid, bbox, exp_sha)
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            nh = name_hash(m.user_name) if m.recognized and m.user_name else ""   # 名字只拿來算雜湊
            r = {"t": now, "result": cat, "nh": nh, "recognized": int(bool(m.recognized)), "sim": float(m.similarity),
                 "bbox": bbox, "w": bbox[2] - bbox[0], "h": bbox[3] - bbox[1], "age": (now - st) if st > 0 else None}
            self.rows.append(r)
            if cat == "no_face":
                print(f"  +{now - self.t0:5.1f}s  無人（user_auth 清除身份）", flush=True)
                return
            fx = a.fx or self.fx or DEFAULT_FX
            d = fx * a.face_width_m / r["w"] if r["w"] > 1 else None
            print(f"  +{now - self.t0:5.1f}s  {zh[cat]:<4} {nh or '-' * 8}  sim {r['sim']:.3f}  bbox 寬 {r['w']:4.0f} px  "
                  f"≈{fmt(d, 2)} m  延遲 {fmt(r['age'], 1)} s", flush=True)

    rclpy.init()
    node = Rec()
    m0 = time.monotonic()
    t0 = node.t0
    print(f"量測 {a.secs:.0f} 秒｜標籤 {a.label}｜預期：{'已註冊（應該認得出來）' if a.expect_registered else '未註冊（應該認不出來）'}")
    if a.expect_registered:
        print(f"  預期身份：{exp.get('recorded', '?')} --watch-registration 記下的那筆（雜湊 {exp_sha[:6]}…）→ 分得出 "
              f"registered_match／other_registered" if exp_sha else
              "  預期身份：沒有記錄（沒跑過 --watch-registration，或加了 --no-expected-id）→ 認出一律記 registered_match，"
              "分不出新檔／舊檔")
    print(f"  距離估計：fx × {a.face_width_m:.3f} m ÷ bbox 寬（人臉模式沒有深度）；只讓受測者一個人入鏡。Ctrl-C 可提早結束。",
          flush=True)
    n_pub, interrupted = None, False
    try:
        while time.monotonic() - m0 < a.secs:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ci_sub is not None and node.fx is not None:
                node.destroy_subscription(node.ci_sub)      # fx 有了就退訂
                node.ci_sub = None
            if n_pub is None and time.monotonic() - m0 > 1.5:
                n_pub = node.count_publishers(a.topic)
                if n_pub == 0:
                    print(f"  ⚠ {a.topic} 目前沒有發布端（「使用者認證 / 決策」單元沒開？）", flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        interrupted = True
    t1, elapsed = time.time(), time.monotonic() - m0
    if n_pub is None:
        try:
            n_pub = node.count_publishers(a.topic)
        except Exception:  # noqa: BLE001 —— Ctrl-C 後 context 可能已關
            pass
    rows, cam_fx, ci_wh = node.rows, node.fx, node.ci_wh
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    # ── 統計 ──
    if a.fx:
        fx, fx_src, method = a.fx, "--fx", "bbox_w+fx:arg"
    elif cam_fx:
        fx, fx_src, method = cam_fx, "camera_info", "bbox_w+fx:camera_info"
    else:
        fx, fx_src, method = DEFAULT_FX, "預設值", "bbox_w+fx:default"
    for r in rows:
        r["dist"] = fx * a.face_width_m / r["w"] if r["result"] != "no_face" and r["w"] > 1 else None
    cnt = collections.Counter(r["result"] for r in rows)
    face = [r for r in rows if r["result"] != "no_face"]
    nf = len(face)
    recog = cnt["registered_match"] + cnt["other_registered"]
    good = cnt["registered_match"] if a.expect_registered else cnt["unknown"]
    sims = [r["sim"] for r in face]
    ws = [r["w"] for r in face if r["w"] > 1]
    wmed, dmed = med(ws), med([r["dist"] for r in face if r["dist"] is not None])
    nom = nominal_m(a.label)
    implied = nom * wmed / fx if (nom and wmed) else None
    hz = len(rows) / elapsed if elapsed > 0 else 0.0
    lc = log_crosscheck(a.log_dir, t0, t1 + 1.0)

    # ── 逐則 CSV ──
    day = time.strftime("%Y%m%d", time.localtime(t0))
    csv_dir = os.path.join(a.out_dir, f"face_test_{day}")
    os.makedirs(csv_dir, exist_ok=True)
    csv_path = os.path.join(csv_dir, f"{time.strftime('%H%M%S', time.localtime(t0))}_{safe(a.label)}.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(CSV_COLS)
        for r in rows:
            w.writerow([dt.datetime.fromtimestamp(r["t"]).isoformat(timespec="milliseconds"), f"{r['t'] - t0:.2f}",
                        r["result"], r["nh"], r["recognized"], f"{r['sim']:.4f}", *(f"{v:.1f}" for v in r["bbox"]),
                        f"{r['w']:.1f}", f"{r['h']:.1f}", cell(r["dist"], 3), method if r["dist"] is not None else "",
                        cell(r["age"], 3)])

    # ── 每輪一列 TSV ──
    tsv_path = os.path.join(a.out_dir, f"face_test_{day}.tsv")
    append_tsv(tsv_path, TSV_COLS, {
        "時間": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t0)), "標籤": a.label,
        "預期": "已註冊" if a.expect_registered else "未註冊",
        "預期身份": f"雜湊{exp_sha[:6]}" if exp_sha else ("無雜湊" if a.expect_registered else "—"),
        "秒數": f"{elapsed:.1f}", "則數": len(rows), "Hz": f"{hz:.2f}", "有臉則": nf,
        "registered_match": cnt["registered_match"], "other_registered": cnt["other_registered"],
        "unknown": cnt["unknown"], "no_face": cnt["no_face"], "名字雜湊次數": hash_counts(rows),
        "辨識率%": cell(pct(recog, nf), 0), "判對%": cell(pct(good, nf), 0),
        "sim平均": cell(statistics.mean(sims) if sims else None, 3),
        "sim最小": cell(min(sims) if sims else None, 3), "sim最大": cell(max(sims) if sims else None, 3),
        "距離中位m": cell(dmed, 2), "bbox寬中位px": cell(wmed, 0), "fx": f"{fx:.1f}", "fx來源": fx_src,
        "臉寬假設m": f"{a.face_width_m:.3f}", "推算臉寬m": cell(implied, 3),
        "延遲中位s": cell(med([r["age"] for r in face if r["age"] is not None]), 2),
        "log成功/未達": f"{lc['ok']}/{lc['rej']}" if lc["files"] else "無log",
        "csv": os.path.relpath(csv_path, a.out_dir),
    })

    # ── 螢幕：一行總結＋補充 ──
    p = lambda x: "—" if x is None else f"{x:.0f}%"
    print(f"\n[{a.label}] {elapsed:.1f}s 收 {len(rows)} 則 {hz:.2f} Hz｜有臉 {nf}（registered_match {cnt['registered_match']}"
          f"／other_registered {cnt['other_registered']}／unknown {cnt['unknown']}）無人 {cnt['no_face']}"
          f"｜名字雜湊 {hash_counts(rows) or '—'}｜辨識率 {p(pct(recog, nf))}｜判對 {p(pct(good, nf))}"
          f"｜sim 平均/最小/最大 {fmt(statistics.mean(sims) if sims else None)}/{fmt(min(sims) if sims else None)}/"
          f"{fmt(max(sims) if sims else None)}｜距離中位 {fmt(dmed, 2)} m（bbox 寬中位 {fmt(wmed, 0)} px、"
          f"fx {fx:.1f} {fx_src}、臉寬 {a.face_width_m:.2f} m）", flush=True)
    if fx_src == "camera_info" and ci_wh:
        print(f"  fx 取自 {a.camera_info_topic}（{ci_wh[0]}x{ci_wh[1]}）")
    elif fx_src == "預設值":
        print(f"  ⚠ 沒收到 {a.camera_info_topic}，fx 用預設 {DEFAULT_FX}（Astra S 規格 HFOV 60° 換算）——距離只能看相對大小")
    if implied is not None:
        print(f"  推算臉寬 {implied:.3f} m（標稱 {nom:.2f} m × bbox 寬 {wmed:.0f} px ÷ fx {fx:.1f}）；各輪一致才表示距離估計可信")
    if lc["files"]:
        ms = lambda xs: f"，sim 平均 {statistics.mean(xs):.3f}" if xs else ""
        print(f"  log 對照（user_auth_*.log 同時段）：身份驗證成功 {lc['ok']} 筆{ms(lc['ok_sims'])}、"
              f"未達門檻 {lc['rej']} 筆{ms(lc['rej_sims'])}（這行 log 有 2 秒節流）、清除身份 {lc['cleared']} 次")
    else:
        print(f"  log 對照：{a.log_dir} 沒有這段時間寫過的 user_auth_*.log")
    if not rows:
        print(f"  ⚠ {elapsed:.0f} 秒內一則 {a.topic} 都沒收到（發布端 {n_pub if n_pub is not None else '?'} 個）。"
              "可能：鏡頭前沒人／臉沒偵測到（只在有臉時才發）、導航開著（導航中不推論）、相機或人臉單元沒開。")
        if lc["ok"] + lc["rej"] > 0:
            print("  ⚠ 但 log 同時段有辨識紀錄 → 話題收不到：檢查 ROS_DOMAIN_ID、話題名，或 smartnav_msgs 只建了部分套件"
                  "（型別雜湊不符時訂閱端靜默收不到）")
    if interrupted:
        print(f"  （Ctrl-C 提早結束，Hz 用實際 {elapsed:.1f} 秒算）")
    print(f"  逐則：{csv_path}\n  總表：{tsv_path}（--summary 看全部）", flush=True)
    return 0


# ── 註冊進度 ────────────────────────────────────────────────
def do_watch(a):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from smartnav_msgs.msg import RegistrationProgress

    class Watch(Node):
        def __init__(self):
            super().__init__("face_test_reg_cc", enable_rosout=False, start_parameter_services=False)
            self.first = None
            self.last = None
            self.final = None
            self.create_subscription(RegistrationProgress, "/registration_progress", self._cb, 10)

        def _cb(self, m):
            # ★ m.user_name 與 m.message（「某某」註冊完成…）都含名字：一律不讀
            now = time.monotonic()
            if self.first is None:
                self.first = now
            st, got, tgt = str(m.status), int(m.collected_samples), int(m.target_samples)
            self.last = (st, got, tgt)
            print(f"  +{now - self.first:5.1f}s  {STATUS_ZH.get(st, st)}  {got}/{tgt} 張", flush=True)
            if not m.active and self.final is None:
                self.final = {"status": st, "collected": got, "target": tgt, "dur": now - self.first,
                              "sha": uid_hash(m.user_uuid) if m.user_uuid else "",
                              "nh": name_hash(m.user_name) if m.user_name else ""}   # 名字只拿來算雜湊

    rclpy.init()
    node = Watch()
    m0 = time.monotonic()
    print(f"等註冊進度（最多 {a.secs:.0f} 秒；只看張數與狀態，不讀名字）。現在去平板：")
    print("  使用者管理 →「註冊新使用者」：輸入姓名（同名會被拒）、選類型 →「即時採樣」、採樣張數（預設 10）→ 按「開始註冊」，")
    print("  受測者馬上站到鏡頭前約 1 m、正對、稍微轉頭，站到顯示完成為止（每 2 秒收 1 張，10 張約 20 秒；45 秒沒收滿會自動取消並刪掉這筆）。",
          flush=True)
    try:
        while time.monotonic() - m0 < a.secs and node.final is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    fin, last, seen = node.final, node.last, node.first is not None
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    now_s = time.strftime("%Y-%m-%d %H:%M:%S")
    if fin:
        reg_path = os.path.join(a.out_dir, f"face_test_{time.strftime('%Y%m%d')}_reg.tsv")
        append_tsv(reg_path, REG_COLS, {"時間": now_s, "狀態": fin["status"], "樣本": fin["collected"],
                                        "目標": fin["target"], "耗時s": f"{fin['dur']:.1f}",
                                        "身份雜湊": fin["sha"][:6] if fin["sha"] else "", "名字雜湊": fin["nh"]})
        msg = f"註冊結果：{STATUS_ZH.get(fin['status'], fin['status'])} {fin['collected']}/{fin['target']} 張，" \
              f"耗時 {fin['dur']:.1f} s（第一則進度 → 結束）；這筆的名字雜湊 {fin['nh'] or '—'}"
        if fin["status"] == "succeeded" and fin["sha"]:
            with open(expected_path(a.out_dir), "w", encoding="utf-8") as fh:
                json.dump({"sha": fin["sha"], "recorded": now_s, "status": fin["status"], "collected": fin["collected"],
                           "target": fin["target"], "duration_s": round(fin["dur"], 1)}, fh, ensure_ascii=False)
            msg += f"；已記下這筆的身份雜湊 {fin['sha'][:6]}…（不存名字、不存 UUID），之後 --expect-registered 會用它"
        elif fin["status"] == "timeout":
            msg += "；user_auth 已自動刪掉這筆使用者，要重新註冊（人要一直正對鏡頭、導航要關）"
        print(msg)
        print(f"  記錄：{reg_path}")
    elif seen:
        print(f"到時間了，註冊還沒結束（最後 {STATUS_ZH.get(last[0], last[0])} {last[1]}/{last[2]} 張）——沒有記下身份雜湊")
    else:
        print(f"{a.secs:.0f} 秒內沒收到任何註冊進度。可能：① 平板顯示「使用者名稱已存在」——同名不會發進度，先刪舊檔或換名字；"
              "② 「使用者認證 / 決策」單元沒開；③ 還沒按「開始註冊」。")
    return 0


def main():
    ap = argparse.ArgumentParser(description="人臉辨識 × 距離量測（唯讀；不記名字）")
    ap.add_argument("--label", help="這輪的標籤，建議寫捲尺量的距離：1.0m、50cm、2.0m；未註冊者可寫 stranger")
    ap.add_argument("--secs", type=float, default=None, help="量多久（秒）；預設 20，--watch-registration 預設 120")
    ap.add_argument("--expect-registered", action="store_true", help="鏡頭前的人是已註冊者（應該認得出來）")
    ap.add_argument("--no-expected-id", action="store_true", help="不用 --watch-registration 記下的身份雜湊")
    ap.add_argument("--watch-registration", action="store_true", help="看註冊進度；成功就記下新註冊者的身份雜湊")
    ap.add_argument("--summary", action="store_true", help="印總表（--date YYYYMMDD 或 all）")
    ap.add_argument("--hash-name", action="store_true", help="輸入名字（不回顯）→ 只印它的名字雜湊")
    ap.add_argument("--date", default=time.strftime("%Y%m%d"))
    ap.add_argument("--face-width-m", type=float, default=0.15, help="bbox 寬對應的實際臉寬（m），預設 0.15")
    ap.add_argument("--fx", type=float, default=None, help="手動指定 fx（px）；不給就用 camera_info")
    ap.add_argument("--topic", default="/user_identity")
    ap.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    ap.add_argument("--out-dir", default=OUT_DIR)
    ap.add_argument("--log-dir", default=LOG_DIR)
    a = ap.parse_args()
    a.out_dir = os.path.expanduser(a.out_dir)
    if a.hash_name:
        import getpass
        print(name_hash(getpass.getpass("名字（不會顯示）：")))
        return 0
    if a.summary:
        return do_summary(a)
    os.makedirs(a.out_dir, exist_ok=True)
    if a.watch_registration:
        a.secs = a.secs or 120.0
        return do_watch(a)
    if not a.label:
        ap.error("要給 --label（例如 --label 1.0m），或用 --summary／--watch-registration")
    a.secs = a.secs or 20.0
    return do_measure(a)


if __name__ == "__main__":
    sys.exit(main())
