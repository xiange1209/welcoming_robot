#!/usr/bin/env python3
"""第 5 步自動化：在兩個地點之間自動來回帶位，每趟結果自動記錄（不用平板、不用每趟按 Enter）。

★ 2026-10-27 新寫。使用者：「這些都沒有自動化的方式?因為這樣很難測跟計算」。
  以前每趟要在平板按帶位、到了回 SSH 按 Enter，新舊兩組 10 趟、急停還要自己找時機按。
  現在：腳本自己發導航（HMI /api/navigate，跟平板同一條路）、等作業結束、用 nav_result_line.py 把這趟寫進 TSV。

用法（導航單元要在跑、而且已經定位）：
    python3 nav_ab_auto.py --goto 櫃台                                   # 先開到起點（不記錄）
    python3 nav_ab_auto.py --a 櫃台 --b 大廳 --trips 4 --tag 新 --tsv nav_ab.tsv   # A→B→A→B，每趟記一列
    python3 nav_ab_auto.py --a 櫃台 --b 大廳 --estop-trip --tag 急停 --tsv nav_ab.tsv   # 開到 B 途中自動按急停
★ 安全：車會自己連續跑。路線淨空、旁邊一定有人拿平板急停。這個視窗按 Ctrl-C＝立刻急停並結束（結束碼 130）。
"""
import argparse
import glob
import json
import os
import re
import subprocess
import sys
import time
import urllib.error
import urllib.request

T = os.path.dirname(os.path.abspath(__file__))
BASE = "http://127.0.0.1:8080"
FINAL = ("succeeded", "failed", "cancelled", "rejected")
TRIP_LIMIT_S = 360          # HMI 的導航作業自己 300 秒逾時；這裡多等一點
# ★ 10/27 傍晚：原本是「作業 running 後 6 秒」就按——當天那一趟按下時車還沒動（規劃＋出發前淨空檢查比 6 秒久），
#   量到「無效：按下時車沒在動」。改成看導航 log 的「開始導航到 X」（車子真的起跑）之後幾秒才按。
ESTOP_AFTER_S = 3.0         # 自動急停：log 出現「開始導航到」後幾秒按
ESTOP_WAIT_START_S = 90.0   # 等起跑最多這麼久；沒起跑就不按（記成沒量到）
NAV_LOGS = os.path.expanduser("~/.smartnav/logs/system_control/nav_*.log")
START_RE = re.compile(r"\[(\d{10}\.\d+)\] \[navigation_action_cc_node\]: 開始導航到 ")
TSV_HEADER = "標籤\t時間\t目標\t結果\t秒\t終點誤差m\t朝向差度\t偏離中位cm\t偏離p95cm\t偏離最大cm\t重新規劃\t折返\t原因\n"


def call(method, path, body=None, token=None, timeout=30):
    h = {"Content-Type": "application/json"}
    if token:
        h["Authorization"] = "Bearer " + token
    req = urllib.request.Request(BASE + path, method=method,
                                 data=json.dumps(body).encode() if body is not None else None, headers=h)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode() or "{}")
    except urllib.error.HTTPError as e:
        raw = e.read().decode(errors="replace")
        try:
            return e.code, json.loads(raw or "{}")
        except Exception:
            return e.code, {"message": raw[:200]}
    except (urllib.error.URLError, OSError) as e:
        return 0, {"message": f"連不到 HMI：{e}"}


def login():
    env = subprocess.run(["systemctl", "show", "smartnav-hmi", "-p", "Environment", "--value"],
                         capture_output=True, text=True).stdout.split()
    kv = dict(x.split("=", 1) for x in env if "=" in x)
    st, js = call("POST", "/api/login", {"username": "admin", "password": kv.get("SMARTNAV_ADMIN_PASSWORD", "")})
    return js.get("token") if st == 200 else None


def waypoints(tok):
    t0 = time.time()
    while True:
        st, js = call("GET", "/api/waypoints", token=tok)
        if st != 503 or time.time() - t0 > 150:
            return st, js.get("waypoints") or [], js.get("message")
        time.sleep(3)


def job_status(tok, jid):
    st, js = call("GET", "/api/jobs", token=tok)
    for j in js.get("jobs") or []:
        if j.get("job_id") == jid:
            return j.get("status", ""), j.get("message", "")
    return "", ""


def started_at(since):
    """這趟「開始導航到 X」寫進導航 log 的時間（epoch 秒）；還沒起跑回 None。只讀最新 log 的最後 300 KB。"""
    logs = sorted(glob.glob(NAV_LOGS), key=os.path.getmtime)
    if not logs:
        return None
    try:
        with open(logs[-1], "rb") as fh:
            fh.seek(0, os.SEEK_END)
            fh.seek(max(0, fh.tell() - 300_000))
            text = fh.read().decode("utf-8", errors="replace")
    except OSError:
        return None
    hits = [float(m.group(1)) for m in START_RE.finditer(text) if float(m.group(1)) >= since]
    return min(hits) if hits else None


def tsv_rows(path):
    try:
        with open(path, encoding="utf-8") as fh:
            return sum(1 for _ in fh)
    except OSError:
        return 0


def trip(tok, wp, tag=None, tsv=None, estop_after=None, label=""):
    """跑一趟。回 (作業狀態, 秒)。有 tsv 就記一列（nav_result_line 抓不到這趟時自己補一列失敗）。"""
    name = wp["waypoint_name"]
    t0 = time.time()
    st, js = call("POST", "/api/navigate", {"waypoint_id": wp["waypoint_id"], "target_name": name}, token=tok)
    if st != 200 or not js.get("success"):
        status, msg = "failed", f"發不出導航（HTTP {st}）：{js.get('message')}"
        jid = None
    else:
        jid = js.get("job_id")
        status, msg, moving_since, pressed = "", "", None, False
        print(f"  {label}→ {name}：", end="", flush=True)
        while time.time() - t0 < TRIP_LIMIT_S:
            status, msg = job_status(tok, jid)
            if estop_after and not pressed:
                if moving_since is None:
                    moving_since = started_at(t0 - 1)
                if moving_since is not None and time.time() - moving_since >= estop_after:
                    call("POST", "/api/estop", {}, token=tok)
                    pressed = True
                    print(" ⏹ 自動按急停", end="", flush=True)
                elif moving_since is None and time.time() - t0 > ESTOP_WAIT_START_S:
                    print(" （等不到起跑，這趟不按急停）", end="", flush=True)
                    estop_after = None
            if status in FINAL:
                break
            print(".", end="", flush=True)
            time.sleep(1.0)
        else:
            call("DELETE", f"/api/jobs/{jid}", token=tok)
            status, msg = "failed", f"{TRIP_LIMIT_S} 秒沒結束，已取消"
        print(f" {status}（{msg}）", flush=True)
    secs = time.time() - t0
    if tsv:
        before = tsv_rows(tsv)
        r = subprocess.run([sys.executable, os.path.join(T, "nav_result_line.py"), "--since", f"{t0 - 1:.0f}",
                            "--tsv", tsv, "--tag", tag or ""], check=False, capture_output=True, text=True)
        if tsv_rows(tsv) > before:
            print("    " + r.stdout.strip().replace("\n", "\n    "), flush=True)
        else:   # 規劃就失敗、或 log 沒有這趟 → 自己補，統計才不會少算
            new = not os.path.exists(tsv)
            with open(tsv, "a", encoding="utf-8") as fh:
                if new:
                    fh.write(TSV_HEADER)
                fh.write("\t".join([tag or "", time.strftime("%H:%M:%S", time.localtime(t0)), name,
                                    "成功" if status == "succeeded" else "失敗", f"{secs:.1f}",
                                    "", "", "", "", "", "", "", f"作業 {status}：{msg}"[:80]]) + "\n")
            print(f"    （log 沒有這趟的導航紀錄，已用作業結果補一列：{status}）")
    return status, secs


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--a", help="地點 A（每組從這裡出發、最後回到這裡）")
    ap.add_argument("--b", help="地點 B")
    ap.add_argument("--trips", type=int, default=4, help="這組幾趟（偶數＝最後回到 A）")
    ap.add_argument("--tag", default="", help="TSV 第一欄的標籤（新／舊／急停）")
    ap.add_argument("--tsv", default="", help="附加寫入的 TSV（day_test 的 nav_ab.tsv）")
    ap.add_argument("--goto", help="只開到這個地點（不記錄），例如每組開始前先回 A")
    ap.add_argument("--estop-trip", action="store_true", help="開到 B 的途中自動按一次急停（量急停反應）")
    a = ap.parse_args()
    tok = login()
    if not tok:
        print("✗ 登入 HMI 失敗")
        return 1
    try:
        st, wps, msg = waypoints(tok)
        if st != 200:
            print(f"✗ 讀不到地點清單（HTTP {st}；{msg!r}）——導航單元有開嗎？")
            return 1
        by_name = {w["waypoint_name"]: w for w in wps}
        need = [n for n in (a.goto, a.a, a.b) if n]
        missing = [n for n in need if n not in by_name]
        if missing:
            print(f"✗ 地圖上沒有：{'、'.join(missing)}（有的是：{'、'.join(by_name) or '（沒有地點）'}）")
            return 1
        if a.goto:
            status, secs = trip(tok, by_name[a.goto], label="先回出發點 ")
            return 0 if status == "succeeded" else 2
        if not (a.a and a.b):
            print("✗ 要給 --a 和 --b（或 --goto）")
            return 1
        if a.estop_trip:
            status, secs = trip(tok, by_name[a.b], tag=a.tag or "急停", tsv=a.tsv, estop_after=ESTOP_AFTER_S,
                                label="［急停測試］")
            return 0
        ok = 0
        for i in range(1, a.trips + 1):
            target = by_name[a.b] if i % 2 == 1 else by_name[a.a]
            status, secs = trip(tok, target, tag=a.tag, tsv=a.tsv, label=f"［{a.tag} {i}/{a.trips}］")
            ok += status == "succeeded"
            time.sleep(3.0)    # 讓車停穩、log 寫完
        print(f"══ {a.tag}：{a.trips} 趟成功 {ok} 趟 ══")
        return 0
    except KeyboardInterrupt:
        call("POST", "/api/estop", {}, token=tok)
        print("\n⏹ 已送出急停並中止（已完成的趟次都有記錄）")
        return 130
    finally:
        call("POST", "/api/logout", {}, token=tok)


if __name__ == "__main__":
    sys.exit(main())
