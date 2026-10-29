#!/usr/bin/env python3
"""定位：自動判斷車子在哪（預設），或設到「車子現在停的那個已存地點」；之後一律做掃描對齊並檢查吻合度。
只改 AMCL 位姿，車不會動。密碼與權杖不印出。

為什麼要這支：平板只有「全域定位」，在走廊或對稱的房間會收斂到錯的地方（8/06 實測錯 3 m、吻合度還顯示 90.4%）。
後端早就有 /api/localize/here（先發 /initialpose，再用 /align_pose 做不必移動的掃描對齊），只是新前端沒有按鈕。
★ 2026-10-27 第一次上機：沒定位就跑故事線，導航亂跑 → day_test.sh 在第 4、5 步開跑前會呼叫這支。
★ 2026-10-27 下午：使用者「不能自己判斷嗎?」→ 加自動判斷（global_localize.py：一筆雷射比整張地圖，
  離線自我測試 25 次：23 次判定且全對、2 次說不確定、0 次錯；中位誤差 1.4 cm／0.3°，每次約 2.3 秒）。

用法（導航單元要在跑）：
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/localize_here.py          # 直接 Enter＝自動判斷；或輸入地點編號／名稱
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/localize_here.py --auto   # 自動判斷（不問）
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/localize_here.py 櫃台      # 設到這個地點（名稱或清單編號；車頭要朝存點時的方向）
    python3 ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/localize_here.py --save 起點   # 用車子現在的位置與朝向存成地點
結束碼：0＝定位確認、2＝吻合度偏低／不確定／方向不對、1＝失敗。
吻合度門檻 80% 的依據：正確 90.8~100% 對錯誤 41~62.6% 完全分開（記憶 localization-verify-score-distribution）。
"""
import glob
import json
import math
import os
import re
import subprocess
import sys
import time
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:8080"
NAV_LOGS = os.path.expanduser("~/.smartnav/logs/system_control/nav_*.log")
ALIGN = re.compile(r"\[(\d{10}\.\d+)\] \[map_service_cc_node\]: 位姿對齊：(.*)$")
MIN_SCORE = 80
# 自動判斷時的放寬：全圖搜尋已經確認「位置唯一」（明顯贏過第二名），map_service 的吻合度（含車上鋁擠條等近距離點）
# 只要 ≥ 65 也算過。80 這條線是在走廊地圖上量的，實驗室新地圖的水準明天才知道。
AUTO_MIN_SCORE, AUTO_MIN_GLOBAL, AUTO_MIN_MARGIN = 65, 0.75, 0.15


def call(method, path, body=None, token=None, timeout=60):
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


def align_lines(since):
    """這次呼叫之後 map_service 寫的「位姿對齊：…」行（吻合度只寫在 log，服務回應裡沒有）"""
    out = []
    for p in sorted(glob.glob(NAV_LOGS), key=os.path.getmtime)[-2:]:
        with open(p, encoding="utf-8", errors="replace") as fh:
            for ln in fh:
                m = ALIGN.search(ln.rstrip("\n"))
                if m and float(m.group(1)) >= since:
                    out.append(m.group(2))
    return out


def ensure_ros_env():
    """自動判斷要讀 /map、/scan：DDS 設定要跟車上節點一樣（env.sh）。沒 source 過就自己重新以 env.sh 執行一次。"""
    if os.environ.get("CYCLONEDDS_URI") or os.environ.get("_LOCALIZE_REEXEC"):
        return
    os.environ["_LOCALIZE_REEXEC"] = "1"
    env_sh = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "env.sh")   # ★ 10/29：repo 的 maprun/env.sh
    os.execvp("bash", ["bash", "-c", f'source "{env_sh}" >/dev/null 2>&1; exec python3 "$0" "$@"',
                       os.path.abspath(__file__)] + sys.argv[1:])


def grab_map_and_scans(timeout=10.0, n_scans=3):
    """讀一次 /map（transient local）與幾筆 /scan。回 (OccupancyGrid 或 None, [LaserScan…])"""
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    rclpy.init()
    node = rclpy.create_node("localize_here_auto")
    got = {"map": None, "scans": []}
    map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(OccupancyGrid, "/map", lambda m: got.__setitem__("map", m), map_qos)
    node.create_subscription(LaserScan, "/scan", lambda m: got["scans"].append(m), qos_profile_sensor_data)
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < timeout and (got["map"] is None or len(got["scans"]) < n_scans):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return got["map"], got["scans"]


def auto_pose():
    """全圖掃描比對。回 (pose 或 None, 說明, cands)"""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import numpy as np
    import global_localize as G
    print("  自動判斷：讀地圖與雷射…", flush=True)
    m, scans = grab_map_and_scans()
    if m is None:
        return None, "收不到 /map（導航單元的定位模式有開嗎？）", []
    if not scans:
        return None, "收不到 /scan（底盤＋雷達有開嗎？）", []
    pts = np.concatenate([G.scan_to_points(s.ranges, s.angle_min, s.angle_increment) for s in scans])
    mi = G.MapIndex(*G.grid_from_msg(m))
    print(f"  全圖搜尋中（{len(pts)} 個雷射點、{len(mi.candidates(0.15))} 個候選位置 × 72 個朝向）…", flush=True)
    cands, dt = G.global_search(mi, pts)
    ok, why = G.verdict(cands)
    for i, c in enumerate(cands[:2]):
        print(f"  {'最像' if i == 0 else '第二'}：({c[1]:.2f}, {c[2]:.2f}) 朝向 {math.degrees(c[3]):.0f}°，吻合 {c[0] * 100:.0f}%")
    print(f"  （{dt:.1f} 秒）{why}")
    return (cands[0][1:] if ok else None), why, cands


def post_and_verify(tok, body, where, auto_cands=None):
    """POST /api/localize/here，再從 nav log 讀掃描對齊的吻合度。回結束碼。"""
    t0 = time.time()
    st, js = call("POST", "/api/localize/here", body, token=tok)
    msg = str(js.get("message") or "")
    print(f"  {msg}")
    if st != 200 or not js.get("success"):
        print(f"✗ 定位失敗（HTTP {st}）")
        return 1
    # ★ 10/27：HMI 對齊沒做成也回 success（訊息裡才寫「讀不到目前位姿」）→ 當天 12:35 就這樣被當成通過
    if any(k in msg for k in ("讀不到目前位姿", "無法對齊", "略過掃描對齊")):
        print("✗ 掃描對齊沒有做成（位置只設了、沒有用雷射確認）：再定位一次")
        return 1
    lines = []
    for _ in range(6):   # log 可能還沒寫進檔案
        lines = align_lines(t0 - 1)
        if lines:
            break
        time.sleep(1)
    for ln in lines:
        print(f"  log：位姿對齊：{ln}")
    if lines and "超過上限" in lines[-1]:
        print(f"⚠ 車頭方向跟「{where}」差太多，對齊被拒絕：把車頭轉到存點時的方向再跑一次（或改用自動判斷）")
        return 2
    # 「吻合度 A% -> B%」取對齊後的 B；「維持原值 (吻合度 A%)」取 A（10/27 修：原本 A -> B 只抓到 A）
    m = re.search(r"吻合度 (\d+)%(?:\s*->\s*(\d+)%)?", " ".join(lines[-1:]))
    if not m:
        # ★ 10/27：以前這裡回 0（通過）→ 對齊其實沒做也會過。沒有吻合度就不能說定位好了。
        print(f"⚠ 已把位置設到「{where}」，但 log 裡沒找到吻合度、無法確認：再定位一次")
        return 2
    score = int(m.group(2) or m.group(1))
    if score >= MIN_SCORE:
        print(f"✓ 已定位到「{where}」，吻合度 {score}%")
        return 0
    if auto_cands:
        best = auto_cands[0][0]
        margin = best - (auto_cands[1][0] if len(auto_cands) > 1 else 0.0)
        if score >= AUTO_MIN_SCORE and best >= AUTO_MIN_GLOBAL and margin >= AUTO_MIN_MARGIN:
            print(f"✓ 已定位到「{where}」：對齊吻合度 {score}%（略低，但全圖只有這裡像：{best * 100:.0f}% 對第二名 "
                  f"{(best - margin) * 100:.0f}%）")
            return 0
    print(f"⚠ 吻合度只有 {score}%（要 ≥ {MIN_SCORE}%）：位置可能不對；換個地方或擺正後再跑一次")
    if score < 60:
        print("  ★ 一直 < 60%：多半是地圖過期（房間擺設變了）——10/27 同一張 9/29 的圖只剩 14~49%（10/20 還有 68~76%），要重建地圖")
    return 2


def main():
    ensure_ros_env()
    env = subprocess.run(["systemctl", "show", "smartnav-hmi", "-p", "Environment", "--value"],
                         capture_output=True, text=True).stdout.split()
    kv = dict(x.split("=", 1) for x in env if "=" in x)
    st, js = call("POST", "/api/login", {"username": "admin", "password": kv.get("SMARTNAV_ADMIN_PASSWORD", "")})
    if st != 200:
        print(f"✗ 登入 HMI 失敗（HTTP {st}）")
        return 1
    tok = js["token"]
    try:
        # ★ 10/27：demo 全開時 CPU 滿載，導航要好一陣子才起來，地點服務先回 HTTP 503 → 等它（最多 150 秒）
        t_wait = time.time()
        said = False
        while True:
            st, js = call("GET", "/api/waypoints", token=tok)
            wps = js.get("waypoints") or []
            if st != 503 or time.time() - t_wait > 150:
                break
            if not said:
                said = True
                print("  導航還在啟動（地點服務還沒好），等它…（最多 150 秒；Ctrl-C＝不等了）", end="", flush=True)
            print(".", end="", flush=True)
            time.sleep(3)
        if said:
            print()
        if st != 200:
            print(f"✗ 讀不到地點清單（HTTP {st}；{js.get('message')!r}）——導航單元（定位模式）有開嗎？")
            return 1
        # ★ 10/27：--save 名稱＝用「車子現在的位置與朝向」建地點（平板只能在地圖上點，跟車子實際停的地方對不準）。
        if len(sys.argv) > 2 and sys.argv[1] == "--save":
            name = sys.argv[2].strip()
            if any(w["waypoint_name"] == name for w in wps):
                print(f"✗ 這張圖已經有「{name}」——先在平板地圖頁刪掉舊的，再存（同名兩個，定位會選錯）")
                return 1
            st, js = call("POST", "/api/waypoints", {"waypoint_name": name, "use_given_pose": False}, token=tok)
            print(f"  {js.get('message')}")
            if st != 200 or not js.get("success"):
                print(f"✗ 存地點失敗（HTTP {st}）")
                return 1
            print(f"✓ 已用車子現在的位置存成「{name}」")
            return 0
        arg = sys.argv[1].strip() if len(sys.argv) > 1 else ""
        if not arg:
            for i, w in enumerate(wps, 1):
                print(f"  {i:2d}. {w['waypoint_name']}")
            try:
                arg = input("車子停哪都可以——直接按 Enter＝自動判斷位置；或輸入地點編號／名稱（車頭要朝存點時的方向）：").strip()
            except EOFError:
                arg = ""
            arg = arg or "--auto"
        if arg == "--auto":
            pose, why, cands = auto_pose()
            if pose is None:
                print(f"⚠ 自動判斷不出來：{why}")
                return 2
            x, y, yaw = pose
            return post_and_verify(tok, {"x": x, "y": y, "yaw": yaw, "align": True},
                                   f"自動判斷的位置 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)", auto_cands=cands)
        if arg.isdigit() and 1 <= int(arg) <= len(wps):
            hit = wps[int(arg) - 1]
        else:
            hit = next((w for w in wps if w["waypoint_name"] == arg), None)
        if hit is None:
            print(f"✗ 沒有這個地點：{arg!r}")
            return 1
        return post_and_verify(tok, {"waypoint_id": hit["waypoint_id"], "align": True}, hit["waypoint_name"])
    finally:
        call("POST", "/api/logout", {}, token=tok)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n（中斷）")
        sys.exit(1)
