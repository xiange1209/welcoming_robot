#!/usr/bin/env python3
"""用 HMI 管理端 API 開／關單元（等同平板「系統開關」的開關）。密碼與權杖不印出。
用法: hmi_unit.py <start|stop> <單元key> [<單元key> ...]"""
import json
import subprocess
import sys
import time
import urllib.error
import urllib.request

action, units = sys.argv[1], sys.argv[2:]
if action not in ("start", "stop", "scenario") or not units:
    sys.exit(__doc__ + "\n       hmi_unit.py scenario <情境key> <等它開起來的單元key> ...")
env = subprocess.run(["systemctl", "show", "smartnav-hmi", "-p", "Environment", "--value"],
                     capture_output=True, text=True).stdout.split()
kv = dict(x.split("=", 1) for x in env if "=" in x)
BASE = "http://127.0.0.1:8080"


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
            return e.code, {"_raw": raw[:200]}


st, js = call("POST", "/api/login", {"username": "admin", "password": kv.get("SMARTNAV_ADMIN_PASSWORD", "")})
if st != 200:
    sys.exit(f"✗ 登入失敗 HTTP {st}")
tok = js["token"]


def running():
    _, j = call("GET", "/api/system/status", token=tok)
    return {(u.get("key") or u.get("unit") or u.get("name")): u for u in j.get("units", [])}


if action == "scenario":
    key, units = units[0], units[1:]
    st, js = call("POST", f"/api/scenarios/{key}", {}, token=tok)
    print(f"  POST scenario {key} → HTTP {st}；{js.get('message')!r}")
    action = "start"
    time.sleep(3)
want = action == "start"
# ★ 2026-10-27：導航要指定啟動方式（「nav」單獨啟動會被 HTTP 400 拒絕）→ 寫成 nav:localization，
#   送出 /api/system/nav/start:localization；等待與「已經開著」都看基本鍵 nav。
base = {u: u.partition(":")[0] for u in units}


def wait_op(op_id, label, limit=150):
    """等 HMI 的開／關作業真的結束（POST 只回 202「已排入」，失敗是之後才發生的）。回 (status, message)。"""
    t_w, last = time.time(), None
    while time.time() - t_w < limit:
        st, js = call("GET", f"/api/system/operations/{op_id}", token=tok)
        js = js.get("operation") or {}   # 回應包在 {"success": …, "operation": {…}} 裡
        if st == 200 and js:
            cur = (js.get("status"), js.get("phase"))
            if cur != last:
                print(f"    {label}：{js.get('phase') or js.get('status')}（{js.get('message')}）"
                      f"｜{time.time() - t_w:.0f}s", flush=True)
                last = cur
            if js.get("status") in ("succeeded", "failed"):
                return js.get("status"), str(js.get("message") or "")
        time.sleep(2)
    return "timeout", f"{limit} 秒沒結束"


def request(unit, act):
    st, js = call("POST", f"/api/system/{unit}/{act}", {}, token=tok)
    print(f"  POST {act} {unit} → HTTP {st}；{js.get('message')!r}", flush=True)
    if st >= 400 or not js.get("operation_id"):
        return ("failed" if st >= 400 else "unknown"), str(js.get("message") or "")
    return wait_op(js["operation_id"], f"{unit} {act}")


try:
    before = running()
    print(f"{action} 前:", {base[u]: before.get(base[u], {}).get("running") for u in units}, flush=True)
    for u in units:
        if bool(before.get(base[u], {}).get("running")) == want:
            print(f"  {u}: 已經是{'開' if want else '關'}的，不動", flush=True)
            continue
        variant = u.partition(":")[2]
        act = f"{action}:{variant}" if (want and variant) else action
        status, msg = request(base[u], act)
        # ★ 2026-10-27：單元的 launch 還在跑、但 HMI 用來偵測的節點掛了（10/27 map_service_cc 當掉）——
        #   HMI 說「沒在跑」卻拒絕再開（作業失敗：「單元已由受管 launch 執行中」），以前就乾等 120 秒、畫面不動。
        #   現在：先停掉那個 launch（HMI 的 stop 停得掉，實測），等它停完再開一次。
        if want and status == "failed" and "受管 launch 執行中" in msg:
            print(f"  ⚠ {base[u]}：launch 還在跑但有節點掛了（HMI 偵測不到）→ 先停掉再重開", flush=True)
            request(base[u], "stop")
            status, msg = request(base[u], act)
        if status not in ("succeeded", "unknown"):
            print(f"  ✗ {base[u]} {act}：{status}（{msg}）", flush=True)
    t0 = time.time()
    while time.time() - t0 < 120:
        time.sleep(3)
        now = running()
        state = {base[u]: now.get(base[u], {}).get("running") for u in units}
        print(f"  +{time.time() - t0:3.0f}s {state}", flush=True)
        if all(bool(v) == want for v in state.values()):
            break
    print("  全部單元:", {k: v.get("running") for k, v in running().items()}, flush=True)
except KeyboardInterrupt:
    print("\n  （中斷：不再等單元開關；已送出的開／關要求 HMI 會照做）", flush=True)
    call("POST", "/api/logout", {}, token=tok)
    sys.exit(130)
call("POST", "/api/logout", {}, token=tok)
