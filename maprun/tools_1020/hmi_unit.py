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
before = running()
print(f"{action} 前:", {k: before.get(k, {}).get("running") for k in units})
for u in units:
    if bool(before.get(u, {}).get("running")) == want:
        print(f"  {u}: 已經是{'開' if want else '關'}的，不動")
        continue
    st, js = call("POST", f"/api/system/{u}/{action}", {}, token=tok)
    print(f"  POST {action} {u} → HTTP {st}；{js.get('message')!r}")
t0 = time.time()
while time.time() - t0 < 120:
    time.sleep(3)
    now = running()
    state = {k: now.get(k, {}).get("running") for k in units}
    print(f"  +{time.time() - t0:3.0f}s {state}")
    if all(bool(v) == want for v in state.values()):
        break
print("  全部單元:", {k: v.get("running") for k, v in running().items()})
call("POST", "/api/logout", {}, token=tok)
