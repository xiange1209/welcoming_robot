# -*- coding: utf-8 -*-
"""2026-10-22：雙板模式下 A 的 HMI 單元管理（supervisor／manager）的行為測試。

背景（筆電端審查車上 agent 的雙板草稿時發現）：
  - A 的 HMI 情境（人臉、測語音、完整迎賓故事線、teach／mapping 的 stop 清單）會在 A 本機開／關 camera／face／
    user_auth／asr_chain；雙板時這四個在 B：A 沒有 Astra 會失敗、A 若也開人臉會重複推論、停掉只會把 B 的感知關掉。
  - HMI 的單元狀態是比對「本機行程指令列」，A 看不到 B 的行程 → B 的單元在 A 的面板上永遠灰燈。
改法：環境變數 SMARTNAV_DUALBOARD=1 時，這四個單元 A 不啟動也不停止（成功回報）；狀態改看 ROS 節點圖。
單板（沒有這個環境變數）行為完全不變。

測什麼（用替身，不需要 ROS）
  ✓ remote_units()：單板空集合、雙板預設四個、環境變數可覆寫
  ✓ supervisor._run_unit_operation：遠端單元不呼叫 _start／_stop 且回報成功；一般單元照舊；單板時遠端單元也照舊
  ✓ run_unit_sync（情境逐步呼叫的入口）：遠端單元 stop／start 都成功、不碰行程
  ✓ manager.system_status：雙板時遠端單元看節點圖（全看到＝running、只看到一半＝partial、都沒有＝離線說明）；
    非遠端單元仍看本機行程；單板時完全看本機行程
  ✓ manager.system_control：雙板時對遠端單元回說明、不排入工作
不測：真的跨板的 DDS 發現（V4／V5，要在兩塊板子上）。

用法：python -X utf8 maprun/tools_1021/test_dualboard_hmi.py
"""
import importlib.abc
import importlib.machinery
import importlib.util
import os
import sys
import tempfile
import types
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

CAR = Path(os.environ.get("SMARTNAV_REPO") or Path(__file__).resolve().parents[2])
SC = CAR / "src/smartnav_hmi/smartnav_hmi/modules/system_control"

# supervisor 會在 ~/.smartnav 底下建資料夾：把家目錄換成暫存區，不要弄髒真的
TMP = tempfile.mkdtemp(prefix="dualboard_hmi_")
os.environ["HOME"] = TMP
os.environ["USERPROFILE"] = TMP
os.environ.pop("SMARTNAV_DUALBOARD", None)
os.environ.pop("SMARTNAV_REMOTE_UNITS", None)

# manager 要 import std_srvs 與相對匯入（..system.procinfo、.supervisor）：建一個最小的假套件樹
STUB_PREFIXES = ("std_srvs",)


class StubModule(types.ModuleType):
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        cls = type(name, (), {"__init__": lambda self, *a, **k: None})
        setattr(self, name, cls)
        return cls


class StubFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def find_spec(self, fullname, path, target=None):
        if fullname.split(".")[0] in STUB_PREFIXES:
            return importlib.machinery.ModuleSpec(fullname, self, is_package=True)
        return None

    def create_module(self, spec):
        m = StubModule(spec.name)
        m.__path__ = []
        return m

    def exec_module(self, module):
        pass


sys.meta_path.insert(0, StubFinder())

PKG = "hmi_pkg"
root = types.ModuleType(PKG)
root.__path__ = []
sys.modules[PKG] = root
for sub, path in ((f"{PKG}.system", SC.parent / "system"), (f"{PKG}.system_control", SC)):
    m = types.ModuleType(sub)
    m.__path__ = [str(path)]
    sys.modules[sub] = m
# manager 用 `from ..system.procinfo import proc_cmdlines`：載入真的 procinfo（或替身）
pi_spec = importlib.util.spec_from_file_location(f"{PKG}.system.procinfo", SC.parent / "system" / "procinfo.py")
procinfo = importlib.util.module_from_spec(pi_spec)
sys.modules[f"{PKG}.system.procinfo"] = procinfo
pi_spec.loader.exec_module(procinfo)


def load(name):
    spec = importlib.util.spec_from_file_location(f"{PKG}.system_control.{name}", SC / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[f"{PKG}.system_control.{name}"] = mod
    spec.loader.exec_module(mod)
    return mod


sup = load("supervisor")
mgr = load("manager")
results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


def dual(on=True, remote=None):
    os.environ.pop("SMARTNAV_REMOTE_UNITS", None)
    if on:
        os.environ["SMARTNAV_DUALBOARD"] = "1"
        if remote is not None:
            os.environ["SMARTNAV_REMOTE_UNITS"] = remote
    else:
        os.environ.pop("SMARTNAV_DUALBOARD", None)


print("── 1. remote_units() ──")
dual(False)
check(sup.remote_units() == frozenset(), "單板：空集合")
dual(True)
check(sup.remote_units() == frozenset({"camera", "face", "user_auth", "asr_chain"}), "雙板：預設四個（相機、人臉、認證、語音）")
dual(True, "face, user_auth")
check(sup.remote_units() == frozenset({"face", "user_auth"}), "SMARTNAV_REMOTE_UNITS 可覆寫（逗號分隔、去空白）")
os.environ["SMARTNAV_DUALBOARD"] = "0"
check(sup.remote_units() == frozenset(), "SMARTNAV_DUALBOARD 不是 1 → 視為單板")

print("── 2. supervisor：遠端單元不碰行程 ──")
S = sup.LaunchSupervisor()
calls = []
S._start = lambda unit, variant, op_id: calls.append(("start", unit)) or f"started {unit}"
S._stop = lambda unit, op_id: calls.append(("stop", unit)) or f"stopped {unit}"
dual(True)
for unit in ("face", "camera", "user_auth", "asr_chain"):
    ok1, msg1 = S.run_unit_sync(unit, "stop")
    ok2, msg2 = S.run_unit_sync(unit, "start", "face" if unit == "camera" else None)
    check(ok1 and ok2 and "B 板" in msg1 and "B 板" in msg2, f"雙板：{unit} 的 stop／start 都成功回報、說明在 B 板")
check(not calls, "雙板：四個遠端單元都沒有呼叫 _start／_stop（A 不碰行程）")
ok, msg = S.run_unit_sync("nav", "start", "localization")
check(ok and ("start", "nav") in calls, "雙板：非遠端單元（nav）照常啟動")
ok, msg = S.run_unit_sync("llm", "stop")
check(ok and ("stop", "llm") in calls, "雙板：非遠端單元（llm）照常停止")
calls.clear()
dual(False)
S.run_unit_sync("face", "stop")
S.run_unit_sync("asr_chain", "start")
check(calls == [("stop", "face"), ("start", "asr_chain")], "單板：face／asr_chain 照舊（行為完全不變）")
check(not S._unit_ops, "各種路徑結束後都清掉進行中的 operation 標記（不會卡住下一次操作）")

print("── 3. manager.system_status ──")


class FakeNode:
    def __init__(self, names):
        self.names = names

    def get_node_names_and_namespaces(self):
        return [(n, "/") for n in self.names]

    def get_logger(self):
        return types.SimpleNamespace(error=lambda *_: None)


def make_manager(graph_names, procs):
    m = object.__new__(mgr.SystemControlManager)
    m.nav_camera = types.SimpleNamespace(paused=lambda: False)   # 10/28 新增的導航中暫停人臉相機；這個測試不涉及
    m._node = FakeNode(graph_names)
    m._supervisor = types.SimpleNamespace(transition=lambda unit: None)
    m._requires_cache = {}
    m._check_requires = lambda requires: ""        # 這個測試不關心 Python 模組有沒有裝
    mgr.proc_cmdlines = lambda: procs
    return m


def unit(status, key):
    return next(u for u in status if u["key"] == key)


# 本機行程：A 上有導航（run_nav_cc.sh 的子行程名含 controller_server 等）；沒有任何感知行程
local_procs = [(1001, "/opt/ros/jazzy/lib/nav2_controller/controller_server"), (1002, "/home/user/x/lslidar_driver_node")]
dual(True)
st = make_manager(["face_node", "user_auth_node", "voice_trigger_node", "speech_recognizer_node", "camera"], local_procs).system_status()
for k in ("face", "user_auth", "asr_chain", "camera"):
    u = unit(st, k)
    check(u["running"] and not u["partial"] and u["remote"] == "B" and "B 板" in u["blocked"],
          f"雙板、節點圖上有 B 的節點：{k} 顯示在跑（remote=B）")
st = make_manager(["voice_trigger_node", "camera"], local_procs).system_status()
check(unit(st, "asr_chain")["partial"] and not unit(st, "asr_chain")["running"], "語音只看到 voice_trigger_node → partial（只起來一半）")
st = make_manager([], local_procs).system_status()
check(all(not unit(st, k)["running"] and "離線" in unit(st, k)["blocked"] for k in ("face", "user_auth", "asr_chain", "camera")),
      "B 完全看不到：四個單元都不在跑，blocked 說明「B 板離線或還沒就緒」")
nav = unit(st, "nav")
check(nav["remote"] == "" and "B 板" not in nav["blocked"], "非遠端單元（nav）沒有 remote 標記")
st = make_manager(["lslidar_driver_node"], [(1002, "/home/user/x/lslidar_driver_node"), (1003, "wheeltec_robot_node")]).system_status()
check(unit(st, "sensors")["running"], "非遠端單元仍看本機行程（sensors：雷達＋底盤行程在）")
st = make_manager(["face_node"], [(1004, "/x/face_embedding"), (1005, "/x/user_auth")]).system_status()
check(not unit(st, "user_auth")["running"], "雙板：本機有 user_auth 行程也不算（要看節點圖；B 的才算）")
dual(False)
st = make_manager(["face_node"], [(1004, "/x/face_embedding"), (1005, "/x/user_auth")]).system_status()
check(unit(st, "face")["running"] and unit(st, "user_auth")["running"] and unit(st, "face")["remote"] == "",
      "單板：照舊看本機行程（節點圖不參與）")

print("── 4. manager.system_control（HTTP 單一單元開關） ──")
requested = []
m = make_manager([], [])
m._supervisor = types.SimpleNamespace(request_unit=lambda u, a, v: requested.append((u, a)) or (True, {}), transition=lambda u: None)
dual(True)
ok, res = m.system_control("face", "start")
check(not ok and "B 板" in res["message"] and not requested, "雙板：按 A 面板上的人臉「啟動」→ 回說明、不排入工作")
ok, res = m.system_control("asr_chain", "stop")
check(not ok and "B 板" in res["message"] and not requested, "雙板：語音「停止」同上")
ok, res = m.system_control("nav", "stop")
check(ok and requested == [("nav", "stop")], "雙板：導航仍可從 A 開關")
dual(False)
requested.clear()
ok, res = m.system_control("face", "stop")
check(ok and requested == [("face", "stop")], "單板：人臉停止照舊排入工作")

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
