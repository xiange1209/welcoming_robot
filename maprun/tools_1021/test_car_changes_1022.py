# -*- coding: utf-8 -*-
"""2026-10-22：車上 agent 在 4 GB 測試板上做的改動（身分權限、帶位喊停、HMI 全部關閉／電源、ASR decoder fp32）
併回 repo 之後的行為測試。這些改動原本只有車上的整合測試（要 ROS、HMI 服務），筆電端沒有任何保護。

測什麼（純函式用真的程式、其餘用替身或原始碼斷言；不需要 ROS）
  ✓ 身分權限：LLM 的 access_policy.py 與 HMI 的 guide/policy.py 是兩份各自維護的規則，這裡逐項比對
    （6 種身分 × 4 組規則 × 5 個地點）——平板點的和嘴巴說的必須一致
  ✓ 規則檔壞掉／缺欄位時退回預設；身分過期當一般訪客；ADMIN 視同 VIP；黑名單可由規則檔放行
  ✓ 對客人說的話一律中性：拒絕訊息、平板訊息、黑名單的系統提示都不出現「黑名單」「BLACKLIST」「辨識」
  ✓ 帶位途中喊停 is_stop_request：該停的說法、不該停的說法（含「停車場在哪裡」不能把車停下來）
  ✓ manager 的 all_off 情境：只停「正在跑」的單元（running 或 partial），順序是上層先關；
    其他情境一樣跳過沒在跑的停止步驟；啟動步驟不受影響
  ✓ 存取控制（原始碼斷言）：關機／重開機與 /api/speak 要登入；/api/estop 刻意不要登入（人人按得到）；
    /api/stats 對未登入遮黑名單姓名
  ✓ LLM 接線：訂 user_identity、三條帶位路徑（程式端直接帶位、navigate_tool、貴賓室工具）都檢查權限、
    等結果時登記目標讓喊停取消得到
  ✓ ASR：int8_components 預設不含 decoder
不測：真的導航、真的 ROS 訂閱、iPad 畫面（見 maprun/tools_1021/hmi_visual_check.mjs）。

用法：python -X utf8 maprun/tools_1021/test_car_changes_1022.py
"""
import importlib.abc
import importlib.machinery
import importlib.util
import itertools
import json
import os
import re
import sys
import tempfile
import time
import types
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

CAR = Path(os.environ.get("SMARTNAV_REPO") or Path(__file__).resolve().parents[2])
LLM = CAR / "src/smartnav_llm/smartnav_llm"
HMI = CAR / "src/smartnav_hmi/smartnav_hmi"
results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def src(path):
    return Path(path).read_text(encoding="utf-8")


# ══ 1. 身分權限：兩份規則逐項比對 ═════════════════════════════
print("── 1. 身分權限：LLM 與 HMI 兩份規則逐項一致 ──")
llm_pol = load_file("llm_access_policy", LLM / "access_policy.py")
hmi_pol = load_file("hmi_guide_policy", HMI / "modules/guide/policy.py")
NOW = 1_000_000.0
IDENTITIES = {
    "沒有人": None,
    "空的": {},
    "認不出來": {"recognized": False, "user_type": "GUEST"},
    "訪客": {"recognized": True, "user_type": "GUEST"},
    "VIP": {"recognized": True, "user_type": "VIP"},
    "ADMIN": {"recognized": True, "user_type": "ADMIN"},
    "黑名單": {"recognized": True, "user_type": "BLACKLIST"},
    "過期的VIP": {"recognized": True, "user_type": "VIP", "age": 60},
    "未知身分字串": {"recognized": True, "user_type": "SOMETHING"},
}
POLICIES = {
    "預設": {},
    "多一個貴賓地點": {"vip_only_waypoints": ["貴賓室", "櫃台"]},
    "黑名單可帶位": {"blacklist_can_navigate": True},
    "身分 3 秒過期": {"identity_fresh_sec": 3.0},
}
WAYPOINTS = ["貴賓室", "門口", "櫃台", "起點", "大廳"]


def as_llm(i):          # LLM 節點存的是收到的時刻 stamp
    if i is None: return None
    d = {k: v for k, v in i.items() if k != "age"}
    d["stamp"] = NOW - i.get("age", 0.5)
    return d


def as_hmi(i):          # HMI 狀態裡存的是 updated_at
    if i is None: return None
    d = {k: v for k, v in i.items() if k != "age"}
    d["updated_at"] = NOW - i.get("age", 0.5)
    return d


mismatch = []
n = 0
for (iname, ident), (pname, override), wp in itertools.product(IDENTITIES.items(), POLICIES.items(), WAYPOINTS):
    pl_llm = {**llm_pol.DEFAULT, **override}
    pl_hmi = {**hmi_pol.DEFAULT, **override}
    t1 = llm_pol.tier_of(as_llm(ident), pl_llm, now=NOW)
    t2 = hmi_pol.tier_of(as_hmi(ident), pl_hmi, now=NOW)
    ok1 = llm_pol.nav_denial(t1, wp, pl_llm) is None
    ok2 = hmi_pol.allowed(t2, wp, pl_hmi)
    n += 1
    if t1 != t2 or ok1 != ok2:
        mismatch.append((iname, pname, wp, t1, t2, ok1, ok2))
check(not mismatch, f"{n} 種組合兩份規則的身分分級與「能不能去」完全一致" + ("" if not mismatch else f"；不一致：{mismatch[:3]}"))
check(llm_pol.DEFAULT["vip_only_waypoints"] == hmi_pol.DEFAULT["vip_only_waypoints"] == ["貴賓室"]
      and llm_pol.DEFAULT["identity_fresh_sec"] == hmi_pol.DEFAULT["identity_fresh_sec"]
      and llm_pol.DEFAULT["blacklist_can_navigate"] is hmi_pol.DEFAULT["blacklist_can_navigate"] is False,
      "兩份預設值相同（貴賓室只給 VIP、身分 10 秒、黑名單不帶位）")

pl = {**llm_pol.DEFAULT}
tiers = {k: llm_pol.tier_of(as_llm(v), pl, now=NOW) for k, v in IDENTITIES.items()}
check(tiers["沒有人"] == tiers["空的"] == tiers["認不出來"] == tiers["訪客"] == tiers["未知身分字串"] == "guest", "沒有人／空的／認不出來／訪客／未知字串 → guest")
check(tiers["VIP"] == tiers["ADMIN"] == "vip", "VIP 與 ADMIN（員工）→ vip")
check(tiers["黑名單"] == "blocked", "黑名單 → blocked")
check(tiers["過期的VIP"] == "guest", "身分超過 10 秒沒更新 → 回到 guest（人走了不能留著貴賓權限）")
check(llm_pol.nav_denial("blocked", "門口", pl) and llm_pol.nav_denial("guest", "貴賓室", pl)
      and llm_pol.nav_denial("vip", "貴賓室", pl) is None and llm_pol.nav_denial("guest", "門口", pl) is None,
      "黑名單任何地點都拒；訪客不能去貴賓室、可以去門口；VIP 都可以")

print("── 2. 規則檔（~/.smartnav/access_policy.json）──")
tmp = Path(tempfile.mkdtemp(prefix="acc_pol_"))
for mod, label in ((llm_pol, "LLM"), (hmi_pol, "HMI")):
    mod.POLICY_PATH = tmp / "none.json"
    check(mod.load_policy() == mod.DEFAULT, f"{label}：沒有規則檔 → 預設值")
    f = tmp / f"{label}.json"
    f.write_text("{壞掉的 json", encoding="utf-8"); mod.POLICY_PATH = f
    check(mod.load_policy() == mod.DEFAULT, f"{label}：規則檔壞掉 → 預設值（不能因為檔案壞了就放行或崩潰）")
    f.write_text(json.dumps({"vip_only_waypoints": ["貴賓室", "經理室"]}), encoding="utf-8")
    got = mod.load_policy()
    check(got["vip_only_waypoints"] == ["貴賓室", "經理室"] and got["identity_fresh_sec"] == 10.0, f"{label}：只寫一個欄位，其餘欄位用預設")

print("── 3. 對客人說的話一律中性 ──")
FORBIDDEN = ("黑名單", "BLACKLIST", "辨識", "名單")
texts = {
    "LLM BLOCKED_REPLY": llm_pol.BLOCKED_REPLY,
    "LLM 貴賓室被拒": llm_pol.nav_denial("guest", "貴賓室", llm_pol.DEFAULT) or "",
    "LLM blocked 被拒": llm_pol.nav_denial("blocked", "門口", llm_pol.DEFAULT) or "",
}
hsrc = src(HMI / "modules/guide/router.py")
m = re.search(r'BLOCKED_MSG\s*=\s*"([^"]+)"', hsrc)
texts["HMI BLOCKED_MSG"] = m.group(1) if m else ""
for k, v in texts.items():
    check(v and not any(w in v for w in FORBIDDEN), f"{k}：「{v[:28]}…」不含黑名單／辨識字樣")
ctx_blocked = llm_pol.prompt_context("blocked", {"user_name": "某人", "description": "備註"})
check("黑名單" not in ctx_blocked and "某人" not in ctx_blocked and "備註" not in ctx_blocked, "黑名單的系統提示：不帶姓名與備註、不出現「黑名單」")
ctx_vip = llm_pol.prompt_context("vip", {"user_name": "測試貴賓", "description": "分行經理"})
check("測試貴賓" in ctx_vip and "分行經理" in ctx_vip, "VIP 的系統提示有姓名與備註")
check(llm_pol.prompt_context("guest", {"user_name": "李大華"}) == "", "一般訪客：不附任何身分資訊")

print("── 4. 帶位途中喊停 is_stop_request ──")
ni = load_file("nav_intent_mod", LLM / "nav_intent.py")
STOP = ["停下來", "請停車", "取消", "算了", "不用了", "stop", "先停一下", "別走了", "不要走了", "不去了", "不要去了", "快停", "別動"]
KEEP = ["請問營業時間", "等等", "不要", "帶我去貴賓室", "我想開戶", "謝謝", "好的", "請問停車場在哪裡"]
for t in STOP:
    check(ni.is_stop_request(t), f"該停：「{t}」")
for t in KEEP:
    check(not ni.is_stop_request(t), f"不該停：「{t}」" + ("（停車場是常見問題，不能因為問它就把帶位中的車停下來）" if "停車場" in t else ""))

print("── 5. manager：all_off 只停正在跑的單元 ──")
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
        mm = StubModule(spec.name); mm.__path__ = []; return mm

    def exec_module(self, module):
        pass


os.environ["HOME"] = tempfile.mkdtemp(prefix="car_changes_home_")
os.environ["USERPROFILE"] = os.environ["HOME"]
os.environ.pop("SMARTNAV_DUALBOARD", None)
sys.meta_path.insert(0, StubFinder())
SC = HMI / "modules/system_control"
PKG = "hmi_pkg2"
root = types.ModuleType(PKG); root.__path__ = []; sys.modules[PKG] = root
for sub, path in ((f"{PKG}.system", SC.parent / "system"), (f"{PKG}.system_control", SC)):
    mm = types.ModuleType(sub); mm.__path__ = [str(path)]; sys.modules[sub] = mm
pi_spec = importlib.util.spec_from_file_location(f"{PKG}.system.procinfo", SC.parent / "system" / "procinfo.py")
procinfo = importlib.util.module_from_spec(pi_spec); sys.modules[f"{PKG}.system.procinfo"] = procinfo; pi_spec.loader.exec_module(procinfo)
for name in ("supervisor", "manager"):
    spec = importlib.util.spec_from_file_location(f"{PKG}.system_control.{name}", SC / f"{name}.py")
    mod = importlib.util.module_from_spec(spec); sys.modules[f"{PKG}.system_control.{name}"] = mod; spec.loader.exec_module(mod)
mgr = sys.modules[f"{PKG}.system_control.manager"]
Mgr = mgr.SystemControlManager


def make(live_units, partial_units=()):
    m = object.__new__(Mgr)
    calls, requests = [], []
    m._supervisor = types.SimpleNamespace(
        run_unit_sync=lambda unit, action, variant=None: calls.append((unit, action, variant)) or (True, f"{action} {unit}"),
        request_scenario=lambda key, steps: requests.append((key, list(steps))) or (True, {"steps": list(steps)}),
    )
    status = [{"key": u, "running": True, "partial": False} for u in live_units] + \
             [{"key": u, "running": False, "partial": True} for u in partial_units] + \
             [{"key": "ghost", "running": False, "partial": False}]
    m.system_status = lambda: status
    return m, calls, requests


spec = Mgr.SCENARIOS["all_off"]
check(spec["start"] == [] and spec["label"] == "全部關閉", "all_off 存在、只停不啟動")
order = spec["stop"]
check(order.index("llm") < order.index("bank_reception") < order.index("face") < order.index("camera") < order.index("nav") < order.index("sensors"),
      "停止順序是上層先關：LLM → 迎賓 → 人臉 → 相機 → 導航 → 底盤雷達")
unknown = [u for u in order if u not in Mgr.SYSTEM_UNITS]
check(not unknown, f"all_off 停止清單的單元 key 都在 SYSTEM_UNITS 裡（打錯字會變成永遠不停那個單元）" + ("" if not unknown else f"；不存在：{unknown}"))

m, calls, _ = make({"llm", "nav"})
ok, steps = m.scenario_apply("all_off")
check(ok and calls == [("llm", "stop", None), ("nav", "stop", None)] and len(steps) == 2, "只有 LLM、導航在跑 → 只對這兩個下「停止」，順序 LLM 先、導航後")
m, calls, _ = make(set())
ok, steps = m.scenario_apply("all_off")
check(ok and calls == [] and steps == [], "什麼都沒開 → 一步都不做、回成功（不會滿畫面紅字）")
m, calls, _ = make(set(), partial_units={"face"})
m.scenario_apply("all_off")
check(calls == [("face", "stop", None)], "只起來一半的單元（partial）也要停")
m, calls, reqs = make({"llm", "face"})
m.scenario_request("all_off")
check(reqs and reqs[0][0] == "all_off" and [(u, a) for u, a, _v in reqs[0][1]] == [("llm", "stop"), ("face", "stop")],
      "scenario_request（HMI 按鈕走的非同步路徑）也只排入正在跑的單元")
demo = Mgr.SCENARIOS["demo"]
m, calls, reqs = make({"nav"})
m.scenario_request("demo")
steps = reqs[0][1]
check(all(u in {"nav"} for u, a, _v in steps if a == "stop") and any(a == "start" for _u, a, _v in steps),
      "其他情境（完整迎賓故事線）：沒在跑的停止步驟一樣跳過，啟動步驟照舊")
ok, steps = make(set())[0].scenario_apply("不存在的情境")
check(not ok, "未知的情境 → 失敗")

print("── 6. 存取控制（原始碼斷言）──")
nav_router = src(HMI / "modules/navigation/router.py")
sc_router = src(SC / "router.py")
chat_router = src(HMI / "modules/chat/router.py")
stats_router = src(HMI / "modules/stats/router.py")
main_py = src(HMI / "api/main.py")


def decorator_of(text, route):
    i = text.index(route)
    line_start = text.rfind("\n", 0, i) + 1
    return text[line_start:text.index("\n", i)]


check("dependencies=admin_only" in decorator_of(sc_router, '"/api/power/{action}"'), "/api/power/{poweroff|reboot}（關機）要登入")
check("dependencies=admin_only" in decorator_of(chat_router, '"/api/speak"'), "/api/speak（讓機器人唸任何字）要登入")
check("dependencies" not in decorator_of(nav_router, '"/api/estop"'), "/api/estop 刻意不要登入（急停人人按得到；誤按只是停車）")
check("poweroff" in sc_router and "reboot" in sc_router and re.search(r'action not in \("poweroff", "reboot"\)', sc_router),
      "電源 API 只接受 poweroff／reboot（不能拿來執行別的 systemctl 動作）")
check('"sudo", "-n", "/usr/bin/systemctl", action' in sc_router, "電源指令是固定的 sudo -n /usr/bin/systemctl <action>（不是組字串、不經 shell）")
check("scenario_apply(\"all_off\")" in sc_router and sc_router.index('scenario_apply("all_off")') < sc_router.index("subprocess.Popen(cmd"),
      "關機前先做 all_off（優雅關閉所有單元），再執行 systemctl")
check("node.auth.valid(token_from_header(authorization))" in stats_router and "（已隱藏）" in stats_router and '"BLACKLIST"' in stats_router,
      "/api/stats 未登入時遮黑名單姓名")
check("guide_router.build_router" in main_py and "assets_signature" in main_py and '"/api/version"' in main_py,
      "api/main.py 同時掛了帶位路由（車上 agent）與 /api/version（筆電端）——兩邊的改動都在")

print("── 7. LLM 接線（原始碼斷言）──")
node_src = src(LLM / "llm_service_node.py")
tools_src = src(LLM / "bank_tools.py")
check("UserIdentity" in node_src and re.search(r'create_subscription\(\s*UserIdentity,\s*"user_identity"', node_src), "訂了 /user_identity")
check(node_src.count("self.nav_denial(") >= 2 and "def nav_denial" in node_src, "程式端直接帶位與 navigate_tool 都呼叫 nav_denial")
check("node.nav_denial(room_name)" in tools_src, "貴賓室帶位工具也檢查權限（黑名單不能靠說「帶我去貴賓室」繞過）")
check("_active_goals[id(goal_handle)]" in node_src and "_active_goals.pop(id(goal_handle)" in node_src, "等結果時登記進行中的目標、結束時移除（喊停取消得到）")
check("is_stop_request(user_input)" in node_src and "cancel_goal_async()" in node_src, "忙碌時先判斷喊停，是就取消目標")
check("set()" not in re.search(r"self\._active_goals[^\n]*", node_src).group(0), "進行中目標用 dict 不用 set（ClientGoalHandle 不能 hash）")

print("── 8. ASR：decoder 不可以用 int8 ──")
asr_src = src(CAR / "src/smartnav_audio/smartnav_audio/speech_recognizer_node.py")
m = re.search(r'"int8_components",\s*"([^"]+)"', asr_src)
check(m and "decoder" not in m.group(1).split(",") and set(m.group(1).split(",")) == {"encoder", "joiner"},
      f"int8_components 預設 = {m.group(1) if m else '（找不到）'}（只有 encoder、joiner 用 int8；decoder 用 fp32）")
check('_pick("decoder-epoch-99-avg-1", "decoder")' in asr_src, "decoder 檔走 _pick(…, 'decoder')，受 int8_components 控制")

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
