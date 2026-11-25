"""HMI 快捷情境（manager.SCENARIOS／_plan_steps）的單元測試：假的單元狀態，不碰真的行程。

★ 2026-11-24：只留「全部開啟／建圖／全部關閉」；全部開啟要跳過已經開著的、導航在定位模式不重開。
"""

import types

import pytest

from smartnav_hmi.modules.system_control.manager import SystemControlManager as Mgr

DEMO_START = ["sensors", "nav", "camera", "face", "user_auth", "bank_reception", "asr_chain", "llm"]
FACE_LABEL = Mgr.SYSTEM_UNITS["camera"]["variants"]["face"]["label"]
OBSTACLE_LABEL = Mgr.SYSTEM_UNITS["camera"]["variants"]["obstacle"]["label"]
ECO_LABEL = Mgr.SYSTEM_UNITS["face"]["variants"]["eco"]["label"]


def make(running=(), partial=(), modes=None, nav_mode="localization"):
    """running/partial：單元 key；modes：{unit: system_status 回報的模式名稱}；nav_mode 可以是例外（問不到）"""
    m = object.__new__(Mgr)
    m.nav_camera = types.SimpleNamespace(paused=lambda: False, forget=lambda: None)
    calls, requests = [], []
    m._supervisor = types.SimpleNamespace(
        run_unit_sync=lambda unit, action, variant=None: calls.append((unit, action, variant)) or (True, "ok"),
        request_scenario=lambda key, steps: requests.append((key, list(steps))) or (True, {"steps": list(steps)}),
    )
    modes = modes or {}
    status = [{"key": u, "running": True, "partial": False, "mode": modes.get(u, "")} for u in running]
    status += [{"key": u, "running": False, "partial": True, "mode": ""} for u in partial]
    m.system_status = lambda: status

    def query():
        if isinstance(nav_mode, Exception):
            raise nav_mode
        return nav_mode
    m.query_nav_mode = query
    return m, calls, requests


def plan(key="demo", **kw):
    m, _calls, reqs = make(**kw)
    ok, _ = m.scenario_request(key)
    assert ok
    return reqs[-1][1]


def test_only_three_scenarios_in_button_order():
    assert list(Mgr.SCENARIOS) == ["demo", "mapping", "all_off"]
    assert [s["label"] for s in Mgr.SCENARIOS.values()] == ["全部開啟", "建圖", "全部關閉"]


def test_every_unit_key_exists():
    for key, spec in Mgr.SCENARIOS.items():
        for item in list(spec["stop"]) + list(spec["start"]) + list(spec.get("keep_if_localization", ())):
            unit, _, variant = item.partition(":")
            assert unit in Mgr.SYSTEM_UNITS, f"{key}：{unit}"
            if variant:
                assert variant in Mgr.SYSTEM_UNITS[unit].get("variants", {}), f"{key}：{item}"


def test_all_on_from_nothing_starts_everything_in_order():
    steps = plan()
    assert [a for _u, a, _v in steps] == ["start"] * len(DEMO_START)
    assert [u for u, _a, _v in steps] == DEMO_START
    assert ("camera", "start", "face") in steps


def test_all_on_skips_running_and_keeps_localization():
    steps = plan(running=["sensors", "nav", "camera", "face"], modes={"camera": FACE_LABEL})
    assert [(u, a) for u, a, _v in steps] == [
        ("user_auth", "start"), ("bank_reception", "start"), ("asr_chain", "start"), ("llm", "start")]


def test_all_on_when_everything_is_already_on_does_nothing():
    assert plan(running=DEMO_START, modes={"camera": FACE_LABEL}) == []


def test_all_on_restarts_nav_in_mapping_mode():
    steps = plan(running=["sensors", "nav"], nav_mode="mapping")
    assert steps[0] == ("nav", "stop", None)
    assert ("nav", "start", None) in steps and ("sensors", "start", None) not in steps


def test_all_on_restarts_nav_when_mode_unknown():
    steps = plan(running=["nav"], nav_mode=RuntimeError("服務沒回應"))
    assert steps[0] == ("nav", "stop", None) and ("nav", "start", None) in steps


def test_wrong_variant_is_stopped_then_started():
    steps = plan(running=["camera", "face"], modes={"camera": OBSTACLE_LABEL, "face": ECO_LABEL})
    stops = [u for u, a, _v in steps if a == "stop"]
    assert set(stops) == {"camera", "face"}
    assert ("camera", "start", "face") in steps and ("face", "start", None) in steps
    first_start = next(i for i, (_u, a, _v) in enumerate(steps) if a == "start")
    assert all(a == "stop" for _u, a, _v in steps[:first_start])     # 停止全部排在啟動前面


def test_partial_unit_is_restarted():
    steps = plan(partial=["asr_chain"])
    assert ("asr_chain", "stop", None) in steps and ("asr_chain", "start", None) in steps


def test_all_off_only_stops_running_in_top_down_order():
    steps = plan("all_off", running=["sensors", "llm", "nav"], partial=["face"])
    assert steps == [("llm", "stop", None), ("face", "stop", None), ("nav", "stop", None), ("sensors", "stop", None)]


def test_mapping_still_stops_heavy_units():
    steps = plan("mapping", running=["face", "llm", "nav", "sensors"])
    stops = [u for u, a, _v in steps if a == "stop"]
    assert {"face", "llm", "nav"} <= set(stops) and "sensors" not in stops
    assert ("nav", "start", None) in steps


def test_scenario_apply_uses_same_plan():
    m, calls, _ = make(running=["sensors", "nav", "camera"], modes={"camera": FACE_LABEL})
    ok, steps = m.scenario_apply("demo")
    assert ok
    assert [c[0] for c in calls] == ["face", "user_auth", "bank_reception", "asr_chain", "llm"]
    assert all(line.startswith("✓ 啟動 ") for line in steps)


def test_unknown_scenario_fails():
    m, _calls, _ = make()
    assert m.scenario_request("face")[0] is False         # 刪掉的舊情境
    assert m.scenario_apply("demo_eco")[0] is False


@pytest.mark.parametrize("key", ["demo", "mapping", "all_off"])
def test_scenario_list_shapes(key):
    m, _calls, _ = make()
    m.SYSTEM_UNITS = Mgr.SYSTEM_UNITS
    item = next(s for s in m.scenario_list() if s["key"] == key)
    assert item["label"] and item["why"]
