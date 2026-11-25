"""HMI 快捷情境（manager.SCENARIOS／_plan_steps）的單元測試：假的單元狀態，不碰真的行程。

★ 2026-11-24：只留「全部開啟／建圖／全部關閉」；全部開啟要跳過已經開著的、導航在定位模式不重開。
"""

import time
import types

import pytest

from smartnav_hmi.modules.system_control.manager import SystemControlManager as Mgr

DEMO_START = ["sensors", "nav", "camera", "face", "user_auth", "bank_reception", "asr_chain", "llm"]
FACE_LABEL = Mgr.SYSTEM_UNITS["camera"]["variants"]["face"]["label"]
OBSTACLE_LABEL = Mgr.SYSTEM_UNITS["camera"]["variants"]["obstacle"]["label"]
ECO_LABEL = Mgr.SYSTEM_UNITS["face"]["variants"]["eco"]["label"]


def make(running=(), partial=(), modes=None, nav_mode="localization", starting=(), stuck=(), started_ago=None):
    """running/partial：單元 key；modes：{unit: system_status 回報的模式名稱}；nav_mode 可以是例外（問不到）"""
    m = object.__new__(Mgr)
    m.nav_camera = types.SimpleNamespace(paused=lambda: False, forget=lambda: None)
    calls, requests = [], []
    m._supervisor = types.SimpleNamespace(
        run_unit_sync=lambda unit, action, variant=None: calls.append((unit, action, variant)) or (True, "ok"),
        request_scenario=lambda key, steps: requests.append((key, list(steps))) or (True, {"steps": list(steps)}),
        session_started_at=lambda unit: None if started_ago is None else time.time() - started_ago,
    )
    modes = modes or {}
    status = [{"key": u, "running": True, "partial": False, "mode": modes.get(u, "")} for u in running]
    status += [{"key": u, "running": False, "partial": True, "mode": ""} for u in partial]
    for u in status:                                       # 啟動中可以同時是 partial（只起來一半）
        u["starting"], u["start_stuck"] = u["key"] in starting, u["key"] in stuck
    known = {u["key"] for u in status}
    status += [{"key": u, "running": False, "partial": False, "mode": modes.get(u, ""), "starting": u in starting,
                "start_stuck": u in stuck} for u in list(starting) + list(stuck) if u not in known]
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


# ── ★ 2026-11-25：「全部開啟」按兩次 ─────────────────────────────
def test_all_on_twice_while_everything_is_starting_does_nothing():
    assert plan(starting=DEMO_START, modes={"camera": FACE_LABEL}) == []


def test_half_up_but_still_starting_is_left_alone():
    steps = plan(running=["sensors", "nav"], partial=["asr_chain"], starting=["asr_chain", "camera"],
                 modes={"camera": FACE_LABEL})
    units = [u for u, _a, _v in steps]
    assert "asr_chain" not in units and "camera" not in units and ("nav", "stop", None) not in steps


def test_nav_that_cannot_answer_yet_is_kept_when_recently_started():
    steps = plan(running=["nav"], nav_mode=RuntimeError("服務還沒起來"), started_ago=40)
    assert ("nav", "stop", None) not in steps and "nav" not in [u for u, _a, _v in steps]


def test_nav_that_never_answers_is_restarted_after_the_settle_time():
    steps = plan(running=["nav"], nav_mode=RuntimeError("壞了"), started_ago=Mgr.NAV_SETTLE_SEC + 5)
    assert steps[0] == ("nav", "stop", None) and ("nav", "start", None) in steps


def test_recent_nav_in_mapping_mode_is_still_restarted():
    steps = plan(running=["nav"], nav_mode="mapping", started_ago=10)
    assert steps[0] == ("nav", "stop", None)


def test_stuck_unit_is_stopped_then_started():
    steps = plan(running=["sensors", "nav"], stuck=["llm"])
    assert ("llm", "stop", None) in steps and ("llm", "start", None) in steps


def test_starting_with_wrong_variant_is_replaced():
    steps = plan(running=["sensors", "nav"], starting=["camera"], modes={"camera": OBSTACLE_LABEL})
    assert ("camera", "stop", None) in steps and ("camera", "start", "face") in steps


def test_all_off_also_stops_units_that_are_still_starting():
    steps = plan("all_off", running=["sensors"], starting=["nav", "llm"], stuck=["camera"])
    assert [u for u, a, _v in steps if a == "stop"] == ["llm", "camera", "nav", "sensors"]
