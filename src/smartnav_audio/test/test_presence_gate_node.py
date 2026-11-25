"""收音距離閘門接到收音狀態機的部分（★ 2026-11-25）：能關、能收尾、導航掛掉會復位。

真的呼叫 VoiceTriggerNode.process_audio_chunk 等方法；麥克風、VAD、話題用替身。
"""

import importlib.util
import sys
import threading
import types
from collections import deque
from pathlib import Path

import numpy as np
import pytest

if importlib.util.find_spec("sounddevice") is None:      # 測試機沒有麥克風套件：型別註解用得到的名稱給個替身
    _sd = types.ModuleType("sounddevice")
    _sd.__getattr__ = lambda name: type(name, (), {})
    sys.modules["sounddevice"] = _sd

from smartnav_audio import voice_trigger_node as vtn  # noqa: E402
from smartnav_audio.presence_gate import PresenceGate  # noqa: E402

BLOCK = np.full(512, 0.1, dtype=np.float32)


@pytest.fixture
def node(monkeypatch):
    t = types.SimpleNamespace(now=100.0)
    monkeypatch.setattr(vtn.time, "monotonic", lambda: t.now)
    n = object.__new__(vtn.VoiceTriggerNode)
    n.t, n.sent, n.logs = t, [], []
    log = types.SimpleNamespace(info=n.logs.append, warning=n.logs.append, debug=lambda m: None, error=n.logs.append)
    n.get_logger = lambda: log
    n.vad = types.SimpleNamespace(is_speech=lambda a: True)
    n._lock = threading.RLock()
    n._state = vtn.TriggerState.IDLE
    n._audio_buffer = deque(maxlen=15)
    n.speech_start_frames, n.command_initial_wait_frames, n.silence_frames_threshold = 3, 4, 8
    n._vad_positive_frames = n._silence_frames = n._command_wait_frames = 0
    n._in_command_initial_wait = False
    n._mon_buf = None
    n._publish_audio = lambda a, is_final=False: n.sent.append(is_final)
    n._gate = PresenceGate(clock=lambda: t.now)
    n._gate_enabled, n._gate_cut_sec, n._gate_closed_since = True, 2.5, None
    n._gate_blocked, n._gate_last_log, n._nav_flag = 0, 0.0, False
    n._gate._scan_at = t.now                     # 有雷射資料
    return n


def person(n, here=True):
    n._gate._scan_at = n.t.now
    if here:
        n._gate._present_at = n.t.now


def feed(n, secs, here):
    for _ in range(int(secs / 0.032)):
        n.t.now += 0.032
        person(n, here)
        n.process_audio_chunk(BLOCK)


def test_blocked_when_nobody_in_front(node):
    feed(node, 2, here=False)
    assert node._state == vtn.TriggerState.IDLE and node.sent == []


def test_switch_off_at_runtime_really_stops_blocking(node):
    res = node._on_set_parameters([types.SimpleNamespace(name="presence_gate_enable", value=False)])
    assert res.successful
    feed(node, 2, here=False)
    assert node._state == vtn.TriggerState.COMMAND and node.sent


def test_utterance_is_closed_when_the_person_leaves_but_noise_goes_on(node):
    feed(node, 1.5, here=True)
    assert node._state == vtn.TriggerState.COMMAND
    feed(node, 3.0 + 2.5 + 0.3, here=False)       # hold 3 秒 + 收尾 2.5 秒；VAD 一直判定有人聲
    assert True in node.sent and any("收尾" in m for m in node.logs)
    assert node._state == vtn.TriggerState.IDLE   # 沒人就不會馬上又開一句


def test_person_still_there_is_never_cut(node):
    feed(node, 20, here=True)
    assert True not in node.sent


def test_cut_can_be_disabled(node):
    node._on_set_parameters([types.SimpleNamespace(name="presence_cut_sec", value=0.0)])
    feed(node, 1.5, here=True)
    feed(node, 10, here=False)
    assert True not in node.sent


def test_not_cut_while_guiding(node):
    feed(node, 1.5, here=True)
    node._nav_active_cb(types.SimpleNamespace(data=True))
    feed(node, 10, here=False)
    assert True not in node.sent


def test_navigation_flag_resets_when_publisher_disappears(node):
    node._nav_active_cb(types.SimpleNamespace(data=True))
    node.count_publishers = lambda topic: 1
    node._nav_watchdog()
    assert node._gate.is_open()[1].startswith("帶位中")
    node.count_publishers = lambda topic: 0
    node._nav_watchdog()
    node.t.now += 10
    person(node, here=False)
    assert node._gate.is_open()[0] is False and node._nav_flag is False


def test_lidar_silence_never_blocks_or_cuts(node):
    feed(node, 1.5, here=True)
    for _ in range(300):
        node.t.now += 0.032
        node.process_audio_chunk(BLOCK)           # 雷射停了（_scan_at 不更新）
    assert True not in node.sent


def _launch_module():
    path = Path(__file__).resolve().parents[1] / "launch" / "asr_chain.launch.py"
    spec = importlib.util.spec_from_file_location("asr_chain_launch", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.parametrize("env,want", [(None, True), ("1", True), ("0", False), ("false", False)])
def test_launch_passes_the_switch_from_environment(monkeypatch, env, want):
    from launch import LaunchContext
    mod = _launch_module()
    if env is None:
        monkeypatch.delenv("SMARTNAV_PRESENCE_GATE", raising=False)
    else:
        monkeypatch.setenv("SMARTNAV_PRESENCE_GATE", env)
    monkeypatch.setattr(mod, "_astra_card", lambda: None)
    monkeypatch.setattr(mod, "_voice_input_mode", lambda: "browser")
    monkeypatch.setattr(mod.time, "sleep", lambda s: None)
    made = []
    monkeypatch.setattr(mod, "Node", lambda **kw: made.append(kw) or types.SimpleNamespace())
    monkeypatch.setattr(mod, "RegisterEventHandler", lambda *a, **k: None)
    monkeypatch.setattr(mod, "OnProcessExit", lambda **k: None)
    ctx = LaunchContext()
    for ent in mod.generate_launch_description().entities:
        if type(ent).__name__ == "DeclareLaunchArgument":
            ent.execute(ctx)
    mod._preflight(ctx)
    trig = next(kw for kw in made if kw["executable"] == "voice_trigger")
    assert trig["parameters"][0]["presence_gate_enable"] is want
