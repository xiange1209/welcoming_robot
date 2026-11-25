"""處理器分時（cpu_share.py）的單元測試：不需要 ROS，用真的子行程驗證每條執行緒的 nice。"""

import os
import resource
import signal
import subprocess
import sys
import time

import pytest

from smartnav_hmi.modules.system_control import cpu_share as cs

CHILD = r"""
import subprocess, sys, threading, time
for _ in range(3):
    threading.Thread(target=time.sleep, args=(60,), daemon=True).start()
if len(sys.argv) > 1 and sys.argv[1] == "spawn":
    subprocess.Popen([sys.executable, "-c", "import threading,time; threading.Thread(target=time.sleep,args=(60,),daemon=True).start(); time.sleep(60)"])
time.sleep(60)
"""


def _spawn(*args, preexec=None):
    proc = subprocess.Popen([sys.executable, "-c", CHILD, *args], start_new_session=True, preexec_fn=preexec)
    time.sleep(0.6)                      # 等執行緒與孫行程開好
    return proc


def _session_threads(sid):
    out = []
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue
        try:
            if cs.read_stat(entry.name)[0] != sid:
                continue
        except OSError:
            continue
        for tid in cs.thread_ids(int(entry.name)):
            try:
                out.append(os.getpriority(os.PRIO_PROCESS, tid))
            except OSError:
                pass
    return out


def _kill(proc):
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    proc.wait(timeout=5)


def _floor_here():
    soft = resource.getrlimit(resource.RLIMIT_NICE)[0]
    return -20 if soft == resource.RLIM_INFINITY else 20 - soft


def test_read_stat_matches_os():
    sid, nice, start = cs.read_stat(os.getpid())
    assert sid == os.getsid(0)
    assert nice == os.getpriority(os.PRIO_PROCESS, 0)
    assert start > 0


def test_nice_floor_matches_rlimit():
    assert cs.nice_floor_of(os.getpid()) == max(-20, _floor_here())


@pytest.mark.parametrize("unit,mode,base,floor,obst,want", [
    ("sensors", cs.NAV, 0, -10, False, -10),
    ("nav", cs.NAV, 0, -10, False, -10),
    ("nav", cs.NAV, 10, -10, False, 10),          # launch 刻意設 +10 的維持原值
    ("asr_chain", cs.GREET, 0, -10, False, -8),
    ("sensors", cs.GREET, 0, -10, False, -10),
    ("nav", cs.GREET, 0, -10, False, -5),
    ("nav", cs.GREET, 10, -10, False, 10),
    ("asr_chain", cs.NAV, -10, -10, False, 5),
    ("camera", cs.NAV, 0, -10, False, 19),
    ("camera", cs.NAV, 0, -10, True, -5),         # 避障模式的相機不往後排
    ("face", cs.GREET, 0, -10, False, 0),         # 迎賓時沒列到＝原值
    ("llm", cs.NAV, 0, -10, False, 5),
    ("speech_synthesizer", cs.NAV, 0, -10, False, 0),
    ("sensors", cs.NAV, 0, -3, False, -3),        # 只能調到上限允許的程度
    ("sensors", cs.NAV, 0, -19, False, -10),      # 不超過 NICE_FLOOR
])
def test_target_nice(unit, mode, base, floor, obst, want):
    assert cs.target_nice(unit, mode, base, floor, obst) == want


def test_apply_and_restore_all_threads_and_grandchildren():
    proc = _spawn("spawn")
    dummy = _spawn()                     # 代替 HMI 自己，免得把 pytest 的優先權改掉
    try:
        core = cs.CpuShareCore(self_pid=dummy.pid)
        before = _session_threads(proc.pid)
        assert len(before) >= 6          # 子行程 4 條＋孫行程 2 條
        st = core.apply(cs.NAV, {"camera": {"sid": proc.pid, "obstacle": False}})
        assert st["processes"] == 2
        assert set(_session_threads(proc.pid)) == {19}
        if _floor_here() <= 0:           # 有權限回到 0 才測回復
            core.apply(cs.GREET, {"camera": {"sid": proc.pid, "obstacle": False}})
            assert set(_session_threads(proc.pid)) == {0}
            core.apply(cs.NAV, {"camera": {"sid": proc.pid, "obstacle": False}})
            core.restore()
            assert set(_session_threads(proc.pid)) == {0}
    finally:
        _kill(proc)
        _kill(dummy)


@pytest.mark.skipif(_floor_here() > -10, reason="這個環境的 RLIMIT_NICE 不夠，測不了調高優先權")
def test_raise_priority_for_navigation():
    proc = _spawn()
    dummy = _spawn()
    try:
        core = cs.CpuShareCore(self_pid=dummy.pid)
        core.apply(cs.NAV, {"sensors": {"sid": proc.pid}})
        assert set(_session_threads(proc.pid)) == {-10}
        core.apply(cs.GREET, {"llm": {"sid": proc.pid}})
        assert set(_session_threads(proc.pid)) == {0}
        assert set(_session_threads(dummy.pid)) == {cs.SELF_NICE}   # 「HMI 自己」也調了
    finally:
        _kill(proc)
        _kill(dummy)


def test_positive_baseline_kept():
    proc = _spawn(preexec=lambda: os.nice(10))
    dummy = _spawn()
    try:
        core = cs.CpuShareCore(self_pid=dummy.pid)
        core.apply(cs.NAV, {"nav": {"sid": proc.pid}})
        assert set(_session_threads(proc.pid)) == {10}
    finally:
        _kill(proc)
        _kill(dummy)


def test_process_that_cannot_return_to_zero_is_left_alone():
    def no_nice_limit():
        resource.setrlimit(resource.RLIMIT_NICE, (0, 0))
    proc = _spawn(preexec=no_nice_limit)
    dummy = _spawn()
    try:
        core = cs.CpuShareCore(self_pid=dummy.pid)
        st = core.apply(cs.NAV, {"camera": {"sid": proc.pid}})
        assert st["skipped"] == 1 and st["processes"] == 0
        assert set(_session_threads(proc.pid)) == {0}     # 沒有被排到 +15（不然回不來）
    finally:
        _kill(proc)
        _kill(dummy)


def test_unknown_session_is_ignored():
    dummy = _spawn()
    try:
        core = cs.CpuShareCore(self_pid=dummy.pid)
        st = core.apply(cs.NAV, {"camera": {"sid": 999999999}})
        assert st["processes"] == 0
    finally:
        _kill(dummy)
