"""收音距離閘門（presence_gate.py）：用合成的雷射掃描，不需要 ROS。★ 2026-11-24"""

import math

import numpy as np
import pytest

from smartnav_audio.presence_gate import PresenceGate

N = 1440                                     # 0.25° 一點，跟車上雷射差不多
INC = 2 * math.pi / N
A0 = -math.pi


def scan_with(points, background=8.0):
    """points：[(x, y)]（雷射座標，x 朝車頭）。其餘方向是 background 公尺外的牆。"""
    r = np.full(N, background, dtype=np.float32)
    for x, y in points:
        i = int(round((math.atan2(y, x) - A0) / INC)) % N
        r[i] = math.hypot(x, y)
    return r


def legs(x, y=0.0):
    """一個人：兩條腿，各約 6 個點"""
    pts = []
    for leg_y in (y - 0.12, y + 0.12):
        for k in range(-3, 3):
            pts.append((x, leg_y + k * 0.01))
    return pts


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def gate():
    c = Clock()
    g = PresenceGate(distance_m=1.0, front_offset_m=0.31, half_width_m=0.6, min_points=3, hold_sec=3.0,
                     stale_sec=2.0, clock=c)
    g.clock = c
    return g


def test_person_80cm_in_front_opens(gate):
    assert gate.on_scan(A0, INC, scan_with(legs(0.31 + 0.8)))
    assert gate.is_open()[0]


def test_person_150cm_in_front_stays_closed(gate):
    assert not gate.on_scan(A0, INC, scan_with(legs(0.31 + 1.5)))
    ok, why = gate.is_open()
    assert not ok and "沒人" in why


def test_person_beside_or_behind_does_not_count(gate):
    assert not gate.on_scan(A0, INC, scan_with(legs(0.9, y=1.2)))      # 側邊 1.2 m
    assert not gate.on_scan(A0, INC, scan_with(legs(-0.8)))            # 車後
    assert not gate.is_open()[0]


def test_own_body_inside_footprint_is_ignored(gate):
    body = [(0.15, y / 100) for y in range(-15, 16)]                   # 車頭外緣以內的鋁擠條
    assert not gate.on_scan(A0, INC, scan_with(body))


def test_hold_keeps_gate_open_briefly(gate):
    gate.on_scan(A0, INC, scan_with(legs(0.31 + 0.7)))
    gate.clock.t += 2.0
    gate.on_scan(A0, INC, scan_with([]))                               # 退了半步、這一圈沒掃到
    assert gate.is_open()[0]
    gate.clock.t += 2.0
    gate.on_scan(A0, INC, scan_with([]))
    assert not gate.is_open()[0]


def test_no_scan_means_no_limit(gate):
    ok, why = gate.is_open()
    assert ok and "沒有雷射" in why                                    # 感測器沒開＝跟以前一樣
    gate.on_scan(A0, INC, scan_with([]))
    assert not gate.is_open()[0]
    gate.clock.t += 2.5                                                # 雷射停了
    assert gate.is_open()[0]


def test_navigation_disables_limit(gate):
    gate.on_scan(A0, INC, scan_with([]))
    gate.set_navigating(True)
    ok, why = gate.is_open()
    assert ok and "帶位" in why
    gate.set_navigating(False)
    assert not gate.is_open()[0]


def test_invalid_ranges_ignored(gate):
    r = scan_with([])
    r[N // 2 - 5:N // 2 + 5] = np.inf
    r[N // 2 + 5:N // 2 + 10] = np.nan
    r[N // 2 + 10:N // 2 + 15] = 0.0
    assert not gate.on_scan(A0, INC, r, range_min=0.05, range_max=12.0)


def test_nearest_distance_reported(gate):
    gate.on_scan(A0, INC, scan_with(legs(0.31 + 0.55)))
    assert gate.nearest_m == pytest.approx(0.55, abs=0.03)
