"""貴賓寬限不可以跨人（nav_arbiter_node._identity_cb → access_policy.next_last_vip）。★ 2026-11-25

真的呼叫節點的 _identity_cb／_current，只是不起 ROS 執行環境。
"""

import threading
import types

import pytest

from smartnav_brain import access_policy as ap
from smartnav_brain import nav_arbiter_node as nan

GUEST, VIP, ADMIN, BLACKLIST = 0, 1, 2, 3
FACE, NOBODY = [10.0, 10.0, 60.0, 60.0], [0.0, 0.0, 0.0, 0.0]


class Arb:
    def __init__(self, monkeypatch, **policy):
        self.t = 100.0
        self.policy = {**ap.DEFAULT, "vip_grace_sec": 30.0, **policy}
        monkeypatch.setattr(nan.time, "monotonic", lambda: self.t)
        monkeypatch.setattr(ap.time, "monotonic", lambda: self.t)
        monkeypatch.setattr(ap, "load_policy", lambda: dict(self.policy))
        n = object.__new__(nan.NavArbiterNode)
        n._identity, n._last_vip, n._identity_lock = {}, None, threading.Lock()
        self.logs = []
        log = types.SimpleNamespace(info=self.logs.append, warning=self.logs.append)
        n.get_logger = lambda: log
        self.n = n

    def see(self, kind=GUEST, recognized=True, bbox=FACE, name="A"):
        self.n._identity_cb(types.SimpleNamespace(
            recognized=recognized, user_type=types.SimpleNamespace(type=kind), user_name=name, description="", bbox=bbox))

    def vip(self):
        self.see(VIP)

    def unknown(self):
        self.see(GUEST, recognized=False, name="Unknown")

    def nobody(self):
        self.see(GUEST, recognized=False, bbox=NOBODY, name="Unknown")

    def tier(self):
        return self.n._current()[0]


@pytest.fixture
def arb(monkeypatch):
    return Arb(monkeypatch)


def test_same_vip_turning_head_stays_vip(arb):
    arb.vip(); arb.t += 5; arb.unknown()
    assert arb.tier() == ap.TIER_VIP


def test_vip_then_blacklist_then_unknown_is_not_vip(arb):
    arb.vip(); arb.t += 4; arb.see(BLACKLIST, name="B")
    assert arb.tier() == ap.TIER_BLOCKED
    arb.t += 2; arb.unknown()
    assert arb.tier() == ap.TIER_GUEST


def test_vip_then_registered_guest_then_unknown_is_not_vip(arb):
    arb.vip(); arb.t += 4; arb.see(GUEST, name="C"); arb.t += 2; arb.unknown()
    assert arb.tier() == ap.TIER_GUEST


def test_vip_then_nobody_then_unknown_is_not_vip(arb):
    arb.vip(); arb.t += 10; arb.nobody(); arb.t += 3; arb.unknown()
    assert arb.tier() == ap.TIER_GUEST


def test_nobody_can_be_kept_by_policy(monkeypatch):
    arb = Arb(monkeypatch, vip_grace_clear_on_absent=False)
    arb.vip(); arb.t += 10; arb.nobody(); arb.t += 3; arb.unknown()
    assert arb.tier() == ap.TIER_VIP


def test_zero_grace_never_covers(monkeypatch):
    arb = Arb(monkeypatch, vip_grace_sec=0)
    arb.vip(); arb.t += 0.5; arb.unknown()
    assert arb.tier() == ap.TIER_GUEST


def test_grace_ends(arb):
    arb.vip(); arb.t += 31; arb.unknown()
    assert arb.tier() == ap.TIER_GUEST


def test_admin_counts_like_vip(arb):
    arb.see(ADMIN, name="S"); arb.t += 3; arb.unknown()
    assert arb.tier() == ap.TIER_VIP


def test_grace_age_is_reported(arb):
    arb.vip(); arb.t += 6; arb.unknown()
    identity = arb.n._current()[1]
    assert identity["vip_grace"] and abs(identity["vip_grace_age"] - 6.0) < 1e-6


def _ask(arb, name="貴賓室"):
    arb.n._waypoints = lambda: [("w1", name)]
    goal = types.SimpleNamespace(source=99, use_target_pose=False, target_name="", waypoint_id="w1")
    return arb.n._authorize(goal)[0]


def test_authorize_logs_when_grace_let_it_through(arb):
    arb.vip(); arb.t += 6; arb.unknown()
    assert _ask(arb) is None and any("靠貴賓寬限放行" in m for m in arb.logs)


def test_authorize_refuses_stranger_after_blacklist(arb):
    arb.vip(); arb.t += 4; arb.see(BLACKLIST, name="B"); arb.t += 2; arb.unknown()
    denied = _ask(arb)
    assert denied is not None and not denied.success


def test_residual_risk_stranger_steps_in_without_a_gap(arb):
    """★ 已知殘留：換人之間沒有「無人」、陌生人又不是註冊者時，程式分不出來 → 寬限期內仍會放行（所以預設關閉）"""
    arb.vip(); arb.t += 4; arb.unknown()
    assert arb.tier() == ap.TIER_VIP


def test_same_situation_is_refused_with_the_default_policy(monkeypatch):
    arb = Arb(monkeypatch, vip_grace_sec=ap.DEFAULT["vip_grace_sec"])
    arb.vip(); arb.t += 4; arb.unknown()
    assert arb.tier() == ap.TIER_GUEST
    denied = _ask(arb)
    assert denied is not None and not denied.success
