"""貴賓寬限（access_policy.with_vip_grace）：認出貴賓後 30 秒內，認不出／沒人仍算貴賓。★ 2026-11-24"""

from smartnav_brain import access_policy as ap

POLICY = {**ap.DEFAULT, "vip_grace_sec": 30.0}


def ident(t, recognized=True, kind="VIP", name="A"):
    return {"recognized": recognized, "user_type": kind, "user_name": name, "description": "", "stamp": t}


def tier(current, last_vip, now, policy=POLICY):
    return ap.tier_of(ap.with_vip_grace(current, last_vip, policy, now=now), policy, now=now)


VIP_AT_0 = ident(0.0)


def test_default_is_off():
    assert ap.DEFAULT["vip_grace_sec"] == 0.0        # ★ 2026-11-25：預設關閉（原因寫在 access_policy.DEFAULT）


def test_unrecognized_within_grace_is_vip():
    assert tier(ident(10.0, recognized=False, kind="GUEST", name="Unknown"), VIP_AT_0, 10.0) == ap.TIER_VIP


def test_nobody_within_grace_is_vip_and_keeps_name():
    g = ap.with_vip_grace(ident(20.0, recognized=False, kind="GUEST", name="Unknown"), VIP_AT_0, POLICY, now=20.0)
    assert g["user_name"] == "A" and g["vip_grace"] and ap.tier_of(g, POLICY, now=20.0) == ap.TIER_VIP


def test_grace_ends_after_30_seconds():
    assert tier(ident(31.0, recognized=False, kind="GUEST"), VIP_AT_0, 31.0) == ap.TIER_GUEST


def test_grace_outlives_identity_fresh_sec():
    # identity_fresh_sec 10 秒，寬限 30 秒：第 25 秒仍是貴賓
    assert tier(ident(25.0, recognized=False, kind="GUEST"), VIP_AT_0, 25.0) == ap.TIER_VIP


def test_recognized_blacklist_is_never_covered():
    assert tier(ident(5.0, kind="BLACKLIST", name="B"), VIP_AT_0, 5.0) == ap.TIER_BLOCKED


def test_recognized_other_registered_guest_is_not_covered():
    assert tier(ident(5.0, kind="GUEST", name="C"), VIP_AT_0, 5.0) == ap.TIER_GUEST


def test_disabled_with_zero():
    assert tier(ident(5.0, recognized=False, kind="GUEST"), VIP_AT_0, 5.0, {**POLICY, "vip_grace_sec": 0}) == ap.TIER_GUEST


def test_no_vip_seen_yet():
    assert tier(ident(5.0, recognized=False, kind="GUEST"), None, 5.0) == ap.TIER_GUEST


def test_policy_file_without_key_gets_default(tmp_path, monkeypatch):
    p = tmp_path / "access_policy.json"
    p.write_text('{"vip_only_waypoints": ["貴賓室"], "identity_fresh_sec": 10.0}', encoding="utf-8")
    monkeypatch.setattr(ap, "POLICY_PATH", p)
    assert ap.load_policy()["vip_grace_sec"] == ap.DEFAULT["vip_grace_sec"]
