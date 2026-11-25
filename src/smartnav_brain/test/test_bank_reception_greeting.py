"""迎賓詞不要一直打斷對話（bank_reception_node._should_greet／identity_callback）。

★ 2026-11-24：使用者「一直說某某貴賓你好 打斷我們講話」；11/24 17:04～18:09 實測 VIP 迎賓 23 次、一般 19 次。
用假時鐘餵 UserIdentity，不需要 ROS 執行環境。
"""

import types

import pytest

from smartnav_brain import bank_reception_node as brn

VIP, GUEST, BLACKLIST = 1, 0, 3


class Sim:
    def __init__(self, monkeypatch):
        self.t = 1000.0
        monkeypatch.setattr(brn.time, "monotonic", lambda: self.t)
        n = object.__new__(brn.BankReceptionNode)
        n.cooldown_sec, n.visitor_cooldown_sec, n.min_confidence = 60.0, 300.0, 0.5
        n.regreet_absent_sec, n.vip_shadow_sec = 180.0, 60.0
        n.visitor_absent_reset_sec, n.quiet_after_talk_sec, n.visitor_confirm_sec = 30.0, 15.0, 4.0
        n.greet_pending_max_sec = 30.0
        n.vip_greeting, n.visitor_greeting = "{name}貴賓您好", "您好歡迎光臨"
        n.staff_alert_vip, n.staff_alert_blacklist, n.staff_alert_visitor = "出現貴賓{name}", "出現黑名單{name}", "出現訪客"
        n._last_seen, n._seen_at = {}, {}
        n._last_face_at = n._last_vip_at = n._last_talk_at = -1e9
        n._state = "IDLE"
        n._unknown_since = None
        n._pending = None
        self.said, self.alerts, self.logged = [], [], []
        n._say = self.said.append
        n._staff_alert = lambda tpl, name: self.alerts.append(tpl.format(name=name))
        n._emit_event = lambda *a: None
        n._log_visit = lambda msg, ptype: self.logged.append(ptype)
        n._send_notification = lambda text: None
        log = types.SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None,
                                    warning=lambda *a, **k: None, error=lambda *a, **k: None)
        n.get_logger = lambda: log
        self.n = n

    def face(self, kind, name="A", uuid="u-a"):
        recognized = kind in (VIP, BLACKLIST)
        msg = types.SimpleNamespace(bbox=[10.0, 10.0, 60.0, 60.0], user_name=name if recognized else "Unknown",
                                    user_type=types.SimpleNamespace(type=kind), recognized=recognized,
                                    similarity=0.8 if recognized else 0.6, user_uuid=uuid if recognized else "")
        self.n.identity_callback(msg)

    def nobody(self):
        self.n.identity_callback(types.SimpleNamespace(bbox=[0.0, 0.0, 0.0, 0.0], user_name="Unknown",
                                                       user_type=types.SimpleNamespace(type=GUEST), recognized=False,
                                                       similarity=0.0, user_uuid=""))

    def run(self, secs, step, fn):
        end = self.t + secs
        while self.t < end:
            fn()
            self.t += step

    def vip_greetings(self):
        return [s for s in self.said if "貴賓" in s]

    def visitor_greetings(self):
        return [s for s in self.said if s == "您好歡迎光臨"]


@pytest.fixture
def sim(monkeypatch):
    return Sim(monkeypatch)


def test_vip_standing_ten_minutes_is_greeted_once(sim):
    sim.run(600, 1.5, lambda: sim.face(VIP))          # 舊版：每 60 秒一次 → 10 次
    assert len(sim.vip_greetings()) == 1


def test_vip_who_leaves_and_comes_back_is_greeted_again(sim):
    sim.run(120, 1.5, lambda: sim.face(VIP))
    sim.run(240, 10, sim.nobody)                       # 離開 4 分鐘
    sim.run(30, 1.5, lambda: sim.face(VIP))
    assert len(sim.vip_greetings()) == 2


def test_flapping_vip_gets_no_visitor_greeting(sim):
    frames = iter(range(10 ** 6))
    # 11/24 實況：認出與「沒過門檻」交錯（成功 234／未達 113），偶爾 10 秒沒臉
    def step():
        k = next(frames)
        if k % 40 in range(30, 38):
            return                                     # 這幾幀沒偵測到臉
        sim.face(VIP if k % 3 else GUEST)
    sim.run(600, 1.5, step)
    assert len(sim.vip_greetings()) == 1
    assert sim.visitor_greetings() == []


def test_new_visitor_after_empty_lobby_is_greeted(sim):
    sim.run(60, 1.5, lambda: sim.face(GUEST))
    sim.run(40, 10, sim.nobody)                        # 鏡頭前空了 40 秒
    sim.run(20, 1.5, lambda: sim.face(GUEST))          # 下一位（300 秒冷卻內）
    assert len(sim.visitor_greetings()) == 2


def test_visitor_looking_away_briefly_is_not_regreeted(sim):
    sim.run(60, 1.5, lambda: sim.face(GUEST))
    sim.t += 12
    sim.nobody()                                       # user_auth 10 秒就發「無人」—— 舊版這裡會解除冷卻
    sim.t += 3
    sim.run(30, 1.5, lambda: sim.face(GUEST))
    assert len(sim.visitor_greetings()) == 1


def test_visitor_after_vip_left_long_enough_is_greeted(sim):
    sim.run(60, 1.5, lambda: sim.face(VIP))
    sim.run(90, 10, sim.nobody)
    sim.run(20, 1.5, lambda: sim.face(GUEST))
    assert len(sim.vip_greetings()) == 1 and len(sim.visitor_greetings()) == 1


def test_greeting_waits_until_talk_stops(sim):
    sim.n._last_talk_at = sim.t - 10                   # 貴賓走進來時，旁邊剛有人講完話
    sim.face(VIP)
    assert sim.vip_greetings() == []                   # 不插話
    assert sim.alerts == ["出現貴賓A"] and sim.logged == ["VIP"]   # 行員通知、紀錄照常（不等）
    sim.run(60, 1.5, lambda: sim.face(VIP))
    assert len(sim.vip_greetings()) == 1               # 安靜 15 秒後補一句，只補一次


def test_no_greeting_while_conversation_keeps_going(sim):
    sim.n._last_talk_at = sim.t - 1
    def step():
        if int(sim.t) % 5 == 0:
            sim.n._on_talk(None)                       # 一直在對話（每 5 秒有人講話或 LLM 回答）
        sim.face(VIP)
    sim.run(120, 1.0, step)
    assert sim.vip_greetings() == []
    assert sim.alerts == ["出現貴賓A"] and sim.logged == ["VIP"]


def test_deferred_greeting_not_said_to_someone_else(sim):
    sim.n._last_talk_at = sim.t - 5
    sim.face(VIP)                                      # 貴賓 A 來了，但有人在講話 → 延後
    sim.run(40, 1.5, lambda: sim.face(VIP, name="C", uuid="u-c"))   # 之後鏡頭前只有貴賓 C
    assert "A貴賓您好" not in sim.said


def test_greeting_spoken_once_conversation_is_old(sim):
    sim.n._last_talk_at = sim.t - 31
    sim.face(VIP)
    assert len(sim.vip_greetings()) == 1


def test_blacklist_alerts_once_per_visit_and_sounds_like_visitor(sim):
    sim.run(300, 1.5, lambda: sim.face(BLACKLIST, name="B", uuid="u-b"))
    assert sim.alerts == ["出現黑名單B"]
    assert sim.said == ["您好歡迎光臨"]


def test_vip_first_seen_unrecognized_gets_only_vip_greeting(sim):
    sim.face(GUEST)                                    # 走近時第一幀認不出
    sim.t += 1.5
    sim.run(60, 1.5, lambda: sim.face(VIP))
    assert len(sim.vip_greetings()) == 1 and sim.visitor_greetings() == []


def test_visitor_greeting_waits_a_few_seconds(sim):
    sim.face(GUEST)
    assert sim.said == []
    sim.run(6, 1.5, lambda: sim.face(GUEST))
    assert sim.said == ["您好歡迎光臨"]


def test_talk_callbacks_update_time(sim):
    sim.n._on_talk(types.SimpleNamespace(data="你好"))
    assert sim.n._last_talk_at == sim.t
    sim.t += 5
    sim.n._on_voice_triggered(types.SimpleNamespace(data=False))
    assert sim.n._last_talk_at == sim.t - 5
    sim.n._on_voice_triggered(types.SimpleNamespace(data=True))
    assert sim.n._last_talk_at == sim.t
