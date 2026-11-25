"""客人會看到／聽到的話（★ 2026-11-25）：英文錯誤不外露、固定答案由程式回答、指示不照唸、被詐騙提 165。

不連 Ollama、不起 ROS：真的呼叫 LLMServiceNode._run_agent_loop，模型與話題用替身。
"""

import threading
import types

import pytest

from smartnav_llm import customer_prompt, llm_service_node as mod, nav_intent as ni

FAQ = """# 示範銀行常見問題

## 地點
- 示範地點：大廳、貴賓室。

## 營業時間
- 週一到週五，上午九點到下午三點半；週六、週日和國定假日不營業。ATM 開放時間沒有資訊。

## 開戶與臨櫃業務
- 開戶請帶身分證。
"""


# ── 純函式 ────────────────────────────────────────────────
@pytest.mark.parametrize("text", ["那我能去哪裡呀", "我可以去哪裡", "你可以帶我去哪裡？", "有哪些地方可以去", "哪裡可以去", "可以去那些地方"])
def test_asks_destinations(text):
    assert ni.asks_destinations(text)


@pytest.mark.parametrize("text", ["我要去哪裡開戶", "可以去哪裡換錢", "帶我去貴賓室", "洗手間在哪裡", "你好", "我去哪裡辦信用卡",
                                  "可以帶我去那裡嗎", "能帶我到那邊嗎", "可以去那邊坐嗎", "我可以到那邊等嗎",
                                  "有什麼地方要簽名", "請問有什麼地方可以掛號"])
def test_not_asking_destinations(text):
    assert not ni.asks_destinations(text)


@pytest.mark.parametrize("text", ["營業時間是什麼時候", "你們幾點關門", "營業到幾點", "週六有營業嗎", "今天有上班嗎", "貴賓室幾點開"])
def test_asks_hours(text):
    assert ni.asks_business_hours(text)


@pytest.mark.parametrize("text", ["ATM 幾點開", "現在幾點", "停車場幾點關門", "我要開戶", "帶我去櫃檯"])
def test_not_asking_hours(text):
    assert not ni.asks_business_hours(text)


def test_hours_sentence_is_copied_from_the_knowledge_base():
    assert ni.faq_business_hours(FAQ) == "週一到週五，上午九點到下午三點半；週六、週日和國定假日不營業。"
    assert ni.faq_business_hours("# 沒有這一節") is None and ni.faq_business_hours("") is None


def test_real_faq_file_has_the_hours_section():
    from pathlib import Path
    faq = (Path(__file__).resolve().parents[1] / "config" / "bank_faq.txt").read_text(encoding="utf-8")
    assert "週一到週五" in ni.faq_business_hours(faq)


@pytest.mark.parametrize("text,yes", [("我被詐騙怎麼辦", True), ("我被騙了", True), ("有可疑的電話", True), ("我要開戶", False)])
def test_mentions_fraud(text, yes):
    assert ni.mentions_fraud(text) is yes


@pytest.mark.parametrize("reply,yes", [
    ("請告知客戶行員即將前來", True), ("已送出通報，告訴客人請稍候", True), ("執行結果: 成功", True),
    ("行員馬上會過來為您服務", False), ("開戶請帶身分證", False), ("如果有問題請告訴我", False)])
def test_leaks_instruction(reply, yes):
    assert ni.leaks_instruction(reply) is yes


def test_guest_context_never_invites_to_vip_room():
    text = customer_prompt.prompt_context("guest", "", "")
    assert "一般訪客" in text and "貴賓專屬" in text
    assert "貴賓您好" in customer_prompt.prompt_context("vip", "王", "")
    assert "貴賓室" not in customer_prompt.prompt_context("blocked", "", "")


# ── 節點 ──────────────────────────────────────────────────
class Handler:
    seen_token, ttft, held = True, None, []

    def __init__(self):
        self.flushed, self.dropped = 0, 0

    def start_turn(self):
        pass

    def begin_turn(self, hold=True):
        pass

    def flush_held(self):
        self.flushed += 1

    def drop_held(self):
        self.dropped += 1


class Reply:
    def __init__(self, content, tool_calls=None):
        self.content, self.tool_calls, self.response_metadata = content, tool_calls or [], {}


class Chain:
    def __init__(self, replies):
        self.replies = list(replies)
        self.calls = 0

    def invoke(self, _data):
        self.calls += 1
        r = self.replies.pop(0)
        if isinstance(r, Exception):
            raise r
        return r


@pytest.fixture
def node():
    n = object.__new__(mod.LLMServiceNode)
    n.said, n.resets, n.logs = [], [], []
    log = types.SimpleNamespace(info=lambda t: n.logs.append(("info", t)), warning=lambda t: n.logs.append(("warn", t)),
                                error=lambda t: n.logs.append(("error", t)), warn=lambda t: n.logs.append(("warn", t)))
    n.get_logger = lambda: log
    n._publish_reply = n.said.append
    n.llm_stream_reset_pub = types.SimpleNamespace(publish=n.resets.append)
    n.stream_handler = Handler()
    n.memory_lock = threading.Lock()
    n.memory = types.SimpleNamespace(get_history_messages=lambda: [], add_message=lambda m: None)
    n.agent_busy_lock = threading.Lock()
    n.is_agent_running = True
    n._access_context, n.access_context_stale_sec = None, 5.0
    n.hold_all_iterations, n.direct_nav, n.nav_guard = True, True, True
    n.num_predict, n._ollama_last, n._llm_busy = 256, 0.0, False
    n._knowledge_section = lambda *a: ""
    n._publish_timing = lambda *a: None
    n._load_system_prompt = lambda: "SYS"
    n._to_traditional = lambda t: t
    n._try_direct_navigation = lambda *a, **k: False
    n.knowledge = types.SimpleNamespace(full_text=FAQ)
    n.tools_map = {"notify_staff_tool": types.SimpleNamespace(invoke=lambda a: "執行結果: 成功, 詳細信息: 行員已收到通知，正在前來")}
    n._guide_destinations = lambda: [("w1", "櫃檯"), ("w2", "大廳")]
    n.retrieved = []
    n._knowledge_section = lambda *a: n.retrieved.append(1) or ""
    return n


def run(node, text, *replies):
    node.chain = Chain(replies)
    node._chain_for = lambda _t: node.chain
    node._run_agent_loop(text)
    assert node.is_agent_running is False
    return node.said


def test_error_is_never_shown_to_the_guest(node):
    said = run(node, "我要開戶", RuntimeError("Server disconnected without sending a response."))
    assert said == [mod.LLMServiceNode.SORRY_REPLY]
    assert node.stream_handler.dropped == 1 and len(node.resets) == 1
    assert any(k == "error" and "Server disconnected" in t for k, t in node.logs)


def test_too_many_steps_is_polite(node):
    call = [{"name": "nope", "args": {}, "id": "1"}]
    said = run(node, "我要開戶", *[Reply("", call) for _ in range(10)])
    assert said == [mod.LLMServiceNode.SORRY_REPLY]


def test_destinations_come_from_the_arbiter_not_the_model(node):
    said = run(node, "那我能去哪裡呀")
    assert said == ["我可以帶您去櫃檯、大廳。"] and node.chain.calls == 0


def test_destinations_fall_back_to_the_model_when_arbiter_is_silent(node):
    node._guide_destinations = lambda: None
    assert run(node, "我可以去哪裡", Reply("請洽詢櫃檯行員。")) == ["請洽詢櫃檯行員。"]


def test_business_hours_are_fixed(node):
    said = run(node, "你們幾點關門")
    assert said == ["本行的營業時間是週一到週五，上午九點到下午三點半；週六、週日和國定假日不營業。"] and node.chain.calls == 0


def test_food_is_not_made_up(node):
    said = run(node, "我肚子餓")
    assert len(said) == 1 and "食堂" not in said[0] and node.chain.calls == 0
    assert node.retrieved == []                   # 固定答案不必先做知識庫檢索


@pytest.mark.parametrize("text", ["我的錢被騙走了，你們幾點開門", "詐騙集團叫我去匯款，你們營業到幾點", "我不舒服，可以去哪裡"])
def test_fixed_answers_never_swallow_a_call_for_help(node, text):
    said = run(node, text, Reply("我請行員過來協助您。"))
    assert node.chain.calls == 1 and "營業時間" not in said[0]


def test_help_is_still_sent_when_the_model_is_down(node):
    said = run(node, "我被詐騙了怎麼辦", RuntimeError("連不上"))
    assert said == ["我已經通知行員過來協助您，請稍候。也可以撥打 165 反詐騙專線。"]


def test_staff_is_not_notified_twice_when_the_model_dies_after_notifying(node):
    sent = []
    node.tools_map["notify_staff_tool"] = types.SimpleNamespace(invoke=lambda a: sent.append(1) or "執行結果: 成功")
    call = [{"name": "notify_staff_tool", "args": {}, "id": "1"}]
    run(node, "叫你們主管出來", Reply("", call), RuntimeError("連不上"))
    assert len(sent) == 1


def test_instruction_leak_is_replaced_after_notifying(node):
    call = [{"name": "notify_staff_tool", "args": {}, "id": "1"}]
    said = run(node, "叫你們主管出來", Reply("", call), Reply("請告知客戶行員即將前來"))
    assert said == ["我已經通知行員，請您稍候。"] and node.stream_handler.flushed == 0


def test_instruction_leak_without_notifying_does_not_claim_it(node):
    said = run(node, "叫你們主管出來", Reply("請告知客戶行員即將前來"))
    assert "已經通知" not in said[0]


def test_fraud_reply_always_mentions_165(node):
    said = run(node, "我被詐騙怎麼辦", Reply("已經將您的要求轉介給專員，他們會為您提供幫助。"))
    assert said == ["已經將您的要求轉介給專員，他們會為您提供幫助。也可以撥打 165 反詐騙專線。"]


def test_fraud_reply_with_165_is_untouched(node):
    said = run(node, "我被詐騙怎麼辦", Reply("請撥打 165 反詐騙專線。"))
    assert said == ["請撥打 165 反詐騙專線。"] and node.stream_handler.flushed == 1


def test_normal_reply_is_untouched(node):
    assert run(node, "我要開戶", Reply("開戶請帶身分證。")) == ["開戶請帶身分證。"]
    assert node.stream_handler.flushed == 1


# ── 地點清單依身分過濾 ─────────────────────────────────────
def _wp(i, name):
    return types.SimpleNamespace(waypoint_id=i, waypoint_name=name, map_id="m",
                                 pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.0, y=0.0)))


@pytest.mark.parametrize("allowed,shown,hidden", [({"w1"}, "櫃檯", "貴賓室"), (None, "貴賓室", None)])
def test_list_waypoints_tool_only_lists_allowed(allowed, shown, hidden):
    from smartnav_llm.tools import make_tools
    res = types.SimpleNamespace(success=True, message="", waypoints_info=[_wp("w1", "櫃檯"), _wp("w9", "貴賓室")])
    fake = types.SimpleNamespace(list_waypoints_client=types.SimpleNamespace(call_async=lambda r: None),
                                 _wait_for_future=lambda f, t: res, _allowed_destination_ids=lambda: allowed)
    out = make_tools(fake, False)["list_waypoints_tool"].invoke({})
    listed = out.split("詳細信息")[0]
    assert shown in listed and (hidden is None or hidden not in listed)
    assert ("不能前往" in out) is (hidden is not None)      # 有被排除的地點時，要告訴模型怎麼回答


# ── 逾時 ──────────────────────────────────────────────────
def test_chat_llm_gets_http_timeouts():
    n = object.__new__(mod.LLMServiceNode)
    n.llm_read_timeout_sec, n.llm_connect_timeout_sec = 45.0, 3.0
    n.get_logger = lambda: types.SimpleNamespace(warning=lambda t: None)
    kw = n._ollama_client_kwargs()
    if kw:                                        # 這版 langchain_ollama 支援才檢查
        t = kw["client_kwargs"]["timeout"]
        assert (t.connect, t.read) == (3.0, 45.0)
        assert n._ollama_client_kwargs(180.0)["client_kwargs"]["timeout"].read == 180.0


def test_default_read_timeout_is_longer_than_the_measured_cold_start():
    from pathlib import Path
    import re
    src = Path(mod.__file__).read_text(encoding="utf-8")
    read = float(re.search(r'"llm_read_timeout_sec", ([\d.]+)', src).group(1))
    assert read > 46.83                              # 2026-10-21 實測：第一次從硬碟載入模型的秒數
