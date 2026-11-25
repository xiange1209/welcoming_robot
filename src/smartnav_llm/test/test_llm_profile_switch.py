"""平板選 LLM 主機／模型（★ 2026-11-24）：pick_model 與 LLMServiceNode 的即時切換。不連真的 Ollama。"""

import threading
import types

import pytest

from smartnav_llm import llm_service_node as mod
from smartnav_llm.ollama_locator import pick_model

PARAM_HOSTS = ["192.168.11.76", "127.0.0.1", "MSI.mshome.net", "MSI.local", "gateway", "192.168.137.1"]
MINE = {"key": "mine", "label": "我的筆電", "model": "qwen2.5:3b",
        "hosts": ["192.168.11.74", "MSI.mshome.net", "MSI.local", "gateway", "192.168.137.1"]}
CLASSMATE = {"key": "classmate", "label": "同學電腦", "model": "llama3.1:8b", "hosts": ["192.168.11.76", "127.0.0.1"]}
AUTO = {"key": "auto", "label": "自動", "model": "", "hosts": []}


@pytest.mark.parametrize("want,have,got", [
    ("qwen2.5:3b", ["qwen2.5:3b", "bge-m3:latest"], "qwen2.5:3b"),
    ("qwen2.5:3b", ["qwen2.5:7b", "bge-m3:latest"], "qwen2.5:7b"),       # 同系列別的版本
    ("llama3.1:8b", ["llama3.1:latest"], "llama3.1:latest"),
    ("qwen2.5", ["qwen2.5:latest"], "qwen2.5:latest"),                   # 沒寫 tag＝:latest
    ("qwen2.5:3b", ["qwen3:4b", "llama3.1:8b"], None),                    # qwen3 不是 qwen2.5
    ("llama3.1:8b", [], None),
])
def test_pick_model(want, have, got):
    assert pick_model(want, have) == got


class FakeLocator:
    made = []
    url = "http://192.168.11.74:11434"

    def __init__(self, cands, on_change, info, warn, retry_sec=15.0):
        self.cands, self.on_change, self.stopped, self.started = list(cands), on_change, False, False
        FakeLocator.made.append(self)

    def resolve_now(self):
        if FakeLocator.url:
            self.on_change(FakeLocator.url)
        return FakeLocator.url

    def start_background(self):
        self.started = True

    def stop(self):
        self.stopped = True


class Resp:
    def __init__(self, names):
        self._names = names

    def json(self):
        return {"models": [{"name": n} for n in self._names]}


@pytest.fixture
def node(monkeypatch):
    FakeLocator.made, FakeLocator.url = [], "http://192.168.11.74:11434"
    monkeypatch.setattr(mod, "OllamaLocator", FakeLocator)
    tags = {"names": ["qwen2.5:7b", "bge-m3:latest"]}
    monkeypatch.setattr(mod.requests, "get", lambda url, timeout=0: Resp(tags["names"]))
    n = object.__new__(mod.LLMServiceNode)
    n._param_model = n._requested_model = n.model_name = "llama3.1:8b"
    n._param_candidates = list(PARAM_HOSTS)
    n.ollama_candidates = list(PARAM_HOSTS)
    n._llm_switch_lock = threading.RLock()
    n._profile_key, n._profile_label = "auto", "自動"
    n._llm_ready_for_profile, n._pending_profile = True, None
    n.ollama_base_url = "http://192.168.11.76:11434"
    n._embedder = None
    n._ollama_locator = FakeLocator(PARAM_HOSTS, n._on_ollama_url, None, None)
    built, published, logs = [], [], []
    n._raw_llm, n._chains, n.stream_handler = object(), {"x": 1}, None
    n._make_chat_llm = lambda cb: built.append((n.ollama_base_url, n.model_name)) or object()
    n.llm_model_pub = types.SimpleNamespace(publish=lambda m: published.append(m.data))
    log = types.SimpleNamespace(info=lambda t: logs.append(("info", t)), warning=lambda t: logs.append(("warn", t)),
                                error=lambda t: logs.append(("error", t)), warn=lambda t: logs.append(("warn", t)))
    n.get_logger = lambda: log
    n._warmup_llm = lambda: None
    n.t = types.SimpleNamespace(built=built, published=published, logs=logs, tags=tags)
    return n


def test_switch_to_my_laptop_uses_same_family_when_exact_missing(node):
    node._switch_profile(MINE)
    assert node.ollama_candidates == MINE["hosts"]
    assert node._ollama_locator.cands == MINE["hosts"] and node._ollama_locator.started
    assert FakeLocator.made[0].stopped                                   # 舊的找主機執行緒停掉
    assert node.ollama_base_url == "http://192.168.11.74:11434"
    assert node.model_name == "qwen2.5:7b"                               # 沒有 3b → 同系列 7b
    assert node.t.built[-1] == ("http://192.168.11.74:11434", "qwen2.5:7b") and node._chains == {}
    assert node.t.published[-1] == "qwen2.5:7b"
    assert any("改用同系列的 qwen2.5:7b" in t for _k, t in node.t.logs)


def test_switch_back_to_auto_restores_parameters(node):
    node._switch_profile(MINE)
    node.t.tags["names"] = ["llama3.1:8b", "bge-m3:latest"]
    FakeLocator.url = "http://192.168.11.76:11434"
    node._switch_profile(AUTO)
    assert node.ollama_candidates == PARAM_HOSTS and node.model_name == "llama3.1:8b"
    assert node.t.published[-1] == "llama3.1:8b"


def test_same_profile_twice_does_nothing(node):
    node._switch_profile(MINE)
    n_built, n_loc = len(node.t.built), len(FakeLocator.made)
    node._switch_profile(MINE)
    assert len(node.t.built) == n_built and len(FakeLocator.made) == n_loc


def test_missing_family_keeps_name_and_says_why(node):
    node.t.tags["names"] = ["gemma3:4b"]
    node._switch_profile(MINE)
    assert node.model_name == "qwen2.5:3b"
    assert any(k == "error" and "ollama pull qwen2.5:3b" in t for k, t in node.t.logs)


def test_unreachable_hosts_keep_looking_in_background(node):
    FakeLocator.url = None
    node._switch_profile(CLASSMATE)
    assert node._ollama_locator.started
    assert any("都連不到" in t for _k, t in node.t.logs)


def test_message_during_startup_is_deferred(node):
    node._llm_ready_for_profile = False
    node._on_llm_profile_msg(types.SimpleNamespace(data='{"key": "mine", "model": "qwen2.5:3b", "hosts": ["h"]}'))
    assert node._pending_profile["key"] == "mine"
    assert node.model_name == "llama3.1:8b"                              # 還沒套用（交給 __init__）


def test_bad_json_is_ignored(node):
    node._on_llm_profile_msg(types.SimpleNamespace(data="not json"))
    assert node.model_name == "llama3.1:8b" and node.t.built == []
