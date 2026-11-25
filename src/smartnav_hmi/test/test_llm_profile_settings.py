"""HMI 的 LLM 主機設定（modules/chat/llm_profile.py）。★ 2026-11-24"""

import json

import pytest

from smartnav_hmi.modules.chat.llm_profile import DEFAULT_KEY, LlmProfileSettings


def make(tmp_path, settings=None, profiles=None):
    s, p = tmp_path / "llm_settings.json", tmp_path / "llm_profiles.json"
    if settings is not None:
        s.write_text(settings, encoding="utf-8")
    if profiles is not None:
        p.write_text(profiles, encoding="utf-8")
    return LlmProfileSettings(path=s, profiles_path=p)


def test_defaults(tmp_path):
    st = make(tmp_path)
    assert [p["key"] for p in st.profiles] == ["auto", "classmate", "mine"]
    assert st.current()["key"] == DEFAULT_KEY == "auto"
    mine = next(p for p in st.profiles if p["key"] == "mine")
    assert mine["model"].startswith("qwen2.5") and "192.168.11.74" in mine["hosts"]
    classmate = next(p for p in st.profiles if p["key"] == "classmate")
    assert classmate["model"] == "llama3.1:8b" and classmate["hosts"][0] == "192.168.11.76"


def test_select_persists(tmp_path):
    st = make(tmp_path)
    assert st.select("mine")["key"] == "mine"
    assert json.loads((tmp_path / "llm_settings.json").read_text(encoding="utf-8")) == {"profile": "mine"}
    assert make(tmp_path).current()["key"] == "mine"                     # 重開 HMI 也記得


def test_unknown_key_rejected(tmp_path):
    st = make(tmp_path)
    with pytest.raises(ValueError):
        st.select("nope")
    assert st.current()["key"] == "auto"


def test_broken_settings_file_falls_back(tmp_path):
    assert make(tmp_path, settings="{broken").current()["key"] == "auto"
    assert make(tmp_path, settings='{"profile": "gone"}').current()["key"] == "auto"


def test_custom_profiles_file(tmp_path):
    custom = [{"key": "lab", "label": "實驗室", "model": "qwen2.5:7b", "hosts": ["10.0.0.5"]}]
    st = make(tmp_path, profiles=json.dumps(custom, ensure_ascii=False))
    assert [p["key"] for p in st.profiles] == ["lab"] and st.current()["key"] == "lab"


def test_broken_profiles_file_uses_defaults(tmp_path):
    assert [p["key"] for p in make(tmp_path, profiles="[{]").profiles] == ["auto", "classmate", "mine"]
    assert [p["key"] for p in make(tmp_path, profiles="[1, 2]").profiles] == ["auto", "classmate", "mine"]
