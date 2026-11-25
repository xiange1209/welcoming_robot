"""LLM 主機／模型的選擇（★ 2026-11-24）。

使用者：「LLM 那邊可以選擇我要開哪個模型嗎？可能是同學的 llama3.1 或是我的千問 2.5」。

存在 ~/.smartnav/llm_settings.json（每台平板看到同一個設定、重開機也記得）。
HMI 啟動時與每次變更時，用 latched 話題 llm_profile（JSON）廣播；llm_service 收到就換主機與模型，**不用重開**。

  auto       自動：照 llm_service 自己的預設（候選主機依序找、模型 llama3.1:8b）—— 跟以前一模一樣
  classmate  同學電腦：192.168.11.76（或反向 SSH 通道的 127.0.0.1），llama3.1:8b
  mine       我的筆電：MSI（192.168.11.74／MSI.mshome.net／熱點 192.168.137.1），qwen2.5:3b
             ★ 主機上沒有指定的那一版時，llm_service 會改用同系列的其他版本（例如 qwen2.5:7b）並在 log 註明

主機或模型要改：在 ~/.smartnav/llm_profiles.json 放一份同格式的清單（list of {key,label,detail,model,hosts}）即可，
不用改程式；model 或 hosts 留空＝用 llm_service 自己的預設。
"""

import json
import threading
from pathlib import Path
from typing import Dict, List, Optional

DEFAULT_PROFILES: List[Dict] = [
    {"key": "auto", "label": "自動", "detail": "先找到哪台就用哪台（照舊）", "model": "", "hosts": []},
    {"key": "classmate", "label": "同學電腦", "detail": "llama3.1:8b", "model": "llama3.1:8b",
     "hosts": ["192.168.11.76", "127.0.0.1"]},
    {"key": "mine", "label": "我的筆電", "detail": "千問 qwen2.5", "model": "qwen2.5:3b",
     "hosts": ["192.168.11.74", "MSI.mshome.net", "MSI.local", "gateway", "192.168.137.1"]},
]
DEFAULT_KEY = "auto"


class LlmProfileSettings:
    def __init__(self, path: Optional[Path] = None, profiles_path: Optional[Path] = None, logger=None):
        home = Path.home() / ".smartnav"
        self.path = path or home / "llm_settings.json"
        self.profiles_path = profiles_path or home / "llm_profiles.json"
        self._logger = logger
        self._lock = threading.Lock()
        self.profiles = self._load_profiles()
        self._key = self._load_key()

    def _warn(self, text: str) -> None:
        if self._logger:
            self._logger.warn(text)

    def _load_profiles(self) -> List[Dict]:
        try:
            raw = json.loads(self.profiles_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return [dict(p) for p in DEFAULT_PROFILES]
        except Exception as exc:  # noqa: BLE001 —— 壞檔不能讓 HMI 起不來
            self._warn(f"LLM 主機清單 {self.profiles_path} 讀取失敗，改用預設：{exc}")
            return [dict(p) for p in DEFAULT_PROFILES]
        out = []
        for p in raw if isinstance(raw, list) else []:
            if isinstance(p, dict) and p.get("key") and p.get("label"):
                out.append({"key": str(p["key"]), "label": str(p["label"]), "detail": str(p.get("detail", "")),
                            "model": str(p.get("model", "")),
                            "hosts": [str(h) for h in (p.get("hosts") or []) if str(h).strip()]})
        if not out:
            self._warn(f"LLM 主機清單 {self.profiles_path} 沒有可用的項目，改用預設")
        return out or [dict(p) for p in DEFAULT_PROFILES]

    def _load_key(self) -> str:
        try:
            key = json.loads(self.path.read_text(encoding="utf-8")).get("profile")
        except Exception:  # noqa: BLE001 —— 沒檔或壞檔都退回預設
            key = None
        keys = [p["key"] for p in self.profiles]
        if key in keys:
            return key
        return DEFAULT_KEY if DEFAULT_KEY in keys else keys[0]

    def current(self) -> Dict:
        with self._lock:
            return dict(next(p for p in self.profiles if p["key"] == self._key))

    def select(self, key: str) -> Dict:
        """換主機並寫檔。不認得的 key 丟 ValueError（呼叫端回 400）"""
        if not any(p["key"] == key for p in self.profiles):
            raise ValueError(f"沒有這個選項：{key!r}（可用：{', '.join(p['key'] for p in self.profiles)}）")
        with self._lock:
            self._key = key
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps({"profile": key}, ensure_ascii=False, indent=1), encoding="utf-8")
        tmp.replace(self.path)                        # 原子替換：寫到一半斷電也不會留下半個檔
        return self.current()
