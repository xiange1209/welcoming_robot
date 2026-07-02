import json
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from threading import Lock, Thread
from typing import Optional


_FALLBACK = {
    "vip": {
        "intent": "NAVIGATION",
        "action": "navigate_to_vip_lounge",
        "target": "VIP貴賓室",
        "reply": "歡迎光臨！請問今天需要什麼協助？",
    },
    "blacklist": {
        "intent": "COMMAND",
        "action": "alert_staff",
        "target": None,
        "reply": "",
    },
    "visitor": {
        "intent": "CHAT",
        "action": None,
        "target": None,
        "reply": "您好，歡迎光臨！請問需要什麼服務？",
    },
}


@dataclass
class LLMDecision:
    intent: str
    action: Optional[str] = None
    target: Optional[str] = None
    reply: str = ""
    thinking: Optional[str] = None
    source: str = "api"
    received_at: float = field(default_factory=time.time)

    @classmethod
    def from_payload(cls, payload: dict, source: str = "api"):
        return cls(
            intent=payload.get("intent", "CHAT"),
            action=payload.get("action"),
            target=payload.get("target"),
            reply=payload.get("reply", ""),
            thinking=payload.get("thinking"),
            source=source,
        )


class LLMApiClient:
    def __init__(self, base_url: str, enabled: bool = False,
                 timeout_sec: float = 2.0, cooldown_sec: float = 8.0,
                 offline_backoff_sec: float = 30.0):
        self.base_url = base_url.rstrip("/")
        self.endpoint = f"{self.base_url}/api/chat"
        self.enabled = enabled
        self.timeout_sec = timeout_sec
        self.cooldown_sec = cooldown_sec
        self.offline_backoff_sec = offline_backoff_sec

        self._lock = Lock()
        self._worker: Optional[Thread] = None
        self._last_requested_at = 0.0
        self._last_requested_signature: Optional[str] = None
        self._latest_signature: Optional[str] = None
        self._latest_decision: Optional[LLMDecision] = None
        self._disabled_until = 0.0

    def submit_if_due(self, payload: dict, signature: str) -> Optional[LLMDecision]:
        now = time.time()
        with self._lock:
            current = self._latest_decision if self._latest_signature == signature else None

            if not self.enabled:
                return current

            if now < self._disabled_until:
                return current

            if self._last_requested_signature == signature and now - self._last_requested_at < self.cooldown_sec:
                return current

            if self._worker is not None and self._worker.is_alive():
                return current

            self._last_requested_at = now
            self._last_requested_signature = signature
            self._worker = Thread(
                target=self._request_worker,
                args=(payload, signature),
                daemon=True,
            )
            self._worker.start()
            return current

    def get_latest_decision(self, signature: Optional[str] = None) -> Optional[LLMDecision]:
        with self._lock:
            if signature is None or self._latest_signature == signature:
                return self._latest_decision
            return None

    def _request_worker(self, payload: dict, signature: str):
        decision = self._request_decision(payload)
        with self._lock:
            self._latest_signature = signature
            self._latest_decision = decision

    def _request_decision(self, payload: dict) -> LLMDecision:
        try:
            request = urllib.request.Request(
                self.endpoint,
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                data = json.loads(response.read().decode("utf-8"))
            return LLMDecision.from_payload(data, source="api")
        except (urllib.error.URLError, TimeoutError, OSError, ValueError, json.JSONDecodeError):
            with self._lock:
                self._disabled_until = time.time() + self.offline_backoff_sec

            fallback_key = payload.get("face_recognition", "visitor")
            fallback = _FALLBACK.get(fallback_key, _FALLBACK["visitor"])
            return LLMDecision.from_payload(fallback, source="fallback")