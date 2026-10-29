"""自動找 Ollama 在哪台機器（2026-10-29）。

為什麼：Ollama 跑在筆電上，筆電的 IP 跟著網路變——開自己的熱點時是 192.168.137.1，
在同學那邊的網路是 192.168.11.156。寫死一個 IP，換網路就整個 LLM 連不到（節點照常啟動，對話時才失敗）。

做法：照順序列出候選（電腦名稱、預設閘道、已知 IP），**同時**對每個候選打 GET /api/tags，
在總期限內有回應的候選裡取「排最前面的」。
- 電腦名稱：Windows 熱點會把筆電登記成 `<電腦名稱>.mshome.net`（車上實測 192.168.137.1 → MSI.mshome.net）；
  其他網路靠 mDNS 的 `<電腦名稱>.local`（車上有 avahi＋libnss-mdns）。
- `gateway`＝這台車現在的預設閘道（開筆電熱點時就是筆電）。
- ★ 名稱解析（尤其 .local）可能卡好幾秒，而且 requests 的 timeout 管不到 DNS——所以每個候選各開一條
  daemon 執行緒，只等總期限，不會因為某個名稱卡住而拖慢啟動。
"""

import socket
import struct
import threading
from typing import Callable, List, Optional, Tuple

import requests

DEFAULT_PORT = 11434


def default_gateway() -> Optional[str]:
    """讀 /proc/net/route 的預設路由閘道（IPv4）；讀不到回 None。"""
    try:
        with open("/proc/net/route", encoding="ascii") as fh:
            for line in fh.readlines()[1:]:
                f = line.split()
                if len(f) >= 4 and f[1] == "00000000" and int(f[3], 16) & 2:   # 目的 0.0.0.0、RTF_GATEWAY
                    return socket.inet_ntoa(struct.pack("<L", int(f[2], 16)))
    except (OSError, ValueError):
        pass
    return None


def candidate_url(entry: str) -> Optional[str]:
    """候選字串 → http://主機:埠。接受 `gateway`、主機名、IP、`主機:埠`、完整 URL；空字串或閘道找不到回 None。"""
    e = (entry or "").strip()
    if not e:
        return None
    if e.lower() == "gateway":
        gw = default_gateway()
        return f"http://{gw}:{DEFAULT_PORT}" if gw else None
    if "://" in e:
        return e.rstrip("/")
    host, _, port = e.partition(":")
    return f"http://{host}:{port or DEFAULT_PORT}"


def find_ollama(candidates: List[str], deadline_sec: float = 3.0,
                log: Optional[Callable[[str], None]] = None) -> Tuple[Optional[str], List[str]]:
    """回 (找到的 base_url 或 None, 實際試過的 URL 清單)。期限內有回應者取排最前面的。"""
    urls: List[str] = []
    for c in candidates:
        u = candidate_url(c)
        if u and u not in urls:
            urls.append(u)
    ok = {}
    lock = threading.Lock()
    done = threading.Event()

    def probe(u: str) -> None:
        try:
            r = requests.get(f"{u}/api/tags", timeout=max(0.5, deadline_sec - 0.2))
            good = r.status_code == 200
        except Exception:  # noqa: BLE001 —— 連不到、名稱解析失敗都算沒有
            good = False
        with lock:
            ok[u] = good
            # 排最前面的已經有結果、而且是好的 → 不用等其他人
            for x in urls:
                if x not in ok:
                    break
                if ok[x]:
                    done.set()
                    break
            if len(ok) == len(urls):
                done.set()

    for u in urls:
        threading.Thread(target=probe, args=(u,), daemon=True).start()
    done.wait(timeout=deadline_sec)
    with lock:
        for u in urls:
            if ok.get(u):
                return u, urls
    if log:
        log(f"Ollama：{deadline_sec:.0f} 秒內 {len(urls)} 個候選都沒有回應（{', '.join(urls)}）")
    return None, urls


class OllamaLocator:
    """`auto` 模式：啟動時找一次；找不到或之後連不上，就在背景每 `retry_sec` 秒重找，找到就呼叫 on_change。"""

    def __init__(self, candidates: List[str], on_change: Callable[[str], None],
                 log_info: Callable[[str], None], log_warn: Callable[[str], None], retry_sec: float = 15.0):
        self._cands = list(candidates)
        self._on_change = on_change
        self._info, self._warn = log_info, log_warn
        self._retry = max(5.0, float(retry_sec))
        self.current: Optional[str] = None
        self._stop = threading.Event()

    def resolve_now(self) -> Optional[str]:
        url, tried = find_ollama(self._cands)
        if url:
            if url != self.current:
                self._info(f"Ollama：自動找到 {url}（依序試了 {', '.join(tried)}）")
                self.current = url
                self._on_change(url)
        return url

    def start_background(self) -> None:
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self) -> None:
        self._stop.set()

    def _alive(self, url: str) -> bool:
        try:
            return requests.get(f"{url}/api/tags", timeout=2.0).status_code == 200
        except Exception:  # noqa: BLE001
            return False

    def _loop(self) -> None:
        warned = False
        while not self._stop.wait(self._retry):
            if self.current and self._alive(self.current):
                warned = False
                continue
            if self.resolve_now():
                warned = False
            elif not warned:
                self._warn(f"Ollama：目前連不到，背景每 {self._retry:.0f} 秒重找一次（筆電的 Ollama 有開嗎？）")
                warned = True
