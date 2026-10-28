#!/usr/bin/env python3

"""即時資訊查詢模組

提供 LLM 查詢會隨時間變動、無法寫死在知識庫裡的資料，全部使用免金鑰的公開來源：

* **匯率**：即時市場參考匯率
* **日期時間**：本機時鐘

所有查詢皆有逾時保護與短期快取，網路異常時回傳明確的失敗訊息，
讓 LLM 據實告知客戶查詢失敗，而不是編造數字。
"""

import logging
import threading
import time
from datetime import datetime
from typing import Any, Dict, Optional, Tuple

import requests

# 共用連線設定
_REQUEST_TIMEOUT = 8.0
_BROWSER_HEADERS = {
    "User-Agent": "Mozilla/5.0 (X11; Linux aarch64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0 Safari/537.36",
    "Accept-Language": "zh-TW,zh;q=0.9",
}

# 資料來源
_RTER_FX_API = "https://tw.rter.info/capi.php"
_FALLBACK_FX_API = "https://open.er-api.com/v6/latest/USD"

# 常見幣別的中英文說法對照 ISO 代碼
_CURRENCY_ALIASES: Dict[str, str] = {
    "美金": "USD", "美元": "USD", "usd": "USD", "美": "USD",
    "日幣": "JPY", "日元": "JPY", "日圓": "JPY", "jpy": "JPY", "円": "JPY",
    "歐元": "EUR", "歐幣": "EUR", "eur": "EUR",
    "人民幣": "CNY", "人民弊": "CNY", "rmb": "CNY", "cny": "CNY", "台幣兌人民幣": "CNY",
    "港幣": "HKD", "港元": "HKD", "hkd": "HKD",
    "澳幣": "AUD", "澳元": "AUD", "aud": "AUD",
    "韓元": "KRW", "韓幣": "KRW", "krw": "KRW",
    "英鎊": "GBP", "英磅": "GBP", "gbp": "GBP",
    "加幣": "CAD", "加元": "CAD", "cad": "CAD",
    "泰銖": "THB", "泰幣": "THB", "thb": "THB",
    "越南盾": "VND", "vnd": "VND",
    "新加坡幣": "SGD", "新幣": "SGD", "sgd": "SGD",
    "瑞士法郎": "CHF", "法郎": "CHF", "chf": "CHF",
    "紐幣": "NZD", "紐西蘭幣": "NZD", "nzd": "NZD",
    "菲律賓披索": "PHP", "披索": "PHP", "php": "PHP",
    "馬來西亞幣": "MYR", "馬幣": "MYR", "令吉": "MYR", "myr": "MYR",
    "印尼盾": "IDR", "idr": "IDR",
    "澳門幣": "MOP", "mop": "MOP",
    "南非幣": "ZAR", "zar": "ZAR",
    "瑞典克朗": "SEK", "sek": "SEK",
}

_CURRENCY_NAMES: Dict[str, str] = {
    "USD": "美元", "JPY": "日圓", "EUR": "歐元", "CNY": "人民幣", "HKD": "港幣",
    "AUD": "澳幣", "KRW": "韓元", "GBP": "英鎊", "CAD": "加幣", "THB": "泰銖",
    "VND": "越南盾", "SGD": "新加坡幣", "CHF": "瑞士法郎", "NZD": "紐西蘭幣",
    "PHP": "菲律賓披索", "MYR": "馬來西亞令吉", "IDR": "印尼盾", "MOP": "澳門幣",
    "ZAR": "南非幣", "SEK": "瑞典克朗",
}

_WEEKDAYS = ["一", "二", "三", "四", "五", "六", "日"]


def _get_logger() -> logging.Logger:
    """取得模組日誌記錄器

    Returns:
        logging.Logger: 已配置格式的 Logger 實例
    """
    logger = logging.getLogger(__name__)
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s"))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
    return logger


class TTLCache:
    """具存活時間的簡易快取

    避免同一個問題重複打外部 API，也讓連續追問時回應更快。
    """

    def __init__(self) -> None:
        self._data: Dict[str, Tuple[float, Any]] = {}
        self._lock = threading.Lock()

    def get(self, key: str, ttl: float) -> Optional[Any]:
        """取得未過期的快取值

        Args:
            key: 快取鍵值
            ttl: 存活秒數

        Returns:
            Optional[Any]: 快取內容，不存在或已過期時為 None
        """
        with self._lock:
            entry = self._data.get(key)
            if not entry:
                return None
            timestamp, value = entry
            if time.time() - timestamp > ttl:
                self._data.pop(key, None)
                return None
            return value

    def set(self, key: str, value: Any) -> None:
        """寫入快取

        Args:
            key: 快取鍵值
            value: 要快取的內容
        """
        with self._lock:
            self._data[key] = (time.time(), value)


_cache = TTLCache()


def _http_get_json(url: str, params: Optional[Dict[str, Any]] = None,
                   timeout: float = _REQUEST_TIMEOUT, retries: int = 2) -> Any:
    """發出 GET 請求並解析 JSON，失敗時自動重試一次

    機器人多半以 Wi-Fi 連線，偶發的逾時比服務真正掛掉常見得多，
    因此短暫失敗先重試一次再放棄。

    Args:
        url: 目標網址
        params: 查詢參數
        timeout: 單次請求逾時秒數

    Returns:
        Any: 解析後的 JSON 內容

    Raises:
        Exception: 重試後仍失敗時，拋出最後一次的例外
    """
    # ★ 2026-08-26（W3）：重試次數改成可調，重試次數由呼叫端決定。
    last_error: Optional[Exception] = None
    for attempt in range(retries):
        try:
            response = requests.get(url, params=params, headers=_BROWSER_HEADERS, timeout=timeout)
            response.raise_for_status()
            return response.json()
        except Exception as exc:
            last_error = exc
            # ★ 最後一次嘗試就不要再睡 —— retries=1 時原本會白等 0.5 秒
            if attempt < retries - 1:
                _get_logger().warning(f"請求失敗將重試一次 ({url}): {exc}")
                time.sleep(0.5)

    raise last_error if last_error else RuntimeError("請求失敗")


def _to_float(value: Any) -> Optional[float]:
    """將 API 回傳的字串安全轉為浮點數

    Args:
        value: 原始值，可能是 "-" 或空字串

    Returns:
        Optional[float]: 轉換結果，無法轉換時為 None
    """
    try:
        number = float(str(value).strip())
    except (TypeError, ValueError):
        return None
    return number


def _load_usd_rates() -> Optional[Dict[str, float]]:
    """載入以美元為基準的匯率表

    主來源失效時自動切換備援來源，兩者皆為免金鑰的公開服務。

    Returns:
        Optional[Dict[str, float]]: 幣別代碼對應 1 美元可兌換的金額，失敗時為 None
    """
    cached = _cache.get("fx_rates", ttl=300.0)
    if cached:
        return cached

    rates: Dict[str, float] = {}
    try:
        for pair, item in _http_get_json(_RTER_FX_API).items():
            if not pair.startswith("USD") or len(pair) != 6:
                continue
            rate = _to_float(item.get("Exrate"))
            if rate:
                rates[pair[3:]] = rate
    except Exception as exc:
        _get_logger().warning(f"主要匯率來源失敗，改用備援來源: {exc}")

    if not rates:
        try:
            payload = _http_get_json(_FALLBACK_FX_API)
            rates = {code: float(value) for code, value in (payload.get("rates") or {}).items()}
        except Exception as exc:
            _get_logger().error(f"備援匯率來源也失敗: {exc}")
            return None

    if rates:
        _cache.set("fx_rates", rates)
    return rates or None


def query_exchange_rate(currency: str, amount: float = 1.0) -> str:
    """查詢外幣對新台幣的即時參考匯率

    Args:
        currency: 幣別名稱或代碼，例如「美金」「日幣」「USD」
        amount: 要換算的外幣金額，預設為 1

    Returns:
        str: 依既有工具慣例組成的「執行結果」字串
    """
    key = (currency or "").strip().lower().replace("幣值", "").replace("匯率", "")
    code = _CURRENCY_ALIASES.get(key) or _CURRENCY_ALIASES.get(key.replace("元", "")) or key.upper()

    if code == "TWD":
        return "執行結果: 失敗, 詳細信息: 請指定要查詢的外幣幣別，例如美金、日幣或歐元"

    rates = _load_usd_rates()
    if not rates:
        return "執行結果: 失敗, 詳細信息: 匯率服務連線失敗，請稍後再試或改由櫃檯查詢牌告匯率"

    usd_twd = rates.get("TWD")
    usd_target = rates.get(code)
    if not usd_twd or not usd_target:
        return f"執行結果: 失敗, 詳細信息: 目前查不到「{currency}」的匯率，請確認幣別名稱"

    # 透過美元作為橋樑換算外幣對台幣
    twd_per_unit = usd_twd / usd_target
    name = _CURRENCY_NAMES.get(code, code)

    try:
        amount = float(amount)
    except (TypeError, ValueError):
        amount = 1.0
    if amount <= 0:
        amount = 1.0

    lines = [f"1 {name}({code}) = {twd_per_unit:.4f} 新台幣"]
    if abs(amount - 1.0) > 1e-9:
        lines.append(f"{amount:g} {name} = {amount * twd_per_unit:,.2f} 新台幣")
    lines.append(f"1000 新台幣 = {1000.0 / twd_per_unit:,.2f} {name}")

    return (
        f"執行結果: 成功, 詳細信息: {'；'.join(lines)}。"
        f"此為即時市場參考匯率，實際換匯以本行臨櫃牌告匯率為準"
    )


def query_datetime() -> str:
    """查詢目前的日期與時間

    語言模型沒有時鐘，任何與「今天」「現在」相關的回答都必須先呼叫本函式。

    Returns:
        str: 依既有工具慣例組成的「執行結果」字串
    """
    now = datetime.now()
    weekday = _WEEKDAYS[now.weekday()]
    return (
        f"執行結果: 成功, 詳細信息: 現在是西元 {now.year} 年 {now.month} 月 {now.day} 日"
        f"(民國 {now.year - 1911} 年)，星期{weekday}，{now.hour:02d}:{now.minute:02d}"
    )
