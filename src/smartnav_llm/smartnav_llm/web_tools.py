#!/usr/bin/env python3

"""即時資訊查詢模組

提供 LLM 查詢會隨時間變動、無法寫死在知識庫裡的資料，全部使用免金鑰的公開來源：

* **台股報價**：臺灣證券交易所 (上市) 與證券櫃檯買賣中心 (上櫃)
* **匯率**：即時市場參考匯率
* **天氣**：Open-Meteo
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
_TWSE_QUOTE_API = "https://mis.twse.com.tw/stock/api/getStockInfo.jsp"
_TWSE_LIST_API = "https://openapi.twse.com.tw/v1/exchangeReport/STOCK_DAY_ALL"
_TPEX_LIST_API = "https://www.tpex.org.tw/openapi/v1/tpex_mainboard_quotes"
_RTER_FX_API = "https://tw.rter.info/capi.php"
_FALLBACK_FX_API = "https://open.er-api.com/v6/latest/USD"
_WEATHER_API = "https://api.open-meteo.com/v1/forecast"
_GEOCODING_API = "https://geocoding-api.open-meteo.com/v1/search"

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

# 台灣主要城市座標，直接命中可省去一次地理編碼查詢
_CITY_COORDINATES: Dict[str, Tuple[float, float]] = {
    "台北": (25.0330, 121.5654), "臺北": (25.0330, 121.5654),
    "新北": (25.0169, 121.4628), "基隆": (25.1276, 121.7392),
    "桃園": (24.9936, 121.3010), "新竹": (24.8138, 120.9675), "苗栗": (24.5602, 120.8214),
    "台中": (24.1477, 120.6736), "臺中": (24.1477, 120.6736),
    "彰化": (24.0518, 120.5161), "南投": (23.9609, 120.9719), "雲林": (23.7092, 120.4313),
    "嘉義": (23.4801, 120.4491),
    "台南": (22.9999, 120.2270), "臺南": (22.9999, 120.2270),
    "高雄": (22.6273, 120.3014), "屏東": (22.5519, 120.5487),
    "宜蘭": (24.7021, 121.7378), "花蓮": (23.9871, 121.6015),
    "台東": (22.7583, 121.1444), "臺東": (22.7583, 121.1444),
    "澎湖": (23.5711, 119.5793), "金門": (24.4321, 118.3171), "馬祖": (26.1608, 119.9494),
}

# WMO 天氣代碼對照中文描述
_WEATHER_CODES: Dict[int, str] = {
    0: "晴天", 1: "大致晴朗", 2: "局部多雲", 3: "陰天",
    45: "有霧", 48: "凍霧",
    51: "毛毛雨", 53: "毛毛雨", 55: "較大毛毛雨",
    56: "凍毛毛雨", 57: "強凍毛毛雨",
    61: "小雨", 63: "中雨", 65: "大雨",
    66: "凍雨", 67: "強凍雨",
    71: "小雪", 73: "中雪", 75: "大雪", 77: "冰珠",
    80: "短暫陣雨", 81: "陣雨", 82: "強陣雨",
    85: "陣雪", 86: "大陣雪",
    95: "雷雨", 96: "雷雨伴隨冰雹", 99: "強雷雨伴隨冰雹",
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
    # ★ 2026-08-26（W3）：重試次數改成可調，見 _load_stock_directory 的說明。
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


def _load_stock_directory() -> Dict[str, Tuple[str, str, str]]:
    """載入上市與上櫃股票的名稱對照表

    對照表一天內不會變動，因此快取 12 小時。

    Returns:
        Dict[str, Tuple[str, str, str]]: 名稱或代號對應 (代號, 名稱, 市場別)
    """
    cached = _cache.get("stock_directory", ttl=43200.0)
    if cached:
        return cached

    directory: Dict[str, Tuple[str, str, str]] = {}
    sources = [(_TWSE_LIST_API, "tse", "Code", "Name"), (_TPEX_LIST_API, "otc", "SecuritiesCompanyCode", "CompanyName")]

    for url, market, code_key, name_key in sources:
            # ★★ 2026-08-26（W3）：20 秒 x 2 次重試 x 2 個來源 = 最壞 81 秒 ★★
            #
            # 這 81 秒內 llm_service_node 的 is_agent_running 一直是 True，
            # 客人講任何話都只會得到「系統目前正在處理上一個指令」。
            # 而這張表只是「名稱 -> 代號」的對照，載不進來時數字代號路徑仍可用，
            # 不值得為它讓整個對話停擺。改成 6 秒、不重試 -> 最壞 12 秒。
        try:
            for item in _http_get_json(url, timeout=6.0, retries=1):
                code = str(item.get(code_key, "")).strip()
                name = str(item.get(name_key, "")).strip()
                if not code or not name:
                    continue
                entry = (code, name, market)
                directory.setdefault(code, entry)
                directory.setdefault(name, entry)
        except Exception as exc:
            _get_logger().warning(f"股票對照表載入失敗 ({market}): {exc}")

    if directory:
        _cache.set("stock_directory", directory)
    return directory


def _resolve_stock(query: str) -> Optional[Tuple[str, str, str]]:
    """將使用者說法解析成股票代號

    支援直接輸入代號、輸入完整公司名稱，或輸入部分名稱模糊比對。

    Args:
        query: 使用者輸入的股票名稱或代號

    Returns:
        Optional[Tuple[str, str, str]]: (代號, 名稱, 市場別)，找不到時為 None
    """
    query = (query or "").strip().replace("股價", "").replace("股票", "").replace("的", "")
    if not query:
        return None

    directory = _load_stock_directory()

    if query in directory:
        return directory[query]

    # 純數字代號但不在對照表 (可能是新上市或 ETF)，直接讓報價 API 判斷市場別
    if query.isdigit():
        return (query, query, "")

    # 模糊比對：優先取名稱開頭相符者
    candidates = [entry for name, entry in directory.items() if not name.isdigit() and query in name]
    if not candidates:
        return None
    candidates.sort(key=lambda entry: (not entry[1].startswith(query), len(entry[1])))
    return candidates[0]


def query_stock_price(stock: str) -> str:
    """查詢台股即時報價

    Args:
        stock: 股票名稱或代號，例如「台積電」或「2330」

    Returns:
        str: 依既有工具慣例組成的「執行結果」字串
    """
    # ★ 2026-08-26（W4）：把「對照表連不上」與「真的查無此股」分開回報。
    #   兩者原本共用「查不到這檔股票」這句話 —— 網路問題會被說成
    #   「機器人不認識台積電」，客人與現場人員都會往錯的方向排查。
    #   純數字代號不需要對照表（_resolve_stock 會直接走代號路徑），所以只擋名稱查詢。
    if not (stock or "").strip().isdigit() and not _load_stock_directory():
        return ("執行結果: 失敗, 詳細信息: 股票名稱查詢服務目前連線失敗，"
                "請稍後再試，或直接提供股票代號")
    resolved = _resolve_stock(stock)
    if not resolved:
        return f"執行結果: 失敗, 詳細信息: 查不到「{stock}」這檔股票，請確認名稱或提供股票代號"

    code, name, market = resolved
    cache_key = f"stock:{code}"
    cached = _cache.get(cache_key, ttl=60.0)
    if cached:
        return cached

    # 市場別未知時，上市與上櫃都查一次
    markets = [market] if market else ["tse", "otc"]
    channels = "|".join(f"{m}_{code}.tw" for m in markets)

    try:
        payload = _http_get_json(_TWSE_QUOTE_API, params={"ex_ch": channels, "json": "1", "delay": "0"})
    except Exception as exc:
        return f"執行結果: 失敗, 詳細信息: 證交所報價服務連線失敗 ({exc})，請稍後再試"

    entries = payload.get("msgArray") or []
    if not entries:
        return f"執行結果: 失敗, 詳細信息: 證交所目前查不到 {name}({code}) 的報價資料"

    info = entries[0]
    name = info.get("n") or name
    previous_close = _to_float(info.get("y"))
    price = _to_float(info.get("z"))

    # 盤中尚未成交時 z 會是 "-"，改用最佳買價，仍取不到則以昨收表示
    if price is None:
        best_bid = str(info.get("b", "")).split("_")[0]
        price = _to_float(best_bid) or previous_close

    if price is None:
        return f"執行結果: 失敗, 詳細信息: {name}({code}) 目前尚無成交價可供參考"

    parts = [f"{name}({code}) 成交價 {price:.2f} 元"]
    if previous_close:
        change = price - previous_close
        percent = change / previous_close * 100.0
        parts.append(f"漲跌 {change:+.2f} ({percent:+.2f}%)，昨收 {previous_close:.2f}")

    for label, key in (("開盤", "o"), ("最高", "h"), ("最低", "l")):
        value = _to_float(info.get(key))
        if value is not None:
            parts.append(f"{label} {value:.2f}")

    volume = _to_float(info.get("v"))
    if volume is not None:
        parts.append(f"成交量 {volume:.0f} 張")

    stamp = f"{info.get('d', '')} {info.get('t', '')}".strip()
    result = (
        f"執行結果: 成功, 詳細信息: {'，'.join(parts)}。"
        f"資料時間 {stamp}，來源為臺灣證券交易所公開資訊，僅供參考不構成投資建議"
    )
    _cache.set(cache_key, result)
    return result


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


def _resolve_city(city: str) -> Optional[Tuple[float, float]]:
    """將城市名稱解析成經緯度

    台灣主要城市直接查內建座標表，其他地名才呼叫地理編碼服務。

    Args:
        city: 城市名稱

    Returns:
        Optional[Tuple[float, float]]: (緯度, 經度)，查不到時為 None
    """
    key = city.replace("市", "").replace("縣", "").strip()
    if key in _CITY_COORDINATES:
        return _CITY_COORDINATES[key]

    cache_key = f"geocode:{city}"
    cached = _cache.get(cache_key, ttl=86400.0)
    if cached:
        return cached

    try:
        results = _http_get_json(_GEOCODING_API, params={"name": city, "count": 1, "language": "zh"}).get(
            "results"
        ) or []
        if not results:
            return None
        location = (float(results[0]["latitude"]), float(results[0]["longitude"]))
    except Exception as exc:
        _get_logger().warning(f"地理編碼失敗 ({city}): {exc}")
        return None

    _cache.set(cache_key, location)
    return location


def query_weather(city: str = "台北") -> str:
    """查詢指定城市的目前天氣

    Args:
        city: 城市名稱，預設為台北

    Returns:
        str: 依既有工具慣例組成的「執行結果」字串
    """
    city = (city or "台北").strip()

    cache_key = f"weather:{city}"
    cached = _cache.get(cache_key, ttl=600.0)
    if cached:
        return cached

    location = _resolve_city(city)
    if not location:
        return f"執行結果: 失敗, 詳細信息: 查不到「{city}」這個地點，請確認地名"

    latitude, longitude = location
    try:
        payload = _http_get_json(
            _WEATHER_API,
            params={
                "latitude": latitude,
                "longitude": longitude,
                "current": "temperature_2m,relative_humidity_2m,apparent_temperature,weather_code",
                "daily": "temperature_2m_max,temperature_2m_min,precipitation_probability_max",
                "timezone": "Asia/Taipei",
                "forecast_days": 1,
            },
        )
    except Exception as exc:
        return f"執行結果: 失敗, 詳細信息: 天氣服務連線失敗 ({exc})，請稍後再試"

    current = payload.get("current") or {}
    if not current:
        return f"執行結果: 失敗, 詳細信息: 查不到「{city}」的天氣資料"

    description = _WEATHER_CODES.get(int(current.get("weather_code", -1)), "天氣狀況不明")
    parts = [f"{city}目前{description}"]

    for label, key, unit in (
        ("氣溫", "temperature_2m", "°C"),
        ("體感", "apparent_temperature", "°C"),
        ("濕度", "relative_humidity_2m", "%"),
    ):
        value = current.get(key)
        if value is not None:
            parts.append(f"{label} {value}{unit}")

    daily = payload.get("daily") or {}
    lows = daily.get("temperature_2m_min") or []
    highs = daily.get("temperature_2m_max") or []
    if lows and highs:
        parts.append(f"今日高低溫 {lows[0]}~{highs[0]}°C")

    rain = daily.get("precipitation_probability_max") or []
    if rain and rain[0] is not None:
        parts.append(f"降雨機率 {rain[0]}%")

    result = f"執行結果: 成功, 詳細信息: {'，'.join(parts)}"
    _cache.set(cache_key, result)
    return result


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
