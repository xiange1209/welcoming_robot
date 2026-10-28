"""這一輪要讓模型看到哪些工具（純函式，不碰 ROS）。

為什麼不全部給、靠提示詞叫它別亂用：llama3.1:8b 只要手上有工具就想用——
對「你好」去查地點並唸出地點清單、對「你們幾點關門」誤叫查時間的工具（2026-10-28 真模型測試，
提示詞已寫明禁止仍然發生）。提示詞是請求，不給工具才是保證。

規則（關鍵字放寬，寧可多給也不要漏掉真正的求助）：
  純問候／道謝／道別            一個工具都不給（直接回話）
  通報行員                      句子有求助意圖（真人、行員、專員、客訴、看不懂、被騙、不舒服…）才給。
                                ★ 不能每輪都給：它變成唯一的工具時，8B 對一般問題（幾點關門、開戶）
                                也會呼叫它，等於對行員的手機亂發通報（2026-10-28 真模型測試實測到）
  查地點、導航                  句子有去／到／帶／導航／在哪／怎麼走這類字才給
  查時間                        句子問「當下」（現在、今天、幾號、星期幾…）才給
  查匯率                        句子提到匯率、換匯或常見幣別才給
"""

import re
from typing import Iterable, List

NOTIFY = "notify_staff_tool"
NAV = ("list_waypoints_tool", "navigate_tool")
DATETIME = "query_datetime_tool"
EXCHANGE = "query_exchange_rate_tool"

# 整句只有問候或道謝（可帶「你／您／喔／啊」與標點）；有任何別的內容就不算
_SOCIAL = re.compile(
    r"^(你好|您好|哈囉|嗨|hello|hi|早安|早|午安|晚安|謝謝|謝啦|感謝|多謝|謝謝你|謝謝您|謝謝妳|"
    r"再見|拜拜|掰掰|辛苦了|沒事了|沒問題|好的|好|ok|okay|嗯|喔|哦|了解|知道了)+(你|您|妳|喔|哦|啊|啦|囉|了)?$",
    re.IGNORECASE,
)
# 「帶什麼／帶哪些證件」是問要帶的東西，不是帶路
_HELP = re.compile(
    r"(真人|行員|專員|人員|客服|經理|主管|人工|找人|有人|櫃檯小姐|轉接|聯絡|打電話|叫.{0,3}來|"
    r"投訴|客訴|申訴|抱怨|不滿|幫不了|幫不上|沒辦法幫|不會用|不會操作|看不懂|不懂|聽不懂|"
    r"幫忙|協助|幫我處理|急|報警|警察|詐騙|被騙|可疑|救命|不舒服|受傷|生病|暈|跌倒|需要幫助)"
)
_NAV = re.compile(r"(去|到|帶(?!什麼|甚麼|哪些|哪個|著|來)|走|導航|在哪|哪裡|哪邊|怎麼|位置|地點|路線)")
_NOW = re.compile(r"(現在|目前|此刻|今天|今日|今晚|明天|昨天|幾號|星期|禮拜|週幾|日期)")
_FX = re.compile(r"(匯率|換匯|換錢|兌換|美金|美元|日幣|日圓|歐元|人民幣|港幣|英鎊|澳幣|韓元|韓幣|新幣|泰銖|外幣)")
_PUNCT = re.compile(r"[\s，。,.!！?？、~～…]+")


def is_social(text: str) -> bool:
    """整句只是問候／道謝／道別／簡短回應"""
    return bool(_SOCIAL.match(_PUNCT.sub("", text or "")))


def allowed_tools(text: str, available: Iterable[str]) -> List[str]:
    """依客人這句話，回傳這一輪可以給模型的工具名稱（保持 available 的順序）"""
    available = list(available)
    if is_social(text):
        return []
    s = text or ""
    allowed = set()
    if _HELP.search(s):
        allowed.add(NOTIFY)
    if _NAV.search(s):
        allowed.update(NAV)
    if _NOW.search(s):
        allowed.add(DATETIME)
    if _FX.search(s):
        allowed.add(EXCHANGE)
    return [name for name in available if name in allowed]
