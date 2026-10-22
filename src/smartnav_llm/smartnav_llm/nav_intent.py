"""帶位意圖、帶位承諾與回音的程式端判斷（★ 2026-10-21）

為什麼不交給模型：10/20 實車（llm_20261020_153753.log，15:40）客人說「帶我去門口」5 次，
qwen2.5:3b **一次都沒呼叫導航工具**——1 次照抄上一題的失敗訊息、3 次空白回應、
最後 1 次回「請跟我來，我會帶您去門口」卻沒發導航指令（車不會動）。
帶位是迎賓故事線的核心，不能交給 3B 模型擲骰子：句子裡有明確的帶路意圖、
又對得到地圖上**唯一**的地點名稱時，由程式直接導航；對不到才交給模型。

全部是純函式（不碰 ROS），測試：maprun/tools_1021/test_nav_intent.py。
"""

import re
from typing import Iterable, List, Optional, Sequence, Tuple

# 這兩個工具會讓車子實際移動。「答應帶路」之後若這一輪沒呼叫過它們，就是嘴上答應。
NAV_TOOL_NAMES = frozenset({"navigate_tool", "guide_to_vip_room_tool"})

# 帶路的說法（命令句）。「去」單獨出現太泛（「我去櫃檯辦」也算），所以只收帶主詞或動詞的組合。
_GO = re.compile(
    r"(帶我去|帶我到|帶我過去|請帶我|麻煩帶我|可以帶我|能帶我|幫我帶路|帶路|引導我|"
    r"我要去|我想去|我要到|我想到|我去(?![年過了])|導航到|導航去|前往)"
)  # 「我去年在櫃檯開戶」的「我去」不是帶路——排除「去年／去過／去了」
# 問位置。bank_faq.txt 寫「以上地點機器人都可以帶您過去」，題庫 ②③ 的預期也是帶位。
_WHERE = re.compile(r"(在哪|在那裡|在哪邊|怎麼去|怎麼走|往哪走)")
# 否定與取消：「不用帶我去」「先不要」——寧可交給模型，也不要誤開車。
_NEGATE = re.compile(r"(不要|不用|不必|別帶|先不|取消|停下|停止)")

# ASR 與使用者用字的常見變體（只用在比對，不改原文）
_ALIASES: Sequence[Tuple[str, str]] = (
    ("櫃台", "櫃檯"),
    ("貴賓廳", "貴賓室"),
    ("vip室", "貴賓室"),
    ("vip", "貴賓室"),
    # ★ 2026-10-21：同一個地方的不同說法。地圖上標「廁所」、客人問「洗手間」也要對得上，
    #   否則會一邊帶去廁所、一邊回答「沒有設定洗手間」。
    ("廁所", "洗手間"),
    ("化妝室", "洗手間"),
)

_STRIP = re.compile(r"[\s，。？！、,.?!：:；;「」『』（）()…~～\-—]")


def normalize(text: str) -> str:
    """比對用的正規化：去標點空白、轉小寫、套用別名、收斂疊字（「貴貴賓賓」→「貴賓」）。

    疊字收斂是因為 10/20 的 ASR 每對相鄰字約 11% 是重複（見 分析_實車20261020.md），
    地點名稱本身沒有疊字，收斂後才對得上。
    """
    s = _STRIP.sub("", text or "").lower()
    for a, b in _ALIASES:
        s = s.replace(a, b)
    return re.sub(r"(.)\1+", r"\1", s)


def _lcs(a: str, b: str) -> int:
    """最長共同子序列長度（字串都很短，O(nm) 足夠）"""
    if not a or not b:
        return 0
    prev = [0] * (len(b) + 1)
    for ca in a:
        cur = [0]
        for j, cb in enumerate(b):
            cur.append(prev[j] + 1 if ca == cb else max(prev[j + 1], cur[j]))
        prev = cur
    return prev[-1]


def _name_score(name_n: str, text_n: str) -> float:
    """地點名稱在句子裡的吻合度：完全包含 > 1；三字以上的名稱允許錯一個字（「貴賓市」）；其餘 0。"""
    if not name_n:
        return 0.0
    if name_n in text_n:
        return 1.0 + len(name_n) / 100.0  # 都完全包含時，長的優先（「貴賓室」勝過「貴賓」）
    if len(name_n) >= 3:
        w = len(name_n) + 1  # 限制在相近的一小段裡比，不讓頭尾散落的字湊出吻合
        best = max((_lcs(name_n, text_n[i:i + w]) for i in range(max(1, len(text_n) - w + 1))), default=0)
        if best >= len(name_n) - 1:
            return best / len(name_n)
    return 0.0


def has_nav_intent(text: str) -> bool:
    s = normalize(text)
    if not s or _NEGATE.search(s):
        return False
    return bool(_GO.search(s) or _WHERE.search(s))


def match_waypoint(text: str, names: Iterable[str]) -> Optional[str]:
    """回傳句子裡提到的地點（原始名稱）。對不到、或兩個地點同分，回 None（交給模型）。"""
    s = normalize(text)
    scored: List[Tuple[float, str]] = []
    for name in names:
        sc = _name_score(normalize(name), s)
        if sc > 0:
            scored.append((sc, name))
    if not scored:
        return None
    scored.sort(key=lambda t: -t[0])
    if len(scored) > 1 and abs(scored[0][0] - scored[1][0]) < 1e-9 and scored[0][1] != scored[1][1]:
        return None
    return scored[0][1]


def detect_nav_request(text: str, names: Iterable[str]) -> Optional[str]:
    """有帶路意圖、而且對得到唯一的地點 → 回傳地點名稱；否則 None。"""
    if not has_nav_intent(text):
        return None
    return match_waypoint(text, names)


# ★ 2026-10-21：bank_faq.txt 第 5 行列的「示範場地沒有設定」的地點。
#   筆電 Ollama 評估（backups/研究數據/LLM評估_20261021/）：FAQ 已併入提示，模型問到洗手間、ATM
#   仍會反問樓層或叫人「呼叫查詢地點工具」。這類問題答案是固定的，交給程式回答。
UNCONFIGURED_PLACES: Sequence[Tuple[str, str]] = (
    # (句子裡會出現的說法, 回答時用的名稱)
    ("洗手間", "洗手間"),
    ("廁所", "洗手間"),
    ("化妝室", "洗手間"),
    ("atm", "ATM"),
    ("提款機", "ATM"),
    ("取號機", "取號機"),
    ("號碼牌", "取號機"),
    ("電梯", "電梯"),
    ("停車場", "停車場"),
)


def unconfigured_place(text: str, names: Iterable[str]) -> Optional[str]:
    """問的是 FAQ 列為「示範場地沒有設定」的地點、而且地圖上也沒有這個地點 → 回傳要講的名稱。

    地圖上真的有（例如有人標了「廁所」）就回 None，交給帶位——地圖才是現場的真相。
    """
    if not has_nav_intent(text):
        return None
    s = normalize(text)
    on_map = {normalize(n) for n in names}
    for word, say in UNCONFIGURED_PLACES:
        w = normalize(word)
        if w in s and w not in on_map and normalize(say) not in on_map:
            return say
    return None


def mentions_place(text: str, name: str) -> bool:
    """句子有沒有提到這個地點（同一套正規化與錯一字容忍）。給「叫錯帶位工具」防呆用。"""
    return _name_score(normalize(name), normalize(text)) > 0


# 只收「把人帶到某處」的說法。單獨的「我帶您」不收：「我帶您了解一下開戶流程」是正常回答。
_PROMISE = re.compile(
    r"(請跟我來|跟我來|請隨我來|帶您去|帶你去|帶您到|帶你到|帶您過去|帶你過去|"
    r"為您帶路|幫您帶路|這就帶您|這就帶你|馬上帶您|馬上帶你|現在帶您過去|現在帶你過去)"
)
_REFUSE = re.compile(r"(無法|不能|沒辦法|沒有辦法|暫時沒有|目前沒有|不好意思|抱歉|失敗)")


def promises_navigation(reply: str) -> bool:
    """回覆是不是在「答應帶路」。「無法帶您去…」「不好意思…不能帶您去」這類拒絕句不算。"""
    s = re.sub(r"\s", "", reply or "")
    if not _PROMISE.search(s):
        return False
    if _REFUSE.search(s) and not re.search(r"(請跟我來|跟我來|請隨我來)", s):
        return False
    return True


# ★ 2026-10-22：帶位途中客人要車停下的說法。以前這時候說什麼都被回「系統目前正在處理上一個指令」丟掉，
#   客人喊「停下來」車照樣走（只剩平板急停）。誤判的代價是「車停下來」，所以可以寬一點；
#   但不收單獨的「等等」「不要」——ASR 疊字常把別的字聽成「等等」，「不要」也太常出現。
#   ★ 筆電端 10/22 併入時補：「停車場在哪裡」是常見問題（FAQ 也有），不能因為客人問它就把帶位中的車停下來，
#     所以「停車」後面不接「場」。
_STOP = re.compile(
    r"(停下|停止|停車(?!場)|先停|停一下|快停|停住|別走了|不要走了|別動|不要動|取消|"
    r"不要去了|不去了|不用去了|不用帶了|不用了|算了|stop)"
)


def is_stop_request(text: str) -> bool:
    """客人是不是要車子停下來（只在車子正在帶位時才拿來用）。"""
    return bool(_STOP.search(normalize(text)))


def _bigrams(s: str) -> set:
    return {s[i:i + 2] for i in range(len(s) - 1)}


def is_robot_echo(user_text: str, robot_said: Iterable[str], overlap: float = 0.8) -> bool:
    """這句「使用者輸入」是不是機器人自己剛講過的話（被麥克風收回來）。

    10/20 14:59 LLM 收到「開始導航到 廁所」——那正是導航節點自己的播報。
    ASR 端的回音過濾比對的是辨識文字，辨識一亂就漏；這裡在 LLM 入口再擋一次。
    規則同 ASR 端：整句是機器人某句話的一段，或相鄰兩字重疊比例 ≥ overlap。
    """
    got = normalize(user_text)
    if len(got) < 4:
        return False
    got_bg = _bigrams(got)
    for said in robot_said:
        ref = normalize(said)
        if not ref:
            continue
        if got in ref:
            return True
        if len(got_bg) >= 3 and len(got_bg & _bigrams(ref)) / len(got_bg) >= overlap:
            return True
    return False
