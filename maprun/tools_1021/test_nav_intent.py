# -*- coding: utf-8 -*-
"""2026-10-21：程式端帶位判斷（smartnav_llm/nav_intent.py）的行為測試。

背景：10/20 實車 15:40「帶我去門口」×5，qwen2.5:3b 零次呼叫導航（3 次空白、1 次「請跟我來」卻沒導航）。
題目盡量用 10/20 log 裡真的出現過的句子（含 ASR 疊字、錯字），地點用 10/20 那張圖的 6 個。

測什麼：帶路意圖、地點比對（含別名、疊字收斂、錯一字）、否定、模型「答應帶路」偵測、回音判斷。
不測：ROS 端真的送出導航（要在車上看 log「🧭 帶位直達」與導航節點的「開始導航到 X」）。

用法：python -X utf8 maprun/tools_1021/test_nav_intent.py   （SMARTNAV_REPO 可指定 repo 根）
"""
import importlib.util
import os
import sys
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

CAR = Path(os.environ.get("SMARTNAV_REPO") or Path(__file__).resolve().parents[2])
spec = importlib.util.spec_from_file_location("nav_intent", CAR / "src/smartnav_llm/smartnav_llm/nav_intent.py")
ni = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ni)

NAMES = ["門口", "起點", "櫃台", "廁所", "大廳", "貴賓室"]  # 10/20 地圖 map_1dda0e955545 的地點
results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


print("── 1. 帶路要求 → 地點 ──")
cases = [
    ("帶我去門口", "門口"),                       # 15:40 講了 5 次的那句
    ("帶我去櫃檯", "櫃台"),                       # 地圖寫「櫃台」、題庫寫「櫃檯」
    ("請問櫃檯在哪裡？", "櫃台"),                 # 題 ②，預期帶位
    ("貴賓室要怎麼去？", "貴賓室"),               # 題 ③
    ("請帶我去大廳。", "大廳"),                   # 題 ⑩
    ("帶帶我去乖乖貴貴賓賓去", "貴賓室"),         # 10/20 15:34 ASR 原文（疊字）
    ("我去貴賓市", "貴賓室"),                     # 10/20 15:36 ASR 原文（錯一字）
    ("你帶我去貴桂賓吃到大大去貴賓貴貴", "貴賓室"),  # 10/20 15:33 ASR 原文
    ("我想去起點", "起點"),
    ("廁所在哪", "廁所"),                         # 地圖上真的有「廁所」——與 FAQ「洗手間沒設定」衝突，明天要決定
]
for text, want in cases:
    got = ni.detect_nav_request(text, NAMES)
    check(got == want, f"「{text}」→ {got}（預期 {want}）")

print("── 2. 不該帶路 ──")
for text in ["請問洗手間在哪裡？", "請問 ATM 在哪裡？", "我不要去門口", "先不用帶我去貴賓室",
             "門口有人在等嗎", "我去年在櫃檯開戶", "帶我去", "請問銀行幾點營業？", "你好",
             "跟他女人左左裡左逼利誘URU"]:
    got = ni.detect_nav_request(text, NAMES)
    check(got is None, f"「{text}」→ {got}（預期 None）")

print("── 3. 正規化 ──")
check(ni.normalize("櫃台") == "櫃檯", "櫃台 → 櫃檯")
check(ni.normalize("貴貴賓賓") == "貴賓", "疊字收斂：貴貴賓賓 → 貴賓")
check(ni.normalize("VIP室") == "貴賓室", "VIP室 → 貴賓室")
check(ni.match_waypoint("帶我去後門口", ["門口", "後門口"]) == "後門口", "都完全包含時取較長的名稱")
check(ni.match_waypoint("帶我去大廳", []) is None, "沒有地點清單 → None")

print("── 4. 模型是不是在「答應帶路」 ──")
for text, want in [
    ("請跟我來，我會帶您去門口。", True),            # 10/20 15:41 實際回覆（沒呼叫導航）
    ("好的，我這就帶您過去。", True),
    ("抱歉，請跟我來。", True),
    ("機器人無法帶您去貴賓室。請確認貴賓室的位置並建立為一個地點，然後再試一次。", False),  # 10/20 15:40
    ("不好意思，導航系統暫時沒有啟動，我不能直接帶您去大廳。請以口頭指引貴賓前往貴賓室。", False),  # 10/20 15:17
    ("本行營業時間為週一到週五上午九點至下午三點半。", False),
    ("我帶您了解一下開戶需要的證件", False),
    ("", False),
]:
    check(ni.promises_navigation(text) == want, f"{want!s:5}「{text[:24]}」")

print("── 5. 回音（機器人自己的話被收回來） ──")
check(ni.is_robot_echo("開始導航到 廁所", ["開始導航到 廁所"]), "10/20 14:59「開始導航到 廁所」＝導航節點自己的播報")
check(ni.is_robot_echo("導航到廁所", ["開始導航到 廁所"]), "播報的一段也算")
check(not ni.is_robot_echo("帶我去門口", ["好的，請跟我來。", "開始導航到 廁所"]), "客人的新要求不算")
check(not ni.is_robot_echo("我要辦理業務", ["請問您要辦理什麼業務呢"]), "用了機器人的詞造新句子不算（bigram 0.6 < 0.8）")
check(not ni.is_robot_echo("你好", ["你好"]), "太短（<4 字）不判")
check("navigate_tool" in ni.NAV_TOOL_NAMES and "guide_to_vip_room_tool" in ni.NAV_TOOL_NAMES, "會讓車移動的兩個工具都在清單")

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
