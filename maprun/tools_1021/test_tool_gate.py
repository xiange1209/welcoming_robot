# -*- coding: utf-8 -*-
"""2026-10-28：tool_gate（每一輪只給模型這句話可能用到的工具）的行為測試。純函式，不需要 ROS／Ollama。

背景：llama3.1:8b 對「你好」去查地點並唸出清單、對「你們幾點關門」誤叫查時間的工具（提示詞已禁止仍發生）。
原則是「寧可多給也不要少給」：找真人（通報行員）是安全相關，除了純問候之外每一輪都給。

用法：python -X utf8 maprun/tools_1021/test_tool_gate.py
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
spec = importlib.util.spec_from_file_location("tool_gate", CAR / "src/smartnav_llm/smartnav_llm/tool_gate.py")
tg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tg)

ALL = ["list_waypoints_tool", "navigate_tool", "notify_staff_tool", "query_exchange_rate_tool", "query_datetime_tool"]
results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


def names(text):
    return tg.allowed_tools(text, ALL)


print("── 1. 純問候／道謝：一個工具都不給 ──")
for t in ["你好", "您好！", "早安", "謝謝", "謝謝你", "謝謝您。", "再見", "好的，謝謝", "嗨", "Hello", "辛苦了", "了解"]:
    check(names(t) == [], f"「{t}」→ 無工具")

print("── 2. 問候加上別的內容：不算純問候 ──")
check("navigate_tool" in names("你好，帶我去大廳"), "「你好，帶我去大廳」仍給導航工具")
check("notify_staff_tool" in names("謝謝，我要找真人"), "「謝謝，我要找真人」仍給通報工具")

print("── 3. 通報行員：有求助意圖才給（一般問題不給，否則 8B 會亂通報行員） ──")
for t in ["我要找真人服務", "我想跟專員談貸款", "你幫不了我", "叫行員來", "我要客訴", "有人可以幫我嗎", "我看不懂這個表格",
          "我被詐騙了", "我不舒服", "請幫我找經理", "我想轉接客服", "這裡有人嗎"]:
    check("notify_staff_tool" in names(t), f"求助「{t}」→ 有通報工具")
for t in ["請問你們幾點關門", "幾點下班", "ATM 幾點關", "假日有沒有開", "請問開戶要帶什麼", "手續費多少", "美金匯率多少",
          "帶我去貴賓室", "現在幾點", "", "貴賓室在哪"]:
    check("notify_staff_tool" not in names(t), f"一般問題「{t}」→ 沒有通報工具")

print("── 4. 查時間：只有問當下才給 ──")
for t, want in [("現在幾點", True), ("今天幾號", True), ("今天星期幾", True), ("禮拜幾", True),
                ("你們幾點關門", False), ("幾點下班", False), ("ATM 幾點關", False), ("假日有沒有開", False), ("請問營業時間", False)]:
    check(("query_datetime_tool" in names(t)) == want, f"{'給' if want else '不給'}時間工具：「{t}」")

print("── 5. 查地點／導航：有去／到／帶路／在哪這類字才給 ──")
for t, want in [("帶我去貴賓室", True), ("貴賓室在哪", True), ("怎麼走到大廳", True), ("導航到櫃檯", True), ("我想去洗手間", True),
                ("請問開戶要帶什麼", False), ("要帶哪些證件", False), ("你好嗎", False), ("手續費多少", False), ("現在幾點", False)]:
    got = "navigate_tool" in names(t) and "list_waypoints_tool" in names(t)
    check(got == want, f"{'給' if want else '不給'}導航工具：「{t}」")
check(("navigate_tool" in names("x")) == ("list_waypoints_tool" in names("x")), "查地點與導航永遠一起給或一起不給")

print("── 6. 查匯率 ──")
for t, want in [("美金匯率多少", True), ("我想換日幣", True), ("外幣可以換嗎", True), ("開戶要帶什麼", False), ("你好嗎", False)]:
    check(("query_exchange_rate_tool" in names(t)) == want, f"{'給' if want else '不給'}匯率工具：「{t}」")

print("── 7. 順序與未知工具 ──")
check(tg.allowed_tools("帶我去大廳", ["navigate_tool", "list_waypoints_tool"]) == ["navigate_tool", "list_waypoints_tool"],
      "保持呼叫端給的順序，且只回傳 available 裡有的")
check(tg.allowed_tools("帶我去大廳", []) == [], "available 為空 → 空")

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
