# -*- coding: utf-8 -*-
"""2026-10-21：LLM 節點今晚的接線（AST 與文字檢查，不需要 ROS／Ollama）。

背景（10/20 實車，分析_實車20261020.md）：銀行十題 對 2／部分 1／錯 7、工具呼叫 5/10；
「帶我去門口」×5 零次導航；閒置 >5 分的第一題 5~8 秒（Ollama keep_alive 5 分鐘，筆電 log 證實重載 9 次）；
雜訊觸發股價工具；機器人自己的播報被當成使用者輸入。

測什麼
  ✓ 新參數與預設：keep_alive "-1"、warmup_on_start、inline_bank_faq、bank_hide_map_tools、direct_nav、nav_guard、robot_echo_sec
  ✓ 股價／天氣在銀行模式預設關（預設值不是常數 True）
  ✓ ChatOllama 只在 _make_chat_llm 建構一次，仍帶 num_ctx=self.num_ctx、num_predict，並展開 _ollama_extra_kwargs()
  ✓ _run_agent_loop：迴圈前先試程式端帶位；空白／答應帶路的防呆在 flush_held 之前；工具迴圈會記錄 nav_called
  ✓ /llm_response 只從 _publish_reply 發（才會記進「機器人講過的話」給回音判斷用）
  ✓ 系統提示：改成「依附在最後的知識庫回答」，並寫明「帶路一定要真的呼叫工具」
不測：真的接上 Ollama 的效果（見 backups/研究數據/LLM評估_20261021/）與實車行為。

用法：python -X utf8 maprun/tools_1021/test_llm_wiring.py
"""
import ast
import os
import sys
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

CAR = Path(os.environ.get("SMARTNAV_REPO") or Path(__file__).resolve().parents[2])
NODE = CAR / "src/smartnav_llm/smartnav_llm/llm_service_node.py"
PROMPT = CAR / "src/smartnav_llm/config/system_prompt_bank.txt"
SRC = NODE.read_text(encoding="utf-8")
tree = ast.parse(SRC)
results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


def decl(name):
    """declare_parameter("name", <預設>) 的預設 AST 節點（找不到 None）"""
    for n in ast.walk(tree):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "declare_parameter"
                and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == name and len(n.args) > 1):
            return n.args[1]
    return None


def const(name):
    d = decl(name)
    return d.value if isinstance(d, ast.Constant) else None


def func(name):
    return next((n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name), None)


print("── 1. 參數 ──")
check(const("keep_alive") == "-1", f"keep_alive 預設 \"-1\"（實際 {const('keep_alive')!r}）")
for p in ("warmup_on_start", "inline_bank_faq", "bank_hide_map_tools", "direct_nav", "nav_guard"):
    check(const(p) is True, f"{p} 預設 True")
check(const("robot_echo_sec") == 20.0, "robot_echo_sec 預設 20 秒")
sw = decl("enable_stock_weather_tools")
check(sw is not None and not (isinstance(sw, ast.Constant) and sw.value is True),
      "enable_stock_weather_tools 不再寫死 True（銀行模式預設關）")
check(const("num_ctx") == 8192 and const("num_predict") == 256, "num_ctx 8192、num_predict 256 沒被動到")

print("── 2. ChatOllama ──")
calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "ChatOllama"]
check(len(calls) == 1, f"只有一處 ChatOllama(...)（找到 {len(calls)}）")
mk = func("_make_chat_llm")
check(mk is not None and any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "ChatOllama"
                             for n in ast.walk(mk)), "那一處在 _make_chat_llm 裡（主迴圈與暖機共用）")
if calls:
    kws = {k.arg: k.value for k in calls[0].keywords}
    check(isinstance(kws.get("num_ctx"), ast.Attribute) and kws["num_ctx"].attr == "num_ctx", "仍帶 num_ctx=self.num_ctx")
    check("num_predict" in kws and "base_url" in kws and "model" in kws, "仍帶 num_predict、base_url、model")
    star = [k.value for k in calls[0].keywords if k.arg is None]
    check(any(isinstance(v, ast.Call) and isinstance(v.func, ast.Attribute) and v.func.attr == "_ollama_extra_kwargs"
              for v in star), "展開 **self._ollama_extra_kwargs()（keep_alive 只在支援時才帶）")
check(func("_warmup_llm") is not None and "threading.Thread(target=self._warmup_llm" in SRC, "啟動時在背景執行緒暖機")

print("── 3. 主迴圈 ──")
loop = func("_run_agent_loop")
lsrc = ast.get_source_segment(SRC, loop) if loop else ""
i_direct = lsrc.find("self._try_direct_navigation(user_input)")
i_for = lsrc.find("for iteration in range(max_iterations)")
check(0 <= i_direct < i_for, "迴圈開始前先試程式端帶位")
i_empty = lsrc.find("if not final_reply.strip()")
i_prom = lsrc.find("nav_intent.promises_navigation(final_reply)")
i_flush = lsrc.find("self.stream_handler.flush_held()")
check(0 <= i_empty < i_flush and 0 <= i_prom < i_flush, "空白與「答應帶路」防呆都在 flush_held 之前（換掉的句子不會先被唸）")
check("force_intent=True" in lsrc, "答應帶路卻沒導航 → 改由程式依地點直接帶（force_intent）")
check("nav_intent.NAV_TOOL_NAMES" in lsrc and "nav_called = True" in lsrc, "工具迴圈記錄這一輪有沒有呼叫導航")

print("── 4. /llm_response 的出口 ──")
pub = func("_publish_reply")
outside = [n for n in ast.walk(tree)
           if isinstance(n, ast.Attribute) and n.attr == "publish" and isinstance(n.value, ast.Attribute)
           and n.value.attr == "llm_response_pub"
           and not (pub and pub.lineno <= n.lineno <= pub.end_lineno)]
check(pub is not None and not outside, f"只從 _publish_reply 發（其他地方直接 publish：{len(outside)} 處）")
cb = func("user_text_callback")
check(cb is not None and "_is_robot_echo(user_input)" in ast.get_source_segment(SRC, cb), "使用者輸入先擋回音")
check('create_subscription(\n            String, "speech_text", self._robot_speech_cb' in SRC, "訂閱 speech_text 記下機器人講過的話")

print("── 5. 系統提示（銀行版） ──")
P = PROMPT.read_text(encoding="utf-8")
check("本行知識庫的全文附在這份提示的最後" in P, "改成依附在最後的知識庫回答")
check("你**必須先呼叫「查詢本行業務規定」工具**" not in P, "不再無條件要求先呼叫 FAQ 工具（工具已拿掉）")
check("帶路一定要真的呼叫工具" in P, "寫明帶路一定要真的呼叫工具")
check("協助管理地圖資料" not in P, "角色不再包含管理地圖（建圖等工具在銀行模式已拿掉）")
check("# 本行知識庫（全文；回答本行業務問題只能依據這一節）" in SRC, "_load_system_prompt 附上的節名與提示文字一致")

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
