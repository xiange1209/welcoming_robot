# -*- coding: utf-8 -*-
"""2026-10-15：LLM 節點必須明確設定 num_ctx。

背景：筆電 Ollama 日誌 `vram-based default context total_vram="4.0 GiB" default_num_ctx=4096`，
而銀行模式固定開銷（系統提示詞＋13 個工具描述）約 3,800 tokens（T5 報告 §6.3），
不設 num_ctx 會被 Ollama 悄悄截斷輸入。

測什麼（AST，不需要 ROS／Ollama）
  ✓ 宣告了 num_ctx 參數，預設 ≥ 8192
  ✓ ChatOllama(...) 呼叫有帶 num_ctx=self.num_ctx
  ✓ num_predict 仍在（沒被誤刪）
不測：真的送進 Ollama 後是否生效（要在筆電看 `ollama ps` 的 CONTEXT 欄與伺服器日誌）。

用法：python maprun/tools_1015/test_num_ctx.py        （SMARTNAV_REPO 可指定 repo 根）
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
tree = ast.parse(NODE.read_text(encoding="utf-8"))

results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


def decl_default(name):
    """找 declare_parameter("name", <常數>)，回傳預設值（找不到 None）"""
    for n in ast.walk(tree):
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "declare_parameter"
                and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == name
                and len(n.args) > 1 and isinstance(n.args[1], ast.Constant)):
            return n.args[1].value
    return None


print("── 1. 參數宣告 ──")
d = decl_default("num_ctx")
check(isinstance(d, int) and d >= 8192, f"num_ctx 已宣告且預設 ≥ 8192（實際 {d}）")
check(decl_default("num_predict") == 256, f"num_predict 仍宣告為 256（實際 {decl_default('num_predict')}）")

print("── 2. ChatOllama 呼叫 ──")
calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "ChatOllama"]
check(len(calls) == 1, f"原始碼只有一處 ChatOllama(...) 建構（找到 {len(calls)}）")
if calls:
    kws = {k.arg: k.value for k in calls[0].keywords}
    v = kws.get("num_ctx")
    check(isinstance(v, ast.Attribute) and v.attr == "num_ctx" and isinstance(v.value, ast.Name) and v.value.id == "self",
          "ChatOllama(..., num_ctx=self.num_ctx)")
    check("num_predict" in kws, "ChatOllama 仍帶 num_predict")
    check("base_url" in kws and "model" in kws, "ChatOllama 仍帶 base_url、model")

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
