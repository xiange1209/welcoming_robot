#!/usr/bin/env python3
"""離線探針：不啟 ROS，直接用節點原始碼裡的工具定義去問 ollama。

工具的 name / docstring / 參數簽章用 AST 從真正的原始碼抽出來（不是手抄，
所以不會跟節點漂移），函式主體換成 stub —— 工具「選哪個」只取決於
schema 與提示詞，跟主體無關。

用途：
  * 量 prompt token 數（驗證有沒有超過 num_ctx 被截斷）
  * 快速比較不同 system prompt / 工具集 / 參數，不用重啟 ROS 節點
"""

import ast
import json
import sys
from pathlib import Path
from typing import Any

from langchain_core.tools import StructuredTool
from langchain_ollama import ChatOllama
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder

SRC = Path("/home/user/welcoming_robot_ws/src/smartnav_ws/src/smartnav_llm/smartnav_llm")
CFG = Path("/home/user/welcoming_robot_ws/src/smartnav_ws/src/smartnav_llm/config")
OLLAMA = "http://192.168.137.1:11434"

_PY2JSON = {"str": str, "float": float, "int": int, "bool": bool}


def extract_tools(path: Path) -> list[dict]:
    """從原始碼抽出所有被 @tool 裝飾的函式的 name / doc / 參數"""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    out = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef):
            continue
        if not any(
            (isinstance(d, ast.Name) and d.id == "tool") for d in node.decorator_list
        ):
            continue
        doc = ast.get_docstring(node) or ""
        args = []
        defaults = node.args.defaults
        pad = len(node.args.args) - len(defaults)
        for i, a in enumerate(node.args.args):
            ann = a.annotation.id if isinstance(a.annotation, ast.Name) else "str"
            dflt = None
            has_default = i >= pad
            if has_default:
                d = defaults[i - pad]
                dflt = d.value if isinstance(d, ast.Constant) else None
            args.append({"name": a.arg, "type": ann, "default": dflt,
                         "required": not has_default})
        out.append({"name": node.name, "doc": " ".join(doc.split()), "args": args})
    return out


def build_tool(spec: dict) -> StructuredTool:
    from pydantic import create_model
    fields: dict[str, Any] = {}
    for a in spec["args"]:
        t = _PY2JSON.get(a["type"], str)
        fields[a["name"]] = (t, ... if a["required"] else a["default"])
    schema = create_model(spec["name"] + "Args", **fields) if fields else None

    def _stub(**kwargs):
        return "執行結果: 成功, 詳細信息: (stub)"

    if schema is None:
        return StructuredTool.from_function(
            func=lambda: "執行結果: 成功", name=spec["name"], description=spec["doc"])
    return StructuredTool.from_function(
        func=_stub, name=spec["name"], description=spec["doc"], args_schema=schema)


def all_tool_specs(enable_web=True, enable_rag=True, enable_bank=True) -> list[dict]:
    node_specs = extract_tools(SRC / "llm_service_node.py")
    bank_specs = extract_tools(SRC / "bank_tools.py")
    web_names = {"query_stock_price_tool", "query_exchange_rate_tool",
                 "query_weather_tool", "query_datetime_tool"}
    keep = []
    for s in node_specs:
        if s["name"] in web_names and not enable_web:
            continue
        if s["name"] == "search_bank_knowledge_tool" and not enable_rag:
            continue
        keep.append(s)
    if enable_bank:
        keep += bank_specs
    return keep


def make_chain(system_file="system_prompt.txt", temperature=0.0, model="qwen2.5:3b",
               num_ctx=None, **flags):
    specs = all_tool_specs(**flags)
    tools = [build_tool(s) for s in specs]
    kw = {}
    if num_ctx:
        kw["num_ctx"] = num_ctx
    llm = ChatOllama(base_url=OLLAMA, model=model, temperature=temperature, **kw)
    bound = llm.bind_tools(tools)
    tmpl = ChatPromptTemplate.from_messages([
        ("system", "{system_prompt}"),
        MessagesPlaceholder(variable_name="history"),
        ("human", "{input}"),
        MessagesPlaceholder(variable_name="agent_scratchpad"),
    ])
    if system_file:
        p = Path(system_file)
        if not p.is_absolute():
            p = CFG / system_file
        sp = p.read_text(encoding="utf-8")
    else:
        sp = ""
    return (tmpl | bound), sp, specs


def ask(chain, sp, text):
    r = chain.invoke({"system_prompt": sp, "history": [], "input": text,
                      "agent_scratchpad": []})
    return r


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "list"
    if mode == "list":
        for s in all_tool_specs():
            print(f"{s['name']:32s} args={[a['name'] for a in s['args']]}")
            print(f"    {s['doc'][:110]}")
    elif mode == "tokens":
        # 量 prompt token 數：拿 response 的 prompt_eval_count
        chain, sp, specs = make_chain()
        r = ask(chain, sp, "你好")
        gi = r.response_metadata
        print("工具數:", len(specs))
        print("prompt_eval_count:", gi.get("prompt_eval_count"))
        print("eval_count:", gi.get("eval_count"))
        print("content:", repr(r.content)[:300])
        print("tool_calls:", r.tool_calls)
    elif mode == "probe":
        sysfile = sys.argv[2] if len(sys.argv) > 2 else "system_prompt.txt"
        n = int(sys.argv[3]) if len(sys.argv) > 3 else 3
        temp = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
        qs = [("請問開戶要帶什麼證件", "query_bank_faq_tool"),
              ("你們幾點關門", "query_bank_faq_tool"),
              ("我要找真人服務", "notify_staff_tool")]
        chain, sp, specs = make_chain(system_file=sysfile, temperature=temp)
        print(f"# system={sysfile} tools={len(specs)} temp={temp}")
        for q, exp in qs:
            got = []
            for _ in range(n):
                r = ask(chain, sp, q)
                got.append(r.tool_calls[0]["name"] if r.tool_calls else
                           f"(無:{str(r.content)[:40]})")
            print(f"{q:14s} 期望 {exp:24s} -> {got}")
