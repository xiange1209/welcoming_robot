#!/usr/bin/env python3
"""離線比較不同 system prompt 的工具選擇準確率。

用 stub 工具（不連 ROS），所以連車子的邊都碰不到，可以放心跑「帶我去貴賓室」。
工具的名稱/描述/簽章是用 AST 從真正的節點原始碼抽出來的。
"""
import sys, json
from collections import Counter
sys.path.insert(0, "/tmp/claude-1000/-home-user/7a056d7a-9908-4e53-993d-789a1a0fa964/scratchpad")
from offline_probe import make_chain, ask

SP = "/tmp/claude-1000/-home-user/7a056d7a-9908-4e53-993d-789a1a0fa964/scratchpad"

QS = [
    ("Q1_開戶證件", "請問開戶要帶什麼證件", "query_bank_faq_tool"),
    ("Q2_幾點關門", "你們幾點關門", "query_bank_faq_tool"),
    ("Q3_找真人", "我要找真人服務", "notify_staff_tool"),
    ("R1_現在幾點", "現在幾點", "query_datetime_tool"),
    ("R2_打招呼", "你好", None),
    ("R3_帶位", "帶我去貴賓室", "guide_to_vip_room_tool"),
    ("R4_匯率", "美金匯率多少", "query_exchange_rate_tool"),
    ("R5_營業時間", "你們營業時間是幾點到幾點", "query_bank_faq_tool"),
]

VARIANTS = {
    "A_原導航版": "system_prompt.txt",
    "B_新銀行版(有工具名)": "system_prompt_bank.txt",
    "C_新銀行版(無工具名)": f"{SP}/variant_C.txt",
    "D_導航版+補真人與營業時間": f"{SP}/variant_D.txt",
}

N = int(sys.argv[1]) if len(sys.argv) > 1 else 3
only = sys.argv[2:] if len(sys.argv) > 2 else None

out = {}
for label, path in VARIANTS.items():
    if only and label not in only:
        continue
    chain, sp, specs = make_chain(system_file=path, enable_rag=False)
    total = hit = 0
    rows = []
    prose = 0            # 「用講的」次數：沒有 tool_call 但內容裡出現工具名
    for qid, q, exp in QS:
        got = []
        for _ in range(N):
            r = ask(chain, sp, q)
            name = r.tool_calls[0]["name"] if r.tool_calls else None
            got.append(name)
            total += 1
            if name == exp:
                hit += 1
            if not r.tool_calls and "_tool" in str(r.content):
                prose += 1
        rows.append((qid, exp, Counter(map(str, got))))
    out[label] = {"hit": hit, "total": total, "prose": prose,
                  "rows": [(a, str(b), dict(c)) for a, b, c in rows]}
    print(f"\n===== {label}  (提示詞 {path.split('/')[-1]}, 工具 {len(specs)} 個) =====")
    for qid, exp, dist in rows:
        ok = dist.get(str(exp), 0)
        print(f"  {qid:14s} 期望 {str(exp):26s} {ok}/{N}  {dict(dist)}")
    print(f"  --> 合計 {hit}/{total}   用講的(prose) {prose} 次")
    sys.stdout.flush()

print("\n########## 總表 ##########")
for label, d in out.items():
    print(f"{label:26s} {d['hit']:2d}/{d['total']}  用講的 {d['prose']}")
json.dump(out, open(f"{SP}/compare_prompts.json", "w"), ensure_ascii=False, indent=1)
