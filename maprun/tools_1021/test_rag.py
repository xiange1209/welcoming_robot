# -*- coding: utf-8 -*-
"""2026-10-28：知識庫檢索（knowledge_base.py）的行為測試。不需要 ROS／Ollama：embedder 用假的。

測什麼
  ✓ 切塊：一條一筆、章節標題併進每一筆、總標題不進索引
  ✓ 關鍵字備援：十個典型問題各命中對的片段；無關的話查不到
  ✓ context_for：查得到附片段、查不到明講「沒有資訊」、沒建好回空字串（由節點退回整份全文）
  ✓ Chroma 索引（有裝 chromadb 才測，否則跳過並說明）：重建、沿用、內容或模型改了會重建、
    查詢時 embedder 壞掉當場退到關鍵字、embedder 一開始就壞掉退到關鍵字且不丟例外
  ✓ 節點接線（原始碼斷言）：純問候不附知識庫、不是工具、追問帶上前一句

不測：真的 Ollama 與 bge-m3 的檢索品質（見 maprun/tools_1021/rag_eval.py，要連 Ollama）。

用法：python -X utf8 maprun/tools_1021/test_rag.py   （要測 Chroma：先把裝有 chromadb 的路徑放進 PYTHONPATH）
"""
import importlib.util
import os
import re
import sys
import tempfile
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

CAR = Path(os.environ.get("SMARTNAV_REPO") or Path(__file__).resolve().parents[2])
LLM = CAR / "src/smartnav_llm"
spec = importlib.util.spec_from_file_location("knowledge_base", LLM / "smartnav_llm/knowledge_base.py")
kb = importlib.util.module_from_spec(spec)
sys.modules["knowledge_base"] = kb
spec.loader.exec_module(kb)
FAQ = LLM / "config/bank_faq.txt"
results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


print("── 1. 切塊 ──")
text = FAQ.read_text(encoding="utf-8")
chunks = kb.split_chunks(text)
check(len(chunks) >= 10, f"bank_faq.txt 切成 {len(chunks)} 筆")
check(all(c.text.startswith("[") and "] " in c.text for c in chunks), "每一筆都帶章節標題 [章節] …")
check(not any("示範銀行常見問題" in c.text for c in chunks), "總標題（# 開頭）不進索引")
check(len({c.id for c in chunks}) == len(chunks), "id 不重複")
check(any(c.text.startswith("[營業時間]") and "三點半" in c.text for c in chunks), "營業時間那條在，且帶章節標題")
loose = kb.split_chunks("## 章節\n第一行\n第二行\n\n## 另一個\n- 條列甲\n- 條列乙\n")
check([c.text for c in loose] == ["[章節] 第一行 第二行", "[另一個] 條列甲", "[另一個] 條列乙"], f"沒有條列的章節整段一筆（{[c.text for c in loose]}）")
check(kb.split_chunks("") == [], "空檔 → 空")
check(kb.signature("a", "m1") != kb.signature("a", "m2") and kb.signature("a", "m1") != kb.signature("b", "m1")
      and kb.signature("a", "m1") == kb.signature("a", "m1"), "簽章：內容或模型不同就不同，相同就相同")

print("── 2. 關鍵字備援（沒有 embedding 時） ──")
KB = kb.KnowledgeBase(FAQ, None, "none", db_dir=None)
check(KB.load() == "keyword" and KB.ready, "embedder=None → 關鍵字模式、可用（不丟例外）")
CASES = [
    ("營業時間是幾點", "營業時間"), ("請問開戶要帶什麼", "開戶"),
    ("假日有沒有開", "營業時間"), ("想換美金", "外幣"), ("貸款要找誰", "貸款"), ("信用卡掉了怎麼辦", "信用卡"),
    ("貴賓室幾點開", "貴賓室"), ("有人詐騙我打電話", "可疑"),
]
for q, want in CASES:
    hits = KB.search(q)
    check(hits and any(want in c.text for c, _ in hits), f"「{q}」→ 命中含「{want}」（{[c.text[:12] for c, _ in hits][:2]}）")
check(KB.search("火星上有幾座火山") == [], "完全無關的話 → 查不到")
check(KB.search("") == [] and KB.search("   ") == [], "空字串 → 查不到")
ctx = KB.context_for("請問開戶要帶什麼")
check("開戶" in ctx and ctx.lstrip().startswith("---") and "本行知識庫" in ctx, "context_for：附片段，標題寫明是檢索出的片段")
check(KB.search("你們幾點關門") == [], "已知限制：關鍵字比不到同義詞（問「關門」、知識庫寫「三點半」）")
check(KB.context_for("你們幾點關門") == "", "關鍵字模式查不到 → 回空字串（節點改附整份 FAQ；召回率低，不能叫模型說沒有資訊）")
cold = kb.KnowledgeBase(FAQ, None, "none")
check(cold.context_for("開戶") == "" and not cold.ready, "還沒 load → 回空字串（節點會退回整份全文）")
missing = kb.KnowledgeBase(Path("/nonexistent/faq.txt"), None, "none")
check(missing.load() == "none" and not missing.ready and missing.context_for("x") == "", "檔案讀不到 → none、不丟例外")


def fake_embed(texts):
    """字元 bigram 雜湊到 128 維的假向量：內容相近的句子向量相近，足夠測流程"""
    out = []
    for t in texts:
        v = [0.0] * 128
        t = re.sub(r"\s+", "", t)
        for i in range(len(t) - 1):
            v[hash(t[i:i + 2]) % 128] += 1.0
        out.append(v)
    return out


print("── 3. Chroma 索引 ──")
try:
    import chromadb  # noqa: F401
    HAVE = True
except Exception as exc:  # noqa: BLE001
    HAVE = False
    print(f"  - 跳過：沒有 chromadb（{type(exc).__name__}）。把裝有 chromadb 的路徑放進 PYTHONPATH 才會測這一節")
if HAVE:
    calls = []

    def counting(texts):
        calls.append(len(texts))
        return fake_embed(texts)

    tmp = Path(tempfile.mkdtemp(prefix="rag_test_"))
    a = kb.KnowledgeBase(FAQ, counting, "fake-1", db_dir=tmp / "db", min_similarity=0.2)
    check(a.load() == "chroma", "第一次 load → chroma 模式")
    check(calls and calls[0] == len(chunks), f"第一次建索引：一次送 {calls[:1]} 筆（共 {len(chunks)} 筆）")
    check(any("營業時間" in c.text for c, _ in a.search("營業時間是幾點")), "向量檢索找得到營業時間")
    calls.clear()
    b = kb.KnowledgeBase(FAQ, counting, "fake-1", db_dir=tmp / "db", min_similarity=0.2)
    b.load()
    check(b.mode == "chroma" and all(n == 1 for n in calls), f"內容沒變 → 沿用索引，不重算整份（呼叫 {calls}）")
    calls.clear()
    c = kb.KnowledgeBase(FAQ, counting, "fake-2", db_dir=tmp / "db", min_similarity=0.2)
    c.load()
    check(calls and calls[0] == len(chunks), "embedding 模型換了 → 重建索引")
    calls.clear()
    f2 = tmp / "faq2.txt"
    f2.write_text(text + "\n- 新增的一條：新資料。\n", encoding="utf-8")
    d = kb.KnowledgeBase(f2, counting, "fake-2", db_dir=tmp / "db", min_similarity=0.2)
    d.load()
    check(calls and calls[0] == len(chunks) + 1, "知識檔內容改了 → 重建索引")

    broken = {"on": False}

    def flaky(texts):
        if broken["on"]:
            raise ConnectionError("Ollama 連不上")
        return fake_embed(texts)

    e = kb.KnowledgeBase(FAQ, flaky, "fake-3", db_dir=tmp / "db2", min_similarity=0.2)
    e.load()
    broken["on"] = True
    hits = e.search("請問開戶要帶什麼")
    check(hits and any("開戶" in c.text for c, _ in hits), "查詢時 Ollama 連不上 → 當場退到關鍵字，仍查得到、不丟例外")

    def dead(texts):
        raise ConnectionError("Ollama 連不上")

    g = kb.KnowledgeBase(FAQ, dead, "fake-4", db_dir=tmp / "db3")
    check(g.load() == "keyword" and g.ready, "建索引時 Ollama 就連不上 → 關鍵字模式、不丟例外")
    check(g.search("請問開戶要帶什麼"), "關鍵字模式仍可檢索")

print("── 4. 節點接線（原始碼斷言） ──")
node = (LLM / "smartnav_llm/llm_service_node.py").read_text(encoding="utf-8")
tools = (LLM / "smartnav_llm/tools.py").read_text(encoding="utf-8")
check("def _knowledge_section" in node and "tool_gate.is_social(user_input)" in node.split("def _knowledge_section")[1][:900],
      "純問候不附知識庫")
check("search" not in tools and "knowledge" not in tools.lower(), "RAG 不是工具（tools.py 沒有檢索）")
check("kb.context_for(query)" in node and "len(user_input) < 6" in node, "短追問帶上前一句再檢索")
check("threading.Thread(target=self.knowledge.load" in node, "建索引在背景執行緒（不擋節點啟動）")
check("knowledge_section = self._knowledge_section" in node and "+ knowledge_section +" in node, "知識庫一節每輪附在系統提示")
check(all(f'"{p}"' in node for p in ("enable_rag", "embed_model", "rag_top_k", "rag_min_similarity", "knowledge_db_dir")),
      "參數都宣告了：enable_rag、embed_model、rag_top_k、rag_min_similarity、knowledge_db_dir")
req = (LLM / "requirements.txt").read_text(encoding="utf-8")
check("chromadb" in req, "requirements.txt 有 chromadb")

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
