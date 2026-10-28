# -*- coding: utf-8 -*-
"""2026-10-28：量知識庫檢索的品質並建議 rag_min_similarity。要連 Ollama（已 pull bge-m3）與 chromadb。

  正向題：每題標出「第一名片段應該含的字」，量第一名命中率與相似度
  負向題：知識庫沒有的話題（火星、天氣、股價…），量最高相似度
  門檻要夾在 負向最高 與 正向最低 之間；夾不進去就代表 embedding 分不開，要調知識內容或換模型

用法：PYTHONPATH=<含 chromadb 的路徑> python3 maprun/tools_1021/rag_eval.py [ollama_url] [embed_model]
"""
import importlib.util
import os
import sys
import tempfile
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

CAR = Path(os.environ.get("SMARTNAV_REPO") or Path(__file__).resolve().parents[2])
URL = sys.argv[1] if len(sys.argv) > 1 else "http://192.168.11.156:11434"
MODEL = sys.argv[2] if len(sys.argv) > 2 else "bge-m3"
spec = importlib.util.spec_from_file_location("kbm", CAR / "src/smartnav_llm/smartnav_llm/knowledge_base.py")
kbm = importlib.util.module_from_spec(spec); sys.modules["kbm"] = kbm; spec.loader.exec_module(kbm)

POS = [
    ("請問你們幾點關門", "營業時間"), ("幾點開門", "營業時間"), ("假日有沒有開", "營業時間"), ("週六有營業嗎", "營業時間"),
    ("請問開戶要帶什麼", "開戶"), ("我要換美金", "外幣"), ("信用卡掉了怎麼辦", "信用卡"), ("貸款要找誰", "貸款"),
    ("貴賓室開到幾點", "貴賓室"), ("洗手間在哪", "洗手間"), ("ATM 在哪裡", "ATM"), ("有人打電話要我給驗證碼", "驗證碼"),
    ("我行動不便需要幫忙", "行動不便"), ("匯款手續費多少", "手續費"),
    ("開護要帶什麼", "開戶"), ("代款要找誰", "貸款"),   # 語音辨識的同音錯字
]
NEG = ["火星上有幾座火山", "今天天氣如何", "台積電股價多少", "你叫什麼名字", "講個笑話給我聽"]

with tempfile.TemporaryDirectory() as tmp:
    emb = kbm.OllamaEmbedder(URL, MODEL)
    kb = kbm.KnowledgeBase(CAR / "src/smartnav_llm/config/bank_faq.txt", emb, MODEL, db_dir=Path(tmp) / "db",
                           top_k=3, min_similarity=-1.0)
    print(f"模式 {kb.load()}，{len(kb.chunks)} 筆，embedding 模型 {MODEL}")
    if kb.mode != "chroma":
        sys.exit("向量模式沒起來（Ollama 連不上、或沒裝 chromadb），無法評估")
    top1_ok, pos_sims = 0, []
    print("\n── 正向題 ──")
    for q, want in POS:
        hits = kb.search(q)
        ok = bool(hits) and want in hits[0][0].text
        in3 = any(want in c.text for c, _ in hits)
        top1_ok += ok
        if ok:
            pos_sims.append(hits[0][1])
        print(f"  {'✓' if ok else ('~' if in3 else '✗')} {hits[0][1]:.3f}  「{q}」→ {hits[0][0].text[:34]}")
    print("\n── 負向題 ──")
    neg_sims = []
    for q in NEG:
        hits = kb.search(q)
        neg_sims.append(hits[0][1])
        print(f"    {hits[0][1]:.3f}  「{q}」→ {hits[0][0].text[:34]}")

print(f"\n第一名命中 {top1_ok}/{len(POS)}（~ = 沒在第一名但在前三、✗ = 前三都沒有）")
lo, hi = min(pos_sims), max(neg_sims)
print(f"正向（第一名命中者）最低 {lo:.3f}；負向最高 {hi:.3f}")
if lo > hi:
    print(f"→ 門檻可以夾在 ({hi:.3f}, {lo:.3f})，建議取中間 {((lo + hi) / 2):.2f}")
else:
    print(f"→ 分不開：負向最高 {hi:.3f} ≥ 正向最低 {lo:.3f}。任何門檻都會誤殺或誤放，需要調知識內容、查詢改寫，或換 embedding 模型")
