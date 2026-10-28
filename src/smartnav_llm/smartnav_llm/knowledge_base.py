#!/usr/bin/env python3

"""本行知識庫的檢索（RAG）

流程：每一輪由程式先檢索，把最相關的幾筆片段附在系統提示最後；**不是工具**，模型不必決定要不要查
（10/20 實測：FAQ 當工具讓 3B 自己決定時，沒呼叫的題目全部答錯或答偏，見 llm_service_node 的歷史註解）。

三層，由好到壞，壞掉自動往下退，對話不會因此中斷：
  1. Chroma（本機持久化）＋ Ollama 的 embedding 模型：語意檢索
  2. 關鍵字（字元 bigram 重疊）：Ollama 連不上、或沒裝 chromadb
  3. 整份 FAQ 全文：知識庫還沒建好或完全查不到時（呼叫端處理）

不依賴 ROS，可單獨測試（測試用假的 embedder，見 maprun/tools_1021/test_rag.py）。
"""

import hashlib
import logging
import math
import re
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional, Sequence, Tuple

import requests

CHUNKER_VERSION = "1"  # 切塊規則改了就要加一，否則舊索引會被當成還有效
COLLECTION = "bank_faq"


@dataclass(frozen=True)
class Chunk:
    id: str
    text: str  # 含章節標題，例如「[營業時間] 週一到週五…」：標題要進 embedding，單獨一句「請洽櫃檯」才有上下文


def split_chunks(text: str) -> List[Chunk]:
    """依 `## 章節` 與 `- 條列` 切塊：一條一筆，章節標題併進每一筆。

    沒有條列的章節（整段文字）就整段當一筆。`# ` 開頭的總標題不進索引。
    """
    chunks: List[Chunk] = []
    title = ""
    loose: List[str] = []

    def flush_loose() -> None:
        body = " ".join(x for x in loose if x).strip()
        if body:
            chunks.append(Chunk(f"c{len(chunks):03d}", f"[{title}] {body}" if title else body))
        loose.clear()

    for raw in text.splitlines():
        line = raw.strip()
        if not line or (line.startswith("#") and not line.startswith("##")):
            continue
        if line.startswith("##"):
            flush_loose()
            title = line.lstrip("#").strip()
        elif line.startswith("- "):
            flush_loose()
            chunks.append(Chunk(f"c{len(chunks):03d}", f"[{title}] {line[2:].strip()}" if title else line[2:].strip()))
        else:
            loose.append(line)
    flush_loose()
    return chunks


def signature(text: str, embed_model: str) -> str:
    return hashlib.sha256(f"{CHUNKER_VERSION}|{embed_model}|{text}".encode("utf-8")).hexdigest()[:16]


class OllamaEmbedder:
    """Ollama 的 /api/embed。模型跑在遠端那台，這裡只送文字、收向量。"""

    def __init__(self, base_url: str, model: str, timeout: float = 30.0):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.timeout = timeout

    def __call__(self, texts: Sequence[str]) -> List[List[float]]:
        resp = requests.post(
            f"{self.base_url}/api/embed", json={"model": self.model, "input": list(texts)}, timeout=self.timeout
        )
        resp.raise_for_status()
        vectors = resp.json()["embeddings"]
        if len(vectors) != len(texts):
            raise RuntimeError(f"embedding 數量不符：送 {len(texts)} 筆、回 {len(vectors)} 筆")
        return vectors


def _bigrams(s: str) -> set:
    s = re.sub(r"[\s，。,.!！?？、：:；;（）()「」\[\]-]+", "", s)
    return {s[i:i + 2] for i in range(len(s) - 1)} if len(s) > 1 else set(s)


# 幾乎每句話都有、不代表主題的字（不然「請問…」會讓所有片段都沾到分數）
_FILLER = ("請問", "你們", "你好", "謝謝", "可以", "要不要", "有沒有", "是不是", "我想", "我要", "一下")


def keyword_scores(query: str, chunks: Sequence[Chunk]) -> List[float]:
    """字元 bigram 的罕見詞加權命中率。沒有 embedding 時的備援。

    只看「知識庫裡出現過」的 query bigram（問句本身的「什麼」「怎麼」不在知識庫就不算分母），
    出現在越少片段的 bigram 權重越高（「開戶」比「櫃檯」更能代表主題）。
    分數 = 命中片段的權重和 ÷ 這句話所有（在知識庫出現過的）bigram 的權重和。
    """
    q = query
    for f in _FILLER:
        q = q.replace(f, "")
    qb = _bigrams(q)
    chunk_bigrams = [_bigrams(c.text) for c in chunks]
    df: dict = {}
    for cb in chunk_bigrams:
        for b in cb:
            df[b] = df.get(b, 0) + 1
    n = max(len(chunks), 1)
    weight = {b: math.log(1 + n / df[b]) for b in qb if b in df}
    total = sum(weight.values())
    if total == 0:
        return [0.0] * len(chunks)
    return [sum(w for b, w in weight.items() if b in cb) / total for cb in chunk_bigrams]


class _NoEmbedder(Exception):
    pass


class KnowledgeBase:
    """知識庫：切塊、建索引、檢索。建索引與檢索都可能呼叫遠端 Ollama，不要在 ROS callback 裡直接呼叫。"""

    def __init__(
        self,
        knowledge_file: Path,
        embedder: Optional[Callable[[Sequence[str]], List[List[float]]]],
        embed_model: str,
        db_dir: Optional[Path] = None,
        top_k: int = 3,
        min_similarity: float = 0.45,
        min_keyword: float = 0.5,
        logger: Optional[logging.Logger] = None,
    ):
        self.knowledge_file = Path(knowledge_file)
        self.embedder = embedder
        self.embed_model = embed_model
        self.db_dir = Path(db_dir) if db_dir else None
        self.top_k = top_k
        self.min_similarity = min_similarity
        self.min_keyword = min_keyword
        self.log = logger or logging.getLogger(__name__)
        self.full_text = ""
        self.chunks: List[Chunk] = []
        self.mode = "none"  # none | chroma | keyword
        self._collection = None
        self._ready = threading.Event()

    # ── 建索引 ────────────────────────────────────────────
    def load(self) -> str:
        """讀檔、切塊、建索引。回傳實際使用的模式。失敗時退到關鍵字，檔案讀不到才是 none。"""
        try:
            self.full_text = self.knowledge_file.read_text(encoding="utf-8").strip()
        except OSError as exc:
            self.log.warning(f"讀不到知識庫 {self.knowledge_file}: {exc}")
            self._ready.set()
            return self.mode
        self.chunks = split_chunks(self.full_text)
        self.mode = "keyword"
        try:
            if self.embedder is None:
                raise _NoEmbedder()
            self._build_chroma()
            self.mode = "chroma"
            self._warm_embedder()
        except _NoEmbedder:
            pass  # 呼叫端明確關掉向量檢索（enable_rag:=false），不是錯誤
        except Exception as exc:  # noqa: BLE001 — 任何原因都退到關鍵字，對話不能因為知識庫壞掉而中斷
            self.log.warning(f"向量索引建不起來，改用關鍵字檢索（{type(exc).__name__}: {exc}）")
        self.log.info(f"✓ 知識庫 {len(self.chunks)} 筆，檢索模式：{self.mode}")
        self._ready.set()
        return self.mode

    def _warm_embedder(self) -> None:
        """先問一次，讓 Ollama 把 embedding 模型載進去——索引沿用時沒有任何呼叫，第一位客人會多等載入時間"""
        try:
            self.embedder(["warmup"])
        except Exception as exc:  # noqa: BLE001
            self.log.warning(f"embedding 暖機失敗（不影響，第一題檢索會慢一點）: {exc}")

    def _build_chroma(self) -> None:
        if self.db_dir is None or not self.chunks:
            raise RuntimeError("沒有索引資料夾或知識庫是空的")
        import chromadb
        from chromadb.config import Settings

        self.db_dir.mkdir(parents=True, exist_ok=True)
        client = chromadb.PersistentClient(path=str(self.db_dir), settings=Settings(anonymized_telemetry=False))
        sig = signature(self.full_text, self.embed_model)
        try:
            col = client.get_collection(COLLECTION)
            if (col.metadata or {}).get("signature") == sig and col.count() == len(self.chunks):
                self._collection = col
                self.log.info("知識庫索引沿用（內容與 embedding 模型都沒變）")
                return
            client.delete_collection(COLLECTION)
        except Exception:  # noqa: BLE001 — 沒有這個 collection
            pass
        col = client.create_collection(COLLECTION, metadata={"hnsw:space": "cosine", "signature": sig})
        vectors = self.embedder([c.text for c in self.chunks])
        col.add(ids=[c.id for c in self.chunks], embeddings=vectors, documents=[c.text for c in self.chunks])
        self._collection = col
        self.log.info(f"知識庫索引重建：{len(self.chunks)} 筆（{self.embed_model}）")

    def wait_ready(self, timeout: float) -> bool:
        return self._ready.wait(timeout)

    @property
    def ready(self) -> bool:
        return self._ready.is_set() and self.mode != "none"

    # ── 檢索 ──────────────────────────────────────────────
    def search(self, query: str) -> List[Tuple[Chunk, float]]:
        """（片段, 分數）由高到低，只留過門檻的。向量檢索失敗當場退到關鍵字。"""
        if not self.ready or not query.strip():
            return []
        if self.mode == "chroma" and self._collection is not None:
            try:
                vec = self.embedder([query])[0]
                res = self._collection.query(query_embeddings=[vec], n_results=min(self.top_k, len(self.chunks)))
                by_id = {c.id: c for c in self.chunks}
                hits = [(by_id[i], 1.0 - d) for i, d in zip(res["ids"][0], res["distances"][0]) if i in by_id]
                return [(c, s) for c, s in hits if s >= self.min_similarity]
            except Exception as exc:  # noqa: BLE001
                self.log.warning(f"向量檢索失敗，這一輪改用關鍵字（{type(exc).__name__}: {exc}）")
        scores = keyword_scores(query, self.chunks)
        ranked = sorted(zip(self.chunks, scores), key=lambda x: -x[1])[: self.top_k]
        return [(c, s) for c, s in ranked if s >= self.min_keyword]

    def context_for(self, query: str) -> str:
        """附在系統提示最後的「本行知識庫」一節。

        回空字串＝「沒辦法用檢索回答」，呼叫端要改附整份全文：知識庫沒建好；或關鍵字模式查不到
        （關鍵字的召回率低，例如問「關門」但知識庫寫「三點半」，查不到不代表真的沒有，不能叫模型說沒有資訊）。
        只有向量模式查不到，才明講沒有相關資料，讓模型照提示詞規則說「沒有資訊」。
        """
        if not self.ready:
            return ""
        hits = self.search(query)
        head = "\n\n---\n\n# 本行知識庫（依客戶這一題檢索出的片段；回答本行業務問題只能依據這一節）\n\n"
        if not hits:
            if self.mode != "chroma":
                return ""
            return head + "（沒有找到與這個問題相關的資料。請照實說這部分沒有資訊，並請客戶洽詢櫃檯行員。）"
        return head + "\n".join(f"- {c.text}" for c, _s in hits)
