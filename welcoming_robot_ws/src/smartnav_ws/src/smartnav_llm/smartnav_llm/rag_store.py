#!/usr/bin/env python3

"""銀行知識庫檢索模組 (RAG)

從 Markdown 知識庫載入行內規定與業務說明，以混合檢索取出最相關的段落供 LLM 引用，
避免小模型自行捏造銀行的營業時間、費率與作業規定。

檢索採用兩路混合：

* **字元 N-gram TF-IDF**：純 Python 實作，不需任何模型或額外套件，中文不需斷詞。
* **語意向量**：透過 Ollama 的 embedding 模型 (如 bge-m3)，可命中換句話說的問法。

語意向量為選用，若 Ollama 沒有對應模型或連線失敗，會自動退回純 TF-IDF 檢索，
確保知識庫在任何情況下都能運作。
"""

import hashlib
import json
import logging
import math
import re
import threading
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import requests


def _get_logger() -> logging.Logger:
    """取得模組日誌記錄器

    Returns:
        logging.Logger: 已配置格式的 Logger 實例
    """
    logger = logging.getLogger(__name__)
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s"))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
    return logger


# 中文連續字串與英數字詞彙
_CJK_RUN_PATTERN = re.compile(r"[一-鿿]+")
_ALNUM_PATTERN = re.compile(r"[a-z0-9]+")
_ALNUM_JOIN_PATTERN = re.compile(r"(?<=[a-z0-9])[-_.](?=[a-z0-9])")

# 中文虛詞：口語問句中大量出現但不帶檢索訊號，保留會嚴重稀釋相似度
# ★★ 2026-08-26（W6）：還沒填寫的知識庫段落一律不進索引 ★★
#
# knowledge/bank_faq.md 是一份**範本**，整份有 62 個「（請填寫）」。
# 它在 enable_bank_tools:=false 時會被 search_bank_knowledge_tool 檢索到，
# 模型會把括號裡的「例如 …」當成本行的規定唸出來 —— 這是最典型的幻覺，
# 而且來源是我們自己餵給它的。
#
# 這不是「過濾掉不好看的字」，是**讓半成品知識庫安全降級成「查無資料」**：
# 系統提示詞 §5 已經規定「查不到就說查不到」，只要不把範本當資料餵進去，
# 那條規則就會生效。
_PLACEHOLDER_MARKER = "請填寫"

_UNIGRAM_WEIGHT = 0.35
_STOPWORD_CHARS = frozenset(
    "的了嗎呢吧啊喔耶我你妳他她它您們請問想要有沒在是不就都也很會能可以怎麼什樣如何哪個那這些多少幫忙一下和跟與及還再又於為被把給對從到說知道且但或"
)
# Markdown 標題與雜訊
_HEADING_PATTERN = re.compile(r"^(#{1,6})\s+(.*?)\s*#*$")
_MD_NOISE_PATTERN = re.compile(r"[*`>_\[\]()]|^\s*[-+]\s+", re.MULTILINE)


def tokenize(text: str) -> List[str]:
    """將文字切成檢索用詞元

    中文以「雙字」N-gram 為主，不需要斷詞套件即可涵蓋大多數銀行術語
    (例如「定存」「換匯」「利率」)。刻意不使用單字元 N-gram：像「想」「要」「在」
    這類虛詞會在每個段落都命中，反而把真正相關的段落壓下去。
    由虛詞組成的雙字詞 (例如「我想」「怎麼」) 同樣會被濾掉。
    英數字則以完整詞彙處理 (例如 atm、app、24)。

    Args:
        text: 待切分的原始文字

    Returns:
        List[str]: 詞元列表，可能包含重複詞元以反映詞頻
    """
    # 先併掉英數字中間的連字號，讓「Wi-Fi」與「wifi」視為同一個詞
    text = _ALNUM_JOIN_PATTERN.sub("", text.lower())
    tokens: List[str] = _ALNUM_PATTERN.findall(text)

    for run in _CJK_RUN_PATTERN.findall(text):
        # 實詞單字仍要保留，「換日幣」與「換外幣」才有共同詞元可比對
        tokens.extend(char for char in run if char not in _STOPWORD_CHARS)

        tokens.extend(
            run[i:i + 2]
            for i in range(len(run) - 1)
            if not (run[i] in _STOPWORD_CHARS and run[i + 1] in _STOPWORD_CHARS)
        )

    return tokens


@dataclass
class KnowledgeChunk:
    """知識庫段落

    Attributes:
        chunk_id: 段落唯一識別碼
        title: 標題路徑，例如「換匯服務 > 外幣現鈔」
        text: 段落內文
        source: 來源檔名
    """

    chunk_id: str
    title: str
    text: str
    source: str

    def to_search_text(self) -> str:
        """組出供索引使用的文字

        標題重複一次以提高標題關鍵字的權重。

        Returns:
            str: 索引用文字
        """
        return f"{self.title} {self.title} {self.text}"

    def to_context(self, index: int) -> str:
        """組出提供給 LLM 的引用格式

        Args:
            index: 這筆資料在檢索結果中的序號 (從 1 開始)

        Returns:
            str: 帶標題與來源的段落文字
        """
        return f"[資料{index}] {self.title} (來源: {self.source})\n{self.text}"


@dataclass
class SearchResult:
    """單筆檢索結果

    Attributes:
        chunk: 命中的知識庫段落
        lexical_score: 關鍵字 (TF-IDF) 餘弦相似度
        dense_score: 語意向量餘弦相似度，未啟用時為 0.0
        fused_score: 融合後的排序分數
    """

    chunk: KnowledgeChunk
    lexical_score: float = 0.0
    dense_score: float = 0.0
    fused_score: float = 0.0


def split_markdown(text: str, source: str, max_chars: int = 500) -> List[KnowledgeChunk]:
    """依 Markdown 標題切分知識庫文件

    以標題階層為界切段，並保留完整標題路徑，讓每個段落脫離上下文後仍可讀。
    過長的段落會再依空行二次切分，每一小段都會冠上同樣的標題路徑。

    Args:
        text: Markdown 全文
        source: 來源檔名，會記錄在段落中供 LLM 標示出處
        max_chars: 單一段落的字數上限，超過則二次切分

    Returns:
        List[KnowledgeChunk]: 切分後的段落列表
    """
    chunks: List[KnowledgeChunk] = []
    heading_stack: List[str] = []
    buffer: List[str] = []

    def flush() -> None:
        """將暫存內容收成段落"""
        body = "\n".join(buffer).strip()
        buffer.clear()
        if not body:
            return

        title = " > ".join(heading_stack) if heading_stack else source
        # 依空行二次切分，避免單一段落過長稀釋檢索訊號
        pieces: List[str] = []
        if len(body) <= max_chars:
            pieces = [body]
        else:
            current = ""
            for paragraph in re.split(r"\n\s*\n", body):
                paragraph = paragraph.strip()
                if not paragraph:
                    continue
                if current and len(current) + len(paragraph) > max_chars:
                    pieces.append(current)
                    current = paragraph
                else:
                    current = f"{current}\n{paragraph}" if current else paragraph
            if current:
                pieces.append(current)

        for piece in pieces:
            chunk_id = f"{source}#{len(chunks):03d}"
            chunks.append(KnowledgeChunk(chunk_id=chunk_id, title=title, text=piece, source=source))

    for line in text.splitlines():
        matched = _HEADING_PATTERN.match(line.rstrip())
        if matched:
            flush()
            level = len(matched.group(1))
            heading = matched.group(2).strip()
            # 依標題階層維護路徑
            heading_stack[:] = heading_stack[: level - 1]
            while len(heading_stack) < level - 1:
                heading_stack.append("")
            heading_stack.append(heading)
            heading_stack[:] = [h for h in heading_stack if h]
        else:
            buffer.append(line)

    flush()
    return chunks


def _term_weight(term: str, term_freq: int, idf: float) -> float:
    """計算單一詞元的 TF-IDF 權重

    中文單字的辨識力遠低於雙字詞 (「首」會命中「首次申請」，但兩者毫無關係)，
    因此單字只給部分權重：既保留「換日幣」對上「換外幣」的橋樑作用，
    又不會讓無關問題靠幾個共用單字湊出高分。

    Args:
        term: 詞元
        term_freq: 詞元在該段落中的出現次數
        idf: 該詞元的逆文件頻率

    Returns:
        float: 詞元權重
    """
    weight = (1.0 + math.log(term_freq)) * idf
    return weight * _UNIGRAM_WEIGHT if len(term) == 1 else weight


class LexicalIndex:
    """字元 N-gram TF-IDF 倒排索引

    知識庫規模通常僅數十至數百段，以純 Python 倒排索引即可在毫秒內完成檢索，
    不需引入 sklearn 或向量資料庫。
    """

    def __init__(self) -> None:
        self.postings: Dict[str, List[Tuple[int, float]]] = {}
        self.idf: Dict[str, float] = {}
        self.doc_count: int = 0

    def build(self, chunks: List[KnowledgeChunk]) -> None:
        """建立索引

        Args:
            chunks: 知識庫段落列表
        """
        self.postings = {}
        self.idf = {}
        self.doc_count = len(chunks)
        if not chunks:
            return

        doc_terms: List[Counter] = [Counter(tokenize(chunk.to_search_text())) for chunk in chunks]

        document_freq: Counter = Counter()
        for terms in doc_terms:
            document_freq.update(terms.keys())

        for term, freq in document_freq.items():
            self.idf[term] = math.log(1.0 + self.doc_count / freq)

        for doc_index, terms in enumerate(doc_terms):
            weights = {term: _term_weight(term, tf, self.idf[term]) for term, tf in terms.items()}
            norm = math.sqrt(sum(w * w for w in weights.values())) or 1.0
            for term, weight in weights.items():
                self.postings.setdefault(term, []).append((doc_index, weight / norm))

    def search(self, query: str) -> Dict[int, float]:
        """檢索並回傳各段落的餘弦相似度

        Args:
            query: 使用者問題

        Returns:
            Dict[int, float]: 段落索引對應的餘弦相似度，未命中的段落不會出現
        """
        if not self.doc_count:
            return {}

        query_terms = Counter(tokenize(query))
        query_weights = {
            term: _term_weight(term, tf, self.idf[term]) for term, tf in query_terms.items() if term in self.idf
        }
        if not query_weights:
            return {}

        norm = math.sqrt(sum(w * w for w in query_weights.values())) or 1.0
        scores: Dict[int, float] = {}
        for term, weight in query_weights.items():
            for doc_index, doc_weight in self.postings.get(term, []):
                scores[doc_index] = scores.get(doc_index, 0.0) + (weight / norm) * doc_weight

        return scores


class DenseIndex:
    """Ollama 語意向量索引

    透過 Ollama 的 embedding API 取得向量，可命中 TF-IDF 抓不到的換句話說問法
    (例如「錢放定期」對應「定期存款」)。模型不存在或連線失敗時會自動停用。
    """

    def __init__(self, base_url: str, model: str, timeout: float = 30.0) -> None:
        """初始化語意索引

        Args:
            base_url: Ollama 服務位址
            model: embedding 模型名稱，例如 bge-m3
            timeout: 單次請求逾時秒數
        """
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.timeout = timeout
        self.available = bool(model)
        self.vectors: List[List[float]] = []
        self._logger = _get_logger()

    def embed(self, texts: List[str]) -> Optional[List[List[float]]]:
        """呼叫 Ollama 取得文字向量

        優先使用新版 /api/embed，遇到舊版 Ollama 則退回 /api/embeddings 逐筆取得。

        Args:
            texts: 待向量化的文字列表

        Returns:
            Optional[List[List[float]]]: 向量列表，失敗時為 None
        """
        if not self.available or not texts:
            return None

        try:
            response = requests.post(
                f"{self.base_url}/api/embed",
                json={"model": self.model, "input": texts},
                timeout=self.timeout,
            )
            if response.status_code == 404:
                return self._embed_legacy(texts)
            response.raise_for_status()
            embeddings = response.json().get("embeddings")
            if not embeddings:
                raise ValueError("回應中沒有 embeddings 欄位")
            return embeddings
        except Exception as exc:
            self._logger.warning(f"語意向量取得失敗，改用純關鍵字檢索 ({self.model}): {exc}")
            self.available = False
            return None

    def _embed_legacy(self, texts: List[str]) -> Optional[List[List[float]]]:
        """以舊版 API 逐筆取得向量

        Args:
            texts: 待向量化的文字列表

        Returns:
            Optional[List[List[float]]]: 向量列表，失敗時為 None
        """
        embeddings: List[List[float]] = []
        for text in texts:
            response = requests.post(
                f"{self.base_url}/api/embeddings",
                json={"model": self.model, "prompt": text},
                timeout=self.timeout,
            )
            response.raise_for_status()
            embeddings.append(response.json()["embedding"])
        return embeddings

    def build(self, chunks: List[KnowledgeChunk], cache_path: Optional[Path] = None, content_hash: str = "") -> bool:
        """建立語意索引，並以內容雜湊快取避免重複計算

        Args:
            chunks: 知識庫段落列表
            cache_path: 向量快取檔路徑，None 表示不使用快取
            content_hash: 知識庫內容雜湊，用於判斷快取是否過期

        Returns:
            bool: 是否成功建立語意索引
        """
        if not self.available or not chunks:
            return False

        if cache_path and cache_path.exists():
            try:
                cached = json.loads(cache_path.read_text(encoding="utf-8"))
                if cached.get("hash") == content_hash and cached.get("model") == self.model:
                    self.vectors = cached["vectors"]
                    self._logger.info(f"✓ 載入語意向量快取: {cache_path.name} ({len(self.vectors)} 段)")
                    return True
            except Exception as exc:
                self._logger.warning(f"語意向量快取讀取失敗，將重新計算: {exc}")

        embeddings = self.embed([chunk.to_search_text() for chunk in chunks])
        if not embeddings:
            return False

        self.vectors = [self._normalize(vector) for vector in embeddings]

        if cache_path:
            try:
                cache_path.parent.mkdir(parents=True, exist_ok=True)
                payload = {"hash": content_hash, "model": self.model, "vectors": self.vectors}
                cache_path.write_text(json.dumps(payload), encoding="utf-8")
            except Exception as exc:
                self._logger.warning(f"語意向量快取寫入失敗 (不影響檢索): {exc}")

        return True

    def search(self, query: str) -> Dict[int, float]:
        """檢索並回傳各段落的語意相似度

        Args:
            query: 使用者問題

        Returns:
            Dict[int, float]: 段落索引對應的餘弦相似度
        """
        if not self.available or not self.vectors:
            return {}

        embeddings = self.embed([query])
        if not embeddings:
            return {}

        query_vector = self._normalize(embeddings[0])
        return {
            index: sum(a * b for a, b in zip(query_vector, vector)) for index, vector in enumerate(self.vectors)
        }

    @staticmethod
    def _normalize(vector: List[float]) -> List[float]:
        """將向量正規化為單位長度

        Args:
            vector: 原始向量

        Returns:
            List[float]: 單位向量
        """
        norm = math.sqrt(sum(v * v for v in vector)) or 1.0
        return [v / norm for v in vector]


class BankKnowledgeStore:
    """銀行知識庫

    負責載入 Markdown 知識庫、建立混合索引，並提供給 LLM 使用的檢索介面。
    支援偵測檔案異動後自動重建索引，行員修改知識庫內容不需重啟節點。
    """

    def __init__(
        self,
        knowledge_dir: Path,
        ollama_base_url: str = "",
        embed_model: str = "",
        top_k: int = 3,
        min_lexical_score: float = 0.10,
        min_dense_score: float = 0.45,
        dense_weight: float = 0.5,
        auto_reload: bool = True,
        cache_dir: Optional[Path] = None,
    ) -> None:
        """初始化知識庫

        Args:
            knowledge_dir: 知識庫資料夾，內含 .md 檔案
            ollama_base_url: Ollama 服務位址，留空表示停用語意檢索
            embed_model: embedding 模型名稱，留空表示停用語意檢索
            top_k: 每次檢索回傳的段落數
            min_lexical_score: 關鍵字相似度門檻，低於此值視為不相關
            min_dense_score: 語意相似度門檻，低於此值視為不相關
            dense_weight: 融合排序時語意分數的權重 (0~1)
            auto_reload: 是否在每次檢索前偵測檔案異動並自動重建索引
            cache_dir: 語意向量快取資料夾，預設為 ~/.smartnav
        """
        self.knowledge_dir = Path(knowledge_dir)
        # 不寫進知識庫資料夾：--symlink-install 會讓它指回原始碼樹，快取檔會污染 repo
        self.cache_dir = Path(cache_dir) if cache_dir else Path.home() / ".smartnav"
        self.top_k = top_k
        self.min_lexical_score = min_lexical_score
        self.min_dense_score = min_dense_score
        self.dense_weight = dense_weight
        self.auto_reload = auto_reload

        self.chunks: List[KnowledgeChunk] = []
        self.lexical = LexicalIndex()
        self.dense = DenseIndex(ollama_base_url, embed_model) if (ollama_base_url and embed_model) else None
        self.dense_ready = False

        self._lock = threading.Lock()
        self._content_hash = ""
        self._logger = _get_logger()

    @property
    def is_ready(self) -> bool:
        """知識庫是否有可檢索的內容

        Returns:
            bool: 是否已載入任何段落
        """
        return bool(self.chunks)

    def load(self, force: bool = False) -> int:
        """載入知識庫並建立索引

        Args:
            force: 即使內容未變動也強制重建索引

        Returns:
            int: 載入的段落數
        """
        with self._lock:
            return self._load_locked(force)

    def _load_locked(self, force: bool = False) -> int:
        """實際執行載入 (呼叫前需持有鎖)

        Args:
            force: 是否強制重建

        Returns:
            int: 載入的段落數
        """
        if not self.knowledge_dir.is_dir():
            self._logger.warning(f"✗ 找不到知識庫資料夾: {self.knowledge_dir}")
            self.chunks = []
            return 0

        files = sorted(p for p in self.knowledge_dir.glob("*.md") if p.is_file())
        raw_documents = []
        hasher = hashlib.sha256()
        for path in files:
            try:
                content = path.read_text(encoding="utf-8")
            except Exception as exc:
                self._logger.warning(f"知識庫檔案讀取失敗 {path.name}: {exc}")
                continue
            hasher.update(path.name.encode("utf-8"))
            hasher.update(content.encode("utf-8"))
            raw_documents.append((path.name, content))

        content_hash = hasher.hexdigest()
        if not force and content_hash == self._content_hash and self.chunks:
            return len(self.chunks)

        chunks: List[KnowledgeChunk] = []
        _skipped = 0        # 略過的「（請填寫）」範本段落數（W6）
        for name, content in raw_documents:
            for _c in split_markdown(content, source=name):
                if _PLACEHOLDER_MARKER in _c.text:
                    _skipped += 1
                    continue
                chunks.append(_c)

        if _skipped:
            self._logger.warning(
                f"知識庫有 {_skipped} 個段落含「{_PLACEHOLDER_MARKER}」尚未填寫，已略過不索引"
                " —— 這些問題會回「查不到」，不會被編造答案")
        self.chunks = chunks
        self._content_hash = content_hash
        self.lexical.build(chunks)

        self.dense_ready = False
        if self.dense and chunks:
            cache_path = self.cache_dir / "rag_embedding_cache.json"
            self.dense_ready = self.dense.build(chunks, cache_path=cache_path, content_hash=content_hash)

        mode = "關鍵字 + 語意向量" if self.dense_ready else "純關鍵字"
        self._logger.info(f"✓ 銀行知識庫已載入: {len(files)} 檔 / {len(chunks)} 段, 檢索模式: {mode}")
        return len(chunks)

    def search(self, query: str, top_k: Optional[int] = None) -> List[SearchResult]:
        """檢索知識庫

        兩路分數各自正規化後加權融合排序，並以絕對相似度門檻濾掉不相關段落，
        寧可回報查無資料，也不讓 LLM 拿著不相干的段落亂答。

        Args:
            query: 使用者問題
            top_k: 回傳段落數，None 表示使用預設值

        Returns:
            List[SearchResult]: 依相關度排序的檢索結果，可能為空列表
        """
        query = (query or "").strip()
        if not query:
            return []

        with self._lock:
            if self.auto_reload or not self.chunks:
                self._load_locked()

            if not self.chunks:
                return []

            lexical_scores = self.lexical.search(query)
            dense_scores = self.dense.search(query) if (self.dense and self.dense_ready) else {}

            candidates = set(lexical_scores) | set(dense_scores)
            if not candidates:
                return []

            max_lexical = max(lexical_scores.values(), default=0.0) or 1.0
            max_dense = max(dense_scores.values(), default=0.0) or 1.0
            dense_weight = self.dense_weight if dense_scores else 0.0

            results: List[SearchResult] = []
            for index in candidates:
                lexical_score = lexical_scores.get(index, 0.0)
                dense_score = dense_scores.get(index, 0.0)

                # 任一路達到門檻即視為相關，避免單一檢索方式的盲點
                if lexical_score < self.min_lexical_score and dense_score < self.min_dense_score:
                    continue

                fused = (1.0 - dense_weight) * (lexical_score / max_lexical) + dense_weight * (
                    dense_score / max_dense
                )
                results.append(
                    SearchResult(
                        chunk=self.chunks[index],
                        lexical_score=lexical_score,
                        dense_score=dense_score,
                        fused_score=fused,
                    )
                )

            results.sort(key=lambda item: item.fused_score, reverse=True)
            return results[: top_k or self.top_k]

    def build_context(self, query: str, top_k: Optional[int] = None) -> str:
        """檢索並組出可直接餵給 LLM 的文字

        Args:
            query: 使用者問題
            top_k: 回傳段落數，None 表示使用預設值

        Returns:
            str: 依既有工具慣例組成的「執行結果」字串
        """
        if not self.is_ready and not self.knowledge_dir.is_dir():
            return "執行結果: 失敗, 詳細信息: 知識庫尚未設定，請確認 knowledge 資料夾存在且含有 .md 檔案"

        results = self.search(query, top_k=top_k)
        if not results:
            return (
                "執行結果: 查無資料, 詳細信息: 知識庫中沒有這個問題的規定。"
                "請據實告知客戶你查不到，並建議由行員櫃檯協助，嚴禁自行推測或編造答案"
            )

        blocks = [result.chunk.to_context(index) for index, result in enumerate(results, start=1)]
        joined = "\n\n".join(blocks)
        return f"執行結果: 成功, 知識庫資料如下 (僅能依據以下內容回答，不足的部分請說查不到):\n{joined}"
