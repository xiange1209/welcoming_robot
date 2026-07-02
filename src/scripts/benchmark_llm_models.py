#!/usr/bin/env python3
"""LLM 模型效能基準測試工具

測量指定 Ollama 模型的「首次輸出時間 (TTFT)」與「平均輸出時間 (總生成時間)」，
輸出成可直接貼進報告的 Markdown 表格。

使用前提：
- 已在本機（通常是筆電）啟動 Ollama：ollama serve
- 已 pull 好要測試的模型，例如：
    ollama pull gemma3:4b
    ollama pull gemma4:e2b
    ollama pull gemma4:e4b
    ollama pull qwen2.5:3b

使用方式：
    python3 src/scripts/benchmark_llm_models.py
    python3 src/scripts/benchmark_llm_models.py --models gemma3:4b qwen2.5:3b --runs 8
    python3 src/scripts/benchmark_llm_models.py --host http://192.168.1.xxx:11434

輸出：
- 終端機列印 Markdown 表格（可直接貼進報告）
- 同時輸出 JSON 明細檔（--output 指定路徑，預設 benchmark_llm_results.json）

注意：
- 本腳本只負責「量測」，不會幫你捏造數據。每次執行的結果會因模型是否常駐、
  GPU 是否已預熱、系統負載而有差異，建議同一批測試在相近條件下一次跑完。
- 第一次呼叫某個模型時 Ollama 需要載入模型到記憶體，延遲會明顯偏高；
  腳本預設會先做 1 次暖機呼叫（不列入統計），讓比較更公平。
"""

import argparse
import json
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from statistics import mean
from typing import Optional

DEFAULT_MODELS = ["gemma3-bank", "gemma4-bank", "qwen2.5:3b"]

SYSTEM_PROMPT = """你是智慧銀行服務機器人的決策系統。
根據人臉辨識結果和客戶語音，輸出結構化 JSON 指令控制機器人行為。

輸出格式（嚴格 JSON，不要加任何其他文字）：
{
  "intent": "NAVIGATION" | "COMMAND" | "CHAT",
  "action": "<具體的動作代碼或 null>",
  "target": "<地點名稱或 null>",
  "reply": "<給用戶的語音回覆，50字以內>"
}"""

DEFAULT_PROMPT = """人臉辨識結果: vip
人員名稱: 陳先生
性別: M
VIP等級: platinum
客戶說: 你好，我想辦理黃金存款"""


@dataclass
class RunResult:
    first_token_ms: Optional[float]
    total_ms: Optional[float]
    ok: bool
    error: Optional[str] = None


@dataclass
class ModelBenchmark:
    model: str
    runs: list = field(default_factory=list)

    @property
    def ok_runs(self):
        return [r for r in self.runs if r.ok]

    @property
    def success_rate(self) -> float:
        if not self.runs:
            return 0.0
        return len(self.ok_runs) / len(self.runs)

    @property
    def avg_first_token_ms(self) -> Optional[float]:
        values = [r.first_token_ms for r in self.ok_runs if r.first_token_ms is not None]
        return mean(values) if values else None

    @property
    def avg_total_ms(self) -> Optional[float]:
        values = [r.total_ms for r in self.ok_runs if r.total_ms is not None]
        return mean(values) if values else None

    @property
    def min_total_ms(self) -> Optional[float]:
        values = [r.total_ms for r in self.ok_runs if r.total_ms is not None]
        return min(values) if values else None

    @property
    def max_total_ms(self) -> Optional[float]:
        values = [r.total_ms for r in self.ok_runs if r.total_ms is not None]
        return max(values) if values else None


def call_ollama_stream(host: str, model: str, prompt: str, timeout: int = 120) -> RunResult:
    """呼叫 Ollama /api/chat（streaming），量測首字延遲與總生成時間"""
    body = {
        "model": model,
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": prompt},
        ],
        "stream": True,
        "options": {"temperature": 0.1, "num_gpu": 99},
    }

    url = host.rstrip("/") + "/api/chat"
    req = urllib.request.Request(
        url,
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    t0 = time.perf_counter()
    first_token_time: Optional[float] = None

    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            for line in resp:
                line = line.strip()
                if not line:
                    continue
                try:
                    chunk = json.loads(line)
                except json.JSONDecodeError:
                    continue

                content = chunk.get("message", {}).get("content")
                if content and first_token_time is None:
                    first_token_time = time.perf_counter()

                if chunk.get("done"):
                    t_end = time.perf_counter()
                    first_token_ms = (
                        (first_token_time - t0) * 1000 if first_token_time is not None else None
                    )
                    total_ms = (t_end - t0) * 1000
                    return RunResult(first_token_ms=first_token_ms, total_ms=total_ms, ok=True)

        # 串流結束但沒有收到 done=True
        return RunResult(first_token_ms=None, total_ms=None, ok=False, error="串流結束但未收到 done 標記")

    except urllib.error.URLError as e:
        return RunResult(first_token_ms=None, total_ms=None, ok=False, error=f"連線失敗: {e}")
    except Exception as e:
        return RunResult(first_token_ms=None, total_ms=None, ok=False, error=str(e))


def benchmark_model(host: str, model: str, prompt: str, runs: int, warmup: bool) -> ModelBenchmark:
    print(f"\n=== 測試模型: {model} ===")
    result = ModelBenchmark(model=model)

    if warmup:
        print("  暖機呼叫中（不計入統計）...")
        warm = call_ollama_stream(host, model, prompt)
        if not warm.ok:
            print(f"  ⚠ 暖機呼叫失敗: {warm.error}")

    for i in range(runs):
        run = call_ollama_stream(host, model, prompt)
        result.runs.append(run)
        if run.ok:
            print(
                f"  第 {i + 1}/{runs} 次: 首字 {run.first_token_ms:.0f} ms | "
                f"總耗時 {run.total_ms:.0f} ms"
            )
        else:
            print(f"  第 {i + 1}/{runs} 次: 失敗 ({run.error})")

    return result


def print_markdown_table(results: list):
    print("\n\n## LLM 模型效能比較（可直接貼進報告）\n")
    print("| 模型 | 首次輸出時間平均 (ms) | 平均輸出時間 (ms) | 最快 (ms) | 最慢 (ms) | 成功率 |")
    print("|---|---|---|---|---|---|")
    for r in results:
        first_token = f"{r.avg_first_token_ms:.0f}" if r.avg_first_token_ms is not None else "N/A"
        total = f"{r.avg_total_ms:.0f}" if r.avg_total_ms is not None else "N/A"
        min_total = f"{r.min_total_ms:.0f}" if r.min_total_ms is not None else "N/A"
        max_total = f"{r.max_total_ms:.0f}" if r.max_total_ms is not None else "N/A"
        print(f"| {r.model} | {first_token} | {total} | {min_total} | {max_total} | {r.success_rate * 100:.0f}% |")


def save_json(results: list, output_path: str):
    payload = []
    for r in results:
        payload.append(
            {
                "model": r.model,
                "avg_first_token_ms": r.avg_first_token_ms,
                "avg_total_ms": r.avg_total_ms,
                "min_total_ms": r.min_total_ms,
                "max_total_ms": r.max_total_ms,
                "success_rate": r.success_rate,
                "runs": [
                    {"first_token_ms": run.first_token_ms, "total_ms": run.total_ms, "ok": run.ok, "error": run.error}
                    for run in r.runs
                ],
            }
        )
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print(f"\n明細已存至: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="LLM 模型效能基準測試")
    parser.add_argument("--host", default="http://localhost:11434", help="Ollama API 位址")
    parser.add_argument("--models", nargs="+", default=DEFAULT_MODELS, help="要測試的模型清單")
    parser.add_argument("--runs", type=int, default=5, help="每個模型測試次數")
    parser.add_argument("--prompt", default=DEFAULT_PROMPT, help="測試用的使用者輸入")
    parser.add_argument("--no-warmup", action="store_true", help="不做暖機呼叫")
    parser.add_argument("--output", default="benchmark_llm_results.json", help="JSON 明細輸出路徑")
    args = parser.parse_args()

    print(f"Ollama 位址: {args.host}")
    print(f"測試模型: {', '.join(args.models)}")
    print(f"每模型測試次數: {args.runs}")

    all_results = []
    for model in args.models:
        result = benchmark_model(args.host, model, args.prompt, args.runs, warmup=not args.no_warmup)
        all_results.append(result)

    print_markdown_table(all_results)
    save_json(all_results, args.output)


if __name__ == "__main__":
    main()
