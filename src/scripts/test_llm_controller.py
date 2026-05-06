#!/usr/bin/env python3
"""
LLM 控制器測試腳本 (文字輸入/輸出)
用途：測試 LLM 是否能輸出正確的結構化 JSON 以控制 ROS2 行為

使用方式：
    # 互動模式（預設）
    python3 src/scripts/test_llm_controller.py

    # 指定 LLM 後端
    python3 src/scripts/test_llm_controller.py --backend ollama --model qwen2.5:1.5b
    python3 src/scripts/test_llm_controller.py --backend anthropic  # 需要 ANTHROPIC_API_KEY

    # 執行預設測試案例
    python3 src/scripts/test_llm_controller.py --run-tests

安裝 Ollama（RPi4）：
    curl -fsSL https://ollama.com/install.sh | sh
    ollama pull qwen2.5:1.5b
"""

import sys
import json
import time
import argparse
import textwrap
from pathlib import Path
from typing import Optional

# ── LLM 系統提示 ──────────────────────────────────────────────────────────────

SYSTEM_PROMPT = """你是智慧銀行服務機器人的決策系統。根據輸入的人臉辨識結果和客戶語音，輸出結構化 JSON 指令控制機器人行為。

輸出格式（嚴格 JSON，不要加其他文字）：
{
  "intent": "<意圖>",
  "response_text": "<機器人語音回覆，中文>",
  "ros2_command": {
    "action": "<動作>",
    "target": "<目標，選填>",
    "speed": <速度 0.0-1.0，選填>,
    "priority": "<high|normal|low>"
  },
  "log_event": "<紀錄到資料庫的事件描述>",
  "confidence": <信心度 0.0-1.0>
}

意圖(intent)選項：
- greet_vip        VIP 迎接
- security_alert   黑名單安全警告
- greet_visitor    一般訪客問候
- answer_query     回答問題
- navigation_help  導引到特定區域
- unknown          無法判斷

動作(action)選項：
- speak            只播放語音，不移動
- navigate_forward 向前移動迎接
- navigate_to      導引到目標位置
- alert_staff      通知工作人員（黑名單用）
- idle             待機

規則：
1. 黑名單人員 → intent=security_alert, action=alert_staff, priority=high
2. VIP → intent=greet_vip, action=navigate_forward, 熱情問候，詢問需求
3. 一般訪客 → intent=greet_visitor, action=speak, 基本問候和導引
4. 有疑問 → intent=answer_query, action=speak, 盡力回答
5. response_text 限制在 50 字以內
"""

# ── 預設測試案例 ──────────────────────────────────────────────────────────────

TEST_CASES = [
    {
        "name": "VIP 迎接",
        "input": {
            "face_recognition": "vip",
            "person_name": "陳大明",
            "vip_level": "platinum",
            "gender": "M",
            "customer_input": ""
        }
    },
    {
        "name": "黑名單警告",
        "input": {
            "face_recognition": "blacklist",
            "person_name": "李三",
            "risk_level": "high",
            "gender": "M",
            "customer_input": ""
        }
    },
    {
        "name": "訪客問候",
        "input": {
            "face_recognition": "visitor",
            "person_name": None,
            "customer_input": "你好，我想辦一個活期存款帳戶"
        }
    },
    {
        "name": "VIP 詢問服務",
        "input": {
            "face_recognition": "vip",
            "person_name": "王美麗",
            "vip_level": "gold",
            "gender": "F",
            "customer_input": "請問黃金存款利率現在是多少？"
        }
    },
    {
        "name": "訪客問ATM位置",
        "input": {
            "face_recognition": "visitor",
            "person_name": None,
            "customer_input": "ATM 在哪裡？"
        }
    },
]


# ── LLM 後端 ──────────────────────────────────────────────────────────────────

def call_ollama(prompt: str, model: str = "qwen2.5:1.5b") -> Optional[str]:
    """呼叫本地 Ollama API"""
    try:
        import urllib.request
        import urllib.error

        payload = json.dumps({
            "model": model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": prompt}
            ],
            "stream": False,
            "options": {"temperature": 0.1}
        }).encode("utf-8")

        req = urllib.request.Request(
            "http://localhost:11434/v1/chat/completions",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST"
        )
        with urllib.request.urlopen(req, timeout=30) as resp:
            result = json.loads(resp.read().decode())
            return result["choices"][0]["message"]["content"]

    except Exception as e:
        print(f"[ERROR] Ollama 呼叫失敗: {e}")
        print("請確認 Ollama 已啟動: ollama serve")
        return None


def call_anthropic(prompt: str, model: str = "claude-haiku-4-5-20251001") -> Optional[str]:
    """呼叫 Anthropic API（PC 測試用）"""
    import os
    api_key = os.environ.get("ANTHROPIC_API_KEY")
    if not api_key:
        print("[ERROR] 未設定 ANTHROPIC_API_KEY")
        return None
    try:
        import anthropic
        client = anthropic.Anthropic(api_key=api_key)
        msg = client.messages.create(
            model=model,
            max_tokens=512,
            system=SYSTEM_PROMPT,
            messages=[{"role": "user", "content": prompt}]
        )
        return msg.content[0].text
    except ImportError:
        print("[ERROR] 未安裝 anthropic: pip install anthropic")
        return None
    except Exception as e:
        print(f"[ERROR] Anthropic API 失敗: {e}")
        return None


# ── 核心邏輯 ──────────────────────────────────────────────────────────────────

def build_user_prompt(face_result: str, person_name: Optional[str],
                      customer_input: str, extra: dict) -> str:
    lines = [f"人臉辨識結果: {face_result}"]
    if person_name:
        lines.append(f"人員名稱: {person_name}")
    for k, v in extra.items():
        lines.append(f"{k}: {v}")
    if customer_input:
        lines.append(f"客戶說: {customer_input}")
    else:
        lines.append("客戶說: (無語音輸入，自動問候)")
    return "\n".join(lines)


def parse_llm_output(raw: str) -> Optional[dict]:
    """從 LLM 輸出中解析 JSON"""
    raw = raw.strip()
    # 尋找 { ... } 區塊
    start = raw.find("{")
    end = raw.rfind("}") + 1
    if start == -1 or end == 0:
        return None
    try:
        return json.loads(raw[start:end])
    except json.JSONDecodeError:
        return None


def print_result(result: dict):
    """格式化印出解析後的 JSON 指令"""
    intent = result.get("intent", "unknown")
    response = result.get("response_text", "")
    cmd = result.get("ros2_command", {})
    conf = result.get("confidence", 0.0)
    log = result.get("log_event", "")

    colors = {
        "greet_vip": "\033[93m",       # 黃
        "security_alert": "\033[91m",  # 紅
        "greet_visitor": "\033[92m",   # 綠
        "answer_query": "\033[96m",    # 青
        "navigation_help": "\033[94m", # 藍
        "unknown": "\033[90m",         # 灰
    }
    reset = "\033[0m"
    c = colors.get(intent, "\033[0m")

    print(f"\n{c}{'─'*55}{reset}")
    print(f"{c}意圖: {intent.upper()}{reset}  (信心度: {conf:.2f})")
    print(f"語音回覆: {response}")
    print(f"ROS2 指令: action={cmd.get('action','?')} | "
          f"target={cmd.get('target','無')} | priority={cmd.get('priority','normal')}")
    if cmd.get("speed") is not None:
        print(f"  速度: {cmd['speed']}")
    print(f"日誌: {log}")
    print(f"{c}{'─'*55}{reset}")


def run_llm(user_input_dict: dict, backend: str, model: str) -> Optional[dict]:
    extra = {k: v for k, v in user_input_dict.items()
             if k not in ("face_recognition", "person_name", "customer_input") and v is not None}
    prompt = build_user_prompt(
        user_input_dict.get("face_recognition", "visitor"),
        user_input_dict.get("person_name"),
        user_input_dict.get("customer_input", ""),
        extra
    )

    print(f"\n[輸入]\n{textwrap.indent(prompt, '  ')}")
    t0 = time.time()

    if backend == "ollama":
        raw = call_ollama(prompt, model)
    elif backend == "anthropic":
        raw = call_anthropic(prompt, model)
    else:
        print(f"[ERROR] 未知後端: {backend}")
        return None

    elapsed = time.time() - t0

    if raw is None:
        return None

    print(f"\n[LLM 原始輸出] (耗時 {elapsed:.2f}s)\n{textwrap.indent(raw, '  ')}")
    result = parse_llm_output(raw)
    if result:
        print_result(result)
    else:
        print("[WARN] 無法解析 JSON 輸出")

    return result


# ── 互動 REPL ─────────────────────────────────────────────────────────────────

def interactive_mode(backend: str, model: str):
    print(f"\n{'='*55}")
    print("  LLM 控制器互動測試")
    print(f"  後端: {backend} | 模型: {model}")
    print(f"{'='*55}")
    print("輸入格式（依序輸入）：")
    print("  face_recognition: vip / blacklist / visitor")
    print("  person_name: 姓名 (或留空)")
    print("  customer_input: 客戶說的話 (或留空)")
    print("\n輸入 'q' 退出，'test' 執行預設測試案例\n")

    while True:
        try:
            face = input("人臉辨識 [vip/blacklist/visitor]: ").strip().lower()
            if face == "q":
                break
            if face == "test":
                run_test_cases(backend, model)
                continue
            if face not in ("vip", "blacklist", "visitor"):
                face = "visitor"

            name = input("姓名 (留空跳過): ").strip() or None
            text = input("客戶語音 (留空=自動問候): ").strip()

            extra = {}
            if face == "vip":
                level = input("VIP 等級 [standard/gold/platinum]: ").strip() or "standard"
                extra["vip_level"] = level
            elif face == "blacklist":
                risk = input("風險等級 [low/medium/high]: ").strip() or "medium"
                extra["risk_level"] = risk

            run_llm({"face_recognition": face, "person_name": name,
                     "customer_input": text, **extra},
                    backend, model)

        except KeyboardInterrupt:
            break
        except EOFError:
            break

    print("\n[結束]")


def run_test_cases(backend: str, model: str):
    print(f"\n{'='*55}")
    print(f"  執行 {len(TEST_CASES)} 個預設測試案例")
    print(f"{'='*55}")

    pass_count = 0
    for i, tc in enumerate(TEST_CASES):
        print(f"\n[測試 {i+1}/{len(TEST_CASES)}] {tc['name']}")
        result = run_llm(tc["input"], backend, model)
        if result and "intent" in result and "ros2_command" in result:
            pass_count += 1

    print(f"\n{'='*55}")
    print(f"通過: {pass_count}/{len(TEST_CASES)}")
    if pass_count < len(TEST_CASES):
        print("未通過的案例：JSON 格式錯誤或 intent/ros2_command 缺失")
    print(f"{'='*55}")


# ── 主程式 ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="LLM 控制器測試腳本")
    parser.add_argument("--backend", choices=["ollama", "anthropic"], default="ollama",
                        help="LLM 後端 (預設: ollama)")
    parser.add_argument("--model", default="qwen2.5:1.5b",
                        help="模型名稱 (Ollama: qwen2.5:1.5b | Anthropic: claude-haiku-4-5-20251001)")
    parser.add_argument("--run-tests", action="store_true",
                        help="執行預設測試案例後退出")
    args = parser.parse_args()

    if args.run_tests:
        run_test_cases(args.backend, args.model)
    else:
        interactive_mode(args.backend, args.model)


if __name__ == "__main__":
    main()
