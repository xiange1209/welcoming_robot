import json
import urllib.request
import urllib.error
from typing import Optional

OLLAMA_URL = "http://localhost:11434/api/chat"

SYSTEM_PROMPT = """你是智慧銀行服務機器人的決策系統。
根據人臉辨識結果和客戶語音，輸出結構化 JSON 指令控制機器人行為。

輸出格式（嚴格 JSON，不要加任何其他文字）：
{
  "intent": "NAVIGATION" | "COMMAND" | "CHAT",
  "action": "<具體的動作代碼或 null>",
  "target": "<地點名稱或 null>",
  "reply": "<給用戶的語音回覆，50字以內>"
}

規則：
1. 黑名單人員 → intent=COMMAND, action=alert_staff, target=null, reply 留空字串
2. VIP platinum/gold → intent=NAVIGATION, action=navigate_to_vip_lounge, target=VIP貴賓室, reply 熱情問候
3. VIP standard → intent=CHAT, action=null, target=null, reply 基本問候
4. 訪客詢問位置/導覽 → intent=NAVIGATION, action=navigate, target=目標地點, reply 引導說明
5. 訪客提問（非導覽） → intent=CHAT, action=null, target=null, reply 回答銀行相關問題
6. 訪客無問題 → intent=CHAT, action=null, target=null, reply 問候並說明可提供哪些服務
7. reply 語氣專業親切，50字以內
8. reply 語言必須與「客戶說」的語言相同；若客戶無語音輸入，預設使用繁體中文"""


def build_user_prompt(req) -> str:
    lines = [f"人臉辨識結果: {req.face_recognition}"]
    if req.person_name:
        lines.append(f"人員名稱: {req.person_name}")
    if req.gender:
        lines.append(f"性別: {req.gender}")
    if req.vip_level:
        lines.append(f"VIP等級: {req.vip_level}")
    if req.risk_level:
        lines.append(f"風險等級: {req.risk_level}")
    if req.customer_input:
        lines.append(f"客戶說: {req.customer_input}")
    else:
        lines.append("客戶說: (無語音輸入，自動問候)")
    return "\n".join(lines)


def _supports_thinking(model: str) -> bool:
    return "gemma4" in model.lower()


def call_ollama(prompt: str, model: str, timeout: int = 120) -> tuple[Optional[str], Optional[str]]:
    body: dict = {
        "model": model,
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": prompt},
        ],
        "stream": False,
        "options": {"temperature": 0.1, "num_gpu": 99},
    }
    if _supports_thinking(model):
        body["think"] = True

    req = urllib.request.Request(
        OLLAMA_URL,
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        result = json.loads(resp.read().decode())
        message = result["message"]
        return message.get("content"), message.get("thinking")


def call_ollama_stream(prompt: str, model: str, timeout: int = 300):
    """串流版本，yield ("thinking"|"content", text)"""
    body: dict = {
        "model": model,
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": prompt},
        ],
        "stream": True,
        "options": {"temperature": 0.1, "num_gpu": 99},
    }
    if _supports_thinking(model):
        body["think"] = True

    req = urllib.request.Request(
        OLLAMA_URL,
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        for line in resp:
            line = line.strip()
            if not line:
                continue
            try:
                chunk = json.loads(line)
            except json.JSONDecodeError:
                continue
            if chunk.get("done"):
                break
            msg = chunk.get("message", {})
            if msg.get("thinking"):
                yield "thinking", msg["thinking"]
            elif msg.get("content"):
                yield "content", msg["content"]


def parse_json_response(raw: str) -> Optional[dict]:
    raw = raw.strip()
    start = raw.find("{")
    end = raw.rfind("}") + 1
    if start == -1 or end == 0:
        return None
    try:
        return json.loads(raw[start:end])
    except json.JSONDecodeError:
        return None
