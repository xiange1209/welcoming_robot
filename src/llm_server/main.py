#!/usr/bin/env python3
"""
智慧銀行 LLM API Server
筆電端執行，RPi4 透過區域網路 HTTP 呼叫

啟動：
    python src/llm_server/main.py
    python src/llm_server/main.py --model gemma4:e4b-q4_k_m
    python src/llm_server/main.py --port 8000 --host 0.0.0.0
"""

import argparse
import socket
import time
import urllib.error
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from llm_client import build_user_prompt, call_ollama, call_ollama_stream, parse_json_response
from models import ChatRequest, ChatResponse

# ── 全域狀態 ──────────────────────────────────────────────────────────────────

START_TIME = time.time()
REQUEST_LOG: deque = deque(maxlen=20)   # 最近 20 筆請求紀錄
STATS = {"total": 0, "success": 0, "failed": 0, "total_latency_ms": 0.0}
LAST_REQUEST_AT: Optional[str] = None

MODEL = "qwen2.5:3b"              # 預設模型（可 --model 覆蓋）
PORT = 8000
HOST = "0.0.0.0"

# ── 離線降級預設回應 ───────────────────────────────────────────────────────────

FALLBACK: dict[str, dict] = {
    "vip": {
        "intent": "NAVIGATION",
        "action": "navigate_to_vip_lounge",
        "target": "VIP貴賓室",
        "reply": "歡迎光臨！請問今天需要什麼協助？",
    },
    "blacklist": {
        "intent": "COMMAND",
        "action": "alert_staff",
        "target": None,
        "reply": "",
    },
    "visitor": {
        "intent": "CHAT",
        "action": None,
        "target": None,
        "reply": "您好，歡迎光臨！請問需要什麼服務？",
    },
}

# ── FastAPI App ────────────────────────────────────────────────────────────────

app = FastAPI(title="智慧銀行 LLM API", version="1.0.0")

STATIC_DIR = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


@app.get("/")
def status_page():
    return FileResponse(str(STATIC_DIR / "index.html"))


@app.get("/chat")
def chat_page():
    return FileResponse(str(STATIC_DIR / "chat.html"))


@app.get("/health")
def health():
    return {"status": "online"}


@app.get("/api/status")
def api_status():
    uptime = time.time() - START_TIME
    avg_latency = (
        STATS["total_latency_ms"] / STATS["success"]
        if STATS["success"] > 0 else 0.0
    )
    try:
        server_ip = socket.gethostbyname(socket.gethostname())
    except Exception:
        server_ip = "127.0.0.1"

    return {
        "status": "online",
        "model": MODEL,
        "uptime_seconds": round(uptime, 1),
        "requests": {
            "total": STATS["total"],
            "success": STATS["success"],
            "failed": STATS["failed"],
            "avg_latency_ms": round(avg_latency, 1),
        },
        "recent_requests": list(REQUEST_LOG),
        "last_request_at": LAST_REQUEST_AT,
        "server_ip": server_ip,
        "server_port": PORT,
    }


@app.post("/api/chat", response_model=ChatResponse)
def api_chat(req: ChatRequest):
    global LAST_REQUEST_AT
    STATS["total"] += 1
    LAST_REQUEST_AT = datetime.now().isoformat(timespec="seconds")
    t0 = time.time()

    prompt = build_user_prompt(req)
    log_entry = {
        "time": LAST_REQUEST_AT,
        "face": req.face_recognition,
        "name": req.person_name,
        "latency_ms": None,
        "thinking_ms": None,
        "ok": False,
    }

    t_thinking_start = time.time()
    try:
        raw, thinking = call_ollama(prompt, MODEL)
        thinking_ms = round((time.time() - t_thinking_start) * 1000, 1)
        data = parse_json_response(raw) if raw else None

        if data is None:
            raise ValueError("LLM 回應無法解析為 JSON")

        elapsed_ms = round((time.time() - t0) * 1000, 1)
        STATS["success"] += 1
        STATS["total_latency_ms"] += elapsed_ms
        log_entry["latency_ms"] = elapsed_ms
        log_entry["thinking_ms"] = thinking_ms
        log_entry["thinking"] = thinking
        log_entry["ok"] = True
        log_entry["intent"] = data.get("intent", "unknown")
        REQUEST_LOG.appendleft(log_entry)

        return ChatResponse(
            intent=data.get("intent", "CHAT"),
            action=data.get("action"),
            target=data.get("target"),
            reply=data.get("reply", ""),
            thinking=thinking,
        )

    except (urllib.error.URLError, ConnectionRefusedError) as e:
        # Ollama 未啟動 → 降級回應
        STATS["failed"] += 1
        log_entry["latency_ms"] = round((time.time() - t0) * 1000, 1)
        log_entry["error"] = "ollama_offline"
        REQUEST_LOG.appendleft(log_entry)

        fb = FALLBACK.get(req.face_recognition, FALLBACK["visitor"])
        return ChatResponse(
            intent=fb["intent"],
            action=fb["action"],
            target=fb["target"],
            reply=fb["reply"],
        )

    except Exception as e:
        STATS["failed"] += 1
        log_entry["latency_ms"] = round((time.time() - t0) * 1000, 1)
        log_entry["error"] = str(e)[:80]
        REQUEST_LOG.appendleft(log_entry)
        raise HTTPException(status_code=500, detail=str(e))


# ── 串流端點 ──────────────────────────────────────────────────────────────────

@app.post("/api/chat/stream")
def api_chat_stream(req: ChatRequest):
    prompt = build_user_prompt(req)

    def generate():
        import json as _json
        t0 = time.time()
        t_thinking_end = None
        full_content: list[str] = []

        try:
            for event_type, text in call_ollama_stream(prompt, MODEL):
                if event_type == "thinking" and t_thinking_end is None:
                    pass
                elif event_type == "content" and t_thinking_end is None:
                    t_thinking_end = time.time()
                full_content.append(text if event_type == "content" else "")
                yield f"data: {_json.dumps({'type': event_type, 'text': text}, ensure_ascii=False)}\n\n"

            elapsed_ms = round((time.time() - t0) * 1000, 1)
            thinking_ms = round((t_thinking_end - t0) * 1000, 1) if t_thinking_end else elapsed_ms
            content_str = "".join(full_content)
            result = parse_json_response(content_str) or {}
            done = {
                "type": "done",
                "elapsed_ms": elapsed_ms,
                "thinking_ms": thinking_ms,
                "result": result,
            }
            yield f"data: {_json.dumps(done, ensure_ascii=False)}\n\n"
        except Exception as e:
            yield f"data: {_json.dumps({'type': 'error', 'message': str(e)})}\n\n"

    return StreamingResponse(
        generate(),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


# ── 主程式 ────────────────────────────────────────────────────────────────────

def main():
    global MODEL, PORT, HOST
    parser = argparse.ArgumentParser(description="智慧銀行 LLM API Server")
    parser.add_argument("--model", default=MODEL, help="Ollama 模型名稱")
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--host", default=HOST)
    args = parser.parse_args()

    MODEL = args.model
    PORT = args.port
    HOST = args.host

    print(f"[LLM Server] 模型: {MODEL}")
    print(f"[LLM Server] 監聽: http://{HOST}:{PORT}")
    print(f"[LLM Server] 狀態頁: http://localhost:{PORT}/")
    uvicorn.run(app, host=HOST, port=PORT, log_level="warning")


if __name__ == "__main__":
    main()
