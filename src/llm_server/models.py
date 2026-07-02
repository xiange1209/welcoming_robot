from typing import Optional
from pydantic import BaseModel, Field


class ChatRequest(BaseModel):
    face_recognition: str = Field(..., description="vip / blacklist / visitor")
    person_name: Optional[str] = None
    vip_level: Optional[str] = None       # standard / gold / platinum
    risk_level: Optional[str] = None      # low / medium / high
    gender: Optional[str] = None          # M / F / Other
    customer_input: str = ""


class ChatResponse(BaseModel):
    intent: str                    # NAVIGATION | COMMAND | CHAT
    action: Optional[str] = None
    target: Optional[str] = None
    reply: str
    thinking: Optional[str] = None


class StatusResponse(BaseModel):
    status: str
    model: str
    uptime_seconds: float
    requests: dict
    last_request_at: Optional[str]
    server_ip: str
    server_port: int
