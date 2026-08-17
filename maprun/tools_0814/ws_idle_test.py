#!/usr/bin/env python3
"""
驗證 HMI 的「踢掉死連線 / 不誤踢活連線」行為。
對 **正在跑的** hmi_server (127.0.0.1:8080) 測試，不重啟服務、不讓車子動。

Test A 殭屍：手刻 WS 握手後完全不回 pong（模擬平板走出 WiFi）。
             期望：伺服器在 ws_ping_interval(25)+ws_ping_timeout(20) ≈ 45 秒內主動關閉。
Test B 活著但閒置：用 websockets 函式庫（網路層自動回 pong），靜靜掛 100 秒不送任何東西。
             期望：**不可以**被踢掉（平板放著沒操作的情境）。
"""
import asyncio, base64, os, socket, ssl, sys, time

HOST, PORT = "127.0.0.1", 8080
T0 = time.monotonic()

def ts():
    return f"[{time.monotonic()-T0:6.1f}s]"

def log(*a):
    print(ts(), *a, flush=True)

# ---------------- Test A: 殭屍連線 ----------------
def zombie(duration=100):
    key = base64.b64encode(os.urandom(16)).decode()
    s = socket.create_connection((HOST, PORT), timeout=10)
    req = (
        f"GET /ws HTTP/1.1\r\nHost: {HOST}:{PORT}\r\n"
        "Upgrade: websocket\r\nConnection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
    )
    s.sendall(req.encode())
    s.settimeout(5)
    try:
        resp = s.recv(4096)
    except Exception as e:
        log("A 殭屍: 握手沒有回應", e); return
    first = resp.split(b"\r\n")[0].decode(errors="replace")
    log("A 殭屍: 握手回應 =", first)
    if b"101" not in resp.split(b"\r\n")[0]:
        log("A 殭屍: 升級失敗，中止"); s.close(); return
    log("A 殭屍: 連線建立，從現在起【完全不回 pong】")
    # 只讀取到「連線被對方關閉」為止；絕不送任何 frame（包含 pong）
    s.settimeout(duration)
    closed_at = None
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        try:
            s.settimeout(max(1, deadline - time.monotonic()))
            data = s.recv(65536)
        except socket.timeout:
            continue
        except OSError as e:
            closed_at = time.monotonic() - T0
            log(f"A 殭屍: socket 錯誤 {e} → 視為被伺服器關閉"); break
        if not data:
            closed_at = time.monotonic() - T0
            log("A 殭屍: 收到 EOF → **伺服器主動關閉了這條死連線**"); break
        # 檢查是不是 close frame (opcode 0x8)
        if len(data) >= 1 and (data[0] & 0x0F) == 0x8:
            closed_at = time.monotonic() - T0
            log("A 殭屍: 收到 WS close frame → **伺服器主動關閉**"); break
    s.close()
    if closed_at is None:
        log(f"A 殭屍: ★失敗★ 掛了 {duration} 秒仍未被踢掉")
    else:
        log(f"A 殭屍: ★通過★ 被踢掉，耗時 {closed_at:.1f} 秒")

# ---------------- Test B: 活著但閒置 ----------------
async def alive(duration=100):
    import websockets
    uri = f"ws://{HOST}:{PORT}/ws"
    try:
        async with websockets.connect(uri, ping_interval=None, open_timeout=10) as ws:
            log("B 活著: 連線建立，靜置不送任何訊息（函式庫會自動回 pong）")
            got = 0
            end = time.monotonic() + duration
            while time.monotonic() < end:
                try:
                    await asyncio.wait_for(ws.recv(), timeout=5)
                    got += 1
                except asyncio.TimeoutError:
                    pass
                except Exception as e:
                    log(f"B 活著: ★失敗★ 連線在 {time.monotonic()-T0:.1f}s 被關閉: {e!r}")
                    return
            log(f"B 活著: ★通過★ 靜置 {duration} 秒仍在線（期間收到 {got} 則狀態推播）")
    except Exception as e:
        log("B 活著: 連線失敗", repr(e))

async def main():
    dur = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    loop = asyncio.get_running_loop()
    log(f"開始，預計 {dur} 秒。ws_ping_interval=25 ws_ping_timeout=20 → 殭屍應在 ~45 秒內被踢")
    await asyncio.gather(
        loop.run_in_executor(None, zombie, dur),
        alive(dur),
    )
    log("測試結束")

asyncio.run(main())
