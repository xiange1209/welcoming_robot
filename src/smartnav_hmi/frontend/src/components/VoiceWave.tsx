import { useEffect, useRef, useState } from "react";
import { clearLocalWave, localWave, WAVE_POINTS } from "../lib/mic";

/**
 * 語音輸入波形（★ 2026-11-03，語音頁測試用）。兩條：
 *   上：平板送出前（只有「平板錄音」且這台在錄時才有）—— 看平板麥克風本身有沒有收到聲音
 *   下：機器人收到後（/ws/voice_monitor，voice_trigger 實際處理的那份）—— 兩種輸入來源都會經過
 *       綠色＝VAD 判定有人在講、藍底＝正在收這一句（收完才送辨識）、灰色＝只是噪音
 * 每 32 ms 一個點、約 12 秒，最新的在右邊。
 */

interface RobotPoint {
  peak: number;
  vad: number;
  cmd: number;
}

function cssColor(name: string, fallback: string): string {
  const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  return v || fallback;
}

export function VoiceWave({ showLocal, resetKey }: { showLocal: boolean; resetKey: string }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const robot = useRef<RobotPoint[]>([]);
  const robotAt = useRef(0);
  const [robotStale, setRobotStale] = useState(true);

  // ★ 換了收音來源就清掉舊波形（後端的暫存也會一起清，見 hmi_server_node.set_voice_modes）。
  //   兩種來源的音量基準不同，舊的留在圖上會誤以為新來源已經有聲音。
  const firstKey = useRef(resetKey);
  useEffect(() => {
    if (resetKey === firstKey.current) return;
    firstKey.current = resetKey;
    robot.current = [];
    robotAt.current = 0;
    clearLocalWave();
  }, [resetKey]);

  // 機器人端：只在這個元件掛著時連線
  useEffect(() => {
    let ws: WebSocket | null = null;
    let retry: number | undefined;
    let alive = true;
    const connect = () => {
      const proto = location.protocol === "https:" ? "wss" : "ws";
      ws = new WebSocket(`${proto}://${location.host}/ws/voice_monitor`);
      ws.onmessage = (e) => {
        const pts = (JSON.parse(e.data as string) as { pts: number[][] }).pts;
        const buf = robot.current;
        for (const p of pts) buf.push({ peak: p[1], vad: p[2], cmd: p[3] });
        if (buf.length > WAVE_POINTS) buf.splice(0, buf.length - WAVE_POINTS);
        robotAt.current = performance.now();
      };
      ws.onclose = () => {
        if (alive) retry = window.setTimeout(connect, 2000);
      };
    };
    connect();
    const stale = window.setInterval(
      () => setRobotStale(performance.now() - robotAt.current > 2000),
      500,
    );
    return () => {
      alive = false;
      window.clearTimeout(retry);
      window.clearInterval(stale);
      ws?.close();
    };
  }, []);

  // 畫圖：rAF，分頁隱藏時瀏覽器自己會停
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    let raf = 0;
    const draw = () => {
      raf = requestAnimationFrame(draw);
      const dpr = window.devicePixelRatio || 1;
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
      if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
        canvas.width = Math.round(w * dpr);
        canvas.height = Math.round(h * dpr);
      }
      const g = canvas.getContext("2d");
      if (!g) return;
      g.setTransform(dpr, 0, 0, dpr, 0, 0);
      g.clearRect(0, 0, w, h);

      const lanes = showLocal ? 2 : 1;
      const laneH = h / lanes;
      const step = w / WAVE_POINTS;
      const green = cssColor("--color-green", "#30d158");
      const blue = cssColor("--color-blue", "#0a84ff");
      const grey = cssColor("--color-label-3", "rgba(255,255,255,0.38)");
      const label = cssColor("--color-label-2", "rgba(255,255,255,0.62)");
      g.font = "11px system-ui, sans-serif";

      const bar = (x: number, top: number, laneHeight: number, peak: number) => {
        // 用平方根放大小聲的部分：說話的峰值常在 0.05~0.3，線性畫幾乎看不到
        const amp = Math.sqrt(Math.min(1, peak)) * (laneHeight / 2 - 8);
        const mid = top + laneHeight / 2 + 6;
        g.fillRect(x, mid - amp, Math.max(1, step - 0.5), amp * 2 || 1);
      };

      let top = 0;
      if (showLocal) {
        const n = localWave.count;
        g.fillStyle = blue;
        for (let i = 0; i < n; i++) {
          const idx = (localWave.head - n + i + WAVE_POINTS) % WAVE_POINTS;
          bar(w - (n - i) * step, top, laneH, localWave.data[idx]);
        }
        g.fillStyle = label;
        g.fillText("平板送出", 6, top + 13);
        top += laneH;
        g.strokeStyle = grey;
        g.beginPath();
        g.moveTo(0, top);
        g.lineTo(w, top);
        g.stroke();
      }

      const pts = robot.current;
      const n = pts.length;
      // 藍底：正在收句
      g.fillStyle = blue;
      g.globalAlpha = 0.2; // 不用 hex 拼透明度：主題色可能是 oklch()
      for (let i = 0; i < n; i++) {
        if (pts[i].cmd) g.fillRect(w - (n - i) * step, top + 18, step + 0.5, laneH - 20);
      }
      g.globalAlpha = 1;
      for (let i = 0; i < n; i++) {
        g.fillStyle = pts[i].vad ? green : grey;
        bar(w - (n - i) * step, top, laneH, pts[i].peak);
      }
      g.fillStyle = label;
      g.fillText("機器人收到", 6, top + 13);
    };
    raf = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(raf);
  }, [showLocal]);

  return (
    <div className="flex flex-col gap-1.5">
      <canvas
        ref={canvasRef}
        className="h-[150px] w-full rounded-ios border border-[var(--color-outline-variant)] bg-[var(--color-well)]"
      />
      <div className="flex flex-wrap items-center gap-x-3 gap-y-1 text-[12px] text-label-3">
        <span>
          <span className="mr-1 inline-block size-2 rounded-full bg-green" />
          VAD 判定有人講話
        </span>
        <span>
          <span className="mr-1 inline-block size-2 rounded-sm bg-blue/30" />
          正在收這一句（收完送辨識）
        </span>
        <span>
          <span className="mr-1 inline-block size-2 rounded-full bg-[var(--color-label-3)]" />
          噪音／靜音
        </span>
        {robotStale && (
          <span className="text-orange">
            機器人端沒有資料：語音辨識（asr_chain）沒在跑，或還沒有聲音送進去
          </span>
        )}
      </div>
    </div>
  );
}
