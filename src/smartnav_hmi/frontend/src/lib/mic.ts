import { createStore } from "./store";
import { toast } from "./toast";

/**
 * 平板麥克風 -> 機器人（★ 2026-11-03，語音輸入＝browser）。
 *
 * 平板只負責「錄」：轉成 16 kHz、單聲道、16-bit PCM，每 100 ms 從 /ws/mic 送一塊給機器人。
 * 斷句（VAD）與辨識全部在機器人上（voice_trigger -> speech_recognizer），
 * ★ 刻意不用瀏覽器內建的語音辨識：Chrome／Safari 會把錄音送到 Google／Apple 雲端，
 *   銀行迎賓場景不允許客人的聲音離開現場（使用者決定）。
 *
 * ★ 瀏覽器只在 https（或 localhost）開放麥克風。用 http://IP:8080 連進來時直接報錯並給 https 網址。
 */

export type MicStatus = "idle" | "connecting" | "listening" | "error";

export interface MicState {
  on: boolean;
  status: MicStatus;
  error: string;
  /** 0~1 的音量，給畫面上的音量條用 */
  level: number;
}

export const micStore = createStore<MicState>({
  on: false,
  status: "idle",
  error: "",
  level: 0,
});

export const micSupported =
  typeof navigator !== "undefined" && !!navigator.mediaDevices?.getUserMedia;

const TARGET_RATE = 16000;
const SEND_SAMPLES = 1600; // 100 ms

/** 目前這頁的 https 版網址（給「請改用 https」的提示用）。port 未知時回空字串 */
export function httpsUrl(httpsPort: number): string {
  if (!httpsPort) return "";
  return `https://${location.hostname}:${httpsPort}${location.pathname}`;
}

let ws: WebSocket | null = null;
let stream: MediaStream | null = null;
let ctx: AudioContext | null = null;
let node: AudioNode | null = null;
let source: MediaStreamAudioSourceNode | null = null;
let retryTimer: number | undefined;
let pending = new Int16Array(SEND_SAMPLES);
let pendingLen = 0;
// 線性內插重新取樣的狀態（跨區塊延續，避免每塊開頭喀一聲）
let resamplePos = 0;
let lastSample = 0;
let levelAt = 0;

/** ★ 本機（平板送出前）的波形：每 32 ms 一個峰值，留最近 375 點（約 12 秒），語音頁畫圖用 */
export const WAVE_POINTS = 375;
export const localWave = { data: new Float32Array(WAVE_POINTS), head: 0, count: 0, version: 0 };
let binPeak = 0;
let binCount = 0;

/** 清掉本機波形（切換語音輸入、重新開始錄音時） */
export function clearLocalWave(): void {
  localWave.data.fill(0);
  localWave.head = 0;
  localWave.count = 0;
  localWave.version++;
}

function pushWave(peak: number): void {
  localWave.data[localWave.head] = peak;
  localWave.head = (localWave.head + 1) % WAVE_POINTS;
  localWave.count = Math.min(WAVE_POINTS, localWave.count + 1);
  localWave.version++;
}

function fail(message: string): void {
  stopInternal();
  toast.error(message);
  micStore.set((s) => ({ ...s, on: false, status: "error", error: message, level: 0 }));
}

function pushSamples(input: Float32Array, rate: number): void {
  // 音量（RMS），10 Hz 更新就夠
  const now = performance.now();
  if (now - levelAt > 100) {
    levelAt = now;
    let sum = 0;
    for (let i = 0; i < input.length; i++) sum += input[i] * input[i];
    const rms = Math.sqrt(sum / Math.max(1, input.length));
    micStore.set((s) => ({ ...s, level: Math.min(1, rms * 4) }));
  }

  const step = rate / TARGET_RATE;
  // resamplePos 是相對於「上一塊最後一個樣本」的位置：-1 對應 lastSample，0 對應 input[0]
  let pos = resamplePos;
  while (pos < input.length - 1) {
    const i = Math.floor(pos);
    const frac = pos - i;
    const a = i < 0 ? lastSample : input[i];
    const b = input[i + 1];
    const v = a + (b - a) * frac;
    pending[pendingLen++] = Math.max(-32768, Math.min(32767, Math.round(v * 32767)));
    const av = v < 0 ? -v : v;
    if (av > binPeak) binPeak = av;
    if (++binCount === 512) {
      pushWave(Math.min(1, binPeak));
      binPeak = 0;
      binCount = 0;
    }
    if (pendingLen === SEND_SAMPLES) {
      if (ws && ws.readyState === WebSocket.OPEN) ws.send(pending.buffer);
      pending = new Int16Array(SEND_SAMPLES);
      pendingLen = 0;
    }
    pos += step;
  }
  resamplePos = pos - input.length;
  lastSample = input[input.length - 1] ?? lastSample;
}

const WORKLET = `
class SmartnavMicTap extends AudioWorkletProcessor {
  process(inputs) {
    const ch = inputs[0] && inputs[0][0];
    if (ch) this.port.postMessage(ch.slice(0));
    return true;
  }
}
registerProcessor("smartnav-mic-tap", SmartnavMicTap);
`;

async function buildGraph(ms: MediaStream): Promise<void> {
  // 能直接開 16 kHz 就讓瀏覽器自己重新取樣（Chrome 支援）；不行就用裝置預設，自己內插
  try {
    ctx = new AudioContext({ sampleRate: TARGET_RATE });
  } catch {
    ctx = new AudioContext();
  }
  await ctx.resume();
  source = ctx.createMediaStreamSource(ms);
  const rate = ctx.sampleRate;
  if (ctx.audioWorklet) {
    const url = URL.createObjectURL(new Blob([WORKLET], { type: "application/javascript" }));
    try {
      await ctx.audioWorklet.addModule(url);
    } finally {
      URL.revokeObjectURL(url);
    }
    const tap = new AudioWorkletNode(ctx, "smartnav-mic-tap");
    tap.port.onmessage = (e: MessageEvent<Float32Array>) => pushSamples(e.data, rate);
    source.connect(tap);
    node = tap;
  } else {
    // 舊版 iOS Safari 沒有 AudioWorklet：退回 ScriptProcessor（已棄用但到處都能用）
    const sp = ctx.createScriptProcessor(2048, 1, 1);
    sp.onaudioprocess = (e) => pushSamples(e.inputBuffer.getChannelData(0), rate);
    source.connect(sp);
    sp.connect(ctx.destination); // ScriptProcessor 要接到輸出才會被驅動（輸出是靜音）
    node = sp;
  }
}

function openSocket(): void {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const sock = new WebSocket(`${proto}://${location.host}/ws/mic`);
  ws = sock;
  sock.onopen = () => {
    if (ws !== sock) return;
    micStore.set((s) => ({ ...s, status: "listening", error: "" }));
  };
  sock.onclose = (e) => {
    if (ws !== sock) return;
    ws = null;
    if (!micStore.get().on) return;
    if (e.code === 4003) {
      fail("機器人目前的語音輸入不是「平板錄音」（到語音頁切換）");
    } else if (e.code === 4001) {
      fail("另一台平板開始錄音，這台已停止");
    } else {
      // 網路抖動：保留麥克風，2 秒後重連
      micStore.set((s) => ({ ...s, status: "connecting" }));
      retryTimer = window.setTimeout(() => {
        if (micStore.get().on) openSocket();
      }, 2000);
    }
  };
}

/** 開始錄音（必須在使用者點擊裡呼叫：瀏覽器要求手勢才能開麥克風與 AudioContext） */
export async function startMic(httpsPort: number): Promise<void> {
  if (micStore.get().on) return;
  if (!window.isSecureContext) {
    const url = httpsUrl(httpsPort);
    fail(url ? `平板錄音需要用 https 開啟：${url}` : "平板錄音需要用 https 開啟這個頁面");
    return;
  }
  if (!micSupported) {
    fail("這個瀏覽器不支援麥克風錄音");
    return;
  }
  micStore.set({ on: true, status: "connecting", error: "", level: 0 });
  clearLocalWave();
  try {
    stream = await navigator.mediaDevices.getUserMedia({
      audio: {
        channelCount: 1,
        // 回音消除：平板自己在唸（語音輸出＝平板）時，盡量別把自己的聲音錄進去。
        // 機器人那端另外有「TTS 起訖強制切句」與文字層回音過濾當後備。
        echoCancellation: true,
        noiseSuppression: true,
        autoGainControl: true,
      },
    });
    if (!micStore.get().on) {
      stopInternal();
      return;
    }
    resamplePos = 0;
    lastSample = 0;
    pendingLen = 0;
    await buildGraph(stream);
    openSocket();
  } catch (e) {
    const err = e as DOMException;
    fail(
      err?.name === "NotAllowedError"
        ? "麥克風權限被拒絕（到瀏覽器的網站設定允許麥克風）"
        : `麥克風開啟失敗：${err?.message || String(e)}`,
    );
  }
}

function stopInternal(): void {
  window.clearTimeout(retryTimer);
  const sock = ws;
  ws = null;
  try {
    sock?.close();
  } catch {
    /* 已經關了 */
  }
  try {
    source?.disconnect();
    node?.disconnect();
  } catch {
    /* 圖已經拆掉 */
  }
  source = null;
  node = null;
  stream?.getTracks().forEach((t) => t.stop());
  stream = null;
  void ctx?.close().catch(() => {});
  ctx = null;
}

export function stopMic(): void {
  stopInternal();
  micStore.set({ on: false, status: "idle", error: "", level: 0 });
}
