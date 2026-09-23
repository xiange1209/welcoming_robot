import { createStore } from "./store";
import type { ChatMessage, Snapshot } from "./types";

export interface Telemetry {
  connected: boolean;
  snap: Snapshot;
}

const EMPTY_SNAPSHOT: Snapshot = { version: -1, now: 0 };

/**
 * 狀態推播。coalesce 讓同一個影格內的多次推播只觸發一次重繪，
 * 而分頁被隱藏時 rAF 不跑 = 完全不重繪（值仍然是最新的，見 store.ts）。
 */
export const telemetryStore = createStore<Telemetry>(
  { connected: false, snap: EMPTY_SNAPSHOT },
  { coalesce: true },
);

/* ── 人臉在不在 ────────────────────────────────────────────
   user_auth_node 只在偵測到人臉時才發布 UserIdentity，沒人時不發任何訊息，
   所以狀態會一直留著上一個人的資料——不能用 user_name 有沒有值來判斷
   （未認出時它是字串 "Unknown"，永遠非空）。只能看時戳夠不夠新。

   3.5 秒必須大於 user_auth_node 的 identity_publish_interval（預設 2.0 秒），
   否則同一個人被節流不發布的空檔會被誤判成人走了。 */
const FACE_FRESH_SEC = 3.5;

let identityAgeAtRecv = Infinity;
let identityRecvAt = 0;

/** 最後一則身分訊息裡有沒有真的人臉框 */
let faceBoxed = false;

export const presenceStore = createStore<{ present: boolean; boxed: boolean }>({
  present: false,
  boxed: false,
});

/** 只用本地經過時間外推，避免平板與機器人的時鐘差造成誤判 */
export function faceAge(): number {
  if (!Number.isFinite(identityAgeAtRecv)) return Infinity;
  return identityAgeAtRecv + (Date.now() - identityRecvAt) / 1000;
}

function recomputePresence(): void {
  const present = faceAge() < FACE_FRESH_SEC;
  const prev = presenceStore.get();
  // 布林值沒翻轉就不通知——這個判斷一秒跑兩次，但一個人來一趟只翻兩次
  if (prev.present === present && prev.boxed === faceBoxed) return;
  presenceStore.set({ present, boxed: faceBoxed });
}

/* 倒數要自己跑，不能只靠推播：鏡頭前沒人時後端不發訊息，
   沒有這個計時器橫幅就會永遠停在上一個人身上。 */
setInterval(recomputePresence, 500);

/* ── 對話 ──────────────────────────────────────────────────
   後端只在對話真的變過時才帶 messages 欄位（LLM 串流期間省下大量
   序列化）。沒帶就沿用上一份，而且**必須是同一個陣列參考**——
   每次都複製的話對話區會被判定成「變了」而每秒重建十次 DOM。 */
let cachedMessages: ChatMessage[] = [];

export function resetCachedMessages(): void {
  cachedMessages = [];
  telemetryStore.set((prev) => ({
    ...prev,
    snap: { ...prev.snap, messages: cachedMessages },
  }));
}

function ingest(raw: Snapshot): void {
  if (Array.isArray(raw.messages)) {
    cachedMessages = raw.messages;
  }

  const id = raw.identity || {};
  identityAgeAtRecv = id.updated_at && raw.now ? Math.max(0, raw.now - id.updated_at) : Infinity;
  identityRecvAt = Date.now();

  /* ★ bbox 面積是唯一能分辨「真的有人臉」與「後端主動說沒人」的欄位。
     後端在 2 秒沒人時會主動發一則 user_name="Unknown"、similarity=0 的訊息，
     那則訊息是**新鮮的**，所以單看時戳會被繞過——畫面會憑空多出一位
     不存在的訪客。無人時 user_auth_node 明確把 bbox 寫死成 [0,0,0,0]。 */
  const bb = id.bbox || [];
  faceBoxed = !(bb.length < 4 || (bb[2] - bb[0] <= 0 && bb[3] - bb[1] <= 0));
  recomputePresence();

  telemetryStore.set((prev) => ({
    connected: prev.connected,
    snap: { ...raw, messages: cachedMessages },
  }));
}

/* ── 連線 ────────────────────────────────────────────────── */
let socket: WebSocket | null = null;
let retryDelay = 500;
let retryTimer: ReturnType<typeof setTimeout> | null = null;
let started = false;

function connect(): void {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  const ws = new WebSocket(`${proto}//${location.host}/ws`);
  socket = ws;

  ws.onopen = () => {
    retryDelay = 500;
    telemetryStore.set((prev) => ({ ...prev, connected: true }));
    // 連上的那一刻要立刻反映到指示點，不要等下一個影格
    telemetryStore.flush();
  };

  ws.onmessage = (ev) => {
    try {
      ingest(JSON.parse(ev.data as string) as Snapshot);
    } catch {
      // 壞掉的單一幀丟掉就好，不要讓它把整條連線帶走
    }
  };

  ws.onclose = () => {
    // 已經被換掉的舊連線關閉時什麼都不要做：否則它會把剛接好的新連線
    // 標成「斷線」，還會再排一次重連，最後變成越重連越多條。
    if (socket !== ws) return;
    socket = null;
    telemetryStore.set((prev) => ({ ...prev, connected: false }));
    telemetryStore.flush();
    // 指數退避：機器人重開機期間前端不要狂打
    retryDelay = Math.min(8000, retryDelay * 1.7);
    retryTimer = setTimeout(connect, retryDelay);
  };

  ws.onerror = () => ws.close();
}

/** 只會真的連一次。React StrictMode 會把 effect 跑兩遍，沒有這道閘就是兩條連線 */
export function startTelemetry(): void {
  if (started) return;
  started = true;
  connect();

  // 平板從睡眠醒來時 WebSocket 常常已經死了但 onclose 還沒來，
  // 回到前景時主動探一次，比等 TCP 逾時快得多。
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) return;
    telemetryStore.flush();
    // CONNECTING 也要放過——正在握手時再開一條只是多一條要收的連線
    if (socket && (socket.readyState === WebSocket.OPEN || socket.readyState === WebSocket.CONNECTING)) {
      return;
    }
    if (retryTimer) clearTimeout(retryTimer);
    retryDelay = 500;
    connect();
  });
}
