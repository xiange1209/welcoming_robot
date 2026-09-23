import { authHeaders } from "./session";
import { createStore } from "./store";

/* ══════════════════════════════════════════════════════════════
   遙控

   設計原則：**車子的移動由「持續收到指令」維持，而不是由狀態維持。**

   前端每 100 ms 送一次目前的方向；放開按鍵就不再送。後端看門狗超過
   0.6 秒沒收到就自動歸零。這樣「連線中斷」和「使用者放開手」對車子
   來說是同一件事——都會停。

   如果反過來做成「按下送 start、放開送 stop」，那則 stop 只要沒送達
   （手機遙控很常見），車子就會一直跑下去。

   整支刻意不放進 React state：方向鍵是 10 Hz 的迴圈，走 state 等於每秒
   重建十次元件樹，而且 setState 的非同步性會讓「放開」慢一拍——
   那一拍是車子多跑的距離。
   ══════════════════════════════════════════════════════════════ */

const SEND_INTERVAL_MS = 100; // 後端 20 Hz 補送，這裡只要跟得上手指
const MAX_LINEAR = 0.18; // m/s，後端上限
const MAX_ANGULAR = 0.45; // rad/s，後端上限

export interface TeleopState {
  status: string;
  moving: boolean;
  /** 0~1，對應到後端上限 */
  speed: number;
}

export const teleopStore = createStore<TeleopState>({
  status: "未連線",
  moving: false,
  speed: 0.6,
});

const setStatus = (status: string, moving = teleopStore.get().moving) =>
  teleopStore.set((prev) => ({ ...prev, status, moving }));

export function setTeleopSpeed(speed: number): void {
  teleopStore.set((prev) => ({ ...prev, speed }));
}

/** 目前按著的所有輸入來源。key 是 pointerId 或鍵名。
 *
 *  用集合而不是單一向量，是因為「按著▲再加按◀」在阿克曼車上是最常用的
 *  動作——單一向量會被後按的那顆整個覆寫掉，變成 lin=0 的純轉向
 *  （車子不會動），而且先放開其中一顆就會整台停下。 */
const held = new Map<string, { lin: number; ang: number }>();

let timer: ReturnType<typeof setInterval> | null = null;
let inFlight = false; // 背壓：上一則還沒回來就不要再送

function vector(): { lin: number; ang: number } {
  let lin = 0;
  let ang = 0;
  for (const v of held.values()) {
    lin += v.lin;
    ang += v.ang;
  }
  // 多顆同向鍵一起按不應該疊成兩倍速
  return { lin: Math.max(-1, Math.min(1, lin)), ang: Math.max(-1, Math.min(1, ang)) };
}

function send(): void {
  /* in-flight 保護：Pi 4 忙起來時一則 POST 可能超過 100 ms，沒有這道閘門
     請求會在瀏覽器連線池裡無上限堆積，延遲隨時間單調增長，放手後那些
     排隊中的舊指令還會繼續餵飽底盤的 1 秒逾時——車子不會停。 */
  if (inFlight) return;

  const v = vector();
  const speed = teleopStore.get().speed;
  inFlight = true;

  fetch("/api/teleop", {
    method: "POST",
    /* 遙控端點掛了 admin_only，沒帶權杖一律回 401。
       而 fetch 只有在網路層失敗才 reject，401 對它來說是「成功完成」——
       少了這個標頭，指令會被靜默丟棄且畫面上不會有任何錯誤。 */
    headers: authHeaders({ "Content-Type": "application/json" }),
    body: JSON.stringify({ linear: v.lin * speed * MAX_LINEAR, angular: v.ang * speed * MAX_ANGULAR }),
  })
    .then((res) => {
      if (res.ok) return;
      halt();
      setStatus(
        res.status === 401
          ? "遙控需要管理者登入（權杖已失效，請重新登入）"
          : `指令被拒絕：HTTP ${res.status}`,
        false,
      );
    })
    .catch(() => {
      // 送不出去就直接停止本地心跳；車子那端看門狗也會停
      halt();
      setStatus("連線中斷，已停車", false);
    })
    .finally(() => {
      inFlight = false;
    });
}

function refresh(): void {
  if (held.size === 0) {
    stop();
    return;
  }
  if (timer === null) {
    send(); // 立刻送一次，不要等第一個間隔
    timer = setInterval(send, SEND_INTERVAL_MS);
  } else {
    send(); // 方向變了就立刻反映，不要等下一個間隔
  }

  const v = vector();
  const dir = v.lin > 0 ? "前進" : v.lin < 0 ? "後退" : "—";
  const turn = v.ang > 0 ? "左" : v.ang < 0 ? "右" : "—";
  setStatus(
    `移動中　前後 ${dir}　轉向 ${turn}` +
      (v.lin === 0 && v.ang !== 0 ? "　⚠ 阿克曼車無法原地轉，請同時按前進或後退" : ""),
    true,
  );
}

/** 只停本地心跳，不送停車指令（送不出去的情境下由後端看門狗收尾） */
function halt(): void {
  if (timer !== null) {
    clearInterval(timer);
    timer = null;
  }
  held.clear();
  teleopStore.set((prev) => (prev.moving ? { ...prev, moving: false } : prev));
  notifyHeld();
}

export function press(key: string, lin: number, ang: number): void {
  held.set(key, { lin, ang });
  notifyHeld();
  refresh();
}

export function release(key: string): void {
  if (!held.delete(key)) return;
  notifyHeld();
  refresh();
}

export function stop(): void {
  const wasMoving = timer !== null || held.size > 0;
  halt();
  /* keepalive：頁面正在關閉或被隱藏時，一般的 fetch 會被瀏覽器直接取消，
     停車指令就送不出去。這個旗標讓請求脫離頁面生命週期繼續送完。
     即使它仍失敗，後端 0.6 秒看門狗還是會停車——這是第二道保險不是唯一一道。 */
  fetch("/api/teleop/stop", {
    method: "POST",
    headers: authHeaders(),
    keepalive: true,
  }).catch(() => {});
  if (wasMoving) setStatus("已停止", false);
}

/* ── 按鈕的「按著」高亮 ────────────────────────────────────
   這一份跟 held 是同一件事，但只給畫面用。分開是因為 held 每秒被讀十次，
   而高亮只在手指按下／放開時變——用同一個 store 會讓方向鍵跟著心跳重繪。 */
export const heldKeysStore = createStore<ReadonlySet<string>>(new Set());

function notifyHeld(): void {
  heldKeysStore.set(new Set(held.keys()));
}

/* ── 全域安全網 ────────────────────────────────────────── */
let guardsInstalled = false;

export function installTeleopGuards(): void {
  if (guardsInstalled) return;
  guardsInstalled = true;

  /* 視窗失焦一定要停車。
     alt-tab、點到別的視窗、跳出系統對話框都**不會**觸發 visibilitychange，
     但會讓 keyup 整個遺失——鍵還「按著」，計時器繼續送，車子就暴衝出去。
     這是鍵盤遙控最容易出事的一種情況。 */
  window.addEventListener("blur", () => {
    if (held.size > 0 || timer !== null) stop();
  });

  // 螢幕關掉／切到別的 App 一定要停車。
  // 後端看門狗本來就會在 0.6 秒內停，這裡只是讓它更即時。
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) stop();
  });

  // 分頁被關掉時最後再送一次（keepalive 讓它能在卸載後送完）
  window.addEventListener("pagehide", () => {
    if (held.size > 0 || timer !== null) stop();
  });
}
