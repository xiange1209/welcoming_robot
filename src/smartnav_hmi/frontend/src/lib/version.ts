import { createStore } from "./store";
import { teleopStore } from "./teleop";
import { ttsStore } from "./tts";

/**
 * 前端版本檢查。
 *
 * iPad 用 Chrome「加入主畫面」開成獨立 App，切走再切回來只是恢復、不會重新載入，
 * 載入時的舊 JS 會一直跑。2026-10-22：朗讀修正已部署，平板還在跑 10/20 載入的舊版，
 * 把「165」唸成一百六十五——症狀看起來像「修正沒生效」，其實是畫面根本沒換新。
 *
 * 做法：後端 /api/version 回報目前部署的 index.html 引用了哪些 /assets/ 檔案
 * （Vite 把內容雜湊放在檔名裡，內容一變檔名就變），這裡拿來和自己載入時的那一組比。
 * 不一樣 → 顯示「有新版本」橫幅，並且：
 *   - App 從背景回到前景（visibilitychange／pageshow）時發現 → 立刻重載。這時沒有人正在操作
 *     （遙控在切到背景時就被 installTeleopGuards 停掉了），是換版最不打擾的時刻。
 *     這個做法是車上 agent 10/22 在 hmi_ipad.patch 的 version.ts 想出來的，併進這一份。
 *   - 一直開在前景：閒置一段時間且沒在遙控、沒在朗讀才自動重載（可能有人正在打字）。
 *
 * ★ 第一次部署這個檢查時，平板上還在跑的舊 JS 沒有它，要完全關掉 App 再開一次；
 *   之後的更新才會自動提示。
 */

export const versionStore = createStore<{ stale: boolean }>({ stale: false });

const POLL_MS = 60_000;
/** 連續這麼久沒碰畫面才自動重載。遙控中、朗讀中一律不重載 */
const IDLE_RELOAD_MS = 45_000;
const RELOAD_MARK = "versionReloadedFor";

/** 取檔名（不含路徑與查詢字串）；不在 /assets/ 底下回空字串。★ 要和後端 core/version.py 的取法一致 */
function assetName(url: string | null): string {
  if (!url) return "";
  const path = url.split("?")[0].split("#")[0];
  return path.includes("/assets/") ? path.slice(path.lastIndexOf("/") + 1) : "";
}

/** 這個頁面載入時用的那一組檔案。用屬性原值而不是 .src，避免被補成絕對網址 */
export function ownSignature(): string {
  const names = new Set<string>();
  document.querySelectorAll('script[type="module"][src]').forEach((el) => {
    const n = assetName(el.getAttribute("src"));
    if (n) names.add(n);
  });
  document.querySelectorAll('link[rel="stylesheet"][href]').forEach((el) => {
    const n = assetName(el.getAttribute("href"));
    if (n) names.add(n);
  });
  return [...names].sort().join("|");
}

let serverSig = "";
let lastActivity = Date.now();

async function check(): Promise<void> {
  try {
    const res = await fetch("/api/version", { cache: "no-store" });
    if (!res.ok) return;
    const { sig } = (await res.json()) as { sig?: string };
    const own = ownSignature();
    // 任一邊讀不到（開發模式、舊後端）就當作不知道，不提示
    if (!sig || !own) return;
    serverSig = sig;
    const stale = sig !== own;
    if (versionStore.get().stale !== stale) versionStore.set({ stale });
  } catch {
    // 後端重啟中或網路斷了：下次再問。背景檢查失敗不該有任何畫面反應
  }
}

/** force＝剛從背景回來（沒有人在操作），不必等閒置 */
function reloadIfAllowed(force: boolean): void {
  if (!versionStore.get().stale || document.hidden) return;
  if (!force && Date.now() - lastActivity < IDLE_RELOAD_MS) return;
  if (teleopStore.get().moving || ttsStore.get().status === "speaking") return;
  /* 重載一次之後簽章還是對不起來（例如部署到一半），不要再自動重載——
     否則每閒置 45 秒就重載一次。同一個版本只自動重載一次；橫幅仍在，可以手動按。 */
  try {
    if (sessionStorage.getItem(RELOAD_MARK) === serverSig) return;
    sessionStorage.setItem(RELOAD_MARK, serverSig);
  } catch {
    return; // sessionStorage 不能用（私密模式）：寧可不自動重載
  }
  location.reload();
}

let started = false;

export function startVersionWatch(): void {
  if (started) return;
  started = true;

  const bump = () => {
    lastActivity = Date.now();
  };
  // pointermove 要算：長按搖桿拖曳時只有第一下 pointerdown，後面都是 pointermove
  for (const ev of ["pointerdown", "pointermove", "keydown", "touchstart", "touchmove", "wheel"] as const) {
    document.addEventListener(ev, bump, { capture: true, passive: true });
  }

  void check();
  // 只在畫面看得到時才問，和其他背景輪詢（useVisibleInterval）一致
  setInterval(() => {
    if (!document.hidden) void check();
  }, POLL_MS);
  // 主畫面 App 從背景回來正是最容易遇到舊版的時刻：立刻問一次，有新版就直接重載
  const onResume = () => {
    bump();
    void check().then(() => reloadIfAllowed(true));
  };
  document.addEventListener("visibilitychange", () => {
    if (!document.hidden) onResume();
  });
  // 從瀏覽器的返回／前進快取（bfcache）還原時不會重新執行腳本，只會觸發 pageshow
  window.addEventListener("pageshow", (e) => {
    if (e.persisted) onResume();
  });
  setInterval(() => reloadIfAllowed(false), 5_000);
}
