import { createStore } from "./store";
import { telemetryStore } from "./telemetry";
import { toast } from "./toast";
import type { ChatMessage } from "./types";

/**
 * 瀏覽器朗讀。
 *
 * 車上沒有喇叭、也沒有音訊輸出鏈（唯一的音訊裝置是 Astra S 的麥克風，
 * 只能錄不能放），所以「機器人開口說話」是由看著畫面的這台平板自己唸。
 * 用瀏覽器內建的 speechSynthesis：不裝套件、不連外、離線也能唸。
 */

export const ttsSupported =
  typeof window !== "undefined" && "speechSynthesis" in window;

export type TtsStatus = "idle" | "speaking" | "error";

export interface TtsState {
  on: boolean;
  status: TtsStatus;
  /** 需要使用者點畫面一下才能發聲（見 startTts 的說明） */
  needsUnlock: boolean;
  rate: number;
  voiceURI: string;
  voices: { uri: string; name: string; lang: string; zh: boolean }[];
}

export const ttsStore = createStore<TtsState>({
  on: localStorage.getItem("tts") === "1",
  status: "idle",
  needsUnlock: false,
  rate: parseFloat(localStorage.getItem("ttsRate") || "1.05"),
  voiceURI: localStorage.getItem("ttsVoice") || "",
  voices: [],
});

/**
 * 偏好的聲音（依序試）。
 * ★ 這份清單只能「優先選用」不能「保證」——聲音是裝置作業系統提供的，
 *   不是機器人提供的，換一台平板清單就不一樣。
 *   美嘉／Meijia 是 Apple 的 zh-TW 內建音；Google 系列在 Android 上常見。
 */
const PREFERRED = [
  /美[嘉佳]/,
  /Meijia/i,
  /Google.*(國語|Chinese \(Taiwan\))/i,
  /Hanhan/i,
];

const isZh = (lang: string) => /^(zh|cmn|yue)/i.test(lang);

let voice: SpeechSynthesisVoice | null = null;

function pickVoice(): SpeechSynthesisVoice | null {
  const all = speechSynthesis.getVoices();
  const want = ttsStore.get().voiceURI;
  if (want) {
    const picked = all.find((v) => v.voiceURI === want);
    if (picked) return picked;
  }
  for (const re of PREFERRED) {
    const hit = all.find((v) => re.test(v.name) && /^(zh|cmn)/i.test(v.lang));
    if (hit) return hit;
  }
  return (
    all.find((v) => /^zh[-_]TW/i.test(v.lang)) ||
    all.find((v) => /^zh/i.test(v.lang)) ||
    null
  );
}

/** 把這台裝置有的聲音填進清單。中文排前面，其餘仍列出（有些系統把中文標成 cmn） */
function refreshVoices(): void {
  if (!ttsSupported) return;
  const all = speechSynthesis.getVoices();
  if (!all.length) return; // 還沒載入，等 onvoiceschanged 再來

  const mapped = all.map((v) => ({
    uri: v.voiceURI,
    name: v.name,
    lang: v.lang,
    zh: isZh(v.lang),
  }));
  mapped.sort((a, b) => Number(b.zh) - Number(a.zh));

  if (!voice) voice = pickVoice();
  ttsStore.set((prev) => ({
    ...prev,
    voices: mapped,
    voiceURI: prev.voiceURI || voice?.voiceURI || "",
  }));
}

/**
 * 回報 TTS 起訖，讓 ASR 在這段期間靜音。
 *
 * 在此之前 ASR 只能用字數估播放時間（0.22 秒/字），兩個壞處：
 * 100 字的回覆會讓它聾 23 秒；平板沒連線時根本沒出聲卻照樣靜音。
 * 失敗就算了——ASR 端還有字數估計當後備，不要因此擋住播報。
 */
function report(active: boolean): void {
  fetch("/api/tts_state", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ active }),
  }).catch(() => {});
}

export function speak(text: string): void {
  if (!ttsSupported || !text) return;
  if (!voice) voice = pickVoice();

  const u = new SpeechSynthesisUtterance(text);
  if (voice) u.voice = voice;
  u.lang = voice?.lang || "zh-TW";
  u.rate = ttsStore.get().rate;

  // onend 與 onerror 都要回報——只綁 onend 的話，唸到一半失敗就會卡在「播放中」
  u.onstart = () => {
    report(true);
    ttsStore.set((prev) => ({ ...prev, status: "speaking" }));
  };
  u.onend = () => {
    report(false);
    ttsStore.set((prev) => ({ ...prev, status: "idle" }));
  };
  u.onerror = (e) => {
    report(false);
    ttsStore.set((prev) => ({ ...prev, status: "error" }));
    /* ★ 沒有這個提示的話，「沒聲音」有三種原因而畫面完全分不出來：
         (1) TTS 根本沒被觸發（LLM 沒回、話題沒通）
         (2) 觸發了但合成失敗（這台裝置沒有中文語音包——iPad 上很常見）
         (3) 裝置音量鍵靜音
       會被誤判成 (1) 然後跑去查 LLM，而真因是 (2)。 */
    toast.error(
      "語音合成失敗" +
        (e?.error ? "：" + e.error : "") +
        "（這台裝置可能沒有中文語音包）",
    );
  };

  // 上一句還沒唸完就被新回覆蓋過時不要排隊，直接換
  speechSynthesis.cancel();
  speechSynthesis.speak(u);
}

export function setRate(rate: number): void {
  localStorage.setItem("ttsRate", String(rate));
  ttsStore.set((prev) => ({ ...prev, rate }));
}

export function setVoice(uri: string): void {
  localStorage.setItem("ttsVoice", uri);
  ttsStore.set((prev) => ({ ...prev, voiceURI: uri }));
  voice = pickVoice();
  speak("這是這個聲音的效果");
}

/** 切換朗讀。★ 必須從使用者手勢的呼叫堆疊裡呼叫，見 startTts */
export function toggle(): void {
  if (!ttsSupported) return;
  const next = !ttsStore.get().on;
  localStorage.setItem("tts", next ? "1" : "0");
  ttsStore.set((prev) => ({
    ...prev,
    on: next,
    needsUnlock: false,
    status: "idle",
  }));
  if (next) {
    refreshVoices();
    // 這一句就是 iOS 的那把鑰匙——它發生在使用者手勢裡
    speak("朗讀已開啟");
  } else {
    speechSynthesis.cancel();
  }
}

/* ── 只唸「新出現的、已完成的」機器人訊息 ────────────────── */
const spoken = new Set<string>();
let seeded = false;

/** ★ 去重鍵用「時間戳＋文字」而不是純文字。
 *
 *  迎賓詞是固定樣板（「{name}貴賓您好，歡迎蒞臨…」），同一位 VIP 每次的
 *  字串完全相同。用文字當鍵的話，第二次迎賓（冷卻 60 秒後走回來）會被判成
 *  「已經唸過」而靜音——錄影時反覆走進走出必定踩到，而且完全沒有徵兆可查。 */
const keyOf = (m: ChatMessage) => `${m.ts || 0}|${m.text}`;

export function onChatUpdate(items: ChatMessage[]): void {
  const done = items.filter((i) => i.role === "robot" && !i.ghost);

  /* ★ seeding 要在「對話是空的」時也完成。
     原本第一行是「沒有已完成訊息就直接 return」，於是新開機（或按過
     「清除對話」後重新整理）時 seeded 一直是 false；等第一句真正的新回覆
     到達，才走進 seeding 分支被登記成「歷史訊息」然後返回——
     **機器人的第一句話永遠不會被唸出來**，第二句以後才正常。 */
  if (!seeded) {
    for (const i of done) spoken.add(keyOf(i));
    seeded = true;
    return;
  }
  if (!done.length) return;

  const last = done[done.length - 1];
  const k = keyOf(last);
  if (spoken.has(k)) return;
  spoken.add(k);
  if (ttsStore.get().on) speak(last.text);
}

/** 清除對話後要跟著重置，否則舊的去重鍵會讓重新產生的同一句話被靜音 */
export function resetSpoken(): void {
  spoken.clear();
  seeded = false;
}

/**
 * 盯著對話。
 *
 * ★ 必須是全域訂閱，不能綁在迎賓頁的元件上：操作者常常停在地圖或遙控頁，
 *   而機器人認出人之後照樣要開口。原版就是在每則推播都跑一次，
 *   與當時看的是哪一頁無關。
 */
export function startChatTts(): void {
  let lastMessages: ChatMessage[] | undefined;
  telemetryStore.subscribe(() => {
    const messages = telemetryStore.get().snap.messages;
    // 參考沒換就是沒變（後端沒帶 messages 時前端沿用同一個陣列）
    if (!messages || messages === lastMessages) return;
    lastMessages = messages;
    onChatUpdate(messages);
  });
}

/* ── 啟動 ──────────────────────────────────────────────── */
let startedTts = false;

export function startTts(): void {
  if (!ttsSupported || startedTts) return;
  startedTts = true;

  // 語音清單是非同步載入的，第一次呼叫幾乎一定是空陣列
  speechSynthesis.onvoiceschanged = () => {
    voice = pickVoice();
    refreshVoices();
  };
  refreshVoices();

  /* ★ 從 localStorage 還原成「已開啟」時，要補一次解鎖。
   *
   *  iOS/Safari 規定**第一次發聲必須發生在使用者手勢的呼叫堆疊裡**，
   *  而那把鑰匙（toggle 裡的 speak）只在「從關轉開」時才轉——按鈕已經是
   *  開著的，操作者不會去按它（按了反而是關掉）。結果是整場都不會有聲音，
   *  也不會有任何錯誤訊息。桌機 Chrome 沒有這個限制，所以在筆電上重現不出來。
   */
  if (!ttsStore.get().on) return;
  ttsStore.set((prev) => ({ ...prev, needsUnlock: true }));

  const unlock = (ev: PointerEvent) => {
    /* ★ 點的就是朗讀鈕本身時不要搶。
       按鈕文案寫著「點一下啟用朗讀」，操作者最自然的動作就是點它。
       但按鈕的 onClick 是無條件 toggle，於是：
         pointerdown -> 這裡解鎖成功、把狀態改回「朗讀中」
         click       -> toggle 把它關掉
       淨結果是「解鎖成功但朗讀被關掉」，要再點第二下才真的開。 */
    const target = ev.target as HTMLElement | null;
    if (target?.closest?.("[data-tts-toggle]")) return;

    document.removeEventListener("pointerdown", unlock);
    // 唸一個空白就夠了：目的只是在使用者手勢裡呼叫過一次 speak()
    try {
      speechSynthesis.speak(new SpeechSynthesisUtterance(" "));
    } catch {
      // 這裡失敗代表這台裝置根本不讓程式發聲，之後的朗讀會走 onerror 提示
    }
    refreshVoices();
    ttsStore.set((prev) => ({ ...prev, needsUnlock: false }));
  };

  document.addEventListener("pointerdown", unlock);
}
