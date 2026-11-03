import { sessionStore } from "./session";
import { telemetryStore } from "./telemetry";
import { speak, ttsStore, ttsSupported } from "./tts";

/**
 * ★ 2026-11-03：只給服務人員平板（已登入，放在別處）聽的語音提示：「出現貴賓 X」「出現黑名單 X」「出現訪客」。
 *   文字在 bank_reception 的 staff_alert_vip／staff_alert_blacklist／staff_alert_visitor 參數（空字串＝不提示）。
 *
 * 後端放在 system.staff_alert（不進對話清單——那份每台平板都會唸、機器人喇叭也會唸）。
 * 這裡**只有已登入的平板**才唸；迎賓平板（沒登入）與機器人喇叭都不出聲，客人那一側聽不到。
 *
 * 跟「朗讀」開關的關係：語音輸出＝平板時尊重朗讀開關（服務人員平板上按「朗讀」就是開關這些提示）；
 * 語音輸出＝機器人喇叭時平板沒有朗讀鈕可按，所以照樣唸（否則服務人員永遠聽不到）。
 */
export function startStaffAlert(): void {
  // 頁面剛打開時，最後一則提示可能是很久以前的：只唸「打開之後才出現」的
  let lastSeq: number | null = null;
  telemetryStore.subscribe(() => {
    const snap = telemetryStore.get().snap;
    const alert = snap.system?.staff_alert;
    if (!alert) return;
    if (lastSeq === null) {
      lastSeq = alert.seq;
      return;
    }
    if (alert.seq === lastSeq) return;
    lastSeq = alert.seq;
    if (!sessionStore.get().admin || !ttsSupported) return;
    const output = snap.system?.voice_output ?? "browser";
    if (output === "browser" && !ttsStore.get().on) return;
    speak(alert.text);
  });
}
