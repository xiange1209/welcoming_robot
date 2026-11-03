import { useStoreSelector } from "../hooks/useStore";
import { telemetryStore } from "./telemetry";
import type { Snapshot } from "./types";

/**
 * ★ 2026-11-03：機器人是不是正在導航（帶位／指定地點／教導路徑重播）。導航按鈕據此改成「導航中…」並停用。
 *
 * 兩個來源取聯集：
 *   - system.navigating（/navigation_active）：涵蓋 LLM 語音發起的帶位，但要等規劃完、真的出發才變 true
 *   - 進行中的 navigate／follow_taught_path 作業：按下去的那一刻就有，補上「規劃中那幾秒」的空窗
 * 後端 nav_arbiter 本來就會擋第二個導航（BUSY），這裡只是讓按鈕看得出來、不讓人以為按了沒反應。
 */
// global_localization：全域定位會開車繞、收斂後還會開回起點（★ 2026-11-03），也算在動
const NAV_ACTIONS = new Set(["navigate", "follow_taught_path", "global_localization"]);

export function navBusy(snap: Snapshot): boolean {
  if (snap.system?.navigating) return true;
  return (snap.jobs ?? []).some(
    (j) => NAV_ACTIONS.has(j.action ?? "") && (j.status === "pending" || j.status === "running"),
  );
}

export function useNavBusy(): boolean {
  return useStoreSelector(telemetryStore, (t) => navBusy(t.snap));
}
