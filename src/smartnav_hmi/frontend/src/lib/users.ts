import { api } from "./api";
import { sessionStore } from "./session";
import { createStore } from "./store";
import { telemetryStore } from "./telemetry";
import { toast } from "./toast";
import type { Registration, UserRecord } from "./types";

export const usersStore = createStore<UserRecord[]>([]);

export async function refreshUsers(): Promise<void> {
  const r = await api<{ users?: UserRecord[] }>("/api/users");
  usersStore.set(r.users || []);
}

/**
 * 註冊進度是**後端廣播**的，不是本地變數——否則只有按下按鈕的那台裝置
 * 看得到進度。這裡在全域監看而不是綁在使用者頁：發起註冊之後切到別的
 * 分頁去看相機，完成提示一樣要跳出來。
 */
let watching = false;

export function startRegistrationWatch(): void {
  if (watching) return;
  watching = true;

  let prev: Registration | null = null;
  let finishedSeen: number | null = null;
  /** 收過第一份快照沒？用來分辨「剛連上」與「真的剛完成」 */
  let synced = false;

  telemetryStore.subscribe(() => {
    const reg = telemetryStore.get().snap.registration ?? null;
    const before = prev;
    prev = reg;

    // 採樣期間張數會一直跳，左邊清單跟著更新才看得到 num_samples 累積
    if (
      reg?.status === "running" &&
      before?.status === "running" &&
      before.collected !== reg.collected &&
      sessionStore.get().admin
    ) {
      void refreshUsers();
    }

    /* 註冊剛結束：所有裝置都跳提示並刷新名單，不只發起的那一台。
       但剛連上時收到的可能是上一輪留下的結果，那份要吞掉不要重跳。 */
    if (
      reg &&
      reg.status !== "running" &&
      reg.finished_at &&
      finishedSeen !== reg.finished_at
    ) {
      const stale = !synced;
      finishedSeen = reg.finished_at;
      if (!stale) {
        if (reg.status === "succeeded") {
          toast.success("任務已完成");
        } else {
          toast.error("任務執行失敗");
        }
        if (sessionStore.get().admin) void refreshUsers();
      }
    }
    synced = true;
  });
}
