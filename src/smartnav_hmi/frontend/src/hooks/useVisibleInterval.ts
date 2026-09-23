import { useEffect, useRef } from "react";

/**
 * 只在「這一頁正被看著」時跑的計時器。
 *
 * Pi 4 的 CPU 不該花在沒人看的畫面上，而平板螢幕關掉或切到別的 App 時
 * 不會觸發任何切頁事件——只有 visibilitychange 會。原版就是漏了這一道，
 * 讓後端在沒人看的情況下持續維持訂閱。
 *
 *   active     這一頁現在是不是使用中的分頁
 *   immediate  掛上時先跑一次（不要等第一個間隔）
 */
export function useVisibleInterval(
  callback: () => void,
  intervalMs: number,
  active: boolean,
  immediate = true,
): void {
  // 存成 ref 讓計時器永遠叫到最新的 callback，而不必把它列進下面的依賴
  // ——列進去的話每次重繪都會重建計時器，間隔就永遠跑不完。
  // 寫入放在 effect 裡而不是元件本體：算繪期間寫 ref 在並行渲染下不安全。
  const savedCallback = useRef(callback);
  useEffect(() => {
    savedCallback.current = callback;
  }, [callback]);

  useEffect(() => {
    if (!active) return;

    let timer: ReturnType<typeof setInterval> | null = null;

    const start = () => {
      if (timer !== null) return;
      if (immediate) savedCallback.current();
      timer = setInterval(() => savedCallback.current(), intervalMs);
    };

    const stop = () => {
      if (timer === null) return;
      clearInterval(timer);
      timer = null;
    };

    const onVisibility = () => (document.hidden ? stop() : start());

    if (!document.hidden) start();
    document.addEventListener("visibilitychange", onVisibility);

    return () => {
      stop();
      document.removeEventListener("visibilitychange", onVisibility);
    };
  }, [active, intervalMs, immediate]);
}

/**
 * 「有人在看」的心跳。
 *
 * 後端的位姿來源（amcl_pose／TF 監聽）與地圖 PNG 渲染都很貴，而且只有
 * 特定分頁看得到，所以做成按需開啟：在那一頁時定期打一次，切走就停，
 * 後端十幾秒後自動把訂閱收掉。**不打這個心跳地圖就不會更新。**
 */
export function useHeartbeat(url: string, intervalMs: number, active: boolean): void {
  useVisibleInterval(() => {
    fetch(url, { method: "POST" }).catch(() => {});
  }, intervalMs, active);
}
