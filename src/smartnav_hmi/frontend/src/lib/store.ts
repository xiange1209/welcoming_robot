/**
 * 極小的外部狀態容器。
 *
 * 為什麼不用 Context：狀態推播是 10 Hz 的，而快照裡有十幾個互不相干的
 * 區塊（身分、電壓、對話、位姿、作業…）。Context 的值一換，**所有**
 * 消費者都會重繪——等於每秒把整棵樹重建十次，Pi 4 接的平板直接掉幀。
 *
 * 這裡走 useSyncExternalStore + selector：每個元件只訂閱自己要的那一小塊，
 * 值沒變就不重繪。selector 本身是取純量的操作，跑一萬次也不花錢。
 */

export type Store<T> = {
  get: () => T;
  set: (next: T | ((prev: T) => T)) => void;
  subscribe: (listener: () => void) => () => void;
  /** 立刻把待發的通知送出（離開背景時用，見 coalesce） */
  flush: () => void;
};

export interface StoreOptions {
  /**
   * 把同一個動畫影格內的多次更新合併成一次通知。
   *
   * 順帶解決一個更重要的問題：分頁被隱藏時 rAF 不會觸發，於是 React
   * **一次都不會重繪**。而 `get()` 永遠回傳最新值，所以回到前景時
   * 畫面不會是舊的——只是中間那幾百次白工被省掉了。
   */
  coalesce?: boolean;
}

export function createStore<T>(initial: T, options: StoreOptions = {}): Store<T> {
  let state = initial;
  let pending = false;
  let frame = 0;
  const listeners = new Set<() => void>();

  const emit = () => {
    pending = false;
    frame = 0;
    // 複製一份再跑：listener 內部可能取消訂閱，直接迭代 Set 會漏掉下一個
    for (const fn of [...listeners]) fn();
  };

  const notify = () => {
    if (!options.coalesce) {
      emit();
      return;
    }
    if (pending) return;
    pending = true;
    frame = requestAnimationFrame(emit);
  };

  return {
    get: () => state,
    set: (next) => {
      const value = typeof next === "function" ? (next as (prev: T) => T)(state) : next;
      if (Object.is(value, state)) return;
      state = value;
      notify();
    },
    subscribe: (listener) => {
      listeners.add(listener);
      return () => {
        listeners.delete(listener);
      };
    },
    flush: () => {
      if (!pending) return;
      if (frame) cancelAnimationFrame(frame);
      emit();
    },
  };
}

/** 逐鍵淺比較。給回傳物件的 selector 用，否則每次都是新物件、永遠不相等 */
export function shallowEqual<T>(a: T, b: T): boolean {
  if (Object.is(a, b)) return true;
  if (typeof a !== "object" || typeof b !== "object" || a === null || b === null) return false;
  const ka = Object.keys(a as object);
  const kb = Object.keys(b as object);
  if (ka.length !== kb.length) return false;
  for (const k of ka) {
    if (!Object.is((a as Record<string, unknown>)[k], (b as Record<string, unknown>)[k])) {
      return false;
    }
  }
  return true;
}
