import { useSyncExternalStore } from "react";
import { useSyncExternalStoreWithSelector } from "use-sync-external-store/shim/with-selector";
import type { Store } from "../lib/store";

/**
 * 訂閱 store 的一小塊。值（依 isEqual 判定）沒變就不重繪。
 *
 * 用 React 官方的 useSyncExternalStoreWithSelector 而不是自己包一層快取：
 * getSnapshot 必須在資料沒變時回傳**同一個參考**，否則 React 會判定「變了」
 * 而無限重繪。那段快取邏輯（還要一併處理 selector 換身分的情況）容易寫錯，
 * 而這支就是 Redux 與其他狀態庫共用的同一份實作。
 *
 * 這是整個高頻資料流優化的地基：推播 10 Hz 進來，但每個元件只在自己那一小塊
 * 真的變了的時候重繪。
 */
export function useStoreSelector<T, S>(
  store: Store<T>,
  selector: (state: T) => S,
  isEqual?: (a: S, b: S) => boolean,
): S {
  return useSyncExternalStoreWithSelector(
    store.subscribe,
    store.get,
    store.get, // 沒有伺服器端渲染，getServerSnapshot 用同一支
    selector,
    isEqual,
  );
}

/** 整包訂閱。只給真的需要整份狀態的地方用（例如小型 store 的完整值）。 */
export function useStoreValue<T>(store: Store<T>): T {
  return useSyncExternalStore(store.subscribe, store.get, store.get);
}
