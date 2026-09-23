import { useCallback, useEffect, useRef, useState } from "react";

/**
 * 掛載時抓一次，並回傳一個可以重抓的函式。
 *
 * 重點不只是少寫幾行：它帶了**取消旗標**。這幾頁的資料都是 HTTP 抓來的，
 * 而操作者常常按下「重新整理」就馬上切到別的分頁——沒有這道閘，
 * 晚回來的那份回應會寫進已經卸載的元件，或是覆蓋掉更新的一份資料
 * （兩則請求的回應順序不保證跟送出順序一致）。
 *
 *   load     取資料。內容變了要用 useCallback 包住，否則每次重繪都會重抓
 *   initial  還沒抓到之前的值
 */
export function useFetch<T>(load: () => Promise<T>, initial: T): [T, () => void] {
  const [data, setData] = useState<T>(initial);
  const [nonce, setNonce] = useState(0);

  // 用 ref 讓 effect 不必把 load 列進依賴：呼叫端常常寫成行內箭頭函式，
  // 列進去的話每次重繪都會重抓一輪。
  const loadRef = useRef(load);
  useEffect(() => {
    loadRef.current = load;
  }, [load]);

  useEffect(() => {
    let cancelled = false;
    void (async () => {
      const result = await loadRef.current();
      if (!cancelled) setData(result);
    })();
    return () => {
      cancelled = true;
    };
  }, [nonce]);

  const reload = useCallback(() => setNonce((n) => n + 1), []);
  return [data, reload];
}
