import { useEffect, useRef } from "react";

/**
 * MJPEG 串流的開關。
 *
 * MJPEG 是一條開著不關的長連線，隱藏的 <img> 照樣會一直收。只讓目前
 * 看得到的那一路連著，切走就把 src 拿掉中斷它。
 *
 * ★ 分頁被隱藏時也要斷。這是整個節點**最貴的一條路徑**（實測約佔 22%
 *   CPU，比地圖與系統頁加起來還貴）：平板停在迎賓頁、螢幕關掉之後，
 *   切頁事件不會觸發，後端看到 _video_viewers > 0 就繼續以 video_fps
 *   搬運影格給**沒有人在看的螢幕**。
 */
export function useVideoStream(active: boolean) {
  const ref = useRef<HTMLImageElement | null>(null);

  useEffect(() => {
    const img = ref.current;
    if (!img) return;

    // 帶 timestamp 避免瀏覽器把中斷過的串流當成快取重用
    const attach = () => {
      if (!img.getAttribute("src")) img.src = `/video?t=${Date.now()}`;
    };
    const detach = () => {
      if (img.getAttribute("src")) img.removeAttribute("src");
    };
    const sync = () => (active && !document.hidden ? attach() : detach());

    sync();
    document.addEventListener("visibilitychange", sync);
    return () => {
      document.removeEventListener("visibilitychange", sync);
      detach();
    };
  }, [active]);

  return ref;
}
