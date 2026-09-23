import { useLayoutEffect, useRef, useState } from "react";
import type { TabId, TabSpec } from "../lib/tabs";
import { cx } from "../lib/cx";
import { IconLock } from "./icons";

/**
 * 底部懸浮導覽列。
 *
 * 版面上是「浮在內容之上的一顆藥丸」而不是貼底的工具列：貼底的版本
 * 會在 iPad 上被 home 指示條壓到，而懸浮 + safe-area 內距兩者都避得開。
 *
 * 選取指示丸用 transform 位移到目標位置，不是各自換背景色。位移才會讓
 * 眼睛跟著它走——這是 iOS 分頁切換的核心手法，HIG 稱之為維持
 * 「空間的連續性」。
 */
export function TabBar({
  tabs,
  active,
  locked,
  onSelect,
  onLocked,
}: {
  tabs: TabSpec[];
  active: TabId;
  /** 這個分頁現在需要登入才能進去 */
  locked: (tab: TabSpec) => boolean;
  onSelect: (id: TabId) => void;
  onLocked: () => void;
}) {
  const trackRef = useRef<HTMLDivElement>(null);
  const itemRefs = useRef(new Map<TabId, HTMLButtonElement>());
  const [pill, setPill] = useState<{ x: number; w: number } | null>(null);
  // 第一次定位不要平滑捲動：開頁就看到畫面自己滑一下會讓人以為按到什麼。
  // 只在 effect 內讀寫，算繪期間完全不碰。
  const settled = useRef(false);

  useLayoutEffect(() => {
    const measure = () => {
      const el = itemRefs.current.get(active);
      if (!el) {
        setPill(null);
        return;
      }
      setPill((prev) =>
        prev && prev.x === el.offsetLeft && prev.w === el.offsetWidth
          ? prev // 尺寸沒變就不要換掉物件，免得 ResizeObserver 每次都觸發重繪
          : { x: el.offsetLeft, w: el.offsetWidth },
      );
      // 窄螢幕上分頁會捲出畫面，把目前這一頁帶回可視範圍
      el.scrollIntoView({
        inline: "nearest",
        block: "nearest",
        behavior: settled.current ? "smooth" : "auto",
      });
      settled.current = true;
    };

    measure();

    // 字體載入完、或裝置轉向時寬度會變，指示丸要跟著重量
    const observer = new ResizeObserver(measure);
    if (trackRef.current) observer.observe(trackRef.current);
    return () => observer.disconnect();
  }, [active, tabs]);

  return (
    <nav className="pointer-events-none fixed inset-x-0 bottom-0 z-50 flex justify-center px-3 pb-[calc(env(safe-area-inset-bottom)+10px)]">
      <div className="surface-strong pointer-events-auto max-w-full overflow-hidden rounded-full p-1.5">
        <div
          ref={trackRef}
          className="no-scrollbar relative flex overflow-x-auto"
        >
          {pill && (
            <div
              aria-hidden
              className="absolute top-0 bottom-0 rounded-full bg-[var(--color-veil)] shadow-[inset_0_1px_0_rgba(255,255,255,0.18)]"
              // 永遠掛著 transition 是安全的：CSS 不會對元素第一次算繪的初始值
              // 做動畫，所以開頁時指示丸直接出現在正確位置，之後換頁才會滑。
              style={{
                transform: `translateX(${pill.x}px)`,
                width: pill.w,
                transition:
                  "transform 420ms var(--ease-ios), width 420ms var(--ease-ios)",
              }}
            />
          )}

          {tabs.map((tab) => {
            const Icon = tab.icon;
            const isLocked = locked(tab);
            const on = tab.id === active && !isLocked;
            return (
              <button
                key={tab.id}
                type="button"
                ref={(el) => {
                  if (el) itemRefs.current.set(tab.id, el);
                  else itemRefs.current.delete(tab.id);
                }}
                onClick={() => (isLocked ? onLocked() : onSelect(tab.id))}
                aria-current={on ? "page" : undefined}
                title={isLocked ? `${tab.label}需要管理者登入` : undefined}
                className={cx(
                  "pressable relative z-10 flex min-w-[68px] flex-none flex-col items-center gap-1 rounded-full px-3 py-2",
                  "text-[11px] font-medium transition-colors duration-300",
                  on
                    ? "text-label"
                    : isLocked
                      ? "text-label-3/60"
                      : "text-label-3",
                )}
              >
                <span className="relative">
                  <Icon
                    className={cx(
                      "size-[22px] transition-transform duration-[420ms]",
                      on && "scale-110",
                      isLocked && "opacity-55",
                    )}
                    style={{ transitionTimingFunction: "var(--ease-spring)" }}
                  />
                  {isLocked && (
                    // 小鎖頭疊在圖示右下角。外面包一層底色圓徽章，讓它從圖示上「浮」起來
                    // ——直接疊圖示上會跟線條糊在一起，一眼看不出是鎖。
                    <span className="absolute -right-2 -bottom-1 grid size-[15px] place-items-center rounded-full bg-ink-soft">
                      <IconLock
                        className="size-[11px] text-label-2"
                        strokeWidth={2.4}
                      />
                    </span>
                  )}
                </span>
                <span className="whitespace-nowrap">{tab.label}</span>
              </button>
            );
          })}
        </div>
      </div>
    </nav>
  );
}
