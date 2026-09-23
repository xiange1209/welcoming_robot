import { useEffect, type ReactNode } from "react";
import { cx } from "../lib/cx";

/**
 * 置中的純色對話框。
 *
 * 高度用 dvh 而不是 vh：手機叫出鍵盤後可視區會變很矮，用 vh 的話
 * 輸入框會被推到畫面外，而使用者正在打字所以捲不動它。
 */
export function Sheet({
  open,
  onClose,
  title,
  subtitle,
  children,
  footer,
  className,
}: {
  open: boolean;
  onClose: () => void;
  title?: ReactNode;
  subtitle?: ReactNode;
  children?: ReactNode;
  footer?: ReactNode;
  className?: string;
}) {
  useEffect(() => {
    if (!open) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") onClose();
    };
    document.addEventListener("keydown", onKey);
    return () => document.removeEventListener("keydown", onKey);
  }, [open, onClose]);

  if (!open) return null;

  return (
    <div
      className="fixed inset-0 z-60 flex items-center justify-center bg-black/55 p-5"
      onPointerDown={(e) => {
        // 只有點在遮罩本身才關。點在對話框裡面拖曳選字時滑到外面也不該關
        if (e.target === e.currentTarget) onClose();
      }}
    >
      <div
        className={cx(
          "surface-strong anim-sheet max-h-[88dvh] w-full max-w-[380px] overflow-y-auto rounded-sheet p-5",
          className,
        )}
      >
        {title && <h2 className="text-[19px] font-semibold">{title}</h2>}
        {subtitle && <p className="mt-1 text-[13px] leading-relaxed text-label-3">{subtitle}</p>}
        {children && <div className="mt-4">{children}</div>}
        {footer && <div className="mt-5 flex gap-2.5">{footer}</div>}
      </div>
    </div>
  );
}
