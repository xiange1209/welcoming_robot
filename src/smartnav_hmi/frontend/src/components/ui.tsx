import type { ComponentPropsWithRef, ReactNode } from "react";
import { cx } from "../lib/cx";

/* ── 按鈕 ──────────────────────────────────────────────── */

type Variant = "plain" | "primary" | "danger" | "ghost";

const VARIANT: Record<Variant, string> = {
  plain:
    "bg-[var(--color-surface)] text-label border border-[var(--color-outline-variant)] hover:bg-[var(--color-veil)]",
  primary: "bg-blue text-on-blue border border-transparent",
  danger: "bg-red/18 text-red border border-red/35 hover:bg-red/25",
  ghost:
    "bg-transparent text-label-2 border border-transparent hover:bg-[var(--color-surface)]",
};

export interface ButtonProps extends ComponentPropsWithRef<"button"> {
  variant?: Variant;
  /** 撐滿一列。表單底部的主要動作幾乎都要 */
  block?: boolean;
}

export function Button({
  variant = "plain",
  block,
  className,
  children,
  ...rest
}: ButtonProps) {
  return (
    <button
      {...rest}
      className={cx(
        "tap pressable inline-flex items-center justify-center gap-1.5 rounded-ios px-3.5 text-[15px] font-medium whitespace-nowrap",
        "disabled:opacity-40 disabled:pointer-events-none",
        VARIANT[variant],
        block && "w-full",
        className,
      )}
    >
      {children}
    </button>
  );
}

/* ── 卡片 ──────────────────────────────────────────────── */

export function Card({
  title,
  actions,
  children,
  className,
  bodyClassName,
}: {
  title?: ReactNode;
  actions?: ReactNode;
  children: ReactNode;
  className?: string;
  bodyClassName?: string;
}) {
  return (
    <section
      className={cx(
        "surface rounded-card flex min-h-0 flex-col p-3.5",
        className,
      )}
    >
      {(title || actions) && (
        <header className="mb-3 flex flex-none items-center gap-2">
          {title && (
            <h2 className="text-[13px] font-semibold tracking-wide text-label-2 uppercase">
              {title}
            </h2>
          )}
          {actions && (
            <div className="ml-auto flex items-center gap-2">{actions}</div>
          )}
        </header>
      )}
      <div className={cx("flex min-h-0 flex-1 flex-col", bodyClassName)}>
        {children}
      </div>
    </section>
  );
}

/* ── 表單 ──────────────────────────────────────────────── */

export function Field({ className, ...rest }: ComponentPropsWithRef<"input">) {
  return <input {...rest} className={cx("field tap w-full", className)} />;
}

export function Select({
  className,
  children,
  ...rest
}: ComponentPropsWithRef<"select">) {
  return (
    <select
      {...rest}
      className={cx("field tap appearance-none pr-8", className)}
    >
      {children}
    </select>
  );
}

/** iOS 分段控制。指示丸用 transform 位移，不是換背景色——位移才有連續感 */
export function Segmented<T extends string>({
  value,
  options,
  onChange,
  className,
}: {
  value: T;
  options: { value: T; label: string }[];
  onChange: (value: T) => void;
  className?: string;
}) {
  const index = Math.max(
    0,
    options.findIndex((o) => o.value === value),
  );
  return (
    <div className={cx("segment", className)}>
      <div
        aria-hidden
        // left 明寫出來，不要靠「絕對定位元素落在靜態位置」的隱性行為——
        // 之後只要有人在它前面插一個節點，指示丸就會整個偏掉
        className="absolute top-[3px] bottom-[3px] left-[3px] rounded-[9px] bg-[var(--color-veil)] shadow-[0_2px_8px_rgba(0,0,0,0.3)] transition-transform duration-[380ms]"
        style={{
          width: `calc((100% - 6px) / ${options.length})`,
          transform: `translateX(${index * 100}%)`,
          transitionTimingFunction: "var(--ease-ios)",
        }}
      />
      {options.map((o) => (
        <button
          key={o.value}
          type="button"
          onClick={() => onChange(o.value)}
          className={cx(
            "relative z-10 flex-1 rounded-[9px] px-3 py-2 text-[14px] font-medium transition-colors duration-200",
            o.value === value ? "text-label" : "text-label-3",
          )}
        >
          {o.label}
        </button>
      ))}
    </div>
  );
}

/* ── 標籤與指示 ────────────────────────────────────────── */

const BADGE_TONE: Record<string, string> = {
  VIP: "bg-gold/20 text-gold border-gold/30",
  ADMIN: "bg-blue/20 text-blue border-blue/30",
  GUEST:
    "bg-[var(--color-well)] text-label-2 border-[var(--color-outline-variant)]",
  BLACKLIST: "bg-red/25 text-red border-red/40 font-bold",
  ok: "bg-green/18 text-green border-green/30",
  bad: "bg-red/18 text-red border-red/30",
  run: "bg-blue/18 text-blue border-blue/30",
  warn: "bg-orange/18 text-orange border-orange/30",
  muted:
    "bg-[var(--color-well)] text-label-3 border-[var(--color-outline-variant)]",
};

export function Badge({
  tone = "muted",
  children,
}: {
  tone?: string;
  children: ReactNode;
}) {
  return (
    <span
      className={cx(
        "inline-flex flex-none items-center rounded-full border px-2 py-0.5 text-[11px] leading-5",
        BADGE_TONE[tone] || BADGE_TONE.muted,
      )}
    >
      {children}
    </span>
  );
}

/** 清單列。整站的「一筆資料 + 右側動作」都長這樣 */
export function Row({
  children,
  className,
}: {
  children: ReactNode;
  className?: string;
}) {
  return (
    <div
      className={cx(
        "flex items-center gap-2.5 rounded-ios border border-[var(--color-outline-variant)] bg-[var(--color-well)] px-3 py-2.5 text-[14px]",
        className,
      )}
    >
      {children}
    </div>
  );
}

/** 可捲清單。外層卡片已經是 flex column，這裡吃掉剩下的高度 */
export function List({
  children,
  className,
}: {
  children: ReactNode;
  className?: string;
}) {
  return (
    <div
      className={cx(
        "flex min-h-0 flex-1 flex-col gap-1.5 overflow-y-auto",
        className,
      )}
    >
      {children}
    </div>
  );
}

export function Hint({
  children,
  className,
}: {
  children: ReactNode;
  className?: string;
}) {
  return (
    <p className={cx("text-[12.5px] leading-relaxed text-label-3", className)}>
      {children}
    </p>
  );
}

export function Empty({ children }: { children: ReactNode }) {
  return <div className="px-1 py-3 text-[13px] text-label-3">{children}</div>;
}
