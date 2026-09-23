import type { SVGProps } from "react";

/*
 * 內嵌 SVG 圖示。
 *
 * 不引入任何圖示套件：HMI 是離線服務，外部 CDN 在展示現場會直接失敗，
 * 而那時你沒有時間查。全部用 currentColor，交給 CSS 決定顏色。
 * 線條粗細 1.8 對齊 SF Symbols 的 Regular 視覺重量。
 */

type IconProps = SVGProps<SVGSVGElement>;

function Base({ children, ...rest }: IconProps) {
  return (
    <svg
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth={1.8}
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden
      {...rest}
    >
      {children}
    </svg>
  );
}

/* 迎賓：一個人 */
export const IconGreet = (p: IconProps) => (
  <Base {...p}>
    <circle cx="12" cy="8" r="3.4" />
    <path d="M4.8 20a7.2 7.2 0 0 1 14.4 0" />
  </Base>
);

/* 到訪統計：長條圖 */
export const IconStats = (p: IconProps) => (
  <Base {...p}>
    <path d="M4 20V11M9.3 20V5M14.7 20v-6M20 20V8" />
  </Base>
);

/* 地圖與導航：定位針 */
export const IconMap = (p: IconProps) => (
  <Base {...p}>
    <path d="M12 21s6.5-6.1 6.5-11a6.5 6.5 0 1 0-13 0C5.5 14.9 12 21 12 21Z" />
    <circle cx="12" cy="10" r="2.4" />
  </Base>
);

/* 遙控建圖：方向鍵 */
export const IconTeleop = (p: IconProps) => (
  <Base {...p}>
    <path d="M12 3.5 14.4 7h-4.8L12 3.5ZM12 20.5 9.6 17h4.8L12 20.5ZM3.5 12 7 9.6v4.8L3.5 12ZM20.5 12 17 14.4V9.6L20.5 12Z" />
    <circle cx="12" cy="12" r="2.2" />
  </Base>
);

/* 系統開關：電源 */
export const IconPower = (p: IconProps) => (
  <Base {...p}>
    <path d="M12 3.5v8" />
    <path d="M17.5 6.6a7.5 7.5 0 1 1-11 0" />
  </Base>
);

/* 使用者管理：兩個人 */
export const IconUsers = (p: IconProps) => (
  <Base {...p}>
    <circle cx="9.5" cy="8.2" r="3.1" />
    <path d="M3.4 19.5a6.1 6.1 0 0 1 12.2 0" />
    <path d="M16.2 5.6a3.1 3.1 0 0 1 0 5.9M17.4 14a6.1 6.1 0 0 1 3.2 5.5" />
  </Base>
);

/* 系統健康：心電圖 */
export const IconHealth = (p: IconProps) => (
  <Base {...p}>
    <path d="M3 12.5h3.6l2-5.2 3.2 10 2.2-6.1 1.5 3.3H21" />
  </Base>
);

/* 語音測試：音波 */
export const IconVoice = (p: IconProps) => (
  <Base {...p}>
    <path d="M4 10.5v3M8 7v10M12 4.5v15M16 8v8M20 10.5v3" />
  </Base>
);

/* 需要管理者登入的分頁：鎖頭 */
export const IconLock = (p: IconProps) => (
  <Base {...p}>
    <rect x="5" y="10.5" width="14" height="9.5" rx="2.4" />
    <path d="M8.2 10.5V7.8a3.8 3.8 0 0 1 7.6 0v2.7" />
  </Base>
);

/* 送出 */
export const IconSend = (p: IconProps) => (
  <Base {...p}>
    <path d="M4.5 12h14M13 6.5l5.5 5.5L13 17.5" />
  </Base>
);

/* 已選取：勾號 */
export const IconCheck = (p: IconProps) => (
  <Base {...p}>
    <path d="M5 12.5l4.5 4.5L19 7.5" />
  </Base>
);

/* 主題設定：調色盤 */
export const IconPalette = (p: IconProps) => (
  <Base {...p}>
    <path d="M12 3.5a8.5 8.5 0 1 0 0 17c1.1 0 1.9-.9 1.9-2 0-.5-.2-1-.5-1.3-.3-.3-.5-.8-.5-1.3 0-1.1.9-2 2-2h1.6c1.7 0 3-1.3 3-3 0-4.1-3.8-7.4-8.5-7.4Z" />
    <circle cx="7.7" cy="11" r="1.15" fill="currentColor" stroke="none" />
    <circle cx="10.2" cy="7.4" r="1.15" fill="currentColor" stroke="none" />
    <circle cx="14.6" cy="7.6" r="1.15" fill="currentColor" stroke="none" />
    <circle cx="16.6" cy="11.3" r="1.15" fill="currentColor" stroke="none" />
  </Base>
);
