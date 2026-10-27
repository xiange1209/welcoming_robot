import type { ComponentType, SVGProps } from "react";
import {
  IconGreet,
  IconHealth,
  IconMap,
  IconPower,
  IconStats,
  IconTeleop,
  IconUsers,
  IconVoice,
} from "../components/icons";

export type TabId =
  | "greet"
  | "stats"
  | "map"
  | "teleop"
  | "system"
  | "users"
  | "health"
  | "voice";

export interface TabSpec {
  id: TabId;
  label: string;
  icon: ComponentType<SVGProps<SVGSVGElement>>;
  /** 未登入時整個分頁不顯示，避免訪客誤按 */
  admin?: boolean;
}

/**
 * 順序即導覽列順序：公開的迎賓頁在前，管理頁接著，診斷頁殿後。
 * ★ 2026-10-26 使用者要求區分管理者與一般使用者：一般使用者（沒登入）只看得到「迎賓」，
 *   到訪統計、系統也改成要登入（訪客不需要看，看了反而以為有一堆功能）。
 * 名稱與舊版逐字一致——縮寫過的名字（「使用者」「地圖導航」）會讓人
 * 懷疑是不是換成了別的功能，在交接與驗收時是純粹的雜訊。
 */
export const TABS: TabSpec[] = [
  { id: "greet", label: "迎賓", icon: IconGreet },
  { id: "stats", label: "到訪統計", icon: IconStats, admin: true },
  { id: "map", label: "地圖與導航", icon: IconMap, admin: true },
  { id: "teleop", label: "遙控建圖", icon: IconTeleop, admin: true },
  { id: "system", label: "系統開關", icon: IconPower, admin: true },
  { id: "users", label: "使用者管理", icon: IconUsers, admin: true },
  { id: "health", label: "系統", icon: IconHealth, admin: true },
  // ★ 2026-10-22：要登入——這頁能讓機器人對客人念任何字（後端 /api/speak 也改成要登入）
  { id: "voice", label: "語音測試", icon: IconVoice, admin: true },
];

/**
 * 只留下現在能用的分頁。
 *
 * ★ 2026-10-26 起 App 用這支（沒登入就完全不顯示管理分頁）。
 * 想回到「顯示成鎖上」：App 裡把 `visibleTabs(admin)` 換回 `TABS`。
 */
export const visibleTabs = (admin: boolean): TabSpec[] =>
  TABS.filter((t) => !t.admin || admin);
