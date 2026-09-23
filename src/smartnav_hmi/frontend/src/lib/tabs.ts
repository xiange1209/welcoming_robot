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
 * 順序即導覽列順序：公開的兩頁在前，管理頁接著，診斷頁殿後。
 * 名稱與舊版逐字一致——縮寫過的名字（「使用者」「地圖導航」）會讓人
 * 懷疑是不是換成了別的功能，在交接與驗收時是純粹的雜訊。
 */
export const TABS: TabSpec[] = [
  { id: "greet", label: "迎賓", icon: IconGreet },
  { id: "stats", label: "到訪統計", icon: IconStats },
  { id: "map", label: "地圖與導航", icon: IconMap, admin: true },
  { id: "teleop", label: "遙控建圖", icon: IconTeleop, admin: true },
  { id: "system", label: "系統開關", icon: IconPower, admin: true },
  { id: "users", label: "使用者管理", icon: IconUsers, admin: true },
  { id: "health", label: "系統", icon: IconHealth },
  { id: "voice", label: "語音測試", icon: IconVoice },
];

/**
 * 只留下現在能用的分頁。
 *
 * 目前**沒有**被使用：導覽列改成把管理分頁顯示成「鎖上」而不是整個隱藏
 * （見 components/TabBar.tsx）。留著這支是因為它是還原成舊行為的開關——
 * App 裡把 `const tabs = TABS` 換成 `visibleTabs(admin)` 就回到完全隱藏。
 */
export const visibleTabs = (admin: boolean): TabSpec[] =>
  TABS.filter((t) => !t.admin || admin);
