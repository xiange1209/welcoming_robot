import {
  Hct,
  SchemeTonalSpot,
  argbFromHex,
  blueFromArgb,
  greenFromArgb,
  hexFromArgb,
  redFromArgb,
} from "@material/material-color-utilities";
import { createStore } from "./store";

/**
 * 主題引擎：Material 3 Tonal Spot。
 *
 * 全站的顏色其實只透過 index.css 裡那一小組 --color-* token 間接使用
 * （bg-blue、text-label-3、bg-white/8…），沒有任何元件寫死原始色碼。
 * 這裡不去改各元件、也不把 token 改名成 M3 的角色命名，而是讓這些
 * token 的**值**隨種子色／深淺模式動態算出、蓋掉 index.css 裡的預設值
 * ——inline style 的 specificity 天生就比 @theme 產生的樣式層高。
 *
 * 跟 tts.ts 走一樣的慣例：純 localStorage 字串 key、模組載入時同步讀出來
 * 灌進 store 初始值、小型 setter 直接 write-through。特意不用 session.ts
 * 的 sessionStorage：那是要讓管理者權杖「關分頁即登出」，但顯示主題是外觀
 * 偏好，應該跟 tts 設定一樣重整、重開機都還在。
 */

export type ThemeMode = "light" | "dark";

export interface SeedColor {
  id: string;
  label: string;
  hex: string;
}

export const SEED_COLORS: SeedColor[] = [
  { id: "classicBlue", label: "經典藍", hex: "#0061A4" },
  { id: "dynamicTeal", label: "活力青", hex: "#006A60" },
  { id: "forestGreen", label: "森林綠", hex: "#2E6B27" },
  { id: "crimsonRed", label: "胭脂紅", hex: "#B3261E" },
  { id: "warmAmber", label: "橄欖褐", hex: "#825500" },
  { id: "roseCoral", label: "珊瑚粉", hex: "#984061" },
];

const DEFAULT_SEED = "classicBlue";

/**
 * 執行中／警告／VIP 這幾個狀態色在全站是固定語意（TopBar 的 TINT、
 * SystemPage 的燈號、各頁的成功／警告標示…），刻意不讓它們跟著種子色
 * 連動——選了森林綠當種子的話，「執行中＝綠燈」會跟主題色混在一起分不清。
 * 只補淺色模式的對應值，沿用原本「取 Apple 系統色」的邏輯（深色沿用既有值，
 * 淺色取 Apple 官方 light-mode 那一組)。
 */
const FIXED_ACCENTS: Record<string, { dark: string; light: string }> = {
  "--color-green": { dark: "#30d158", light: "#34c759" },
  "--color-orange": { dark: "#ff9f0a", light: "#ff9500" },
  "--color-gold": { dark: "#ffd60a", light: "#ffcc00" },
  "--color-purple": { dark: "#bf5af2", light: "#af52de" },
  "--color-teal": { dark: "#40cbe0", light: "#30b0c7" },
};

interface ThemeState {
  seed: string;
  mode: ThemeMode;
}

const readMode = (): ThemeMode =>
  localStorage.getItem("themeMode") === "light" ? "light" : "dark";

const readSeed = (): string => {
  const saved = localStorage.getItem("themeSeed");
  return saved && SEED_COLORS.some((s) => s.id === saved)
    ? saved
    : DEFAULT_SEED;
};

export const themeStore = createStore<ThemeState>({
  seed: readSeed(),
  mode: readMode(),
});

/** 同一個 on-surface 顏色疊不同透明度：深色模式下它接近白、淺色模式下自動
 *  接近黑，髮絲線／文字層級不用手動維護兩組數值。 */
const overlay = (argb: number, alphaPct: number): string =>
  `rgb(${redFromArgb(argb)} ${greenFromArgb(argb)} ${blueFromArgb(argb)} / ${alphaPct}%)`;

function applyTheme(seed: string, mode: ThemeMode): void {
  const seedHex =
    SEED_COLORS.find((s) => s.id === seed)?.hex ?? SEED_COLORS[0].hex;
  const seedHct = Hct.fromInt(argbFromHex(seedHex));
  const scheme = new SchemeTonalSpot(seedHct, mode === "dark", 0);
  const root = document.documentElement.style;

  root.setProperty("--color-ink", hexFromArgb(scheme.background));
  root.setProperty("--color-ink-soft", hexFromArgb(scheme.surfaceContainerLow));
  root.setProperty("--color-label", hexFromArgb(scheme.onSurface));
  root.setProperty("--color-label-2", overlay(scheme.onSurface, 62));
  root.setProperty("--color-label-3", overlay(scheme.onSurface, 38));
  root.setProperty("--color-surface", hexFromArgb(scheme.surfaceContainer));
  root.setProperty(
    "--color-surface-strong",
    hexFromArgb(scheme.surfaceContainerHigh),
  );
  root.setProperty("--color-outline", hexFromArgb(scheme.outline));
  root.setProperty(
    "--color-outline-variant",
    hexFromArgb(scheme.outlineVariant),
  );
  root.setProperty("--color-well", overlay(scheme.onSurface, 10));
  root.setProperty("--color-veil", overlay(scheme.onSurface, 16));
  root.setProperty("--color-blue", hexFromArgb(scheme.primary));
  root.setProperty("--color-on-blue", hexFromArgb(scheme.onPrimary));
  root.setProperty("--color-red", hexFromArgb(scheme.error));
  root.setProperty("--color-on-red", hexFromArgb(scheme.onError));

  for (const [token, { dark, light }] of Object.entries(FIXED_ACCENTS)) {
    root.setProperty(token, mode === "dark" ? dark : light);
  }

  // 提供原生表單控制項 CSS hook
  document.documentElement.dataset.theme = mode;

  document
    .querySelector('meta[name="theme-color"]')
    ?.setAttribute("content", hexFromArgb(scheme.background));
}

export function initTheme(): void {
  applyTheme(themeStore.get().seed, themeStore.get().mode);
}

export function setSeed(id: string): void {
  if (!SEED_COLORS.some((s) => s.id === id)) return;
  localStorage.setItem("themeSeed", id);
  themeStore.set((prev) => ({ ...prev, seed: id }));
  applyTheme(id, themeStore.get().mode);
}

export function setMode(mode: ThemeMode): void {
  localStorage.setItem("themeMode", mode);
  themeStore.set((prev) => ({ ...prev, mode }));
  applyTheme(themeStore.get().seed, mode);
}
