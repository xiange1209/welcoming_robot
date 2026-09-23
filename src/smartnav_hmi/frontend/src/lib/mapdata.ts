import { api } from "./api";
import { createStore } from "./store";
import { telemetryStore } from "./telemetry";
import type { MapEntry, Waypoint } from "./types";

/**
 * 地圖影像與清單。
 *
 * 影像只在 map_meta.version 變動時重抓一次——建圖過程中後端會一直更新
 * 版本號，而每次重抓都是一張完整 PNG，不能跟著 10 Hz 的推播走。
 */

export const mapImageStore = createStore<{ image: HTMLImageElement | null; version: number }>({
  image: null,
  version: -1,
});

export const waypointsStore = createStore<Waypoint[]>([]);
export const mapsStore = createStore<MapEntry[]>([]);

let loadedVersion = -1;

function loadMapImage(version: number): void {
  const img = new Image();
  img.onload = () => {
    mapImageStore.set({ image: img, version });
  };
  img.onerror = () => {
    // 載不到就維持舊圖：建圖中短暫的 404 不該讓畫面整個空掉
    loadedVersion = -1;
  };
  img.src = `/api/map.png?v=${version}`;
}

/** 開始盯著推播裡的地圖版本。只會真的掛一次 */
let started = false;
export function startMapLayer(): void {
  if (started) return;
  started = true;
  telemetryStore.subscribe(() => {
    const meta = telemetryStore.get().snap.map_meta;
    if (!meta || meta.version === loadedVersion) return;
    loadedVersion = meta.version;
    loadMapImage(meta.version);
  });
}

/** 手動重抓（「重新整理」按鈕）。版本沒變時也要強制拉一次 */
export function reloadMapImage(): void {
  const meta = telemetryStore.get().snap.map_meta;
  if (!meta) return;
  loadedVersion = meta.version;
  loadMapImage(meta.version);
}

export async function refreshWaypoints(): Promise<void> {
  const r = await api<{ waypoints?: Waypoint[] }>("/api/waypoints");
  waypointsStore.set(r.waypoints || []);
}

export async function refreshMaps(): Promise<void> {
  const r = await api<{ maps?: MapEntry[] }>("/api/maps");
  mapsStore.set(r.maps || []);
}
