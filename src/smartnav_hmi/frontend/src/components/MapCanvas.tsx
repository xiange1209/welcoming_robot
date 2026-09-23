import { useEffect, useRef } from "react";
import { mapImageStore, waypointsStore } from "../lib/mapdata";
import { drawScene, eventToWorld } from "../lib/mapdraw";
import { telemetryStore } from "../lib/telemetry";
import { cx } from "../lib/cx";

export interface PickedPose {
  x: number;
  y: number;
  yaw: number;
}

/*
 * 地圖畫布。
 *
 * 這是整個介面上更新最頻繁的東西（位姿 10 Hz、建圖時地圖本身也在變），
 * 所以它**完全不走 React 的重繪**：直接訂閱 store 標記 dirty，用 rAF
 * 迴圈畫。好處有三個——
 *   1. 畫面更新率由瀏覽器決定，推播再快也不會堆積。
 *   2. 分頁被隱藏時 rAF 自動停，連算都不算。
 *   3. 拖曳設定朝向時每一個 pointermove 都能立刻反映，不必等 state 回來。
 */
export function MapCanvas({
  active,
  minimal,
  picked,
  onPick,
  onPickEnd,
  className,
  placeholder = "等待地圖…",
}: {
  active: boolean;
  /** 小地圖：只畫地圖與車子，不畫地點名稱與目標點 */
  minimal?: boolean;
  picked?: PickedPose | null;
  /** 回傳 null 代表點到地圖範圍外 */
  onPick?: (pose: PickedPose | null, phase: "down" | "move") => void;
  onPickEnd?: () => void;
  className?: string;
  placeholder?: string;
}) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const hintRef = useRef<HTMLSpanElement>(null);
  const pickedRef = useRef<PickedPose | null>(picked ?? null);
  const dragStart = useRef<{ x: number; y: number } | null>(null);
  const dirty = useRef(true);

  // picked 是 React state（使用者點的）。畫布是命令式的，所以把它同步到 ref
  // 再標記重畫。放在 effect 裡而不是元件本體：算繪期間寫 ref 在並行渲染下不安全。
  useEffect(() => {
    pickedRef.current = picked ?? null;
    dirty.current = true;
  }, [picked]);

  useEffect(() => {
    if (!active) return;
    const canvas = canvasRef.current;
    const ctx = canvas?.getContext("2d");
    if (!canvas || !ctx) return;

    let frame = 0;
    let lastHint = "";

    const mark = () => {
      dirty.current = true;
    };

    const tick = () => {
      frame = requestAnimationFrame(tick);
      if (!dirty.current) return;
      dirty.current = false;

      const snap = telemetryStore.get().snap;
      const hint = drawScene(canvas, ctx, {
        image: mapImageStore.get().image,
        meta: snap.map_meta ?? null,
        pose: snap.robot_pose ?? null,
        navPath: snap.nav_path ?? null,
        waypoints: waypointsStore.get(),
        picked: pickedRef.current,
        minimal,
      });

      // 提示文字直接寫 DOM。走 React 的話這一行會讓整頁每秒重繪十次，
      // 而它的內容大多數影格根本沒變。
      if (hint !== lastHint && hintRef.current) {
        lastHint = hint;
        hintRef.current.textContent = hint;
      }
    };

    const unsubscribes = [
      telemetryStore.subscribe(mark),
      mapImageStore.subscribe(mark),
      waypointsStore.subscribe(mark),
    ];
    const observer = new ResizeObserver(mark);
    observer.observe(canvas);

    dirty.current = true;
    frame = requestAnimationFrame(tick);

    return () => {
      cancelAnimationFrame(frame);
      observer.disconnect();
      for (const off of unsubscribes) off();
    };
  }, [active, minimal]);

  const toWorld = (ev: { clientX: number; clientY: number }) => {
    const canvas = canvasRef.current;
    const ctx = canvas?.getContext("2d");
    if (!canvas || !ctx) return null;
    return eventToWorld(
      canvas,
      ctx,
      mapImageStore.get().image,
      telemetryStore.get().snap.map_meta ?? null,
      ev,
    );
  };

  const interactive = !!onPick;

  return (
    <div
      className={cx(
        "relative min-h-0 overflow-hidden rounded-ios bg-black/45",
        className,
      )}
    >
      <canvas
        ref={canvasRef}
        className={cx("block size-full", interactive ? "cursor-crosshair" : "")}
        style={{ touchAction: "none" }}
        onPointerDown={(e) => {
          if (!interactive) return;
          const w = toWorld(e);
          if (!w) return;
          dragStart.current = w;
          onPick?.({ ...w, yaw: 0 }, "down");
          e.currentTarget.setPointerCapture(e.pointerId);
        }}
        onPointerMove={(e) => {
          if (!interactive || !dragStart.current) return;
          const w = toWorld(e);
          if (!w) return;
          const dx = w.x - dragStart.current.x;
          const dy = w.y - dragStart.current.y;
          // 拖超過 10 公分才視為在指定朝向，否則手指微抖會亂轉
          if (Math.hypot(dx, dy) > 0.1) {
            onPick?.({ ...dragStart.current, yaw: Math.atan2(dy, dx) }, "move");
          }
        }}
        onPointerUp={() => {
          if (!dragStart.current) return;
          dragStart.current = null;
          onPickEnd?.();
        }}
      />

      <span
        ref={hintRef}
        // 這顆浮貼在即時地圖／攝影機畫面上，底色故意跟主題脫鉤（不然淺色
        // 模式的黑底配深色文字幾乎看不見）——底色跟文字都固定不隨主題變。
        className="pointer-events-none absolute bottom-2 left-2 rounded-full bg-black px-2.5 py-1 text-[11px] text-white/70"
      >
        {placeholder}
      </span>
    </div>
  );
}
