import { useCallback, useEffect, useState } from "react";
import { useStoreSelector, useStoreValue } from "../hooks/useStore";
import { post, run } from "../lib/api";
import { mapsStore } from "../lib/mapdata";
import { sessionStore } from "../lib/session";
import { shallowEqual } from "../lib/store";
import * as teleop from "../lib/teleop";
import { faceAge, presenceStore, telemetryStore } from "../lib/telemetry";
import { IconPalette } from "./icons";
import { ThemeSheet } from "./ThemeSheet";
import { Badge, Button } from "./ui";
import { cx } from "../lib/cx";

/* ══════════════════════════════════════════════════════════════
   頂部狀態列

   每一個數值都是獨立訂閱的小元件。推播是 10 Hz，而這一列裡只有
   車速真的每次都在變——分開訂閱之後，其餘幾個一分鐘才動一次的
   數值就不會跟著每秒重繪十次。
   ══════════════════════════════════════════════════════════════ */

/** 身分橫幅的底色。黑名單用紅色而且明顯——這是要讓行員一眼看到的警示。
 *
 *  ⚠ 但車上的語音**刻意只說中性的話**（「已通知行員前來協助」）：
 *    當著本人的面播報「偵測到黑名單」會讓情況立刻升溫，而且對誤判的人
 *    是嚴重冒犯。細節只給行員看，不出聲。 */
const TINT: Record<string, string> = {
  VIP: "from-gold/22 to-transparent",
  ADMIN: "from-blue/22 to-transparent",
  BLACKLIST: "from-red/30 to-transparent",
};

function Identity() {
  const { present, boxed } = useStoreValue(presenceStore);

  const id = useStoreSelector(
    telemetryStore,
    (t) => {
      const i = t.snap.identity || {};
      return {
        recognized: !!i.recognized,
        name: i.user_name || "",
        type: (i.user_type as string) || "",
        similarity: i.similarity,
        description: i.description || "",
      };
    },
    shallowEqual,
  );

  // 不新鮮、或這一幀根本沒有人臉框，就當成「沒有人」
  const shown = present && boxed ? id : null;

  const who = shown
    ? shown.recognized
      ? shown.name
      : shown.name
        ? "訪客"
        : "等待辨識…"
    : "等待辨識…";
  const sub = shown
    ? shown.recognized
      ? `${shown.description || "已認證"}　相似度 ${shown.similarity}`
      : `未在資料庫中（相似度 ${shown.similarity}）`
    : Number.isFinite(faceAge())
      ? "鏡頭前沒有人"
      : "尚未偵測到人臉";

  const tint = shown?.recognized ? TINT[shown.type] : undefined;

  return (
    <div
      className={cx(
        "relative min-w-0 flex-1 rounded-ios px-3 py-1.5 transition-colors duration-500",
        tint && `bg-gradient-to-r ${tint}`,
      )}
    >
      <div className="flex items-center gap-2">
        <span className="truncate text-[20px] leading-tight font-semibold">
          {who}
        </span>
        {shown?.recognized && shown.type && (
          <Badge tone={shown.type}>{shown.type}</Badge>
        )}
      </div>
      <div className="truncate text-[12.5px] text-label-3">{sub}</div>
    </div>
  );
}

/** 一格狀態數值。warn 為真時轉紅，其餘維持灰色不搶注意力 */
function Chip({ text, warn }: { text: string; warn?: boolean }) {
  return (
    <span
      className={cx(
        "tnum rounded-full px-2.5 py-1 text-[12px] whitespace-nowrap transition-colors",
        warn ? "bg-red/15 font-semibold text-red" : "text-label-3",
      )}
    >
      {text}
    </span>
  );
}

function Connection() {
  const connected = useStoreSelector(telemetryStore, (t) => t.connected);
  return (
    <span className="flex items-center gap-1.5 px-1 text-[12px] text-label-3">
      <span
        className={cx(
          "size-2 rounded-full transition-colors duration-300",
          connected
            ? "bg-green shadow-[0_0_8px_rgba(48,209,88,0.8)]"
            : "bg-red",
        )}
      />
      連線
    </span>
  );
}

function Fps() {
  const fps = useStoreSelector(
    telemetryStore,
    (t) => t.snap.system?.camera_fps,
  );
  return <Chip text={"相機 " + (fps ? `${fps} fps` : "—")} />;
}

function Voltage() {
  // 電壓與充電狀態要一起看：充電中電壓會被充電器拉高，
  // 只看電壓會把「插著電」誤讀成「電池很飽」。
  const v = useStoreSelector(
    telemetryStore,
    (t) => {
      const s = t.snap.system || {};
      return {
        voltage: s.voltage,
        charging: !!s.charging,
        current: s.charge_current,
      };
    },
    shallowEqual,
  );

  let text = "電壓 " + (v.voltage != null ? `${v.voltage} V` : "—");
  if (v.charging) text += v.current ? ` ⚡充電 ${v.current}A` : " ⚡充電中";

  return (
    <Chip
      text={text}
      warn={v.voltage != null && !v.charging && v.voltage < 22.0}
    />
  );
}

function Speed() {
  // 脫困、防撞死鎖那類問題的第一個問診點是「有下指令但車不動」
  const speed = useStoreSelector(telemetryStore, (t) => t.snap.system?.speed);
  const text =
    speed != null
      ? Math.abs(speed) < 0.01
        ? "靜止"
        : `${speed.toFixed(2)} m/s`
      : "—";
  return <Chip text={`車速 ${text}`} />;
}

function Cpu() {
  // Pi 4 四核：負載 > 4 就是滿載；溫度 80°C 開始降頻，導航跑起來很接近
  const c = useStoreSelector(
    telemetryStore,
    (t) => ({ temp: t.snap.system?.cpu_temp, load: t.snap.system?.cpu_load }),
    shallowEqual,
  );
  const text =
    "CPU " +
    (c.temp != null ? `${c.temp}°C` : "—") +
    (c.load != null ? ` / 負載 ${c.load}` : "");
  return (
    <Chip
      text={text}
      warn={
        (c.temp != null && c.temp >= 75) || (c.load != null && c.load >= 4.0)
      }
    />
  );
}

function MapName() {
  const mapId = useStoreSelector(telemetryStore, (t) => t.snap.system?.map_id);
  const maps = useStoreValue(mapsStore);
  const hit = maps.find((m) => m.map_id === mapId);
  return <Chip text={"地圖 " + (hit ? hit.map_name : mapId || "—")} />;
}

/**
 * 全域緊急停止。
 *
 * 綁 pointerdown 不是 click：click 要等 pointerup 才觸發，觸控裝置上還可能
 * 有延遲。急停不能等。
 *
 * 刻意**不做二次確認**——誤按緊急停止的代價只是車子停下來，而確認對話框
 * 會在真正需要停車的那幾秒鐘要人多做一個動作。對危險操作要確認、對安全
 * 操作不要，這條的方向是後者。
 */
function EmergencyStop() {
  const [busy, setBusy] = useState(false);
  const [done, setDone] = useState(false);

  const fire = useCallback(async () => {
    if (busy) return;
    // 先停本地遙控心跳，不要等網路來回
    teleop.stop();
    setBusy(true);
    setDone(false);

    try {
      await run(post("/api/estop", {}), () => {
        setDone(true);
        setTimeout(() => setDone(false), 1200);
      });
    } finally {
      setBusy(false);
    }
  }, [busy]);

  // 鍵盤：空白鍵當急停（筆電操作時最直覺）
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.code !== "Space") return;
      const t = e.target as HTMLElement | null;
      if (
        t &&
        (t.tagName === "INPUT" ||
          t.tagName === "TEXTAREA" ||
          t.isContentEditable)
      )
        return;
      e.preventDefault();
      void fire();
    };
    document.addEventListener("keydown", onKey);
    return () => document.removeEventListener("keydown", onKey);
  }, [fire]);

  return (
    <button
      type="button"
      title="立即停止所有動作（空白鍵）"
      // 送出中擋住重複點擊，但不改變大小以免手指跟丟
      className={cx(
        "tap pressable flex-none rounded-ios border border-red/50 bg-red/85 px-3.5 text-[13px] font-semibold text-on-red",
        busy && "pointer-events-none opacity-55",
        done && "anim-ripple",
      )}
      onPointerDown={(e) => {
        e.preventDefault();
        void fire();
      }}
    >
      緊急停止
    </button>
  );
}

export function TopBar({
  onLogin,
  onLogout,
}: {
  onLogin: () => void;
  onLogout: () => void;
}) {
  const { admin } = useStoreValue(sessionStore);
  const [themeOpen, setThemeOpen] = useState(false);

  return (
    <>
      <header className="surface sticky top-0 z-40 flex flex-col gap-1.5 rounded-none border-x-0 border-t-0 px-3 py-2 pt-[calc(env(safe-area-inset-top)+8px)]">
        <div className="flex items-center gap-2">
          <Identity />
          <div className="flex flex-none items-center gap-2">
            <Button
              variant="ghost"
              aria-label="主題設定"
              onClick={() => setThemeOpen(true)}
            >
              <IconPalette className="size-5" />
            </Button>
            {admin && <EmergencyStop />}
            <Button
              variant={admin ? "primary" : "plain"}
              onClick={admin ? onLogout : onLogin}
            >
              {admin ? "登出" : "管理者登入"}
            </Button>
          </div>
        </div>

        <div className="flex flex-wrap items-center gap-1">
          <Connection />
          <Fps />
          <Voltage />
          <Speed />
          <Cpu />
          <MapName />
        </div>
      </header>

      {themeOpen && <ThemeSheet onClose={() => setThemeOpen(false)} />}
    </>
  );
}
