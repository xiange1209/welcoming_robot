import { memo, useCallback, useEffect, useLayoutEffect, useRef, useState } from "react";
import type { PointerEvent as RPointerEvent } from "react";
import { IconSend } from "../components/icons";
import { MapCanvas } from "../components/MapCanvas";
import { Button, Card, Field, Hint, Select } from "../components/ui";
import { cx } from "../lib/cx";
import { useStoreSelector, useStoreValue } from "../hooks/useStore";
import { useVideoStream } from "../hooks/useVideoStream";
import { useVisibleInterval } from "../hooks/useVisibleInterval";
import { post, run } from "../lib/api";
import { confirmDialog } from "../lib/dialog";
import { resetCachedMessages, telemetryStore } from "../lib/telemetry";
import * as tts from "../lib/tts";
import type { ChatMessage, ChatStats } from "../lib/types";

const EMPTY: ChatMessage[] = [];

/** 把後端量到的耗時整理成一行標籤 */
function StatsLine({ stats }: { stats: ChatStats }) {
  const parts: [string, string][] = [];
  if (stats.total != null) parts.push(["總耗時", `${stats.total}s`]);
  // 「等待首字」是客人說完到第一個字出現，含檢索與工具；真正的 TTFT 只算模型那一段
  if (stats.think != null) parts.push(["等待首字", `${stats.think}s`]);
  // 拆解：檢索（知識庫）＋工具（第一個字之前跑的）＋TTFT（請求送出到第一個 token）＋其他（ROS、組提示）
  if (stats.retrieve != null) parts.push(["└ 檢索", `${stats.retrieve}s`]);
  if (stats.tool != null) parts.push(["工具", `${stats.tool}s`]);
  if (stats.ttft != null) parts.push(["TTFT", `${stats.ttft}s`]);
  if (stats.other != null) parts.push(["其他", `${stats.other}s`]);
  if (stats.model != null) parts.push(["模型", `${stats.model}s`]);
  if (stats.generate != null) parts.push(["產生", `${stats.generate}s`]);
  if (stats.cps != null) parts.push(["", `${stats.cps} 字/秒`]);
  if (stats.chars != null) parts.push(["", `${stats.chars} 字`]);
  if (!parts.length) return null;

  return (
    <div className="tnum mt-0.5 ml-1 flex flex-wrap gap-2.5 text-[11px] text-label-3">
      {parts.map(([label, value], i) => (
        <span key={i}>
          {label && `${label} `}
          <b className="font-semibold text-label-2">{value}</b>
        </span>
      ))}
    </div>
  );
}

function Bubble({ message }: { message: ChatMessage }) {
  const mine = message.role === "user";

  /* 對話框只放客人說的話與 LLM 的回覆。黑名單公告、「我要倒車…」這類非 LLM 的語音
     平板照樣會唸（朗讀讀的是 messages 清單，見 tts.ts），只是不畫成泡泡。 */
  if (message.hidden) return null;

  return (
    <div className={cx("flex flex-col", mine ? "items-end" : "items-start")}>
      <div
        className={cx(
          "max-w-[86%] rounded-[18px] px-3.5 py-2 text-[15px] leading-relaxed whitespace-pre-wrap portrait:max-lg:max-w-[92%] portrait:max-lg:text-[20px]",
          mine ? "bg-blue text-on-blue" : "bg-[var(--color-well)] text-label",
          message.ghost && "opacity-55 italic",
        )}
      >
        {message.text}
      </div>
      {message.stats && !mine && <StatsLine stats={message.stats} />}
    </div>
  );
}

/**
 * 已完成的訊息。
 *
 * 只訂閱 messages，而後端在對話沒變時**不會帶這個欄位**（前端沿用同一個
 * 陣列參考）——所以 LLM 逐字串流的那整段期間，這份清單一次都不重繪。
 * 動的只有下面那顆 ghost 泡泡。
 */
const MessageList = memo(function MessageList({
  messages,
}: {
  messages: ChatMessage[];
}) {
  return (
    <>
      {messages.map((m, i) => (
        <Bubble key={`${m.ts ?? 0}-${i}`} message={m} />
      ))}
    </>
  );
});

/** 串流中的暫時訊息。後端會把 token 累積起來，所以這裡是逐字長出來的完整段落 */
function GhostBubbles() {
  const ghosts = useStoreSelector(
    telemetryStore,
    (t) => {
      const list: ChatMessage[] = [];
      if (t.snap.llm_streaming)
        list.push({ role: "robot", text: t.snap.llm_streaming, ghost: true });
      if (t.snap.partial_text)
        list.push({ role: "user", text: t.snap.partial_text, ghost: true });
      return list;
    },
    // 內容相同就回舊參考，否則每一則推播都會重建泡泡
    (a, b) =>
      a.length === b.length &&
      a.every((m, i) => m.text === b[i].text && m.role === b[i].role),
  );

  return (
    <>
      {ghosts.map((m, i) => (
        <Bubble key={`ghost-${m.role}-${i}`} message={m} />
      ))}
    </>
  );
}

function ChatScroller() {
  const messages = useStoreSelector(
    telemetryStore,
    (t) => t.snap.messages ?? EMPTY,
  );
  const streamLength = useStoreSelector(
    telemetryStore,
    (t) =>
      (t.snap.llm_streaming?.length ?? 0) + (t.snap.partial_text?.length ?? 0),
  );
  const boxRef = useRef<HTMLDivElement>(null);

  // 串流中每多幾個字就要跟著往下捲，否則新字長在畫面外
  useLayoutEffect(() => {
    const box = boxRef.current;
    if (!box) return;
    box.scrollTop = box.scrollHeight;
  }, [messages, streamLength]);

  return (
    <div
      ref={boxRef}
      className="flex min-h-0 flex-1 flex-col gap-2 overflow-y-auto pr-1"
    >
      {messages.length === 0 && streamLength === 0 && (
        <p className="m-auto text-[13px] text-label-3">
          還沒有對話。打一句話試試看。
        </p>
      )}
      <MessageList messages={messages} />
      <GhostBubbles />
    </div>
  );
}

/** 朗讀設定。聲音清單來自這台裝置的作業系統，不是機器人，所以換一台平板就會不一樣 */
function TtsControls() {
  const state = useStoreValue(tts.ttsStore);
  if (!state.on) return null;

  return (
    <div className="mt-2 flex flex-wrap items-center gap-2 rounded-ios border border-[var(--color-outline-variant)] bg-[var(--color-well)] p-2.5">
      <span className="text-[12px] text-label-3">聲音</span>
      <Select
        className="min-w-[150px] flex-1 text-[13px]"
        value={state.voiceURI}
        onChange={(e) => tts.setVoice(e.target.value)}
      >
        {state.voices.map((v) => (
          <option key={v.uri} value={v.uri}>
            {v.name}（{v.lang}）
          </option>
        ))}
      </Select>

      <span className="text-[12px] text-label-3">語速</span>
      <input
        type="range"
        min={0.6}
        max={1.6}
        step={0.05}
        value={state.rate}
        className="w-[110px]"
        onChange={(e) => tts.setRate(parseFloat(e.target.value))}
        // 放開才試聽——拖曳中每動一格就唸一次會吵死
        onPointerUp={() => tts.speak("語速調整後聽起來像這樣")}
      />
      <span className="tnum w-12 text-[12px] text-label-3">
        {state.rate.toFixed(2)}×
      </span>

      <Button
        className="text-[13px]"
        onClick={() => tts.speak("您好，歡迎光臨。請問需要我帶您過去嗎？")}
      >
        試聽
      </Button>
    </div>
  );
}

function TtsButton() {
  const state = useStoreValue(tts.ttsStore);

  if (!tts.ttsSupported) {
    return (
      <Button disabled title="這個瀏覽器沒有 speechSynthesis">
        朗讀不支援
      </Button>
    );
  }

  const label = state.needsUnlock
    ? "🔇 點一下啟用朗讀"
    : state.status === "error"
      ? "⚠ 朗讀失敗"
      : state.status === "speaking"
        ? "🔊 唸讀中…"
        : state.on
          ? "🔊 朗讀中"
          : "🔇 朗讀";

  return (
    <Button
      // startTts 的解鎖監聽靠這個屬性認出「點的就是這顆按鈕」
      data-tts-toggle
      variant={
        state.status === "error" ? "danger" : state.on ? "primary" : "plain"
      }
      title="用這台裝置的喇叭唸出機器人的回覆"
      onClick={tts.toggle}
    >
      {label}
    </Button>
  );
}

interface Destination {
  waypoint_id: string;
  name: string;
  vip_only: boolean;
}
interface GuideInfo {
  success: boolean;
  can_navigate: boolean;
  tier: string;
  destinations: Destination[];
  busy?: boolean;
}

/* ★ 2026-10-22：訪客帶位。依人臉辨識到的身分顯示能去的地點：
   一般訪客＝一般地點；VIP＝再加貴賓專屬（★）；黑名單＝整列不出現——平板對著客人，什麼都不說最中性。
   這裡只負責顯示，能不能去由後端 /api/guide 依同一份規則再判一次。
   導航沒開（地點服務沒起來）時整列不出現，不跳錯誤提示：迎賓頁不是除錯的地方。 */
function GuideChips({ active }: { active: boolean }) {
  const [info, setInfo] = useState<GuideInfo | null>(null);
  const identityKey = useStoreSelector(telemetryStore, (t) => {
    const i = t.snap.identity || {};
    return `${!!i.recognized}|${i.user_type || ""}|${i.user_name || ""}`;
  });

  const load = useCallback(async () => {
    try {
      const res = await fetch("/api/guide/destinations", { cache: "no-store" });
      const data = (await res.json()) as GuideInfo;
      setInfo(data.success ? data : null);
    } catch {
      setInfo(null);
    }
  }, []);

  useVisibleInterval(load, 15000, active);
  // 換人（身分變了）就馬上重拿，不等下一輪
  useEffect(() => {
    if (active) void load();
  }, [identityKey, active, load]);

  if (!info || !info.can_navigate || info.destinations.length === 0) return null;

  const go = async (d: Destination) => {
    const ok = await confirmDialog({
      title: `要我帶您到「${d.name}」嗎？`,
      message: "請跟著機器人走。途中想停下來，說「停下來」就可以。",
      confirmLabel: "出發",
    });
    if (!ok) return;
    await run(post("/api/guide", { waypoint_id: d.waypoint_id }), () => void load());
  };

  return (
    <div className="mt-3 flex flex-none flex-wrap items-center gap-2">
      <span className="text-[13px] text-label-3">
        {info.tier === "vip" ? "貴賓帶位" : "帶我去"}
      </span>
      {info.destinations.map((d) => (
        <Button
          key={d.waypoint_id}
          disabled={info.busy}
          variant={d.vip_only ? "primary" : "plain"}
          onClick={() => void go(d)}
        >
          {d.vip_only ? `★ ${d.name}` : d.name}
        </Button>
      ))}
      {info.busy && <span className="text-[12px] text-label-3">帶位中…</span>}
    </div>
  );
}

function LlmModel() {
  // 空字串代表 llm_service_node 還沒啟動（模型名稱是 latched 話題，一啟動就會收到）
  const model = useStoreSelector(
    telemetryStore,
    (t) => t.snap.system?.llm_model,
  );
  return <Hint className="mt-2">模型 {model || "— 未連線"}</Hint>;
}

/* ── 現場影像／帶位地圖（★ 2026-10-29）────────────────────────
   帶位中（後端 system.navigating，來自 /navigation_active）：左卡換成地圖、放大到路線，
   相機縮成可以拖動的小窗（× 可以先藏起來，藏起來時影像串流也停）；到點自動換回相機大畫面。
   <img> 從頭到尾是同一個元素，只換外框樣式——useVideoStream 綁的是這個元素，重新掛載會斷串流。
   小窗的位置與隱藏只記在這台平板（localStorage；讀不到就用右下角）。 */
const PIP_KEY = "greet.pip.v1";

interface PipState {
  x: number; // 0～1：在可移動範圍內的水平位置
  y: number;
  hidden: boolean;
}

function clamp01(v: number) {
  return Math.min(1, Math.max(0, v));
}

function loadPip(): PipState {
  try {
    const v = JSON.parse(localStorage.getItem(PIP_KEY) ?? "null") as Partial<PipState> | null;
    if (v && typeof v.x === "number" && typeof v.y === "number") {
      return { x: clamp01(v.x), y: clamp01(v.y), hidden: !!v.hidden };
    }
  } catch {
    /* 無痕模式或儲存被擋：用預設位置 */
  }
  return { x: 1, y: 1, hidden: false };
}

function savePip(p: PipState) {
  try {
    localStorage.setItem(PIP_KEY, JSON.stringify(p));
  } catch {
    /* 存不了就只在這次有效 */
  }
}

function LiveView({ active }: { active: boolean }) {
  const navigating = useStoreSelector(telemetryStore, (t) => !!t.snap.system?.navigating);
  const [routeView, setRouteView] = useState(true); // true＝放大到路線、false＝整張地圖
  const [pip, setPip] = useState<PipState>(loadPip);
  const videoRef = useVideoStream(active && !(navigating && pip.hidden));
  const boxRef = useRef<HTMLDivElement>(null);
  const [box, setBox] = useState({ w: 0, h: 0 });
  const drag = useRef<{ px: number; py: number; left: number; top: number } | null>(null);

  useLayoutEffect(() => {
    const el = boxRef.current;
    if (!el) return;
    const measure = () => setBox({ w: el.clientWidth, h: el.clientHeight });
    const ro = new ResizeObserver(measure);
    ro.observe(el);
    measure();
    return () => ro.disconnect();
  }, []);

  const pw = Math.min(box.w * 0.34, 260);
  const ph = pw * 0.75;
  const freeW = Math.max(0, box.w - pw);
  const freeH = Math.max(0, box.h - ph);
  const left = pip.x * freeW;
  const top = pip.y * freeH;

  const onDown = (e: RPointerEvent<HTMLDivElement>) => {
    e.currentTarget.setPointerCapture(e.pointerId);
    drag.current = { px: e.clientX, py: e.clientY, left, top };
  };
  const onMove = (e: RPointerEvent<HTMLDivElement>) => {
    const d = drag.current;
    if (!d) return;
    const x = freeW > 0 ? clamp01((d.left + e.clientX - d.px) / freeW) : 0;
    const y = freeH > 0 ? clamp01((d.top + e.clientY - d.py) / freeH) : 0;
    setPip((p) => ({ ...p, x, y }));
  };
  const onUp = () => {
    if (!drag.current) return;
    drag.current = null;
    setPip((p) => {
      savePip(p);
      return p;
    });
  };
  const setHidden = (hidden: boolean) =>
    setPip((p) => {
      const next = { ...p, hidden };
      savePip(next);
      return next;
    });

  return (
    <Card
      title={navigating ? "帶位中" : "現場影像"}
      bodyClassName="min-h-[200px]"
      actions={
        navigating ? (
          <div className="flex gap-2">
            <Button variant="ghost" onClick={() => setRouteView((v) => !v)}>
              {routeView ? "全圖" : "放大路線"}
            </Button>
            {pip.hidden && (
              <Button variant="ghost" onClick={() => setHidden(false)}>
                顯示相機
              </Button>
            )}
          </div>
        ) : undefined
      }
    >
      <div
        ref={boxRef}
        className="relative flex min-h-0 flex-1 items-center justify-center overflow-hidden rounded-ios bg-black"
      >
        {navigating && (
          <div className="absolute inset-0">
            <MapCanvas active={active} focusRoute={routeView} className="size-full" placeholder="帶位中…" />
          </div>
        )}
        <div
          className={
            navigating
              ? cx(
                  "absolute z-10 touch-none overflow-hidden rounded-ios border-2 border-white/80 bg-black shadow-lg",
                  pip.hidden && "hidden",
                )
              : "flex size-full items-center justify-center"
          }
          style={navigating ? { left, top, width: pw, height: ph } : undefined}
          onPointerDown={navigating ? onDown : undefined}
          onPointerMove={navigating ? onMove : undefined}
          onPointerUp={navigating ? onUp : undefined}
          onPointerCancel={navigating ? onUp : undefined}
        >
          <img
            ref={videoRef}
            alt="現場影像"
            draggable={false}
            className="pointer-events-none size-full object-contain"
          />
          {navigating && (
            <button
              type="button"
              aria-label="先把相機小窗藏起來"
              className="absolute right-1 top-1 rounded-full bg-black/60 px-2 text-[13px] leading-5 text-white"
              onPointerDown={(e) => e.stopPropagation()}
              onClick={() => setHidden(true)}
            >
              ×
            </button>
          )}
        </div>
      </div>
    </Card>
  );
}

export function GreetPage({ active }: { active: boolean }) {
  const [text, setText] = useState("");

  const send = () => {
    const value = text.trim();
    if (!value) return;
    setText("");
    void run(post("/api/say", { text: value }));
  };

  const clearChat = async () => {
    const ok = await confirmDialog({
      title: "清除所有對話？",
      message: "機器人的對話記憶也會一併清空，無法復原。",
      confirmLabel: "清除",
      destructive: true,
    });
    if (!ok) return;

    await run(post("/api/chat/clear"), () => {
      resetCachedMessages();
      tts.resetSpoken();
    });
  };

  return (
    /* ★ 2026-10-22：iPad Air 11 吋直放（820×1180）時影像 2：對話 3。
       原本兩張卡平分高度，對話只剩 311 px（約四則訊息），影像卡反而左右留黑邊。
       只在直向且未達 lg 時套用，橫放與寬螢幕維持左右兩欄。 */
    <div className="grid min-h-0 flex-1 gap-3 lg:grid-cols-[1.1fr_1fr] portrait:max-lg:grid-rows-[minmax(0,2fr)_minmax(0,3fr)]">
      <LiveView active={active} />

      <Card title="對話">
        <ChatScroller />
        <GuideChips active={active} />

        <div className="mt-3 flex flex-none flex-wrap items-center gap-2">
          <Field
            value={text}
            placeholder="輸入要對機器人說的話…"
            autoComplete="off"
            className="min-w-[160px] flex-1"
            onChange={(e) => setText(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === "Enter") send();
            }}
          />
          <Button variant="primary" onClick={send} aria-label="送出">
            <IconSend className="size-[18px]" />
          </Button>
          <TtsButton />
          <Button variant="danger" onClick={clearChat}>
            清除
          </Button>
        </div>

        <TtsControls />
        <LlmModel />
      </Card>
    </div>
  );
}
