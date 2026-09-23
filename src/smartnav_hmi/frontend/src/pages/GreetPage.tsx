import { memo, useLayoutEffect, useRef, useState } from "react";
import { IconSend } from "../components/icons";
import { Button, Card, Field, Hint, Select } from "../components/ui";
import { cx } from "../lib/cx";
import { useStoreSelector, useStoreValue } from "../hooks/useStore";
import { useVideoStream } from "../hooks/useVideoStream";
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
  if (stats.think != null) parts.push(["首字延遲", `${stats.think}s`]);
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
  return (
    <div className={cx("flex flex-col", mine ? "items-end" : "items-start")}>
      <div
        className={cx(
          "max-w-[86%] rounded-[18px] px-3.5 py-2 text-[15px] leading-relaxed whitespace-pre-wrap",
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

function LlmModel() {
  // 空字串代表 llm_service_node 還沒啟動（模型名稱是 latched 話題，一啟動就會收到）
  const model = useStoreSelector(
    telemetryStore,
    (t) => t.snap.system?.llm_model,
  );
  return <Hint className="mt-2">模型 {model || "— 未連線"}</Hint>;
}

export function GreetPage({ active }: { active: boolean }) {
  const videoRef = useVideoStream(active);
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
    <div className="grid min-h-0 flex-1 gap-3 lg:grid-cols-[1.1fr_1fr]">
      <Card title="現場影像" bodyClassName="min-h-[200px]">
        <div className="flex min-h-0 flex-1 items-center justify-center overflow-hidden rounded-ios bg-black">
          <img
            ref={videoRef}
            alt="現場影像"
            className="size-full object-contain"
          />
        </div>
      </Card>

      <Card title="對話">
        <ChatScroller />

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
