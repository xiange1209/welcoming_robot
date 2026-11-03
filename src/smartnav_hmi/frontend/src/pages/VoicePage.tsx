import { useState } from "react";
import {
  Badge,
  Button,
  Card,
  Field,
  Hint,
  Row,
  Segmented,
} from "../components/ui";
import { VoiceWave } from "../components/VoiceWave";
import { useStoreSelector, useStoreValue } from "../hooks/useStore";
import { post, run } from "../lib/api";
import { confirmDialog } from "../lib/dialog";
import { httpsUrl, micStore, startMic, stopMic } from "../lib/mic";
import { telemetryStore } from "../lib/telemetry";
import { toast } from "../lib/toast";
import * as tts from "../lib/tts";
import type { VoiceMode } from "../lib/types";

/** ★ 2026-11-03：機器人朗讀開關（跟迎賓頁「朗讀」鈕是同一個設定） */
export function RobotTtsToggle({ compact = false }: { compact?: boolean }) {
  const on = useStoreSelector(
    telemetryStore,
    (t) => t.snap.system?.robot_tts_on ?? true,
  );
  const [busy, setBusy] = useState(false);
  return (
    <Button
      variant={on ? "primary" : "plain"}
      className={compact ? undefined : "self-start text-[13px]"}
      disabled={busy}
      title="機器人喇叭朗讀開關：關掉時機器人不合成、不出聲（正在播的也會停）"
      onClick={async () => {
        setBusy(true);
        await run(post("/api/voice/robot_tts", { on: !on }));
        setBusy(false);
      }}
    >
      {on ? "🔊 朗讀中" : "🔇 朗讀"}
    </Button>
  );
}

/** ★ 2026-11-03：語音輸出／輸入切換。設定存在機器人上，每台平板看到的都一樣 */
function VoiceModeCard() {
  const sys = useStoreSelector(telemetryStore, (t) => t.snap.system);
  const output: VoiceMode = sys?.voice_output ?? "browser";
  const input: VoiceMode = sys?.voice_input ?? "browser";
  const hw = sys?.audio_hw;
  const status = sys?.voice_input_status ?? "";
  const httpsPort = sys?.https_port ?? 0;
  const [busy, setBusy] = useState(false);

  const save = async (patch: { output?: VoiceMode; input?: VoiceMode }) => {
    // ★ 切到「機器人」前先看硬體：偵測不到就先問，免得切過去才發現沒聲音／聽不到
    if (patch.input === "robot" && hw && !hw.mic) {
      const go = await confirmDialog({
        title: "偵測不到機器人的麥克風",
        message: `${hw.mic_detail}。切過去會完全收不到聲音（Astra S 的麥克風跟相機同一顆，沒插好會一起消失）。仍要切換？`,
        confirmLabel: "仍要切換",
      });
      if (!go) return;
    }
    if (patch.output === "robot" && hw && hw.speaker === "none") {
      const go = await confirmDialog({
        title: "偵測不到機器人的播放裝置",
        message: `${hw.speaker_detail}。切過去機器人合成了也放不出聲音，平板也不會唸。仍要切換？`,
        confirmLabel: "仍要切換",
      });
      if (!go) return;
    }
    setBusy(true);
    await run(post("/api/voice/settings", patch), () =>
      toast.success("語音設定已更新"),
    );
    setBusy(false);
  };

  const micBadge =
    input === "browser" ? (
      sys?.mic_connected ? (
        <Badge tone="ok">平板麥克風已連線</Badge>
      ) : (
        <Badge tone="muted">尚無平板在錄音</Badge>
      )
    ) : status === "robot:ok" ? (
      <Badge tone="ok">機器人麥克風收音中</Badge>
    ) : status.startsWith("robot:error") ? (
      <>
        <Badge tone="bad">機器人麥克風開不起來</Badge>
        <span className="text-[12px] text-label-3">
          {status.slice("robot:error:".length)}
        </span>
      </>
    ) : (
      <Badge tone="muted">ASR 未回報（asr_chain 沒在跑？）</Badge>
    );

  const secureUrl = httpsUrl(httpsPort);

  return (
    <Card title="語音輸出／輸入" className="flex-none">
      <div className="flex flex-col gap-2.5">
        <Row>
          <span className="w-20 flex-none text-label-2">語音輸出</span>
          <Segmented<VoiceMode>
            className="flex-1"
            value={output}
            options={[
              { value: "browser", label: "平板朗讀" },
              { value: "robot", label: "機器人喇叭" },
            ]}
            onChange={(v) => !busy && v !== output && void save({ output: v })}
          />
        </Row>
        {output === "robot" && hw && hw.speaker !== "usb" && (
          <Hint className={hw.speaker === "none" ? "text-red" : "text-orange"}>
            ⚠{" "}
            {hw.speaker === "none"
              ? `偵測不到播放裝置：${hw.speaker_detail}`
              : "只偵測到 Pi 的耳機孔——硬體上無法確認有沒有插喇叭，請自己確認有接、音量有開。"}
          </Hint>
        )}
        <Hint>
          {output === "browser" ? (
            <>
              平板用瀏覽器內建語音唸（迎賓頁的「朗讀」鈕要開）。機器人的 TTS
              模型此時不合成、不出聲。
            </>
          ) : (
            <>
              機器人用本機 TTS
              模型合成、從耳機孔的喇叭播放，平板不唸。需要「系統開關」裡的
              <b className="text-label-2"> 語音合成 </b>與
              <b className="text-label-2"> 語音播放 </b>
              兩個單元在跑，而且耳機孔要接喇叭。
            </>
          )}
        </Hint>

        <Row>
          <span className="w-20 flex-none text-label-2">語音輸入</span>
          <Segmented<VoiceMode>
            className="flex-1"
            value={input}
            options={[
              { value: "browser", label: "平板錄音" },
              { value: "robot", label: "機器人麥克風" },
            ]}
            onChange={(v) => !busy && v !== input && void save({ input: v })}
          />
        </Row>
        <div className="flex flex-wrap items-center gap-2">{micBadge}</div>
        {input === "robot" && hw && !hw.mic && (
          <Hint className="text-red">
            ⚠ 偵測不到機器人的麥克風：{hw.mic_detail}。Astra S
            的麥克風跟相機是同一顆 USB 裝置，手動插拔一次試試。
          </Hint>
        )}
        <Hint>
          {input === "robot" ? (
            <>
              機器人自己的麥克風收音、本機 ASR 模型辨識。需要「系統開關」的
              <b className="text-label-2"> 語音辨識（asr_chain）</b>
              在跑，而且機器人要接上麥克風。
            </>
          ) : (
            <>
              平板只負責錄音，聲音傳回機器人用本機 ASR 模型辨識——
              <b className="text-label-2">不經過 Google／Apple 雲端</b>。
              一樣需要<b className="text-label-2"> 語音辨識（asr_chain）</b>
              在跑（斷句與辨識在機器人上），機器人不必接麥克風。 在迎賓頁按「🎤
              開始聆聽」開始；同一時間只收一台平板。
              <br />
              瀏覽器只在 <b className="text-label-2">https</b> 下開放麥克風：
              {window.isSecureContext ? (
                <> 這台已經是安全連線 ✓</>
              ) : secureUrl ? (
                <>
                  {" "}
                  這台目前是 http，請改用{" "}
                  <a className="text-blue underline" href={secureUrl}>
                    {secureUrl}
                  </a>
                  （自簽憑證，第一次要按「繼續前往」）
                </>
              ) : (
                <> 機器人沒有開 https（看 HMI 啟動 log）</>
              )}
            </>
          )}
        </Hint>
      </div>
    </Card>
  );
}

/** ★ 2026-11-03：語音輸入波形＋辨識結果，測試收音用 */
function VoiceMonitorCard() {
  const mic = useStoreValue(micStore);
  const input = useStoreSelector(
    telemetryStore,
    (t) => t.snap.system?.voice_input ?? "browser",
  );
  const httpsPort = useStoreSelector(
    telemetryStore,
    (t) => t.snap.system?.https_port ?? 0,
  );
  const partial = useStoreSelector(
    telemetryStore,
    (t) => t.snap.partial_text ?? "",
  );
  const lastHeard = useStoreSelector(telemetryStore, (t) => {
    const list = t.snap.messages ?? [];
    for (let i = list.length - 1; i >= 0; i--)
      if (list[i].role === "user") return list[i].text;
    return "";
  });

  return (
    <Card
      title="語音輸入監看"
      className="flex-none"
      actions={
        input === "browser" ? (
          <Button
            variant={
              mic.status === "error" ? "danger" : mic.on ? "primary" : "plain"
            }
            className="text-[13px]"
            onClick={() => (mic.on ? stopMic() : void startMic(httpsPort))}
          >
            {mic.status === "listening"
              ? "🎤 停止平板錄音"
              : mic.status === "connecting"
                ? "🎤 連線中…"
                : "🎤 開始平板錄音"}
          </Button>
        ) : undefined
      }
    >
      <VoiceWave showLocal={input === "browser"} resetKey={input} />
      {mic.error && <Hint className="mt-1 text-red">{mic.error}</Hint>}
      <div className="mt-2 flex flex-col gap-1 text-[13px]">
        <div>
          <span className="text-label-3">辨識中：</span>
          <span className="text-label-2">{partial || "—"}</span>
        </div>
        <div>
          <span className="text-label-3">最後一句：</span>
          <span>{lastHeard || "—"}</span>
        </div>
      </div>
      <Hint className="mt-1">
        {input === "browser"
          ? "上面一條是平板麥克風本身收到的、下面一條是機器人實際處理的。上面有波形、下面沒有＝平板有錄但沒送到機器人（https？asr_chain？）。"
          : "機器人麥克風模式只有下面一條。完全平的線＝麥克風沒收到聲音（沒插好或音量 0）。"}
        講完一句後下方會有一段藍底（收句），收完才送辨識；說話時變綠才代表 VAD
        有聽到人聲。
      </Hint>
    </Card>
  );
}

export function VoicePage() {
  const [text, setText] = useState("歡迎光臨，我是智慧銀行迎賓機器人");

  return (
    <div className="flex min-h-0 flex-1 flex-col gap-3 overflow-y-auto">
      <VoiceModeCard />
      <VoiceMonitorCard />
      <Card title="語音測試（跳過 LLM 直接播報）" className="flex-none">
        <div className="flex flex-none flex-wrap gap-2">
          <Field
            className="min-w-[200px] flex-1"
            value={text}
            onChange={(e) => setText(e.target.value)}
          />

          {/* ① 瀏覽器播報：不管「語音輸出」設哪一邊，這顆都用這台平板唸（測試用） */}
          <Button
            variant="primary"
            onClick={() => {
              const value = text.trim();
              if (!value) return;
              if (!tts.ttsSupported) {
                toast.error("這個瀏覽器不支援語音合成");
                return;
              }
              tts.speak(value);
              toast.info("已用這台裝置播報 (音量與靜音鍵要開)");
            }}
          >
            用這台裝置播報
          </Button>

          {/* ② 機器人的 ROS 語音鏈（speech_text -> speech_synthesizer -> voice_playback） */}
          <Button
            onClick={async () => {
              const value = text.trim();
              if (!value) return;
              void run(post("/api/speak", { text: value }), () => {
                toast.info(
                  tts.outputIsBrowser()
                    ? "已發到 speech_text —— 語音輸出目前是「平板朗讀」，機器人喇叭不會出聲"
                    : "已發到 speech_text —— 沒聲音的話看語音合成／語音播放單元有沒有在跑",
                );
              });
            }}
          >
            走機器人語音鏈
          </Button>
        </div>

        <Hint className="mt-4 leading-8">
          ①「用這台裝置播報」——瀏覽器內建的 <code>speechSynthesis</code>，聲音從
          <b className="text-label-2">你手上這台平板</b>出來，跟上面的設定無關。
          <br />
          ②「走機器人語音鏈」——發到 <code>speech_text</code>，由機器人上的
          <code>speech_synthesizer_node</code>（sherpa-onnx）合成、
          <code>voice_playback_node</code>
          從耳機孔播放。語音輸出設成「平板朗讀」時機器人刻意不出聲（不然兩邊會同時講）。
        </Hint>
      </Card>
    </div>
  );
}
