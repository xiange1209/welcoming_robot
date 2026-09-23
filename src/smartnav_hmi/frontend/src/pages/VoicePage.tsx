import { useState } from "react";
import { Button, Card, Field, Hint } from "../components/ui";
import { post, run } from "../lib/api";
import { toast } from "../lib/toast";
import * as tts from "../lib/tts";

export function VoicePage() {
  const [text, setText] = useState("歡迎光臨，我是智慧銀行迎賓機器人");

  return (
    <Card title="語音測試（跳過 LLM 直接播報）" className="min-h-0 flex-1">
      <div className="flex flex-none flex-wrap gap-2">
        <Field
          className="min-w-[200px] flex-1"
          value={text}
          onChange={(e) => setText(e.target.value)}
        />

        {/* ① 瀏覽器播報。這是目前唯一真的會出聲的路徑（車上沒有喇叭） */}
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

        {/* ② 原本的 ROS 語音鏈。留著是為了之後 speech_synthesizer_node 上線時
              能單獨驗它，但現在按下去不會有聲音——把原因講清楚，
              不要讓人以為是網頁壞了。 */}
        <Button
          onClick={async () => {
            const value = text.trim();
            if (!value) return;
            void run(post("/api/speak", { text: value }), () => {
              toast.info(
                "已發到 speech_text —— 沒聲音代表 speech_synthesizer_node 沒在跑",
              );
            });
          }}
        >
          走 ROS 語音鏈
        </Button>
      </div>

      <Hint className="mt-4 leading-8">
        <b className="text-label-2">
          兩條路不一樣，按下去沒反應多半是按到第二條：
        </b>
        <br />
        ①「用這台裝置播報」——走瀏覽器內建的 <code>speechSynthesis</code>
        ，也就是「迎賓」頁那顆朗讀鈕 用的同一套。
        <b className="text-label-2">車上沒有喇叭</b>（唯一的音訊裝置是 Astra S
        的麥克風， 只能錄不能放），所以聲音是從
        <b className="text-label-2">你手上這台平板</b>出來的。這條現在就能用。
        <br />
        ②「走 ROS 語音鏈」——發到 <code>speech_text</code> 話題，需要
        <code>speech_synthesizer_node</code> 在跑、而且 sherpa-onnx 的 TTS
        模型已下載。 現況：
        <b className="text-label-2">兩者都還沒有，所以這條按了不會有聲音</b>
        ，這是預期行為。
      </Hint>
    </Card>
  );
}
