#!/bin/bash
# 語音輸入鏈路啟動器：麥克風 -> VAD -> /audio_in -> ASR -> /user_text
#
# 2026-08-14 建立。只碰 smartnav_audio 的兩個節點，不動相機/人臉/導航。
#
# ── 為什麼要有這支 ────────────────────────────────────────
# 1. Astra S 的音效卡編號每次開機都會變（今天一天內 1 -> 2 -> 3），
#    所以裝置索引一定要當場問 sounddevice，絕對不能寫死。
# 2. 增益有兩個控制項 'Mic',0 與 'Mic',1，兩個都要設，只設一個沒有用。
# 3. 這兩個節點要一起跑才有意義：voice_trigger 只在 VAD 判定有人在講話時
#    才把音訊送上 /audio_in，speech_recognizer 收到才會吐 /user_text。
#
# 用法：
#   run_asr_cc.sh            啟動
#   run_asr_cc.sh stop       停止

set +u
source /home/user/maprun/env.sh

LOGDIR=/home/user/maprun/logs_0814
mkdir -p "$LOGDIR"

if [ "$1" = "stop" ]; then
    /home/user/maprun/kill_node_cc.sh smartnav_audio voice_trigger
    /home/user/maprun/kill_node_cc.sh smartnav_audio speech_recognizer
    exit 0
fi

# ── 1. 找出 Astra S 的錄音卡號與 sounddevice 索引 ─────────
CARD=$(arecord -l 2>/dev/null | grep -i 'ASTRA S' | sed -E 's/^card ([0-9]+):.*/\1/' | head -1)
if [ -z "$CARD" ]; then
    echo "[run_asr] 找不到 ASTRA S 錄音裝置。相機沒插好就沒有麥克風，請重插 Astra S。" >&2
    exit 1
fi

IDX=$(python3 - <<'PYEOF' 2>/dev/null
import sounddevice as sd
for i, d in enumerate(sd.query_devices()):
    if d["max_input_channels"] > 0 and "ASTRA" in d["name"].upper():
        print(i)
        break
PYEOF
)
if [ -z "$IDX" ]; then
    echo "[run_asr] sounddevice 看不到 ASTRA S（PortAudio 沒裝好？）" >&2
    exit 1
fi
echo "[run_asr] ALSA card=$CARD, sounddevice index=$IDX"

# ── 2. 設定增益：兩個控制項都要動 ──────────────────────────
# ★ 2026-08-14 實測校準：48 (24 dB) 太小，VAD 一次都沒觸發過。
#   同一支麥克風、同一個人、同樣距離的三檔對照（15 秒實錄，看底噪與人聲的差）：
#
#     48  (24 dB)  底噪 -35 dBFS，人聲埋在底噪裡，整段 30 區間全平坦  -> VAD 不觸發
#     90  (45 dB)  底噪 -14 dBFS、峰值 0.0 dBFS **削波**，放大的是噪音 -> 一樣不觸發
#     66  (33 dB)  底噪 -26.5 dBFS，人聲高出 6~13 dB，5/30 區間明顯   -> ★ 可用
#
#   66 這一檔實測餵給 sherpa-onnx 認得出「你好请问柜台在哪里」，即時率 0.76x。
#   ⚠ 這是在**安靜環境、距離約 30~50 cm** 量的。換場地或距離要重量，方法：
#     arecord -D hw:$CARD,0 -f S16_LE -r 16000 -c 2 -d 15 /tmp/t.wav
#     然後比較「安靜時的 RMS」與「講話時的 RMS」，差 8 dB 以上才夠 VAD 用。
#   ★ 左聲道才是真的麥克風（實測比右聲道大 5 dB），分析時取 channel 0。
GAIN="${ASR_MIC_GAIN:-66}"
amixer -c "$CARD" sset 'Mic',0 "$GAIN" cap >/dev/null 2>&1
amixer -c "$CARD" sset 'Mic',1 "$GAIN" cap >/dev/null 2>&1
echo "[run_asr] Mic,0 與 Mic,1 增益都設為 $GAIN (0-120)"

# ── 3. 起節點 ─────────────────────────────────────────────
# num_threads 壓到 2：Pi 4 只有四核，導航與相機還在跑，開 4 條會把機器拖垮。
nohup ros2 run smartnav_audio speech_recognizer \
    --ros-args -p num_threads:=2 \
    > "$LOGDIR/asr_recognizer.log" 2>&1 &
echo "[run_asr] speech_recognizer 已啟動（載入模型約需 10-25 秒）"

nohup ros2 run smartnav_audio voice_trigger \
    --ros-args -p device:=$IDX -p vad_num_threads:=1 \
    > "$LOGDIR/asr_voice_trigger.log" 2>&1 &
echo "[run_asr] voice_trigger 已啟動"

echo
echo "[run_asr] 看辨識結果： grep 最終結果 $LOGDIR/asr_recognizer.log"
echo "[run_asr] 停止：       $0 stop"
