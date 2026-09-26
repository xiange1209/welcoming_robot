#!/bin/bash
# 錄一段 Astra S 的雙聲道音訊，給雙麥克風分析用。
#
# 用法：
#   rec_dual_cc.sh 安靜 10      錄 10 秒安靜（不要講話、不要走動）
#   rec_dual_cc.sh 講話 15      錄 15 秒，站在車頭正前方 50 cm 講話
#
# 檔案會存到 /home/user/maprun/tools_0817/rec/<名稱>_<時間>.wav
#
# ★ 卡號每次開機都會變，所以一定要當場抓，不能寫死。
# ★ 會先停掉 ASR，否則錄音裝置被 voice_trigger 佔住，arecord 會失敗。

NAME="${1:-錄音}"
SECS="${2:-10}"
OUTDIR=/home/user/maprun/tools_0817/rec
mkdir -p "$OUTDIR"

CARD=$(arecord -l 2>/dev/null | grep -i 'ASTRA S' | sed -E 's/^card ([0-9]+):.*/\1/' | head -1)
if [ -z "$CARD" ]; then
    echo "✗ 找不到 ASTRA S 錄音裝置。相機沒插好就沒有麥克風，請重插 Astra S。" >&2
    exit 1
fi

# 停掉 ASR 以釋放裝置（用 kill_node_cc.sh，不要用 pkill -f）
/home/user/maprun/run_asr_cc.sh stop >/dev/null 2>&1

# 增益固定在實測甜蜜點 66 (33 dB)，兩個控制項都要設
amixer -c "$CARD" sset 'Mic',0 66 cap >/dev/null 2>&1
amixer -c "$CARD" sset 'Mic',1 66 cap >/dev/null 2>&1

OUT="$OUTDIR/${NAME}_$(date +%H%M%S).wav"
echo "card=$CARD  增益=66(33dB)  時間=${SECS}s"
echo "→ 開始錄音：$OUT"
echo
arecord -D "hw:$CARD,0" -f S16_LE -r 16000 -c 2 -d "$SECS" "$OUT" || exit 1
echo
echo "完成：$OUT"
echo "接著跑： python3 /home/user/maprun/tools_0817/mic_snr_cc.py --quiet 安靜檔 --speech $OUT"
