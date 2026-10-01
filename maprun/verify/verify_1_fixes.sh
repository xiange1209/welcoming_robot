#!/usr/bin/env bash
# V1 驗證 9/18 那批修正「真的生效」，不是「有部署」。
#
# ★★ 本專案鐵則：ros2 param get 不算驗證。
#    9/17 就是這樣被騙的 —— param get 說 15、驅動 log 也印 15，
#    ros2 topic hz 一量是 29.64 Hz。只有量實際輸出才算數。
#
# 前置：先用 HMI 或指令把「人臉模式」開起來
#   ~/maprun/run_camera_face_cc.sh 15 85
#   ~/maprun/run_node_cc.sh smartnav_vision face_embedding face --ros-args -p enable_gpu:=false
#
# 用法：~/maprun/verify/verify_1_fixes.sh

source "$(dirname "$0")/_lib.sh"

echo "V1 修正生效驗證   $(date '+%Y-%m-%d %H:%M:%S')"

# ── 1. 相機幀率：量輸出，不看參數 ────────────────────────────
hdr "相機幀率（9/17 的 color_fps 假象）"
echo "  量測 12 秒..."
HZ=$(hz_of /camera/color/image_raw/compressed 12)
if [ "$(awk -v h="$HZ" 'BEGIN{print (h==0)?1:0}')" = 1 ]; then
  rec "彩色影像實際幀率" FAIL "量不到" "相機沒開？先跑 run_camera_face_cc.sh 15 85"
elif [ "$(awk -v h="$HZ" 'BEGIN{print (h<18)?1:0}')" = 1 ]; then
  rec "彩色影像實際幀率" PASS "${HZ} Hz" "要求 <18；9/17 實測 29.64 Hz 代表降幀從未生效"
else
  rec "彩色影像實際幀率" FAIL "${HZ} Hz" "降幀仍未生效！★ 報告引用過的 CPU 節省值要重算"
fi

# ── 2. 人臉節點 CPU：驗 process_interval_sec 預設值改了沒 ────
# ★ 2026-09-30：cpu_of 改量瞬時值（/proc 差分，取樣 5 秒）。舊版 ps pcpu 是生涯平均，
#   9/17 的「127%」是假象；車子端 9/22 瞬時量到 1.0 → 203%、2.0 → 114%。預設已改 2.0。
hdr "人臉辨識 CPU（process_interval_sec 預設 2.0，量瞬時值）"
FCPU=$(cpu_of "face_embedding" 5)
if [ "$(awk -v c="$FCPU" 'BEGIN{print (c<1)?1:0}')" = 1 ]; then
  rec "face_embedding CPU" WARN "${FCPU}%" "節點沒在跑，這項跳過"
elif [ "$(awk -v c="$FCPU" 'BEGIN{print (c<160)?1:0}')" = 1 ]; then
  rec "face_embedding CPU" PASS "${FCPU}%" "預設 2.0 約 114%（9/22 瞬時量測）；1.0 約 203%＝綁定的 2 核吃滿"
else
  rec "face_embedding CPU" FAIL "${FCPU}%" "≥160% = 還在用 1.0 或 0.0（約 200%），語音鏈會被餓死；車上是 9/30 之後的版本嗎？有人 param set 過嗎？"
fi

# 偵測輸出頻率：每 2 秒處理一幀 → 有臉時約 0.5 Hz（單幀推論約 1004 ms，所以 1.0 秒間隔幾乎等於不限制）
EHZ=$(hz_of /face_embedding 12)
rec "人臉偵測輸出" INFO "${EHZ} Hz" "鏡頭前有人時預期約 0.5 Hz；沒人時 0（只在偵測到臉時才發布）"

# 全機負載
LOAD=$(cut -d' ' -f1 /proc/loadavg)
rec "系統 1 分鐘負載" INFO "$LOAD" "四核心，9/17 預設值飽和時是 14.7"

# ── 3. 語音模型路徑：找不到現在會印 error ────────────────────
hdr "語音模型路徑（原本靜默失效）"
# ★★ 2026-09-22 車子端修正（9/30 併回）：原本寫 ~/maprun/logs/asr.log、vad.log，但**沒有任何程式
#    會產生那兩個檔** —— run_asr_cc.sh 的 LOGDIR 是 logs_0814（名字會騙人：裝的是**現在**的 ASR log），
#    檔名是 asr_recognizer.log／asr_voice_trigger.log。照原路徑這兩項永遠 WARN，等於驗不到。
# ★ 2026-09-30：9/26 起平板改用 LaunchSupervisor 起語音鏈，recognizer 與 trigger 的輸出都寫在
#    ~/.smartnav/logs/system_control/asr_chain_<時間>.log（每次啟動一個新檔）—— 兩邊一起找、取最新
SC_LOGDIR="$HOME/.smartnav/logs/system_control"
ALOG=$(ls -t "$HOME/maprun/logs_0814/asr_recognizer.log" "$SC_LOGDIR"/asr_chain_*.log 2>/dev/null | head -1)
VLOG=$(ls -t "$HOME/maprun/logs_0814/asr_voice_trigger.log" "$SC_LOGDIR"/asr_chain_*.log 2>/dev/null | head -1)
if [ -z "$ALOG" ]; then
  rec "ASR 模型載入" WARN "找不到語音鏈的 log" "先在平板開「語音輸入」（或跑 run_asr_cc.sh）再回來驗"
elif grep -q "找不到 asr 模型目錄" "$ALOG"; then
  rec "ASR 模型載入" FAIL "找不到模型" "$(grep -A6 '找不到 asr 模型目錄' "$ALOG" | tail -5 | tr '\n' ' ')"
elif grep -q "找到 asr 模型目錄" "$ALOG"; then
  rec "ASR 模型載入" PASS "$(grep '找到 asr 模型目錄' "$ALOG" | tail -1 | sed 's/.*目錄: //')" "修正前這裡只印 warning 就繼續，辨識器從未建立"
else
  rec "ASR 模型載入" WARN "log 裡沒有相關訊息" "確認跑的是新版"
fi

if [ -n "$ALOG" ]; then
  rec "（分析的語音 log）" INFO "$ALOG" "要是這一次的啟動；舊的就先重開語音鏈"
fi
if [ -n "$VLOG" ] && grep -q "找到 vad 模型目錄" "$VLOG"; then
  rec "VAD 模型載入" PASS "已找到" ""
elif [ -n "$VLOG" ]; then
  rec "VAD 模型載入" FAIL "未找到" "VAD 不觸發 = 講話完全沒反應；~/models/vad 底下要有 silero_vad.onnx"
fi

# ── 4. 麥克風增益預設值 ──────────────────────────────────────
hdr "麥克風增益（家裡 66 → 實驗室 60 + left）"
G=$(grep -o 'ASR_MIC_GAIN:-[0-9]*' "$HOME/maprun/run_asr_cc.sh" 2>/dev/null | grep -o '[0-9]*$')
# ★★ 2026-09-22 車子端修正（9/30 併回）：`ASR_MIC_MODE:-left` 只有**一個**減號，原本的
#    `cut -d- -f3` 恆為空 → 這項永遠 FAIL 並印「增益 60、?」，其實設定是對的。改成 f2。
M=$(grep -o 'ASR_MIC_MODE:-[a-z]*' "$HOME/maprun/run_asr_cc.sh" 2>/dev/null | cut -d- -f2)
if [ "$G" = "60" ] && [ "$M" = "left" ]; then
  rec "啟動腳本預設（run_asr_cc.sh）" PASS "增益 $G、$M" "刻度實測 1.38 dB/單位；66 會削波 581 個取樣"
else
  rec "啟動腳本預設（run_asr_cc.sh）" FAIL "增益 ${G:-?}、${M:-?}" "應為 60 + left"
fi
# ★ 2026-09-30：9/26 起平板與 demo 走 asr_chain.launch.py，它有自己的預設（9/29 前是 80，會削波）
AC="$HOME/welcoming_robot_ws/src/smartnav_audio/launch/asr_chain.launch.py"
AG=$(grep -o 'ASR_MIC_GAIN", default_value="[0-9]*' "$AC" 2>/dev/null | grep -o '[0-9]*$')
AM=$(grep -o 'ASR_MIC_MODE", default_value="[a-z]*' "$AC" 2>/dev/null | grep -o '[a-z]*$')
if [ "$AG" = "60" ] && [ "$AM" = "left" ]; then
  rec "平板語音鏈預設（asr_chain）" PASS "增益 $AG、$AM" ""
else
  rec "平板語音鏈預設（asr_chain）" FAIL "增益 ${AG:-?}、${AM:-?}" "應為 60 + left；80 會削波（b87dfe8 已修，車上是舊版？）"
fi
[ -n "${ASR_MIC_GAIN:-}" ] && rec "ASR_MIC_GAIN 環境變數" WARN "$ASR_MIC_GAIN" "會蓋過上面兩個預設值"

banner_done
echo "★ 這四項只要有一項 FAIL，後面的實驗數字就不要拿去寫報告。"
