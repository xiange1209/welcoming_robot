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
hdr "人臉辨識 CPU（process_interval_sec 預設 0.0→1.0）"
FCPU=$(cpu_of "face_embedding")
if [ "$(awk -v c="$FCPU" 'BEGIN{print (c<1)?1:0}')" = 1 ]; then
  rec "face_embedding CPU" WARN "${FCPU}%" "節點沒在跑，這項跳過"
elif [ "$(awk -v c="$FCPU" 'BEGIN{print (c<160)?1:0}')" = 1 ]; then
  rec "face_embedding CPU" PASS "${FCPU}%" "9/17 預設 0.0 時是 198%，改 1.0 後應降到 127% 附近"
else
  rec "face_embedding CPU" FAIL "${FCPU}%" "仍接近 198% —— 參數回呼或預設值沒生效，語音鏈會被餓死"
fi

# 偵測輸出頻率：CPU 降了但產出不該掉（單幀推論本來就要約 1004 ms，上限就在 1 Hz）
EHZ=$(hz_of /face_embedding 10)
rec "人臉偵測輸出" INFO "${EHZ} Hz" "預期 0.9~1.0 Hz；降 CPU 不該讓產出變少"

# 全機負載
LOAD=$(cut -d' ' -f1 /proc/loadavg)
rec "系統 1 分鐘負載" INFO "$LOAD" "四核心，9/17 預設值飽和時是 14.7"

# ── 3. 語音模型路徑：找不到現在會印 error ────────────────────
hdr "語音模型路徑（原本靜默失效）"
ALOG="$HOME/maprun/logs/asr.log"
if [ ! -f "$ALOG" ]; then
  rec "ASR 模型載入" WARN "還沒有 asr.log" "先跑 run_asr_cc.sh 再回來驗"
elif grep -q "找不到 asr 模型目錄" "$ALOG"; then
  rec "ASR 模型載入" FAIL "找不到模型" "$(grep -A6 '找不到 asr 模型目錄' "$ALOG" | tail -5 | tr '\n' ' ')"
elif grep -q "找到 asr 模型目錄" "$ALOG"; then
  rec "ASR 模型載入" PASS "$(grep '找到 asr 模型目錄' "$ALOG" | tail -1 | sed 's/.*目錄: //')" "修正前這裡只印 warning 就繼續，辨識器從未建立"
else
  rec "ASR 模型載入" WARN "log 裡沒有相關訊息" "確認跑的是新版"
fi

VLOG="$HOME/maprun/logs/vad.log"
if [ -f "$VLOG" ] && grep -q "找到 vad 模型目錄" "$VLOG"; then
  rec "VAD 模型載入" PASS "已找到" ""
elif [ -f "$VLOG" ]; then
  rec "VAD 模型載入" FAIL "未找到" "VAD 不觸發 = 講話完全沒反應"
fi

# ── 4. 麥克風增益預設值 ──────────────────────────────────────
hdr "麥克風增益（家裡 66 → 實驗室 60 + left）"
G=$(grep -o 'ASR_MIC_GAIN:-[0-9]*' "$HOME/maprun/run_asr_cc.sh" 2>/dev/null | grep -o '[0-9]*$')
M=$(grep -o 'ASR_MIC_MODE:-[a-z]*' "$HOME/maprun/run_asr_cc.sh" 2>/dev/null | cut -d- -f3)
if [ "$G" = "60" ] && [ "$M" = "left" ]; then
  rec "啟動腳本預設" PASS "增益 $G、$M" "刻度實測 1.38 dB/單位；66 會削波 581 個取樣"
else
  rec "啟動腳本預設" FAIL "增益 ${G:-?}、${M:-?}" "應為 60 + left"
fi

banner_done
echo "★ 這四項只要有一項 FAIL，後面的實驗數字就不要拿去寫報告。"
