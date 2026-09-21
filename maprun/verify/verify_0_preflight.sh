#!/usr/bin/env bash
# V0 開工前檢查 —— 電池、裝置、時鐘、程式版本。
# 這一關沒過就不要往下做，後面量到的數字都不可信。
#
# 用法：~/maprun/verify/verify_0_preflight.sh

source "$(dirname "$0")/_lib.sh"

echo "V0 開工前檢查   $(date '+%Y-%m-%d %H:%M:%S')"

# ── 電池 ────────────────────────────────────────────────────
# ★ 低壓保護在 20 V，但 20 V 只切馬達不切舵機
#   症狀是「方向舵會轉但車不走」，很容易誤判成程式壞掉。
hdr "電池"
V=$(timeout 8 ros2 topic echo /PowerVoltage --once --field data 2>/dev/null | head -1)
if [ -z "$V" ]; then
  rec "電池電壓" WARN "讀不到" "底盤節點沒起來？先跑 run_sensors_cc.sh 再回來"
else
  ok=$(awk -v v="$V" 'BEGIN{print (v>=23.0)?1:0}')
  if [ "$ok" = 1 ]; then
    rec "電池電壓" PASS "${V} V" "門檻 23.0 V"
  else
    rec "電池電壓" FAIL "${V} V" "低於 23.0 V —— 先充電，不要開始實驗"
  fi
fi

# ── 裝置 ────────────────────────────────────────────────────
# ★ 硬體問題用列舉指令回答，不要用文件回答
hdr "裝置（直接問核心，不看文件）"
for dev in /dev/ttyUSB0 /dev/ttyUSB1; do
  [ -e "$dev" ] && rec "$dev" PASS "存在" "" || rec "$dev" FAIL "不存在" "底盤/雷達沒插好或編號跑掉"
done

if lsusb 2>/dev/null | grep -qi '2bc5:'; then
  rec "Astra 相機" PASS "$(lsusb | grep -i '2bc5:' | head -1 | sed 's/.*ID //')" ""
else
  rec "Astra 相機" FAIL "lsusb 找不到 2bc5:" "換 USB 孔試試"
fi

# ⚠ 音效卡編號每次開機都會變（一天內實測 1→2→3），所以只確認「有」，不寫死編號
CARD=$(arecord -l 2>/dev/null | grep -o '^card [0-9]*' | head -1 | grep -o '[0-9]*')
[ -n "$CARD" ] && rec "麥克風" PASS "card $CARD" "★ 編號每次開機都會變，run_asr_cc.sh 會當場問" \
                || rec "麥克風" FAIL "arecord -l 沒有裝置" ""

# ── 時鐘 ────────────────────────────────────────────────────
# 時鐘沒同步的話所有 log 時戳都不能拿來比對
hdr "系統時鐘"
if timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -q yes; then
  rec "時鐘同步" PASS "synchronized" ""
else
  rec "時鐘同步" WARN "未同步" "log 時戳無法跨機比對，寫報告時要注意"
fi

# ── 程式版本 ────────────────────────────────────────────────
hdr "車上程式版本"
BUILT=$(stat -c %y "$HOME/welcoming_robot_ws/install/setup.bash" 2>/dev/null | cut -d. -f1)
rec "install 建置時間" INFO "${BUILT:-找不到}" "要晚於你 scp 上來的時間，否則是舊版在跑"

# 四項關鍵修正是否真的在車上（檢查原始碼，不是檢查有沒有 scp）
SRC="$HOME/welcoming_robot_ws/src/smartnav_ws/src"
chk() { # chk <說明> <檔案> <關鍵字>
  if [ -f "$2" ] && grep -q "$3" "$2" 2>/dev/null; then
    rec "$1" PASS "已部署" ""
  else
    rec "$1" FAIL "未部署" "車上還是舊版 —— 重新 scp + colcon build"
  fi
}
chk "修正1 模型路徑可覆寫"  "$SRC/smartnav_audio/smartnav_audio/voice_utils.py"          "model_path_candidates"
chk "修正2 人臉參數回呼"    "$SRC/smartnav_vision/smartnav_vision/face_embedding_node.py" "add_on_set_parameters_callback"
chk "修正3 回音共同前綴"    "$SRC/smartnav_audio/smartnav_audio/speech_recognizer_node.py" "_common_prefix_len"
chk "修正4 HMI 測試情境"    "$SRC/smartnav_hmi/smartnav_hmi/hmi_server_node.py"           "SCENARIOS"
[ -x "$HOME/maprun/run_camera_face_cc.sh" ] \
  && rec "修正5 人臉相機啟動入口" PASS "run_camera_face_cc.sh 可執行" "" \
  || rec "修正5 人臉相機啟動入口" FAIL "缺少或沒有執行權限" "chmod +x ~/maprun/run_camera_face_cc.sh"

banner_done
