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

# ── 目錄結構（2026-09-23 攤平之後）─────────────────────────
# ★ 套件直接放在 ~/welcoming_robot_ws/src/<套件>，不再有 src/smartnav_ws/src/ 這層。
WS="$HOME/welcoming_robot_ws"
SRC="$WS/src"
hdr "目錄結構"
if [ -d "$SRC/smartnav_ws" ]; then
  rec "舊巢狀目錄" FAIL "src/smartnav_ws/ 還在" \
      "新舊兩份套件並存 → colcon 會報 Duplicate package names。先跑 ~/maprun/verify/migrate_layout.sh"
else
  rec "舊巢狀目錄" PASS "已清除" ""
fi
[ -f "$SRC/smartnav_hmi/package.xml" ] \
  && rec "新結構套件" PASS "src/smartnav_hmi 在正確位置" "" \
  || rec "新結構套件" FAIL "找不到 src/smartnav_hmi/package.xml" "src 解壓到錯的地方？要在 ~/welcoming_robot_ws 底下解壓"

# frontier 在 9/23 改成 git 子模組。打包時沒 init，車上就會少這整個套件
[ -f "$SRC/frontier_exploration_ros2/package.xml" ] \
  && rec "frontier 套件" PASS "原始碼在" "" \
  || rec "frontier 套件" FAIL "src/frontier_exploration_ros2 是空的" "筆電端打包前沒跑 git submodule update --init"

# 前端改成 React，dist/ 要先建置才會被 setup.py 裝進 share
DIST="$WS/install/smartnav_hmi/share/smartnav_hmi/frontend/dist/index.html"
[ -f "$DIST" ] \
  && rec "HMI 前端（React）" PASS "dist 已安裝" "" \
  || rec "HMI 前端（React）" FAIL "install 裡沒有 frontend/dist" \
         "平板打開會是空白頁 —— 筆電端打包前要 npm run build，車上再 colcon build"

# ── 關鍵修正是否真的在車上（檢查原始碼，不是檢查有沒有 scp）───
hdr "關鍵修正"
chk() { # chk <說明> <檔案> <關鍵字>
  if [ -f "$2" ] && grep -q "$3" "$2" 2>/dev/null; then
    rec "$1" PASS "已部署" ""
  else
    rec "$1" FAIL "未部署" "車上還是舊版 —— 重新 scp + colcon build"
  fi
}
HMI_SC="$SRC/smartnav_hmi/smartnav_hmi/modules/system_control"
chk "修正1 模型路徑可覆寫"  "$SRC/smartnav_audio/smartnav_audio/voice_utils.py"          "model_path_candidates"
chk "修正2 人臉參數回呼"    "$SRC/smartnav_vision/smartnav_vision/face_embedding_node.py" "add_on_set_parameters_callback"
chk "修正3 回音共同前綴"    "$SRC/smartnav_audio/smartnav_audio/speech_recognizer_node.py" "_common_prefix_len"
chk "修正4 HMI 測試情境"    "$HMI_SC/manager.py"                                          "SCENARIOS"
chk "修正6 HMI 一鍵驗證"    "$HMI_SC/manager.py"                                          "VERIFY_SCRIPTS"
# 9/24~9/25 自動建圖修正（沒部署的話 V5 的分類會失準，情境也可能讓車自己開走）
NCC="$SRC/smartnav_navigation_cc/smartnav_navigation_cc"
chk "修正7 探索中卡住讓步"   "$NCC/stuck_detector_cc_node.py"                             "defer_grace_sec"
chk "修正8 停滯存圖＋STOP 後備" "$NCC/map_service_cc_node.py"                             "_cancel_all_nav_goals"
chk "修正9 換情境先停導航"   "$HMI_SC/manager.py"                                          '"localization", "false"'
chk "修正10 LLM 建圖等 1920 秒" "$SRC/smartnav_llm/smartnav_llm/llm_service_node.py"         "1920.0"
# ★ 上游 explorer 的修補（C++，要重新 colcon build 才生效）。沒修補的話看門狗會把每個
#   開超過 15 秒的目標誤判失敗 —— 自動建圖「常常卡住」很可能就是這個
chk "修正11 frontier 看門狗濾 0" "$SRC/frontier_exploration_ros2/src/frontier_suppression.cpp" "smartnav patch"
# 上面只證明「原始碼」是修補版。C++ 要重新編譯才生效，而 make 看的是檔案時間：
# 編出來的 .o 比原始碼舊 = 解壓後還沒重新 colcon build，車上跑的仍是舊的 explorer（2026-09-25 審查）
SUPP_SRC="$SRC/frontier_exploration_ros2/src/frontier_suppression.cpp"
SUPP_OBJ=$(find "$WS/build/frontier_exploration_ros2" -name 'frontier_suppression.cpp.o' 2>/dev/null | head -1)
if [ -z "$SUPP_OBJ" ]; then
  rec "修正11b frontier 已重新編譯" WARN "找不到編譯產物" "還沒 colcon build？（第一次升新結構會整個重建）"
elif [ -f "$SUPP_SRC" ] && [ "$(stat -c %Y "$SUPP_OBJ")" -lt "$(stat -c %Y "$SUPP_SRC")" ]; then
  rec "修正11b frontier 已重新編譯" FAIL "原始碼比編出來的新" \
      "解壓後沒重新建置 —— cd ~/welcoming_robot_ws && colcon build --symlink-install --packages-select frontier_exploration_ros2"
else
  rec "修正11b frontier 已重新編譯" PASS "已用目前的原始碼編譯" ""
fi
chk "修正12 卡住預算與後備"   "$NCC/map_service_cc_node.py"                             "exploration_stuck_repeat_limit"
# 9/26：防撞狀態 10 秒沒變就停止抬速度 → 指令被死區吃掉、車子停住（模擬實測誤擋 3084 則）
chk "修正13 死區守門員不誤擋" "$NCC/cmd_vel_floor_cc_node.py"                            "_state_publisher_alive"
[ -x "$HOME/maprun/run_camera_face_cc.sh" ] \
  && rec "修正5 人臉相機啟動入口" PASS "run_camera_face_cc.sh 可執行" "" \
  || rec "修正5 人臉相機啟動入口" FAIL "缺少或沒有執行權限" "chmod +x ~/maprun/run_camera_face_cc.sh"

banner_done
