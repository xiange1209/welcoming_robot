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
# ★★ 2026-09-30（車子端 9/29 抓到）：底盤沒開時 ros2 topic echo 把「WARNING: topic … not published」
#    印到 **stdout**，原本 head -1 就把它當電壓，awk 對非數字做**字串比較**（"W" > "2"）→ 判 PASS。
#    鐵則「電池 ≥ 23 V 才開始」這道關卡在底盤沒開時會自己放行。只接受整行是數字的輸出。
V=$(timeout 8 ros2 topic echo /PowerVoltage --once --field data 2>/dev/null | grep -Eo '^[0-9]+([.][0-9]+)?$' | head -1)
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

# ★ 2026-09-30（車子端 9/22 發現）：車上 HMI 是 systemd 服務 smartnav-hmi（ros2 run smartnav_hmi hmi_server），
#   colcon build 之後**不會自己重啟**。前端 index.html 是軟連結所以是新的，後端卻是舊程式 ——
#   卡片畫得出來、內容抓不到，看起來像「功能沒做」。服務的啟動時間必須晚於最後一次建置。
HMI_PID=$(systemctl show -p MainPID --value smartnav-hmi 2>/dev/null)
if [ -z "$HMI_PID" ] || [ "$HMI_PID" = 0 ]; then
  HMI_PID=$(ps -eo pid,args --no-headers | grep '[h]mi_server' | awk 'NR==1{print $1}')
fi
if [ -z "$HMI_PID" ]; then
  rec "HMI 跑的是新程式" INFO "HMI 沒在跑" ""
else
  HMI_START=$(date -d "$(ps -o lstart= -p "$HMI_PID" 2>/dev/null)" +%s 2>/dev/null)
  BUILT_S=$(stat -c %Y "$HOME/welcoming_robot_ws/install/setup.bash" 2>/dev/null)
  if [ -n "$HMI_START" ] && [ -n "$BUILT_S" ] && [ "$HMI_START" -lt "$BUILT_S" ]; then
    rec "HMI 跑的是新程式" FAIL "HMI 啟動早於最後一次建置" "sudo systemctl restart smartnav-hmi（重啟後平板要重新登入）"
  else
    rec "HMI 跑的是新程式" PASS "啟動晚於最後一次建置" ""
  fi
fi

# ── 語音模型 ────────────────────────────────────────────────
# ★ 2026-09-30：找不到模型時語音節點照跑、燈號照亮，只是**永遠不出字**。9/17 是靠 install/ 底下的
#   暫時軟連結才找到，重建就沒了；~/models/asr 是 9/22 在車上手動建的，部署包不含。
#   搜尋順序照 voice_utils.model_path_candidates：環境變數 > 套件 share > ~/models/<type>
hdr "語音模型（找不到時節點照跑、只是不出字）"
SHARE_M="$HOME/welcoming_robot_ws/install/smartnav_audio/share/smartnav_audio/models"
for t in asr:tokens.txt vad:silero_vad.onnx; do
  typ=${t%%:*}; f=${t#*:}; hit=""
  for d in "${SMARTNAV_AUDIO_MODELS_DIR:+$SMARTNAV_AUDIO_MODELS_DIR/$typ}" "$SHARE_M/$typ" "$HOME/models/$typ"; do
    if [ -n "$d" ] && [ -f "$d/$f" ]; then hit="$d"; break; fi
  done
  if [ -n "$hit" ]; then
    rec "$typ 模型" PASS "$hit" ""
  else
    rec "$typ 模型" FAIL "三個位置都沒有 $f" "建 ~/models/$typ 指向模型目錄（見 HANDOFF_car_agent 第 4.5 節）"
  fi
done

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
# 9/26：explorer 選點要離牆與未知格 0.35 m、到達目標後抑制看不透的死角（fork 修補 0002）
chk "修正14 frontier 選點淨空" "$SRC/frontier_exploration_ros2/src/frontier_search.cpp"       "candidate_clearance_m"
# 上面只證明「原始碼」是修補版。C++ 要重新編譯才生效，而 make 看的是檔案時間：
# 編出來的 .o 比原始碼舊 = 解壓後還沒重新 colcon build，車上跑的仍是舊的 explorer（2026-09-25 審查）
# ★ 9/26 起修補改到 6 個 .cpp，每一個都要比（只看一個的話，漏編其他檔也會 PASS）
STALE=""; NOOBJ=""
for f in frontier_suppression frontier_search frontier_explorer_core_dispatch \
         frontier_explorer_core_init frontier_explorer_core_frontiers frontier_explorer_node; do
  S="$SRC/frontier_exploration_ros2/src/$f.cpp"
  O=$(find "$WS/build/frontier_exploration_ros2" -name "$f.cpp.o" 2>/dev/null | head -1)
  if [ -z "$O" ]; then NOOBJ="$NOOBJ $f"
  elif [ -f "$S" ] && [ "$(stat -c %Y "$O")" -lt "$(stat -c %Y "$S")" ]; then STALE="$STALE $f"; fi
done
if [ -n "$NOOBJ" ]; then
  rec "修正11b frontier 已重新編譯" WARN "找不到編譯產物：$NOOBJ" "還沒 colcon build？（第一次升新結構會整個重建）"
elif [ -n "$STALE" ]; then
  rec "修正11b frontier 已重新編譯" FAIL "原始碼比編出來的新：$STALE" \
      "解壓後沒重新建置 —— cd ~/welcoming_robot_ws && colcon build --symlink-install --packages-select frontier_exploration_ros2"
else
  rec "修正11b frontier 已重新編譯" PASS "6 個修補檔都已用目前的原始碼編譯" ""
fi
chk "修正12 卡住預算與後備"   "$NCC/map_service_cc_node.py"                             "exploration_stuck_repeat_limit"
# 9/26：防撞狀態 10 秒沒變就停止抬速度 → 指令被死區吃掉、車子停住（模擬實測誤擋 3084 則）
chk "修正13 死區守門員不誤擋" "$NCC/cmd_vel_floor_cc_node.py"                            "_state_publisher_alive"
# 9/29：9/23 重構與 e56f49b 帶進來的回歸（筆電 b87dfe8）。四個都「不報錯」：工具靜默消失、
#   削波、按鈕回 500、定位模式偷開 explorer
chk "修正15 LLM 銀行工具 import"    "$SRC/smartnav_llm/smartnav_llm/llm_service_node.py"      "from smartnav_llm.bank_tools import"
chk "修正16 ASR 增益預設 60"        "$SRC/smartnav_audio/launch/asr_chain.launch.py"           'ASR_MIC_GAIN", default_value="60"'
chk "修正17 一鍵驗證 import asyncio" "$HMI_SC/router.py"                                        "^import asyncio"
# ★ 修正9 查的是 manager.py 的 cmd 欄位 —— 9/26 起沒有程式讀它（改走 LaunchSupervisor），
#   真正決定定位模式開不開 explorer 的是 system_control.launch.py
chk "修正18 定位模式不開探索"       "$SRC/smartnav_hmi/launch/system_control.launch.py"        '"localization": ("localization", "false")'
# 9/30：path_teach 丟掉自身扇區 0.25 m 內的回波（9/29 四條重播全因鋁擠條回波在 5 秒內進脫困；
#   模擬 A/B 驗過）。現場懷疑它出問題可帶 self_echo_filter:=false 退回舊行為，不必改程式
chk "修正19 重播過濾鋁擠條回波"     "$NCC/path_teach_cc_node.py"                               "_drop_self_echo"
# 9/30：人臉間隔 1.0→2.0 的連動（只改間隔的話註冊會在 20 秒逾時、把剛建的使用者刪掉）
chk "修正20 人臉間隔 2.0 連動"      "$SRC/smartnav_brain/smartnav_brain/user_auth_node.py"     "face_registration_timeout_sec"
# 10/01：探索中自己脫困（家具旁 Start occupied／推不動 → 靜音 nav2 → /unwedge）。兩半都要在
chk "修正21 探索中自己脫困"         "$NCC/map_service_cc_node.py"                              "_maybe_unwedge"
chk "修正22 脫困時 nav2 指令靜音"   "$NCC/cmd_vel_floor_cc_node.py"                            "mute_nav_cmd"
chk "修正23 防撞狀態不再 1 秒過期"   "$NCC/path_teach_cc_node.py"                               "notifyActionState"
chk "修正24 新障礙不脫困、交回上層" "$NCC/path_teach_cc_node.py"                               "_classify_blocker"
chk "修正25 通道被擋住就等它淨空"   "$NCC/navigation_action_cc_node.py"                        "_wait_for_passage"
chk "修正26 規劃前等障礙標回去"     "$NCC/path_teach_cc_node.py"                               "plan_clear_settle_sec"
chk "修正27 車頭車尾貼住直線拉開"   "$NCC/path_teach_cc_node.py"                               "_unwedge_straight"
chk "修正28 日誌不在同一行換等級"     "$NCC/map_service_cc_node.py"                              "Logger severity cannot be changed"
[ -x "$HOME/maprun/run_camera_face_cc.sh" ] \
  && rec "修正5 人臉相機啟動入口" PASS "run_camera_face_cc.sh 可執行" "" \
  || rec "修正5 人臉相機啟動入口" FAIL "缺少或沒有執行權限" "chmod +x ~/maprun/run_camera_face_cc.sh"

banner_done
