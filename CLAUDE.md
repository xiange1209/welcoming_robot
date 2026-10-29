# 專案目錄與工作區規範 (Project Directory Rules)

> ⚠️ 最高優先級路徑規則 (CRITICAL PATH RULE):
> **repo 裡面**：所有源碼與 Package 必須「直接」放在 repo 根目錄的 `./src/` 內，repo 裡不可以再出現任何 `*_ws/` 子目錄。
> **車上（部署）**：repo 被 clone 在 colcon workspace 的 `src/` 底下（見 §2）。這是**刻意的部署方式，不是錯誤**——
> 不要把它「搬回」家目錄、不要攤平、不要另外再建 workspace。

---

## 1. repo 的目錄結構（筆電與 GitHub 上都一樣）

```text
.                          <-- repo 根目錄（GitHub: xiange1209/welcoming_robot）
├── src/                   <-- 所有 Package 直接放這裡
│   ├── smartnav_msgs/          介面定義（RequestNavigation.action 等跨套件常數的單一來源）
│   ├── smartnav_brain/         唯一的業務決策點：user_auth、bank_reception、nav_arbiter＋access_policy
│   ├── smartnav_llm/           迎賓對話（工具呼叫、tool_gate、知識庫檢索 RAG、Ollama 自動尋找）
│   ├── smartnav_audio/         喚醒、語音辨識、語音合成與播放
│   ├── smartnav_vision/        人臉偵測與向量擷取
│   ├── smartnav_navigation_cc/ 建圖、定位、路徑規劃與追蹤、教導-重現、自動探索；提供 /navigate
│   ├── smartnav_hmi/           平板網頁、遙控、管理功能、系統開關（單元與情境）、緊急停止
│   ├── smartnav_bringup/       分階段啟動
│   ├── smartnav_sim/           筆電 WSL 的 Gazebo 模擬（不上車）
│   └── frontier_exploration_ros2/  git 子模組（xiange1209/frontier_exploration_ros2，分支 smartnav-patches）
├── maprun/                <-- 車上用的工具與腳本（day_test.sh、tools_10xx、verify…）
├── scripts/  patches/  pack_car_code.py  pre_pack_gate.py  deploy_hmi.sh  cyclonedds.xml
├── README.md              <-- 架構與「套件職責邊界」
└── CLAUDE.md
```

❌ repo 裡嚴禁：`./welcoming_robot_ws/…`、`./smartnav_ws/…`、任何 `*_ws/` 子目錄、把 `./src` 複製到子目錄。

---

## 2. 車上的部署方式（Raspberry Pi 4，主機名 wheeltec-r680，使用者 user）

```text
~/welcoming_robot_ws/              <-- colcon workspace 根目錄（在這裡 build）
├── src/
│   ├── smartnav_ws/               <-- 本 repo，master 分支（git@github.com:xiange1209/welcoming_robot.git）
│   └── wheeltec_ws/               <-- 同一個 repo 的 wheeltec 分支＝廠商驅動（底盤、雷達、Astra 相機…）
├── build/  install/  log/
~/.smartnav/                       <-- 執行期資料：地圖、地點、教導路徑、人臉庫、知識庫、secrets、各單元 log
~/maprun/logs/                     <-- path_teach 每趟的逐拍紀錄（replay_*_live_*.csv）
~/cyclonedds.xml                   <-- HMI 服務用的 DDS 設定（與 repo 的 cyclonedds.xml 內容相同）
```

- 上面兩個 clone 的位置是固定的：**不要搬、不要改名、不要另外再 clone 一份**。
- 建置：`cd ~/welcoming_robot_ws && colcon build --packages-select <套件>`。
  ★ 目前是**複製式安裝（非 --symlink-install）**：改了 src **一定要 build** 才會生效，build 完要重啟用到它的東西
  （HMI：`sudo systemctl restart smartnav-hmi`；其他單元：平板「系統開關」停止再啟動）。
- 環境：`~/.bashrc` 已 source `/opt/ros/jazzy/setup.bash` 與 `~/welcoming_robot_ws/install/setup.bash`，
  `CYCLONEDDS_URI` 指向 repo 的 `cyclonedds.xml`。HMI 由 systemd 的 `smartnav-hmi.service` 開機自動啟動。
- 工具路徑：工具在 `~/welcoming_robot_ws/src/smartnav_ws/maprun/`（測試主程式 `day_test.sh` 與它用的 `tools_1020/`），
  程式位置一律由腳本自己的所在資料夾推算；**資料**照舊寫在 `~/maprun/verify_out`、`~/maprun/logs`（不進 git）。
  HMI 的「一鍵驗證」已移除（2026-10-29）。
- 執行期資料（`~/.smartnav`）**不進 git**；`~/.smartnav/secrets/`、人臉庫、`visit_log.db` 絕對不能 commit、印出或打包帶走
  （repo 是**公開**的）。

---

## 3. 系統架構重點（細節見 README「套件職責邊界」）

- **導航的唯一入口是 `nav_arbiter`**（smartnav_brain）：LLM、迎賓頁、管理者都送 `RequestNavigation`
  （來源 LLM／GUEST_UI／OPERATOR），只有 nav_arbiter 會呼叫 `/navigate`；它沒在跑＝帶位一律被拒（預設拒絕）。
  **緊急停止不經 nav_arbiter**：HMI 直接對 `/navigate` 與 nav2 送取消。
- HMI「系統開關」的單元一律經 `ros2 launch smartnav_hmi system_control.launch.py unit:=<單元> [variant:=<模式>]` 啟動。
  單元：sensors、nav（mapping／explore／localization，**nav_arbiter 跟著 nav 一起起停**）、camera（face／obstacle）、
  face（normal／eco）、user_auth、bank_reception、llm、asr_chain、speech_synthesizer、voice_playback。
- 帶位中（`/navigation_active`）迎賓頁左卡換成地圖（自動放大到路線），相機縮成可拖動的小窗；到點自動換回。
  可以讓 HMI 在導航中暫停「人臉模式」相機（`nav_camera.py`，參數 `pause_camera_while_navigating`，**預設關**）。
- 帶位被「地圖上沒有的障礙」擋住時：先講「不好意思，請讓我過一下」、等一下，重新規劃時**保留**新障礙讓路線繞開
  （navigation_action_cc `keep_new_obstacles_on_replan`／`new_obstacle_speech*`、path_teach `keep_new_obstacles_sec`）。
- LLM 跑在筆電的 Ollama：`ollama_base_url` 預設 `auto`，依序找 `MSI.mshome.net`、`MSI.local`、預設閘道、已知 IP
  （參數 `ollama_candidates`）；筆電的 Ollama 要聽區網（`OLLAMA_HOST=0.0.0.0`）。
- 原本的導航套件（`smartnav_navigation`）已刪除，導航只用 `smartnav_navigation_cc`；不要再建或啟動舊套件。

---

## 4. ROS / ROS2 建置與指令規範

當生成 Bash 腳本、Dockerfile、CMakeLists.txt 或建置指令時：

1. 在筆電／一般開發環境：以 repo 根目錄為準，直接針對 `./src` 編譯。
2. 在車上：一律在 `~/welcoming_robot_ws` 執行 `colcon build`（它會同時找到 `src/smartnav_ws` 與 `src/wheeltec_ws` 的套件）。
3. 切勿生成會建立新的 `*_ws` 資料夾、把 `./src` 複製到別處、或搬動車上兩個 clone 的指令。
4. 停止節點用 HMI（平板或 API）、`kill -INT <PID>`、`sudo systemctl stop smartnav-hmi`；**不要用 `pkill -f`／`pgrep -f`**。

---

## 5. Git 分支與版本控制嚴格規範 (Git Rules)

> ⚠️ 最高優先級 Git 規則:
>
> 1. 嚴禁刪除任何分支！尤其是 wheeltec 分支。
> 2. 嚴禁擅自將 wheeltec 分支合併 (Merge) 或 Rebase 到 master / main 分支。
> 3. 嚴禁執行任何包含 git branch -d、git branch -D 或 git push origin --delete 的指令。
> 4. 嚴禁使用中文命名 Git 分支！所有新建立的分支名稱必須完全使用全英文、數字、斜線 (/) 或連字號 (-)，例如：feature/navigation 或 fix/laser-scan。

當處理 Git 操作時：

1. 除非使用者明確指示「請幫我合併到 master」，否則絕對不可以執行切換到 master 並進行 merge 的操作。
2. 若需要建立新分支，請務必確認分支名稱完全沒有任何中文字元，且必須使用半形英文命名。
3. 若需要提供 Git 指令，僅提供 git commit 或 git push origin <當前分支>，切勿加上刪除或跨分支合併的指令。
4. 車上：`src/smartnav_ws` 在 master、`src/wheeltec_ws` 在 wheeltec——**不要在 wheeltec_ws 裡切到 master，也不要反過來**。
   使用者沒有要求就不要 commit／push。

---

## 6. AI 輸出前自檢清單 (Checklist)

每次生成程式碼、檔案路徑或 Bash 指令前，請確認：

- [ ] repo 內的原始碼路徑是否都以 `./src/` 開頭？（車上的完整路徑是 `~/welcoming_robot_ws/src/smartnav_ws/src/…`，這是部署位置，正確）
- [ ] 是否沒有要建立新的 `*_ws/` 目錄、搬動車上的 clone、或把 src 複製到別處？
- [ ] 改了車上的程式之後，有沒有 build＋重啟（複製式安裝）？
- [ ] 我剛才生成的指令或分支名稱中，是否有包含中文字元？
- [ ] 我剛才生成的指令中，是否有包含刪除分支 (-d/-D) 或合併至 master 的操作？
- [ ] 有沒有碰到 `~/.smartnav/secrets`、人臉庫或訪客紀錄？（不能印出、不能進 git）
