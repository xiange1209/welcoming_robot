# 對上游 frontier_exploration_ros2 的修補

**修補已經 commit 進我們的 fork，不用手動套。** 子模組 `src/frontier_exploration_ros2` 指向
[`xiange1209/frontier_exploration_ros2`](https://github.com/xiange1209/frontier_exploration_ros2/tree/smartnav-patches)
的 `smartnav-patches` 分支＝上游 `ec530d2`（mertgulerx，2026-09-25 時仍是 main 的 HEAD）＋下面兩個修補
（0001＝`10b06cf`、0002＝`343ac1c`）。

這個資料夾留著 `.patch` 檔當紀錄：看得到我們到底改了上游哪幾行，日後要回報給上游也用得到。

- 2026-09-25 以前 clone 的，子模組還指著上游原版 → `git pull` 之後依序執行兩行（分開打，Windows PowerShell 5.1 不認 `&&`）：
  ```
  git submodule sync
  git submodule update --init
  ```
  `pack_car_code.py` 發現任何一個修補檔沒有 `[smartnav patch]` 標記會拒絕打包，車上的 `verify_0_preflight.sh` 也會檢查（修正11、11b、14）
- 它是 C++，車上要重新 `colcon build` 才生效（打包時會把修補過的檔案時間設成打包當下，保證會重編）
- 要跟上游同步新版：到子模組裡依序 `git fetch upstream`、`git merge upstream/main`、`git push origin smartnav-patches`，
  再回主 repo 把 `src/frontier_exploration_ros2` commit 起來。★ 用 merge 不要用 rebase：rebase 會改寫 `10b06cf`／`343ac1c`，
  推回 fork 得 `--force`，之後舊的子模組指標就抓不到了
- 以後要在 fork 上加新的修補：commit 到 `smartnav-patches`，並在這裡放一份 `.patch`（打包靠它知道哪些檔要保證重編）

## 0001：看門狗忽略 distance_remaining = 0 的回報

**問題**：explorer 的「無進展看門狗」拿每個 nav2 目標的**第一筆** `distance_remaining` 當基準，
之後剩餘距離要比基準少 0.25 m 才算有進展。而 nav2（Jazzy）在**還沒有路徑的時候回報 0.0**：

- 每個目標結束時 `haltAllActions` → BT.CPP 4.x 的 `haltTree` 會 halt **每一個**節點，
  `ComputePathToPose::halt` 把 `{path}` 設成空的（Humble 只 halt 正在執行的節點，所以不是每次都這樣）
- 新目標不清 blackboard，第一次規劃完成前 `{path}` 是空的 → `navigate_to_pose.cpp` 丟例外、被吞掉，
  feedback 照發，`distance_remaining` 是訊息預設值 0.0
- 目標途中規劃失敗一次，`{path}` 也會被清空

基準一旦是 0，就不可能再「比 0 少 0.25」→ **每個開超過逾時（我們設 15 秒）的目標都被判失敗並抑制**，
跟車子有沒有在前進無關。這很可能就是自動建圖「常常卡住」的主要原因。

**證據**（2026-09-25）：逐行讀 nav2 jazzy 分支（1.3.1～1.3.13 這段邏輯沒變）與 BT.CPP 4.6/4.10 原始碼；
把上游 `frontier_suppression.cpp` 原檔接上測試殼編譯實測：

| 情境（參數照 frontier_explore_cc.yaml） | 原版 | 修補後 |
|---|---|---|
| 6 m 路徑、0.2 m/s 前進中，開頭 1.5 秒回報 0.0（Jazzy 每個目標都這樣） | 第 16 秒取消，還剩 3.1 m | 不取消 |
| 途中規劃失敗一次（回報 0.0 一秒） | 第 25 秒取消 | 不誤判 |
| 真的卡住（距離不變） | 16 秒取消 | 16 秒取消 |
| 完全收不到回報／一直規劃不出路 | 15～16 秒取消 | 15 秒取消 |
| 中途繞遠路 +3 m | 23 秒取消 | 23 秒取消（**沒修到**，剩下的限制） |

**修法**：`note_goal_progress` 開頭，`distance_remaining <= 0.001` 的樣本直接略過（當作沒有資訊）。
一直規劃不出路的目標照樣會被抓到——追蹤在 nav2 接受目標時就以接受時間起算了。

**沒修到的**：繞遠路（路徑突然變長 >3 m）、nav2 原生搶佔時沿用舊路徑的剩餘長度。這兩種比較少見，
要修得在 explorer 與 nav2 之間加一個轉接節點（2026-09-25 有完整設計與反駁檢驗，但需要 ROS 環境測試才能上車）。

## 0002：選點淨空＋到達抑制（2026-09-26）

**問題**（Gazebo 模擬，模擬車帶實車的死區與左右迴轉半徑；8 組只調參數的實驗都沒有穩定改善，
紀錄在本機 `專題/backups/研究數據/模擬調參_20260926/`）：

1. **目標點貼著還沒看到的牆**。上游只從「frontier 旁邊那一格空地」挑目標，篩選只看全域 costmap。
   從大廳斜看進 0.99 m 走廊時，遠處的牆還是未知、沒有膨脹成本，目標就落在牆邊甚至牆裡
   （例：(5.47, 0.58) 在北牆範圍 0.495～0.595 內）→ 規劃必敗 → 被抑制 → 走廊一次都沒進去過。
2. **到了還在的 frontier 一直回去**。到達目標什麼都不記。沙發、櫃檯和牆之間 0.35～0.45 m 的縫從外面看不透，
   frontier 永遠在，每次回去約耗 18 秒，時間被吃光後停滯收尾。

**修法**（三個新參數，預設值＝上游行為）：

| 參數 | 我們的值 | 作用 |
|---|---|---|
| `frontier_candidate_clearance_m` | 0.35 | 目標點離「牆與未知格」都至少這麼遠（chamfer 距離轉換，每次搜尋算一次，O(格數)） |
| `frontier_candidate_search_radius_m` | 1.0 | 從 frontier 旁的空格往外找幾公尺；找不到就不提供這個 frontier（車子到不了夠寬的地方） |
| `frontier_suppress_on_arrival` | true | 到達目標（nav2 SUCCEEDED 或進入 complete_if_within）算一次嘗試，還在的 frontier 就被抑制 |

**效果**（同一場地、同一組 nav2 參數，每組一次）：

| 場地 | 修補前 | 修補後 |
|---|---|---|
| 瓶頸測試場、北門關閉（全場約 36 m²） | 23.0 m²，東走廊一次都沒去（E 組） | **33.0 m²**，135 秒進東走廊、205 秒到盡頭房（H 組） |
| 瓶頸測試場、北門打開（全場約 42 m²） | 26.1～31.0 m²，3/4 組被困在北門或北走廊（A～D 組） | **39.6 m²**，進過北走廊也出得來（I 組） |

規劃失敗 62～81 次 → 8～9 次；到達目標 0 → 8～14 次。上游 141 項 gtest ＋ 新增 6 項，147/147 通過
（新增的「到達後立刻重派」測試：上游會把同一個 frontier 再派一次，修補後不會）。

