# 對上游 frontier_exploration_ros2 的修補

**修補已經 commit 進我們的 fork，不用手動套。** 子模組 `src/frontier_exploration_ros2` 指向
[`xiange1209/frontier_exploration_ros2`](https://github.com/xiange1209/frontier_exploration_ros2/tree/smartnav-patches)
的 `smartnav-patches` 分支＝上游 `ec530d2`（mertgulerx，2026-09-25 時仍是 main 的 HEAD）＋下面的修補（`10b06cf`）。

這個資料夾留著 `.patch` 檔當紀錄：看得到我們到底改了上游哪幾行，日後要回報給上游也用得到。

- 2026-09-25 以前 clone 的，子模組還指著上游原版 → `git pull` 之後依序執行兩行（分開打，Windows PowerShell 5.1 不認 `&&`）：
  ```
  git submodule sync
  git submodule update --init
  ```
  `pack_car_code.py` 發現沒修補會拒絕打包，車上的 `verify_0_preflight.sh` 也會檢查（修正11、11b）
- 它是 C++，車上要重新 `colcon build` 才生效（打包時會把修補過的檔案時間設成打包當下，保證會重編）
- 要跟上游同步新版：到子模組裡依序 `git fetch upstream`、`git merge upstream/main`、`git push origin smartnav-patches`，
  再回主 repo 把 `src/frontier_exploration_ros2` commit 起來。★ 用 merge 不要用 rebase：rebase 會改寫 `10b06cf`，
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
