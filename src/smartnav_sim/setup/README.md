# 自動建圖模擬環境：安裝步驟

在筆電的 WSL2 裡裝 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic，用來模擬 senior_akm 的自動建圖（frontier 探索）。
**全部裝在 D 槽，C 槽幾乎不佔。**

> 這裡只講**安裝**。裝好之後怎麼跑模擬、怎麼看結果 → [`../README.md`](../README.md)。
> 2026-09-26 起模擬套件本體已經寫好；之前裝過環境的人 `git pull` 後再跑一次 `setup_ubuntu.sh`（重跑不會壞）就會一起建進去。

| 步驟 | 誰做 | 時間 | 佔用 |
|---|---|---|---|
| 0. 啟用虛擬機器平台 | 你（要系統管理員＋重開機） | 5 分鐘 | — |
| 1. `setup_wsl.ps1` | 你執行 | 5~10 分鐘 | D 槽約 1 GB |
| 2. `setup_ubuntu.sh` | 你在 Ubuntu 裡執行 | 30~60 分鐘（大多在等下載） | D 槽再約 4 GB |
| 3. 冒煙測試 | 你 | 5 分鐘 | — |

合計 D 槽約 5~7 GB（依 apt 索引實算：Ubuntu 1 GiB＋ROS/Gazebo 3.3 GiB＋build 與快取）。

**開始前確認**：Windows 10 版本 19041 以上（這台是 19045 ✓）、D 槽剩 15 GB 以上、有這台電腦的系統管理員密碼。

---

## 第 0 步：啟用虛擬機器平台（只要做一次）

這台電腦的 WSL 程式有裝，但 **WSL2 還不能跑**（`wsl --status` 會說「不支援 WSL2，請啟用虛擬機器平台」）。
BIOS 的虛擬化已經開了，缺的只是 Windows 的一個元件。

1. 按鍵盤的 **Windows 鍵**，打 `PowerShell`
2. 在搜尋結果「Windows PowerShell」上按**右鍵** →「**以系統管理員身分執行**」→ 跳出「是否允許變更」按「**是**」
   （視窗標題列會寫「系統管理員: Windows PowerShell」，沒有這幾個字就是開錯了）
3. 貼上這行（在 PowerShell 視窗裡**按滑鼠右鍵**就是貼上），按 Enter：
   ```
   wsl.exe --install --no-distribution
   ```
   會看到「正在安裝: 虛擬機器平台 … 已安裝 虛擬機器平台 … 要求的作業成功。變更將在系統重新開機後生效。」
4. **重新開機**（一定要，不然下一步會失敗）
5. 重開後開一個**一般的** PowerShell（不用系統管理員），打 `wsl --status`：
   不再出現「不支援 WSL2」就成功了

## 第 1 步：安裝 Ubuntu 24.04 到 D 槽

1. 開一般權限的 PowerShell，貼上：
   ```
   powershell -ExecutionPolicy Bypass -File "$env:USERPROFILE\Desktop\專題\車子\src\smartnav_sim\setup\setup_wsl.ps1"
   ```
2. 它會依序顯示 `1/4 … 4/4`：檢查第 0 步有沒有做 → 檢查 D 槽空間 → 把 Ubuntu 裝到 `D:\WSL\Ubuntu-24.04` →
   建 `.wslconfig`（WSL 最多用 12 GB 記憶體、8 核心，swap 放 D 槽）
3. 第 3 步途中會**開一個新的 Ubuntu 視窗**，要你設 Linux 帳號：
   - `Enter new UNIX username:` → 打一個**全小寫英文**的名字（例如 `smartnav`），Enter
   - `New password:` → 打密碼。**打字時畫面不會顯示任何東西，這是正常的**，打完按 Enter，再打一次確認
   - 這組帳密跟 Windows 無關，**自己記下來**，第 2 步會用到
   - 看到綠色的 `smartnav@電腦名:~$` 提示字元就是好了，打 `exit` 關掉這個視窗
4. 回到原本的 PowerShell，它會把最後幾步做完，最後印出：
   ```
   完成。下一步：開始功能表開「Ubuntu 24.04」，在裡面執行：
     bash "/mnt/c/Users/<你的使用者名稱>/Desktop/專題/車子/src/smartnav_sim/setup/setup_ubuntu.sh"
   ```
   **把印出來的那一行 `bash "…"` 整行複製起來**（已經換成你這台電腦的路徑了）

## 第 2 步：在 Ubuntu 裡裝 ROS 2 + Gazebo

1. 按 Windows 鍵，打 `Ubuntu`，開「**Ubuntu 24.04**」
2. 在 Ubuntu 視窗裡**按滑鼠右鍵貼上**第 1 步最後複製的那一行，按 Enter
3. 會問 `[sudo] password for smartnav:` → 打第 1 步設的 Linux 密碼（一樣看不到字）
4. 接著就是等。畫面會依序出現 `▶ 0/7 前置檢查` … `▶ 7/7 環境變數`，最久的是 `3/7`（下載約 0.7 GB、裝約 3.3 GB）
5. 最後出現 `▶ 完成`，並印出 ROS 2 與整個 Ubuntu 佔了多少空間

**做的事**：ROS 2 官方 apt 來源 → 裝 ros-base、ros-gz（Gazebo Harmonic）、nav2、slam_toolbox、rviz2 →
在 `~/sim_ws/src` 用 symlink 連到 Windows 這份 repo（**在 Windows 改檔，WSL 裡立刻生效**）→ colcon build →
在 `~/.bashrc` 加環境變數。

**中途斷掉（關視窗、網路斷、電腦睡著）沒關係**，重新貼同一行再跑一次，做過的步驟會自動跳過。

## 第 3 步：冒煙測試

**開一個新的** Ubuntu 視窗（舊的那個還沒讀到新的環境變數），一行一行試：

```
gz sim -v4 -r shapes.sdf                       # ① 應該跳出一個有幾個方塊的 3D 畫面，關掉視窗就結束
sudo apt install -y mesa-utils && glxinfo -B | grep -i renderer   # ② 看到 D3D12 (NVIDIA...) = 顯卡有用上
ros2 pkg list | grep -E "smartnav|frontier"    # ③ 應該列出 smartnav_msgs、smartnav_navigation_cc、smartnav_sim、frontier_exploration_ros2
ros2 launch smartnav_sim sim_explore.launch.py # ④ 自動建圖模擬（用法見 ../README.md）
```

①②③ 都過，環境就裝好了；④ 能跑，模擬就可以用了。

---

## 疑難排解

| 症狀 | 原因 | 對策 |
|---|---|---|
| 第 1 步說「WSL2 還不能用」 | 第 0 步沒做、或做了沒重開機 | 回第 0 步，**記得重開機** |
| 錯誤碼 `0x80370102` | BIOS 的虛擬化被關掉了 | 開機進 BIOS 打開 Intel VT-x／AMD-V（SVM） |
| `wsl --install` 說不認得 `--location` | WSL 版本太舊 | 先在 PowerShell 跑 `wsl --update`，再跑一次第 1 步 |
| 打密碼時畫面沒反應 | Linux 輸入密碼本來就不顯示 | 照打，打完按 Enter |
| `gz sim` 視窗一開就崩潰 | WSL 上 ogre2 渲染引擎初始化失敗（gz-rendering #662） | 加 `--render-engine ogre` |
| 視窗間歇性崩潰 | WSLg 已知問題（gz-sim #3335） | 模擬本身用 `-s --headless-rendering`（不開視窗），另開 rviz2 看 |
| `glxinfo` 顯示 llvmpipe | 沒用上顯卡，退回 CPU 軟體算圖（microsoft/WSL #12412） | 能跑但慢；確認 `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` 有設、顯卡驅動是新的 |
| 模擬很慢、車動作像慢動作 | RTF（模擬時間／真實時間）< 1 | ★ 部分節點用牆鐘計時，RTF < 1 時逾時會提早觸發，模擬裡的「卡住」可能是假象 |
| build 出現 `\r` 或 bad interpreter | Windows 端檔案變成 CRLF | repo 的 `.gitattributes` 已設 `eol=lf`；frontier 子模組要另外設（見 repo README「兩人協作」） |
| 3 步的 ③ 少了 frontier | 子模組沒初始化 | 在 Windows 的 `車子/` 跑 `git submodule update --init`，再重跑第 2 步 |
| 看到車上 Pi 的話題 | 跟 Pi 在同一個網路、互相發現 | `~/.bashrc` 已設 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`，開新視窗生效 |

## 完整移除

```
wsl --unregister Ubuntu-24.04     # ★ 會刪掉 D:\WSL\Ubuntu-24.04 裡所有東西，無法復原
```

再手動刪掉 `D:\WSL\` 與 `%USERPROFILE%\.wslconfig`。repo 本身不受影響（WSL 裡只是 symlink）。
