# 自動建圖模擬環境：安裝步驟

在筆電的 WSL2 裡裝 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic，用來模擬 senior_akm 的自動建圖（frontier 探索）。
**全部裝在 D 槽，C 槽幾乎不佔。**

| 步驟 | 誰做 | 時間 | 佔用 |
|---|---|---|---|
| 0. 啟用虛擬機器平台 | 你（要系統管理員＋重開機） | 5 分鐘 | — |
| 1. `setup_wsl.ps1` | 你執行 | 5~10 分鐘 | D 槽約 1 GB |
| 2. `setup_ubuntu.sh` | 你在 Ubuntu 裡執行 | 30~60 分鐘 | D 槽再約 4 GB |
| 3. 冒煙測試 | 你 | 5 分鐘 | — |

合計 D 槽約 5~7 GB（依 apt 索引實算：Ubuntu 1 GiB＋ROS/Gazebo 3.3 GiB＋build 與快取）。

---

## 第 0 步：啟用虛擬機器平台（只要做一次）

這台電腦的 WSL 程式有裝，但 **WSL2 還不能跑**（`wsl --status` 會說「不支援 WSL2，請啟用虛擬機器平台」）。
BIOS 的虛擬化已經開了，缺的只是 Windows 的一個元件。

1. 開始功能表搜尋 **PowerShell** → 右鍵 → **以系統管理員身分執行**
2. 貼上：
   ```
   wsl.exe --install --no-distribution
   ```
3. **重新開機**
4. 重開後開一般 PowerShell，執行 `wsl --status`：不再出現「不支援 WSL2」就成功了

## 第 1 步：安裝 Ubuntu 24.04 到 D 槽

一般權限的 PowerShell：

```
powershell -ExecutionPolicy Bypass -File "$env:USERPROFILE\Desktop\專題\車子\src\smartnav_sim\setup\setup_wsl.ps1"
```

它會：檢查第 0 步有沒有做 → 檢查 D 槽空間 → 把 Ubuntu 裝到 `D:\WSL\Ubuntu-24.04` → 建 `.wslconfig`（WSL 最多用 12 GB 記憶體、8 核心，swap 放 D 槽）。

途中會開一個 Ubuntu 視窗要你設 Linux 使用者名稱與密碼（跟 Windows 的無關，自己記得就好）。

## 第 2 步：在 Ubuntu 裡裝 ROS 2 + Gazebo

開始功能表開 **Ubuntu 24.04**，貼上第 1 步最後印出的那一行（已經換成你這台電腦的路徑），長這樣：

```
bash "/mnt/c/Users/<Windows 使用者名稱>/Desktop/專題/車子/src/smartnav_sim/setup/setup_ubuntu.sh"
```

途中會問你剛設的 Linux 密碼（sudo 用）。重跑不會壞，中斷了就再跑一次。

**做的事**：ROS 2 官方 apt 來源 → 裝 ros-base、ros-gz（Gazebo Harmonic）、nav2、slam_toolbox、rviz2 →
在 `~/sim_ws/src` 用 symlink 連到 Windows 這份 repo（**在 Windows 改檔，WSL 裡立刻生效**）→ colcon build →
在 `~/.bashrc` 加環境變數。

## 第 3 步：冒煙測試

開一個**新的** Ubuntu 視窗：

```
gz sim -v4 -r shapes.sdf                       # 應該出現一個有幾個方塊的 3D 畫面
sudo apt install -y mesa-utils && glxinfo -B | grep -i renderer   # 看到 D3D12 (NVIDIA...) = 顯卡有用上
ros2 launch smartnav_sim sim_explore.launch.py # 自動建圖模擬
```

---

## 疑難排解

| 症狀 | 原因 | 對策 |
|---|---|---|
| `gz sim` 視窗一開就崩潰 | WSL 上 ogre2 渲染引擎初始化失敗（gz-rendering #662） | 加 `--render-engine ogre` |
| 視窗間歇性崩潰 | WSLg 已知問題（gz-sim #3335） | 模擬本身用 `-s --headless-rendering`（不開視窗），另開 rviz2 看 |
| `glxinfo` 顯示 llvmpipe | 沒用上顯卡，退回 CPU 軟體算圖（microsoft/WSL #12412） | 能跑但慢；確認 `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` 有設、顯卡驅動是新的 |
| 模擬很慢、車動作像慢動作 | RTF（模擬時間／真實時間）< 1 | ★ 見 `../README.md` 的「RTF」一節 —— 部分節點用牆鐘計時，RTF < 1 時逾時會提早觸發，模擬裡的「卡住」可能是假象 |
| build 出現 `\r` 或 bad interpreter | Windows 端檔案變成 CRLF | repo 的 `.gitattributes` 已設 `eol=lf`；frontier 子模組要另外設（見專題 CLAUDE.md 第 7 條） |
| 看到車上 Pi 的話題 | 跟 Pi 在同一個網路、互相發現 | `~/.bashrc` 已設 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`，開新終端機生效 |

## 完整移除

```
wsl --unregister Ubuntu-24.04     # ★ 會刪掉 D:\WSL\Ubuntu-24.04 裡所有東西，無法復原
```

再手動刪掉 `D:\WSL\` 與 `%USERPROFILE%\.wslconfig`。repo 本身不受影響（WSL 裡只是 symlink）。
