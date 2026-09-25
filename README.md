# 迎賓機器人 — `wheeltec` 分支：WHEELTEC 廠商驅動

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204B%208GB-red)

> **這個分支只放車子要用的廠商驅動。**專案本體（我們寫的程式、說明、實測數據）在 [`master`](../../tree/master)。
> 兩個分支的 git 歷史各自獨立，**不合併**。

## 怎麼用

在 Pi 上把這個分支 clone 成**獨立的底層 workspace** `~/wheeltec_ws`，先建它，主線再疊在上面：

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install -y libuvc-dev libgoogle-glog-dev          # astra 相機驅動要的，rosdep 蓋不到
git clone -b wheeltec --single-branch https://github.com/xiange1209/welcoming_robot.git ~/wheeltec_ws
cd ~/wheeltec_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build && source install/setup.bash                  # ★ 先 source 這裡，再去建主線
```

接著照 master 的 README「部署到 Pi」clone 並建 `~/welcoming_robot_ws`；它的 `setup.bash` 會自動帶上這裡。

**為什麼不放進主線的 `src/`**：主線 repo 規定所有套件直接放在 `./src`、不准有 `*_ws/` 子目錄（master 的 `CLAUDE.md`）。
現在車上的放法是 `~/welcoming_robot_ws/src/wheeltec_ws/`、跟主線一起建——那樣也能跑，**不用搬**；
master 的 `.gitignore` 也擋住了那個路徑，不會被誤 commit 進 master。

## 內容：`src/` 底下 17 個套件

| 套件（資料夾） | 用途 | 我們有沒有用 |
|---|---|---|
| `turn_on_wheeltec_robot` | 底盤驅動（STM32 串口）、EKF、整車 launch | ✅ 直接用 |
| `serial`（`serial_ros2/`） | 串口函式庫 | ✅ 底盤依賴 |
| `wheeltec_robot_msg` | 底盤自訂訊息 | ✅ 底盤依賴 |
| `wheeltec_robot_urdf` | 車體模型（TF） | ✅ 底盤 launch 會載入 |
| `lslidar_driver`、`lslidar_msgs` | N10 光達 | ✅ |
| `astra_camera`、`astra_camera_msgs` | Astra 深度相機 | ✅ |
| `ldlidar`（`ldlidar_ros2/ldlidar/`） | 另一款光達 | 底盤 launch 解析時會查到，照建 |
| `wheeltec_mic_ros2`、`wheeltec_mic_msg` | 6 麥克風陣列 | ❌ 車上沒有這個硬體（麥克風用相機內建的雙麥） |
| `wheeltec_slam_toolbox`、`wheeltec_cartographer`、`wheeltec_nav2`（`wheeltec_robot_nav2/`） | 廠商版建圖／導航 | ❌ 改用 master 的 `smartnav_navigation_cc` |
| `nav2_waypoint_cycle` | 定點巡航 | ❌ 巡邏模式待接 |
| `ollama_ros_chat`、`ollama_ros_msgs` | 廠商的 LLM 範例 | ❌ 改用 master 的 `smartnav_llm` |

沒用到的也照車上現況一起建，不影響執行，只是多花建置時間。

`scripts/` 是專案早期（2026-04～08）的工具，保留作紀錄；現行版本在 master。

## 規則

- **改廠商碼**：到 `~/wheeltec_ws/`（這個分支的 clone）裡改 → commit → `git push origin wheeltec`。
  改之前先想清楚——車子能跑是建立在這些原廠碼上面。
- **不要把這個分支合併進 master**，也**不要刪這個分支**。
- STM32 韌體是原廠的，不在這裡，也不要改。
- 2026-09-25 核對：這個分支的 `src/` 與車上實際在跑的 528 個檔案逐一相同。

## 授權

[LICENSE](LICENSE)（MIT）適用於本 repo 自有的部分；`src/` 下的 WHEELTEC 廠商程式碼依其原始授權條款。
