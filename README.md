# 迎賓機器人 — `wheeltec` 分支：WHEELTEC 廠商驅動

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204B%208GB-red)

> **這個分支只放車子要用的廠商驅動。**專案本體（我們寫的程式、說明、實測數據）在 [`master`](../../tree/master)。
> 兩個分支的 git 歷史各自獨立，**不合併**。

## 怎麼用

在 Pi 上把這個分支 clone 到主線 workspace 的 `src/wheeltec_ws/`，跟我們的套件一起 `colcon build`
（車上現在就是這個結構）：

```bash
git clone --recurse-submodules https://github.com/xiange1209/welcoming_robot.git ~/welcoming_robot_ws
git clone -b wheeltec --single-branch https://github.com/xiange1209/welcoming_robot.git ~/welcoming_robot_ws/src/wheeltec_ws
```

完整步驟（apt 相依、前端建置、建置指令）見 master 的 README「部署到 Pi」。
master 的 `.gitignore` 已經擋掉 `src/wheeltec_ws/`，所以放在裡面不會被誤 commit 進 master。

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

- **改廠商碼**：到 `src/wheeltec_ws/` 裡改 → commit → `git push origin wheeltec`。
  改之前先想清楚——車子能跑是建立在這些原廠碼上面。
- **不要把這個分支合併進 master**，也**不要刪這個分支**。
- STM32 韌體是原廠的，不在這裡，也不要改。
- 2026-09-25 核對：這個分支的 `src/` 與車上實際在跑的 528 個檔案逐一相同。

## 授權

[LICENSE](LICENSE)（MIT）適用於本 repo 自有的部分；`src/` 下的 WHEELTEC 廠商程式碼依其原始授權條款。
