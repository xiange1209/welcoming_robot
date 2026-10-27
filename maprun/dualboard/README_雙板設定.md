# 雙板設定（草稿，2026-10-22 車上 agent）

> ★★ **2026-10-23 起改用一份設定檔＋一支工具**：`board_profiles.ini`（single／A／B 每一項要什麼值、改在哪裡）＋`board_profile.py`
>   `python3 board_profile.py status`（這塊板子是哪一種、逐項 ✓／✗）→ `plan A`（只列出）→ `sudo python3 board_profile.py apply A`（備份＋套用＋再檢查）。
>   回單板＝`apply single`。測試 `test_board_profile.py`（沙盒 57 項：三種來回、重複執行、記憶體不符拒絕、重現 10/22 的 B 狀態）。
>   下面第 0～5 節的各支腳本還在、行為沒變，但 **`set_board_name.sh` 加的 /etc/hosts 兩行重開機會被洗掉**：
>   user-data 的 `manage_etc_hosts: true` 蓋過 cloud.cfg.d 的 `false`，cloud-init 每次開機依範本重寫 /etc/hosts（10/23 在 B 上看到）。
>   `board_profile.py` 改的是範本 `/etc/cloud/templates/hosts.debian.tmpl`，開機前後一致。
> ★ **2026-10-23 這張 SD 卡（4 GB 測試板）已改回 wheeltec-r680 單板**（第 0 節那些 B 設定全部拿掉；紀錄與原檔備份 `revert_r680_20261023/`）。

> **2026-10-22 筆電端審查後的版本**（車上 agent 草稿 → 審查 → 修正 6 處，見第 9 節）。這裡的檔案**除了第 0 節列的 B 測試板那幾項之外都沒有套用**：校時、服務、A 板都沒動，其餘只做了語法、解析與替身測試。
> ★ **2026-10-22 使用者決定：先在單板把所有功能驗完，之後才換雙板**——所以現在不要套用（單板驗證用的交辦在 `HANDOFF_car_agent_20261022.md`）。B 測試板那些設定跟著 SD 卡走，8 GB 板要用單板時先還原（同一份交辦 §0）。
> 與草稿最大的差別：**終端機與所有腳本照舊 `source ~/maprun/env.sh` 就好**（雙板環境由標記檔 `~/.smartnav/dualboard_role` 決定，見第 9 節 ①）。
> 起草時所在的是 **4 GB 測試板**，什麼周邊都沒接；SD 卡是 10/21 在車上（8 GB）用過的那張。

## 0. ★ 10/22 已在 B（4 GB 測試板）實際套用、實測的部分（紀錄 `apply_log_B_20261022.txt`）
| 做了什麼 | 結果 |
|---|---|
| `sudo nm_eth0_setup.sh B` | ✓ 連線設定 `dualboard-eth0`（優先權 100）；**沒插網路線、連結 DOWN，eth0 仍保有 10.10.10.2/24**（NM 顯示 connected）→ `ignore-carrier` 在 NM 1.46 有效 |
| Wi-Fi 有沒有受影響 | ✓ 預設路由仍走 wlan0、ping 熱點 0% 掉包 |
| DDS 綁 eth0（沒插線） | ✓ `ros2 topic list` 正常（同一份設定在套用前失敗：`eth0: does not match an available interface`）；同板兩個程式經 eth0 互傳成功 |
| 單板環境（env.sh） | ✓ 不受影響，照樣看得到 HMI |
| `check_dualboard.sh` | ✓ 沒有對方時正確回報（第一次因 1000 次 ping 等 ARP 卡住，已修：先試 3 次） |
| 命名 `set_board_name.sh B` | ✓ hostname **smartnav-b**、cloud-init 覆寫（不再改回 wheeltec-r680）、/etc/hosts 加 A／B、提示字元 `[B·4GB]`；重跑不會重複加 |

沒做（要第二塊板子、網路線或 Astra S）：校時設定、停 B 的 HMI、裝感知服務、V1／V2／V4~V9。原本的「Wired connection 1」（DHCP）沒動，靠優先權讓位。

## 1. 分工（使用者 10/22 定案）

| | A：8 GB（車上，車電） | B：4 GB（行動電源） | 筆電 | 平板 |
|---|---|---|---|---|
| 負責 | 底盤、雷達、IMU/EKF、導航（含教導重播、防撞、急停）、**HMI**、LLM 節點、迎賓劇本 | **Astra S（相機＋麥克風）**、人臉特徵、使用者認證（人臉資料庫）、語音偵測＋辨識 | Ollama（LLM 推論）、SSH／VS Code 控制 | 開 A 的 HMI |
| 連線 | Wi-Fi（熱點）＋網路線 eth0 `10.10.10.1/24` | 只有網路線 eth0 `10.10.10.2/24` | Wi-Fi 熱點 `192.168.137.1` | Wi-Fi 熱點 |

- **Astra S 同時是相機和麥克風** → 相機、人臉、收音都必須在插著它的 B。
- **會讓車動、與安全有關的全部留在 A**：B、網路線或行動電源掛了，車照樣能安全行駛，只是不會認人、聽不到。
- 跨板只傳小東西：`/user_identity`、`/user_text`、`/speech_text`、`/playback_status`，以及給 HMI 的壓縮影像（建議約 6 fps）。人臉註冊走 ROS 服務（`register_face`、`list_users`…，`hmi_server_node.py:416-420`）→ **可以跨板**，HMI 留在 A 沒問題。
- 依 10/20 實測（8 GB 單板全開 CPU 閒置 0%）：A 拿掉視覺與語音後約等於「人臉＋相機關」那組（閒置 38%、/odom 20 Hz）；B 視覺約 2 核以上，偏緊 → 建議同時採用「待機人臉每 4 秒」（實測省約 65 點）。

## 2. 檔案（都在 `~/maprun/dualboard/`）

| 檔案 | 放哪塊 | 裝到哪 | 作用 |
|---|---|---|---|
| `overlay.sh` | A、B | 原地（由 `env.sh` 自動 source） | **雙板環境的唯一來源**：DDS 只走 eth0、domain 42、探索範圍 SUBNET、`SMARTNAV_DUALBOARD=1`、`SMARTNAV_BOARD=A/B` |
| `dualboard_mode.sh` | A、B | 原地 | `A`／`B`／`off`／`status`：寫、刪標記檔 `~/.smartnav/dualboard_role`。**標記檔存在＝雙板環境，不存在＝單板** |
| `env_dualboard.sh` | B（systemd 感知服務） | 原地 | 不管有沒有標記檔都強制套用 overlay（薄包裝：`source env.sh` ＋ `source overlay.sh`） |
| `eth0_static_fallback.sh` | A、B（**備案**，V3 失敗才用） | 原地 | NetworkManager 不管 eth0、由 systemd 開機時用 `ip` 設固定位址（不靠 NM 的 ignore-carrier） |
| `cyclonedds_dualboard.xml` | A、B | 原地 | 由 `~/cyclonedds.xml` 複製，只改網卡為 eth0 |
| `nm_eth0_setup.sh` ＋ `nm/60-dualboard-eth0.conf` | A、B | `/etc/NetworkManager/conf.d/`＋NM 連線「dualboard-eth0」 | 固定 IP；**沒插線也保有位址**（ignore-carrier 兩種寫法都放，V3 實測） |
| `chrony/50-dualboard-server.conf` | A | `/etc/chrony/conf.d/` | A 當 B 的校時伺服器 |
| `timesyncd/50-dualboard.conf` | B | `/etc/systemd/timesyncd.conf.d/` | B 只向 A 校時 |
| `systemd/smartnav-hmi.service.d/50-dualboard.conf` | A | `/etc/systemd/system/smartnav-hmi.service.d/` | HMI 服務改用雙板的 DDS 環境 |
| `systemd/smartnav-perception.service` ＋ `launch/perception_b.launch.py` | B | `/etc/systemd/system/`；launch 原地 | B 開機自動開相機（人臉）、人臉、認證、語音輸入 |
| `b_power_log.sh` ＋ `systemd/smartnav-b-powerlog.service` | B（選用） | `/etc/systemd/system/` | 每 10 秒記欠壓／降頻旗標 → `~/maprun/logs/b_power_<日期>.csv` |
| `check_dualboard.sh` | A、B | 原地 | 唯讀檢查：網路線、頻寬、校時、跨板節點 |
| `set_board_name.sh` ＋ `hosts_dualboard.txt` | A、B | `/etc/hosts`、hostname、cloud-init 覆寫、~/.bashrc | 名字 smartnav-a／smartnav-b（car-a／car-b）＋提示字元 [A·8GB]／[B·4GB] |
| `ssh_config_laptop.txt` | 筆電 | `%USERPROFILE%\.ssh\config` | `ssh car-b` 經 A 跳板 |

## 3. 安裝順序

**A（8 GB，車上；挑車子停著時做）**
0. `sudo ~/maprun/dualboard/set_board_name.sh A`（hostname smartnav-a、提示字元 [A·8GB]；A 的卡若是複製 B 這張，名字會是 smartnav-b，一定要改）
1. `sudo ~/maprun/dualboard/nm_eth0_setup.sh A`
2. `sudo apt install chrony iperf3`，裝 `chrony/50-dualboard-server.conf`，`sudo systemctl restart chrony`
3. `/etc/hosts` 加 `hosts_dualboard.txt` 的兩行
4. **先做驗證 V3（拔線照樣能動）再往下**
5. 裝 HMI 的 drop-in（`systemd/smartnav-hmi.service.d/`，**先不要 restart**）
6. `~/maprun/dualboard/dualboard_mode.sh A`（寫標記檔；**之後所有 `source ~/maprun/env.sh` 的腳本與終端機自動吃雙板環境，不用換指令**）
7. 平板先把所有單元關掉 → `sudo systemctl daemon-reload && sudo systemctl restart smartnav-hmi`（會一併停掉 HMI 開著的單元）
8. `~/maprun/dualboard/check_dualboard.sh` 的 ⑤ 要全 ✓（HMI 服務的環境＝overlay）

**B（4 GB）**
1. 準備 SD 卡：沿用 10/22 這張測試卡（**已改名 smartnav-b、eth0 已設好**），或複製 A 的映像後跑 `sudo set_board_name.sh B`、`sudo nm_eth0_setup.sh B`。★ 雙板需要**兩張各自完整的 SD 卡**（A 一張、B 一張）；目前這張卡同時帶著車上的資料與 B 的設定，分卡方式由使用者決定。
2. 安裝期間先讓 B 連 Wi-Fi：`sudo apt install iperf3`，確認 `/etc/sysctl.d/60-ros2-dds.conf` 在（影像接收緩衝）。
3. `sudo ~/maprun/dualboard/nm_eth0_setup.sh B`；`/etc/hosts` 加兩行；裝 `timesyncd/50-dualboard.conf`、`sudo systemctl enable systemd-time-wait-sync`；`~/maprun/dualboard/dualboard_mode.sh B`
4. `sudo systemctl disable --now smartnav-hmi`（B 不跑 HMI）
5. 裝 `smartnav-perception.service`（＋選用 powerlog）→ `enable --now`
6. 都好了再關掉 B 的 Wi-Fi 自動連線（`nmcli con modify "<熱點名稱>" connection.autoconnect no`）——B 不該再出現在 Wi-Fi 網段上（同名、同設定會互相干擾）。
7. 資料：人臉資料庫要在 B（認證節點在 B）；A 的人臉資料庫可留可不留。秘密（Telegram token）B 用不到。

**筆電**：`ssh_config_laptop.txt` 貼進 `%USERPROFILE%\.ssh\config`。

## 4. 驗證（全部通過才算雙板可用）

| # | 做什麼 | 通過標準 |
|---|---|---|
| V1 | 桌上網路測試（交接文件 §5-10）：`check_dualboard.sh` ①② | 1000Mb/s Full、ping 1000 次 0% 掉包、iperf3 雙向 ≥ 300 Mbit/s |
| V2 | 校時：`check_dualboard.sh` ③ | B 的 Server＝10.10.10.1、Offset 毫秒等級；B 重開機後感知服務在校時之後才起來 |
| **V3** | **A 拔掉網路線、重開機**：感測、導航、HMI、遙控、急停都照常 | `ip -br addr show eth0` 仍是 10.10.10.1；`ros2 node list` 正常；車能動。**不過就不能上車**；位址消失 → 先試備案 `sudo ~/maprun/dualboard/eth0_static_fallback.sh A`，再重做 V3 |
| V4 | 跨板節點：`check_dualboard.sh` ④（A、B 各跑一次） | 全部 ✓ |
| V5 | 跨板資料流：A 上 `ros2 topic hz /user_identity`、`/user_text`；平板看得到 B 的相機畫面 | 有資料、畫面不卡 |
| V6 | 從平板（A 的 HMI）註冊／列出使用者 | 成功，資料寫在 B |
| V7 | 兩板同時跑 `~/maprun/tools_1020/load_monitor.py`（全開） | A 閒置明顯 > 0、`/odom` ≈ 24 Hz；B 的 ASR overflow 接近 0 |
| V8 | 行動電源續航：B 全開掛著，看 `b_power_<日期>.csv` | 不出現 `undervolt_now=1`；記下撐幾小時 |
| V9 | 雙板故事線 ×10 | 交接文件 §5-3 的四項標準 |
| **V10** | **A 的平板「系統開關」頁**（HMI 新增的雙板顯示） | B 的相機／人臉／認證／語音四個單元**亮燈**、旁邊寫「在 B 板…」（灰字，不是紅色警告）；按它們的啟動／停止只會得到「A 不管它」；跑「完整迎賓故事線」情境不會去開／關它們；**拔掉網路線約 12～15 秒後四個都變紅色「B 板離線」** |
| V0 | A、B 各跑一次一鍵驗證 | 雙板時另一塊板子的項目記 INFO（A：Astra／麥克風／語音模型在 B；B：電池／底盤在 A），**不能有 FAIL** |

## 5. 回到單板（任何一步出問題）
- ★ 2026-10-23 起：`sudo python3 ~/maprun/dualboard/board_profile.py apply single`（下面三條它都會做，另外還原 hostname、/etc/hosts、.bashrc、校時）。
- A：刪 HMI drop-in → `daemon-reload` → `restart smartnav-hmi`；終端機改回 `source ~/maprun/env.sh`。網路線設定可以留著（沒用到不影響），要拿掉照 `nm_eth0_setup.sh` 檔頭。
- B：`sudo systemctl disable --now smartnav-perception`；要當單板用就 `enable --now smartnav-hmi`。
- 沒有修改任何既有檔案（`env.sh`、`~/cyclonedds.xml`、原本的服務檔都沒動），所以回復只是「不用新的那幾個」。

## 6. 筆電端要改的程式（設定檔做不到的）——2026-10-22 已做完
1. **HMI 單元管理**（已做，測試 `maprun/tools_1021/test_dualboard_hmi.py` 27 項）：`SMARTNAV_DUALBOARD=1` 時 A 的 HMI 對 camera／face／user_auth／asr_chain **不啟動也不停止**（成功回報，情境的其他步驟照常）；單板（沒有這個環境變數）行為完全不變。`SMARTNAV_REMOTE_UNITS`（逗號分隔）可覆寫清單。
   ★ 草稿寫「狀態顯示可沿用節點偵測（跨板看得到）」**是錯的**：HMI 的 `detect` 比對的是**本機行程指令列**，A 看不到 B 的行程。已改成雙板時這四個單元看 **ROS 節點圖**（`face_node`、`user_auth_node`、`voice_trigger_node`＋`speech_recognizer_node`、`camera`），跨板看得到。
2. **B 離線監看**（已做，併在 1）：B 的節點從節點圖消失（約 12 秒，DDS 租約）→ 四個單元在 A 面板變成紅色「B 板（感知板）離線或還沒就緒」；迎賓頁的相機畫面也會顯示 NO CAMERA SIGNAL。沒有另外做彈窗。
3. **打包分兩種**：**不需要**。同一包、角色由標記檔決定（B 的 SD 卡是 A 的映像複本，程式碼本來就一樣）。收資料：A、B 各跑一次 `~/maprun/verify/collect_data.sh --all`（B 的 log `perception_b_*.log` 在同一個 `~/.smartnav/logs/system_control/`，現有腳本就收）；`asr_cer.py` 用 `--asr-log` 指到 B 的 log。
4. 選擇性調參（10/20 實測依據在報告 §12-2）：待機人臉每 4 秒；影像先降頻再轉 JPEG。**尚未做**（要等雙板實測的 V7 數字）。

## 7. 已知風險與還沒確認的事
- **eth0 沒有位址 → 該板 DDS 起不來 → 全部 ROS 節點失敗**。靠 NetworkManager 的 `ignore-carrier` 保住位址；NM 1.46 這個選項要實機確認（V3）。
- 沒設 `<Peers>`，靠直連的多播探索；中間加交換器時要重想。
- 行動電源要能穩定輸出 5V/3A（PD），短而粗的線；沒電時 B 直接斷電有 SD 卡風險 —— 要不要做「欠壓自動關機」由你們決定（`b_power_log.sh` 目前只記錄）。
- B 若沿用這張測試卡：它是 A 映像的完整複本，含 Wi-Fi 設定、相同 hostname、秘密檔 —— 照第 3 節改。
- 記憶體：B 上 VS Code 伺服器＋Claude 約吃 2.2 GB，正式運作不要用 VS Code 連 B。

## 8. 起草時做過的檢查（4 GB 測試板，什麼都沒接）
見同目錄 `check_log_20261022.txt`。

## 9. 筆電端審查紀錄（2026-10-22）
車上 agent 草稿的分工、DDS 隔離（綁 eth0＋domain 42＋SUBNET）、校時（A chrony、B timesyncd）、systemd 單元與 launch 的設計都合理，註解誠實標出了每個「尚未實機驗證」。審查找到 **6 處要修**（已修）與 **3 處待實機確認**：

| # | 問題 | 嚴重度 | 處理 |
|---|---|---|---|
| ① | **`env.sh` 會把雙板環境洗掉**：HMI 的「定位＋導航」單元走 `run_nav_cc.sh`，它一開頭就 `source env.sh`，而 `env.sh` 無條件設 `ROS_DOMAIN_ID=0`、`LOCALHOST`、單板的 xml → 這個單元跑在 domain 0，HMI（domain 42）**看不到自己開出來的導航**。12 支腳本都 source `env.sh` | 高 | `env.sh` 結尾加區塊：標記檔存在才 source `overlay.sh`；沒有標記檔＝完全單板。`test_dualboard_env.py` 19 項（含「overlay 的值＝HMI drop-in 的值」防漂移） |
| ② | **README 說 A 的面板「跨板看得到」B 的單元，是錯的**（HMI 看本機行程） | 高 | 雙板時看 ROS 節點圖（見第 6 節 1） |
| ③ | A 的情境會去開／關 B 的四個單元（A 沒 Astra 會失敗、人臉重複推論、停掉 B 的感知） | 高 | 見第 6 節 1 |
| ④ | **V0 在 A 上會誤報 FAIL**（Astra、麥克風、語音模型在 B；B 上則是電池、底盤）→「V0 沒全過就不能往下」卡死流程 | 中 | `verify_0_preflight.sh` 認 `SMARTNAV_BOARD`，另一塊的項目記 INFO；單板不變 |
| ⑤ | **V3 的唯一保險（NM `ignore-carrier`）寫在 `[device-…]` 區段**，NM 1.46 是否認這個寫法無法在筆電驗證 | 中 | `[main]` 的寫法也放；再加**備案** `eth0_static_fallback.sh`（NM 不管 eth0，systemd 設位址，核心不會因沒載波移除 IPv4 位址） |
| ⑥ | HMI drop-in 與 overlay 是兩份重複的值，會漂移；`check_dualboard.sh` 沒檢查 | 低 | drop-in 補 `SMARTNAV_BOARD=A`；`check_dualboard.sh` ⑤ 逐項比對 HMI 服務的環境 |

**待實機確認（筆電端無法驗證）**：①V3（拔線重開機 eth0 位址還在；不過就用備案）②DDS 多播在直連線上的跨板發現（V4）③B 的記憶體與 CPU 在「人臉＋語音」同開時夠不夠（V7）。
另外 `ExecStartPre=/bin/sleep 20` 與 `time-sync.target` 讓 B 在 A 沒開機時**一直等**（`systemd-time-wait-sync` 無逾時），這是刻意的——B 沒有 A 也沒有用——但開機後看不到 B 的單元就先檢查 A 開了沒、線插了沒。
