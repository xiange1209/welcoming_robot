# maprun/tools_1020 — 車上 agent 的實測小工具

2026-10-20 起由車上（Pi，`wheeltec-r680`）的 Claude agent 寫成，使用者 2026-10-22 同意收進 repo；10/22 晚上併入它在 4 GB 測試板（沒有任何周邊）上新增與修改的版本。
**只在車上用**：`python3`（Pi 的系統 Python）＋ `bash`，路徑假設在 `~/maprun/tools_1020/`，先 `source ~/maprun/env.sh`；筆電（Windows）上不要跑。

**可信度（老實說）**：這些工具最初只用假資料／假 LLM 驗證流程，沒有單元測試；之後在 10/20、10/21（與 10/22 測試板）的實測中用過，「用過」欄以 10/20 報告、`manual_results_*.tsv` 與上車日誌為準，「報告有提到」＝我只確認過報告提到它，沒逐筆核對用法。內文裡「交接文件 §5-x」指 10/20 當天的交接檔章節，現在已不適用。

| 工具 | 做什麼 | 會讓車動或改設定？ | 10/20~22 |
|---|---|---|---|
| `asr_cer.py` | 使用者照稿唸 10 句，逐字比對 `/user_text` 算 CER（`--print-script` 印稿） | 否（LLM 單元要先關，否則辨識結果會被送進 LLM） | 用過（ASR 五輪） |
| `face_test.py` | 人臉辨識 × 距離；`--watch-registration` 記下新註冊那筆的雜湊，分辨本人與別的已註冊者 | 否（導航要關） | 用過 |
| `estop_timing.py` | 平板急停計時：HMI 送零速並發 `/unwedge_abort` 當 t0，到車停下來；預設門檻 1.3 s | 否（只收訊息） | 用過（急停 2/2） |
| `load_monitor.py` | 多工負載時間序列：CPU 閒置%、load、可用記憶體、溫度、降頻旗標、ASR input overflow 次數、各話題則數 | 否（唯讀） | 用過 |
| `llm10.py` | 銀行十題：自動送題、收答案、量延遲、存 CSV（含「工具呼叫」欄）；`--settle`（帶位題第二則「X到了」收完才換題）、`--extra-nav` | **②③④⑩ 會讓車導航**（④ 因為現用地圖有「廁所」地點，10/22 起也在前面停下來問）；導航關著可加 `--nav-down-ok` | 用過（10/20、10/22 測試板） |
| `run_steer_experiment.sh` | 轉向分級曲率實驗一鍵腳本（直接發 `/cmd_vel`） | **會動車**；平板急停停不了它，只能在跑它的終端機按 Ctrl-C；前後左右 ≥ 2 m | 用過（10/21） |
| `result.sh` | 手動測試結果登記，寫 `~/maprun/verify_out/manual_results_<日期>.tsv` 並用 `trip_mark.sh` 在上車日誌留一行 | 否 | 用過 |
| `check_after_run.sh` | 故事線 ×10 跑完後的客觀檢查（只讀 log） | 否 | 報告有提到 |
| `live_status.py` | 實時狀態一頁看完：電池、溫度、CPU、各單元、ROS 節點、nav2 狀態、話題頻率、TF（唯讀，預設量 6 秒） | 否 | 報告有提到 |
| `hmi_unit.py` | 用 HMI 管理端 API 開／關單元或情境（等同平板「系統開關」）；管理密碼與權杖在執行時才從 `smartnav-hmi` 服務的環境讀，不印出、也沒寫死在檔案裡 | 會開關單元 | 報告有提到 |
| `planfail.py` | 唯讀：把教導路徑畫在地圖與現在的雷射上，看為什麼規劃失敗（`planfail.py <path_id> <out.png>`） | 否 | 報告有提到 |
| `mock_nav_cc.py` | 靜態測試用「假導航後端」：提供 `list_waypoints`／`list_maps`／latched `current_map`／`navigate`（等 N 秒回成功、不動車）。有真的提供者在線就拒絕啟動 | 否（不發 `/cmd_vel`、不寫資料庫） | 10/22 測試板（無底盤）用過：LLM 帶位 18/18、HMI 地圖頁 |
| `identity_access_test.py` | 身分權限整合測試：發假 `/user_identity`（虛構姓名）輪流扮演沒有人／訪客／VIP／黑名單，同時測平板帶位 API 與 LLM 回答；要 HMI＋LLM＋`mock_nav_cc.py`，**迎賓劇本要關著** | 否 | 10/22 測試板：4 種身分全符合 |
| `fake_identity_cc.py` | 發假身分（2 Hz）給 HMI／LLM 靜態測試；★ 迎賓劇本開著時不要發 `BLACKLIST`（會真的通報） | 否 | 10/22 用過 |
| `voice_stop_test.py` | 帶位途中說「停下來」：A 應 1~2 秒內取消並回「車子已經停下來了」，B 說別的照舊；配 `mock_nav_cc.py --secs 8` | 否 | 10/22 測試板：喊停 0.03 s 取消 |
| `asr_bench.py` | 語音辨識離線基準：TTS 合成測試句＋疊實驗室真實背景音，用車上同一套 ASR 設定辨識，量 CER／RTF／收尾延遲。★ 合成語音是乾淨標準口音的上限情境，實車 42~51% 才是現場值，只拿來比較設定 | 否 | 10/22 用過：找到 decoder int8 讓 CER 從約 22% 變 81% |
| `face_bench.py` | 人臉辨識離線基準：每幀延遲、不同臉寬／亮度／模糊的相似度；用 insightface 內附公開樣本圖，**不讀人臉資料庫** | 否 | 10/22 用過 |
| `face_e2e_bench.py` | 人臉→身分→迎賓端對端延遲（發假相機影像，量每一段）；要先開 face／user_auth／bank_reception（dryrun） | 否 | 10/22 用過：間隔 1／2／4 s 迎賓中位 1.35／1.57／2.16 s |
| `plan_bench/` | 離線規劃基準：地點兩兩規劃（`ComputePathToPose`），量規劃時間、繞行比、倒車長度、最小曲率半徑 | 否 | 10/22 用過：6 個地點兩兩規劃，28/30 條要倒車、總路程 33% 是倒車（原因是地點存的朝向） |
| `deploy_hmi_dist_cc.sh` | 只換 HMI 前端（不重跑 colcon build）：放進新 dist、補 `--symlink-install` 的逐檔連結、確認每個新檔 HTTP 200，**最後才換 index.html**（先換會整頁空白） | 會換前端檔案 | 10/22 用過 |

## 注意
- `llm10.py` 的題目與預期是**虛構示範資料**，答案對不對要人（或筆電端）依「預期」欄判，工具不判對錯。
- 打包解壓時，車上同名的檔會被覆蓋。車上的 `~/maprun/tools_1020` 可能比這裡新：先 `cp -a ~/maprun/tools_1020 ~/maprun/tools_1020.bak_<日期時分>`，解完 `diff -r` 兩邊。
- 檔案都是 LF 換行（`pack_car_code.py` 會擋 CRLF）。
