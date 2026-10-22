# maprun/tools_1020 — 車上 agent 的實測小工具

2026-10-20 由車上（Pi，`wheeltec-r680`）的 Claude agent 寫成，使用者 2026-10-22 同意收進 repo。
**只在車上用**：`python3`（Pi 的系統 Python）＋ `bash`，路徑假設在 `~/maprun/tools_1020/`，先 `source ~/maprun/env.sh`；筆電（Windows）上不要跑。

**可信度（老實說）**：這些工具最初只用假資料／假 LLM 驗證流程，沒有單元測試；之後在 10/20、10/21（與 10/22 測試板）的實測中用過，「用過」欄以 10/20 報告、`manual_results_*.tsv` 與上車日誌為準，「報告有提到」＝我只確認過報告提到它，沒逐筆核對用法。內文裡「交接文件 §5-x」指 10/20 當天的交接檔章節，現在已不適用。

| 工具 | 做什麼 | 會讓車動或改設定？ | 10/20~22 |
|---|---|---|---|
| `asr_cer.py` | 使用者照稿唸 10 句，逐字比對 `/user_text` 算 CER（`--print-script` 印稿） | 否（LLM 單元要先關，否則辨識結果會被送進 LLM） | 用過（ASR 五輪） |
| `face_test.py` | 人臉辨識 × 距離；`--watch-registration` 記下新註冊那筆的雜湊，分辨本人與別的已註冊者 | 否（導航要關） | 用過 |
| `estop_timing.py` | 平板急停計時：HMI 送零速並發 `/unwedge_abort` 當 t0，到車停下來；預設門檻 1.3 s | 否（只收訊息） | 用過（急停 2/2） |
| `load_monitor.py` | 多工負載時間序列：CPU 閒置%、load、可用記憶體、溫度、降頻旗標、ASR input overflow 次數、各話題則數 | 否（唯讀） | 用過 |
| `llm10.py` | 銀行十題：自動送題、收答案、量延遲、存 CSV（含「工具呼叫」欄） | **②③⑩ 會讓車導航**（導航開著時；現用地圖有「廁所」地點時 ④ 也會），這三題前會停下來問；④ 不會 | 用過（10/20、10/22 測試板） |
| `run_steer_experiment.sh` | 轉向分級曲率實驗一鍵腳本（直接發 `/cmd_vel`） | **會動車**；平板急停停不了它，只能在跑它的終端機按 Ctrl-C；前後左右 ≥ 2 m | 用過（10/21） |
| `result.sh` | 手動測試結果登記，寫 `~/maprun/verify_out/manual_results_<日期>.tsv` 並用 `trip_mark.sh` 在上車日誌留一行 | 否 | 用過 |
| `check_after_run.sh` | 故事線 ×10 跑完後的客觀檢查（只讀 log） | 否 | 報告有提到 |
| `live_status.py` | 實時狀態一頁看完：電池、溫度、CPU、各單元、ROS 節點、nav2 狀態、話題頻率、TF（唯讀，預設量 6 秒） | 否 | 報告有提到 |
| `hmi_unit.py` | 用 HMI 管理端 API 開／關單元或情境（等同平板「系統開關」）；管理密碼與權杖在執行時才從 `smartnav-hmi` 服務的環境讀，不印出、也沒寫死在檔案裡 | 會開關單元 | 報告有提到 |
| `planfail.py` | 唯讀：把教導路徑畫在地圖與現在的雷射上，看為什麼規劃失敗（`planfail.py <path_id> <out.png>`） | 否 | 報告有提到 |

## 注意
- `llm10.py` 的題目與預期是**虛構示範資料**，答案對不對要人（或筆電端）依「預期」欄判，工具不判對錯。
- 打包解壓時，車上同名的檔會被覆蓋。車上的 `~/maprun/tools_1020` 可能比這裡新：先 `cp -a ~/maprun/tools_1020 ~/maprun/tools_1020.bak_<日期時分>`，解完 `diff -r` 兩邊。
- 檔案都是 LF 換行（`pack_car_code.py` 會擋 CRLF）。
