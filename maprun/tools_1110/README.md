# tools_1110：11/10 實車驗證用的量測工具

資料一律寫在 `~/maprun/verify_out/day_1110/`（不進 git）。每支的用法寫在檔頭。

| 工具 | 用途 |
|---|---|
| `trip_report.py` | 每一趟帶位的 log 與循跡重播摘要（成功與否、終點誤差、重新規劃次數） |
| `estop_slide.py` | 急停的反應時間與滑行距離（訂 `/unwedge_abort` 與 `/odom`） |
| `localize_settle.py` | 先設位姿、等 3 秒再做掃描對齊（繞過 HMI「定位到這裡」只等 0.5 秒的問題） |
| `injection_test.py` | 話術防騙：用假身分＋假導航送 20 種說法，記錄導航仲裁器有沒有放行（車不動） |
| `audit_ss.py`／`audit_report.py` | 連線稽核：每 0.5 秒記錄連線，與 tcpdump 對起來，分出區網／外網 |
| `battery_log.py` | 每 30 秒記錄電池電壓 |
| `cpu_snap.py` | N 秒的 CPU 快照（各程式用了多少） |
| `cpu_scenarios.py` | CPU 四種情況（只開導航／加人臉／全開／全開但關人臉相機）；只自測過 |
| `block_external.py` | 封鎖對外連線並自動解除的測試流程；只自測過 |
| `voice_pack.py` | 收音測試三輪合成逐句表（遮掉旁人講話與迎賓詞內容） |
