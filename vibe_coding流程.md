# Vibe Coding 流程 - Claude + 樹莓派 AI 開發指南

## 📖 概述

這是一套使用 Claude Code 進行遠程開發樹莓派 AI 項目的完整工作流程。適合團隊協作、快速迭代和即時測試。

---

## 🔄 完整開發迴圈

```
┌─────────────────────────────────────────────────────────────┐
│  1️⃣  需求/Bug 分析                                         │
│  📝 您描述問題或需求                                        │
└─────────────────────────────────────────┬───────────────────┘
                                          ↓
┌─────────────────────────────────────────────────────────────┐
│  2️⃣  Claude 設計與代碼                                      │
│  🤖 我閱讀現有代碼、分析架構、生成/修改代碼文件            │
│     在 Windows 上的 C:\Users\陳佳憲\Desktop\專題           │
└─────────────────────────────────────────┬───────────────────┘
                                          ↓
┌─────────────────────────────────────────────────────────────┐
│  3️⃣  SCP 傳輸（Windows → 樹莫派）                          │
│  💾 scp -r C:\Users\陳佳憲\Desktop\專題                    │
│     xiange@192.168.1.125:~/ai_bank_robot/                 │
└─────────────────────────────────────────┬───────────────────┘
                                          ↓
┌─────────────────────────────────────────────────────────────┐
│  4️⃣  遠程執行與測試（樹莫派）                              │
│  🖥️  ssh xiange@192.168.1.125                              │
│     python3 scripts/benchmark.py                           │
│     python3 scripts/realtime_detection.py                  │
└─────────────────────────────────────────┬───────────────────┘
                                          ↓
┌─────────────────────────────────────────────────────────────┐
│  5️⃣  結果反饋與迭代                                        │
│  📊 您報告測試結果、性能數據、錯誤信息                      │
│  🔄 回到步驟 1，重複迴圈                                    │
└─────────────────────────────────────────────────────────────┘
```

---

## 📋 逐步詳細指南

### Phase 1️⃣：需求分析（您 ↔ Claude）

#### 場景示例

```
❌ 問題反饋
您：樹莓派運行 benchmark.py 時內存使用達到 1.8GB，超過目標

✅ Claude 的回應
1. 讀取現有的 scripts/benchmark.py
2. 分析內存洩漏點
3. 提出優化方案
4. 修改代碼並解釋改進
```

#### 最佳實踐

- **具體描述問題**（包括錯誤信息）
- **提供上下文**（硬件規格、預期結果）
- **提供性能指標**（CPU、內存、延遲）
- **說明重現步驟**（如何觸發該問題）

---

### Phase 2️⃣：代碼生成與修改（Claude）

#### Claude 的工作內容

```markdown
🔍 代碼閱讀
  - 分析現有架構
  - 識別模式和約束
  - 理解項目規範

🏗️ 設計方案
  - 考慮樹莓派限制（8GB RAM，4核 CPU）
  - 考慮延遲目標（<500ms）
  - 檢查依賴衝突

✍️ 代碼編寫/修改
  - 創建新檔案
  - 修復現有問題
  - 添加測試工具

📝 上傳修改
  - 使用 Write/Edit 工具將代碼保存到 Windows 項目文件夾
```

#### 常見場景

**場景 A：新模塊開發**
```
1. Claude 讀取 config/inference_config.yaml
2. 基於配置創建新文件（如 vision_ai/liveness_checker.py）
3. 文件直接寫入 Windows 項目根目錄
4. 您接下來 SCP 傳輸
```

**場景 B：Bug 修復**
```
1. 您報告樹莓派上的錯誤
2. Claude 讀取源碼，找到問題
3. 使用 Edit 工具修改特定行
4. 您再次 SCP 傳輸新版本
```

**場景 C：性能優化**
```
1. 您運行 scripts/benchmark.py，報告結果
2. Claude 分析輸出，識別瓶頸
3. 修改推理引擎、模型配置或算法
4. 您測試新版本，反饋結果
```

---

### Phase 3️⃣：傳輸代碼到樹莫派

#### Windows 終端操作

```bash
# 打開 PowerShell 或 Command Prompt
cd C:\Users\陳佳憲\Desktop\專題

# 查看最近修改的文件（確認有更新）
dir /s /B *.py | findstr "vision_ai" | sort /R

# 傳輸整個項目到樹莫派
scp -r . xiange@192.168.1.125:~/ai_bank_robot/

# 或傳輸特定文件
scp vision_ai/face_detector.py xiange@192.168.1.125:~/ai_bank_robot/vision_ai/
scp scripts/benchmark.py xiange@192.168.1.125:~/ai_bank_robot/scripts/
```

#### 優化技巧

```bash
# 傳輸前檢查磁盤速度
ping -n 1 192.168.1.125

# 使用 rsync 只傳輸更改的文件（如果樹莓派安裝了 rsync）
rsync -avz --exclude='.git' --exclude='*.pyc' . xiange@192.168.1.125:~/ai_bank_robot/

# 後台傳輸（不中斷其他工作）
# 在 PowerShell 中使用 &
scp -r . xiange@192.168.1.125:~/ai_bank_robot/ &
```

---

### Phase 4️⃣：遠程執行與測試

#### 連接樹莫派

```bash
# SSH 連接
ssh xiange@192.168.1.125

# 進入項目目錄
cd ~/ai_bank_robot

# 激活虛擬環境
source ~/ai_bank_robot_env/bin/activate

# 驗證環境
python3 -c "from vision_ai import InferenceEngine; print('✓ 環境就緒')"
```

#### 常見測試命令

```bash
# ========== 驗證環境 ==========
# 檢查 Python 版本
python3 --version

# 檢查核心套件
pip list | grep -E 'opencv|onnx|mediapipe|pyyaml'

# 驗證推理引擎
python3 -c "from vision_ai import InferenceEngine; engine = InferenceEngine(); print(f'Platform: {engine.platform}')"

# ========== 數據庫操作 ==========
# 初始化數據庫
python3 -c "from database.schema import DatabaseSchema; DatabaseSchema().initialize()"

# 查詢數據庫
sqlite3 database/ai_bank_robot.db "SELECT COUNT(*) FROM visit_logs;"

# ========== 性能測試 ==========
# 基準測試
python3 scripts/benchmark.py

# 實時人臉檢測（需要 X11 轉發或顯示器）
# DISPLAY=:0 python3 scripts/realtime_detection.py

# ========== 資源監控 ==========
# 實時監控 CPU 和內存
top -b -n 1 | head -15

# 磁盤使用量
df -h

# 溫度檢查
vcgencmd measure_temp

# ========== 代碼測試 ==========
# 運行特定 Python 模塊
python3 -m vision_ai.face_detector

# 交互式 Python 測試
python3
>>> from vision_ai import FaceDetector
>>> detector = FaceDetector()
>>> detections = detector.detect(image)
>>> exit()
```

#### 保存輸出供反饋

```bash
# 重定向輸出到文件
python3 scripts/benchmark.py > benchmark_result.log 2>&1

# 複製輸出
cat benchmark_result.log | xclip -selection clipboard

# 或用 SCP 傳回 Windows
scp benchmark_result.log xiange@192.168.1.125:~/  # (結果保存)
scp ~/benchmark_result.log $(your_windows_path)/  # (複製回 Windows)
```

---

### Phase 5️⃣：結果反饋與迭代

#### 反饋信息應包含

```markdown
# 測試報告 - [日期]

## ✅ 成功項目
- [ ] 模塊 A 加載正常
- [ ] 模塊 B 性能達標

## ⚠️ 警告/問題
- 內存峰值: 1.8GB (目標: <2GB)
- 推理延遲: 250ms (目標: <200ms)

## ❌ 錯誤信息
```
ModuleNotFoundError: No module named 'xyz'
Traceback: ...
```

## 📊 性能數據
- CPU 使用: 65%
- FPS: 8.5 (目標: 10)
- 溫度: 62°C

## 🔍 詳細日誌
[看上面的 benchmark_result.log]

## 💡 觀察
- 攝像頭幀延遲明顯
- 某某函數調用過頻
```

#### 迴圈回到 Phase 1

```
您的反饋 → Claude 分析 → 修改代碼 → 傳輸測試 → ...
```

---

## 🛠️ 常用命令快速參考

### Windows PowerShell

```powershell
# 連接樹莫派（快速查看日誌）
ssh xiange@192.168.1.125 "cd ~/ai_bank_robot && python3 scripts/benchmark.py 2>&1"

# 同時傳輸和執行
scp -r . xiange@192.168.1.125:~/ai_bank_robot/ && ssh xiange@192.168.1.125 "cd ~/ai_bank_robot && python3 scripts/benchmark.py"

# 監控實時性能
ssh xiange@192.168.1.125 "watch -n 1 'free -h && echo && df -h && echo && vcgencmd measure_temp'"

# 查看最近修改
ssh xiange@192.168.1.125 "cd ~/ai_bank_robot && git log --oneline -10"
```

### 樹莫派 Bash

```bash
# 快速環境檢查
source ~/ai_bank_robot_env/bin/activate && cd ~/ai_bank_robot && python3 -c "from vision_ai import InferenceEngine; print(f'✓ Ready ({InferenceEngine().platform})')"

# 兩秒內查看資源
(while true; do clear; date; free -h; df -h | head -3; vcgencmd measure_temp; sleep 2; done)

# 提交代碼到 Git
cd ~/ai_bank_robot && git add -A && git commit -m "Update: [description]"

# 清理快取
rm -rf __pycache__ **/__pycache__ *.pyc

# 更新虛擬環境
source ~/ai_bank_robot_env/bin/activate && pip install --upgrade -q pyyaml mediapipe
```

---

## 📊 工作流程總結表

| 階段 | 誰負責 | 工具 | 輸出 |
|-----|-------|------|------|
| 1️⃣ 需求分析 | 您 | 文字描述 | 問題陳述 |
| 2️⃣ 代碼生成 | Claude | Read/Write/Edit | .py 文件 |
| 3️⃣ 傳輸代碼 | 您 | scp | 代碼到樹莫派 |
| 4️⃣ 遠程測試 | 您 | SSH | 控制台輸出 |
| 5️⃣ 反饋結果 | 您 | 複製粘貼 | 性能數據 |

---

## 💡 最佳實踐

### DO ✅

```markdown
✅ 定期提交代碼到 Git
   git add . && git commit -m "Clear commit message"

✅ 保存 benchmark 結果用於對比
   python3 scripts/benchmark.py > results/$(date +%Y%m%d_%H%M%S).log

✅ 在修改前備份
   cp vision_ai/face_detector.py vision_ai/face_detector.py.bak

✅ 使用描述性的錯誤報告
   "推理延遲 250ms，超過目標 200ms，CPU 使用 75%"

✅ 一次只改一件事
   不要在同一個 commit 中混合多個無關的修改

✅ 定期檢查磁盤空間
   df -h  # 確保始終有 >5GB 可用
```

### DON'T ❌

```markdown
❌ 不要直接編輯樹莫派上的文件後又用舊版本 scp 覆蓋
   總是從 Windows 傳輸最新版本

❌ 不要忘記激活虛擬環境
   source ~/ai_bank_robot_env/bin/activate

❌ 不要跳過測試直接提交
   總是運行 python3 scripts/benchmark.py 驗證

❌ 不要在樹莫派上使用 pip install 而不記錄
   任何新依賴都應加入 setup_rpi4_dev.sh

❌ 不要混合不同平台的代碼（RPi4 vs Jetson）
   始終檢查 config/inference_config.yaml 中的 platform 設置
```

---

## 🔍 故障排除

### 常見問題

#### 問題 1：SCP 傳輸太慢

```bash
# 解決方案 1：只傳輸必要文件
scp vision_ai/*.py xiange@192.168.1.125:~/ai_bank_robot/vision_ai/

# 解決方案 2：檢查網絡連接
ping 192.168.1.125
iperf3 -c 192.168.1.125  # 測速（如果已安裝）

# 解決方案 3：使用 rsync（更快）
rsync -avz --delete . xiange@192.168.1.125:~/ai_bank_robot/
```

#### 問題 2：樹莫派上找不到新文件

```bash
# 檢查文件是否傳輸
ssh xiange@192.168.1.125 "ls -la ~/ai_bank_robot/vision_ai/ | head -10"

# 檢查文件時間戳
ssh xiange@192.168.1.125 "stat ~/ai_bank_robot/vision_ai/face_detector.py"

# 如果文件存在但 Python 找不到，清除快取
ssh xiange@192.168.1.125 "find ~/ai_bank_robot -name '__pycache__' -exec rm -rf {} +"
```

#### 問題 3：ModuleNotFoundError

```bash
# 檢查虛擬環境是否激活
echo $VIRTUAL_ENV

# 檢查已安裝的套件
pip list | grep modulename

# 重新安裝依賴
pip install --no-cache-dir pyyaml mediapipe -q
```

#### 問題 4：內存不足

```bash
# 檢查可用內存
free -h

# 清理快取
sudo apt-get clean
rm -rf ~/.cache/pip/*

# 檢查是否有洩漏
top -b -n 1 | head -20
```

---

## 📈 改進建議

### 可選：自動化傳輸

```python
# deploy.py - 自動傳輸和測試
import subprocess
import datetime

def deploy_and_test():
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    # 傳輸
    print(f"📤 傳輸代碼... ({timestamp})")
    subprocess.run(["scp", "-r", ".", "xiange@192.168.1.125:~/ai_bank_robot/"])

    # 測試
    print("🧪 運行測試...")
    result = subprocess.run([
        "ssh", "xiange@192.168.1.125",
        "cd ~/ai_bank_robot && source ~/ai_bank_robot_env/bin/activate && python3 scripts/benchmark.py"
    ], capture_output=True, text=True)

    # 保存結果
    with open(f"test_results_{timestamp}.log", "w") as f:
        f.write(result.stdout)
        f.write(result.stderr)

    print(f"✅ 結果保存到 test_results_{timestamp}.log")

if __name__ == "__main__":
    deploy_and_test()
```

使用：
```bash
cd C:\Users\陳佳憲\Desktop\專題
python3 deploy.py
```

---

## 📝 開發日誌範本

```markdown
# 開發日誌 - [日期]

## 🎯 目標
- [ ] 實現 XXX 功能
- [ ] 修復 YYY bug
- [ ] 優化 ZZZ 性能

## 📝 步驟
1. Claude 分析現有代碼 (30 分鐘)
2. 修改 vision_ai/xxx.py (60 分鐘)
3. 傳輸到樹莫派 (5 分鐘)
4. 樹莫派上運行測試 (15 分鐘)
5. 分析結果，反饋優化點 (20 分鐘)

## ✅ 完成項目
- vision_ai/face_detector.py 修復
- 性能提升 15%

## ⚠️ 待改進
- 還需要測試 XXX 場景
- 需要下載 YYY 模型

## 📊 性能數據
| 指標 | 前 | 後 | 優化 |
|-----|----|----|------|
| 推理延遲 | 250ms | 220ms | -12% |
| 內存 | 1.8GB | 1.6GB | -11% |
| FPS | 8.5 | 9.2 | +8% |

## 🔗 相關 Commit
git commit: abc1234567
```

---

## 总结

```
┌─────────────┐
│  您在 Windows  │ ← 描述問題、反饋結果
└──────┬──────┘
       │
    communicate
       │
       ↓
┌──────────────────────┐
│  Claude 修改代碼      │ ← 讀取、分析、寫入
└──────┬───────────────┘
       │
    upload (scp)
       │
       ↓
┌──────────────────────┐
│ 樹莫派上執行測試      │ ← 驗證、性能、日誌
└──────┬───────────────┘
       │
    feedback
       │
       └── 迴圈 ──→
```

**享受高效的 Vibe Coding！🚀**
