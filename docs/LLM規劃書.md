# 智慧銀行 LLM 規劃書

更新日期：2026-07-02

本文件整理目前 repo 內 Qwen2.5、Gemma 3、Gemma 4 的定位、實測延遲結果、現有程式碼支援狀態與後續測試建議。重要原則只有一個：區分已驗證、已配置但未驗證、以及純規劃中的內容，避免把估計值誤當成實測值。

## 狀態標記

- 已配置：repo 內已有對應 Modelfile 或程式碼支援路徑。
- 待實測：尚未在這個 workspace 直接留下可重現的紀錄。
- 已驗證：有可重現的命令、模型版本、硬體與結果紀錄。目前「首 token 延遲」與「完整回應延遲」已於 2026-07-02 達到此標準（見「2026-07-02 實測結果」章節），其餘指標仍待測。

## 模型比較總表

延遲數據為 2026-07-02 於 Windows 筆電（RTX 3050 Laptop 4GB VRAM、Ollama 0.24.0）的實測平均值，詳細方法見「2026-07-02 實測結果」章節。

| 模型（Ollama tag） | 對應 Modelfile | 模型檔大小 | 首 token 平均（ms） | 總生成平均（ms） | RTX 3050 4GB 適配性（實測結論） | 中文能力 | thinking 支援 | 目前狀態 |
|------|----------|-----------|--------------------|------------------|--------------------------------|----------|---------------|----------|
| qwen2.5:3b | Modelfile_qwen | 約 1.9 GB | 2218 | 3076 | 可完整放入 VRAM，全部模型中最快 | 強 | 否 | 延遲已實測，建議主模型 |
| gemma3:4b（gemma3-bank） | Modelfile_gemma3 | 約 4 GB | 2303 | 4700 | 可放入 VRAM，速度次快 | 中到高 | 否 | 延遲已實測，中文備選 |
| gemma4:e2b（gemma4e2b-bank） | Modelfile_gemma4e2b | 約 7.2 GB | 42235 | 47909 | 超出 VRAM，Ollama 卸載到 CPU，慢約 20~40 倍，不適合即時互動 | 中到高 | 是 | 延遲已實測，此硬體不採用 |
| gemma4:e4b（gemma4e4b-bank） | Modelfile_gemma4e4b | 約 9.6 GB | 95066 | 107123 | 超出 VRAM，Ollama 卸載到 CPU，不適合即時互動 | 高 | 是 | 延遲已實測，此硬體不採用 |

> 表中「中文能力」「thinking 支援」欄位仍屬模型規格與使用經驗整理，不是本 repo 內已完成的品質測試結果；延遲欄位則為實測值。

## 2026-07-02 實測結果

### 測試環境與方法

- 硬體：Windows 筆電，RTX 3050 Laptop GPU（4GB VRAM）
- 軟體：Ollama 0.24.0
- 腳本：`scripts/benchmark_llm_models.py`
- 流程：每個模型 1 次暖機 + 5 次正式測量，透過串流 `/api/chat` 計時
- 提示語：銀行場景 system prompt，temperature 0.1，num_ctx 512
- 原始明細：`benchmark_llm_results.json`（每次 run 的 first_token_ms / total_ms / 成功與否）

### 結果總表

| 模型（Ollama tag） | 基底模型 | 首次輸出平均（ms） | 總生成平均（ms） | 最快（ms） | 最慢（ms） | 成功率 |
|------|------|------|------|------|------|--------|
| gemma3-bank | gemma3:4b | 2303 | 4700 | 4583 | 4755 | 100% |
| gemma4e2b-bank | gemma4:e2b | 42235 | 47909 | 44838 | 50206 | 100% |
| gemma4e4b-bank | gemma4:e4b | 95066 | 107123 | 100231 | 123839 | 100% |
| qwen2.5:3b | qwen2.5:3b | 2218 | 3076 | 3017 | 3196 | 100% |

### 關鍵解讀

- gemma4 系列模型檔 7.2 GB（e2b）與 9.6 GB（e4b）遠超出 4GB VRAM，Ollama 自動卸載到 CPU 推理，導致延遲比放得進 VRAM 的模型慢 20~40 倍。
- gemma3:4b（約 4 GB）與 qwen2.5:3b（約 1.9 GB）能塞進 VRAM，延遲落在可即時互動的範圍。
- 在此硬體上：qwen2.5:3b 最快（TTFT 約 2.2 秒、總時長約 3.1 秒），gemma3:4b 次之（總時長約 4.7 秒），gemma4 系列不適合即時互動。
- 所有模型 5 次正式測量成功率均為 100%，數據穩定（最快與最慢差距小）。

## 為什麼建議主模型改用 qwen2.5:3b

- 實測 TTFT 約 2.2 秒、總生成約 3.1 秒，是四個模型中最快的，最接近即時對話體驗。
- 模型檔約 1.9 GB，在 4GB VRAM 下留給系統與其他程序的空間最充足。
- 中文理解與短指令輸出是目前場景最需要的能力，而不是長篇生成。
- 若要給組員最快速重現，qwen2.5:3b 的失敗面最小。

注意：`smartnav_llm` 的 `model_name` 參數目前預設值是 `gemma4:e2b`（沿用同學 repo 的設定）。依本次實測，建議啟動時以參數覆寫為 `qwen2.5:3b`，或直接修改預設值。

## 為什麼還保留 Gemma 系列

- gemma3:4b：實測可放入 VRAM 且延遲可接受（總時長約 4.7 秒），保留為中文品質比較的備選模型。
- gemma4:e2b / e4b：支援 thinking，但在 RTX 3050 4GB 上實測會 CPU 卸載、延遲數十秒，此硬體上不採用。Modelfile 保留，供未來換到更大 VRAM 的機器時直接使用。

## 目前程式碼實際支援的點（smartnav_llm ROS 2 節點）

LLM 已改為 ROS 2 節點架構（`src/smartnav_llm/`），舊的 FastAPI server（src/llm_server/）已刪除。

- `src/smartnav_llm/smartnav_llm/llm_service_node.py`：LangChain + Ollama（`ChatOllama`）的對話式 Agent 節點。
- 訂閱 `/user_text`（std_msgs/String）作為使用者輸入；回應發布到 `llm_response`，逐 token 串流發布到 `llm_stream`，斷句後的語音文字發布到 `speech_text`。
- 參數：`ollama_base_url`（預設 `http://localhost:11434`）、`model_name`（預設 `gemma4:e2b`）、`temperature`（預設 0.0）。
- 雙機部署：RPi4 上啟動時以 `ollama_base_url:=http://<筆電IP>:11434` 指向筆電的 Ollama，並可同時覆寫模型：

  ```bash
  ros2 launch smartnav_bringup smartnav.launch.py \
    ollama_base_url:=http://192.168.1.xxx:11434 \
    model_name:=qwen2.5:3b
  ```

- 工具集全為導航導向：`create_map` / `list_maps` / `switch_map` / `create_waypoint` / `list_waypoints` / `navigate` / `global_localization`（透過 smartnav_msgs 的服務與動作呼叫 smartnav_navigation）。
- ReAct 多步驟決策迴圈（上限 10 步）、對話記憶保留最近 2 組對話。
- system prompt 讀自 `config/system_prompt.txt`，內容為導航機器人設定。

已知限制（誠實記錄）：

- 尚無銀行場景工具（例如「帶 VIP 到貴賓室」「通報行員處理黑名單」），人臉辨識事件經 `recognition_text_bridge` 轉成文字進入 `/user_text` 後，只會被當成一般對話處理。
- 整個 ROS 2 workspace（含 smartnav_llm 節點與 vision→LLM 橋接）尚未在 RPi4 實機 colcon build 與運行，屬「已實作、待實機驗證」。

## 建議建立的 Ollama 模型

### qwen2.5:3b

```bash
ollama pull qwen2.5:3b
ollama create qwen-bank -f Modelfile_qwen
```

### gemma3:4b

```bash
ollama pull gemma3:4b
ollama create gemma3-bank -f Modelfile_gemma3
```

### gemma4:e2b

```bash
ollama pull gemma4:e2b
ollama create gemma4e2b-bank -f Modelfile_gemma4e2b
```

### gemma4:e4b

```bash
ollama pull gemma4:e4b
ollama create gemma4e4b-bank -f Modelfile_gemma4e4b
```

> 註：2026-07-02 的基準測試中，Gemma 系列測的是上述 `-bank` 衍生模型，Qwen 測的是原始 `qwen2.5:3b` tag。

## 建議的實測欄位

正式比較模型時，至少要有下列數據；未完成的項目不應在簡報或 README 中寫成效能結論：

| 指標 | 說明 | 狀態 |
|------|------|------|
| 首 token 延遲 | 從送出請求到第一段有效內容出現 | 已完成（2026-07-02，`scripts/benchmark_llm_models.py`） |
| 完整回應延遲 | 銀行場景 prompt 下的總生成耗時 | 已完成（2026-07-02，`scripts/benchmark_llm_models.py`） |
| 工具呼叫 / JSON 成功率 | 連跑多次，LangChain Agent 能正確產生並解析 tool call 的比例 | 待測 |
| 中文回覆品質 | 是否保持繁中、是否符合銀行場景 | 待測 |
| GPU / CPU 占用 | 是否發生 CPU 卸載或卡頓 | 待測（本次僅由模型檔大小與延遲差距間接推斷 gemma4 發生 CPU 卸載，未留下占用率紀錄） |

## 建議的測試順序（後續）

延遲基準已完成，剩餘測試建議順序：

1. 先在 qwen2.5:3b 上測 LangChain 工具呼叫成功率與中文回覆品質，取得基準線。
2. 再用 gemma3:4b 對照中文品質，決定是否值得多花約 1.6 秒的總生成時間。
3. gemma4 系列不在此硬體上續測；若未來取得更大 VRAM 的機器，再回頭驗證 thinking 能力。
4. 補上 GPU / CPU 占用率的量測紀錄（例如 `nvidia-smi` 與工作管理員截圖），把「CPU 卸載」從推斷升級為實測。

## 目前結論

- 即時互動場景選 qwen2.5:3b：實測 TTFT 約 2.2 秒、總生成約 3.1 秒，是此硬體上唯一接近即時體驗的選項。
- gemma3:4b 為中文備選：可放入 VRAM，總生成約 4.7 秒，若後續中文品質測試明顯優於 qwen2.5:3b 再考慮切換。
- gemma4:e2b / e4b 在此硬體（RTX 3050 4GB）不採用：模型檔超出 VRAM 導致 CPU 卸載，延遲 48~107 秒，無法用於即時互動。
- 若要使用 gemma4:e2b 的 thinking 能力，需換更大 VRAM 的機器再驗證。
- `smartnav_llm` 的 `model_name` 預設仍是 `gemma4:e2b`，部署時務必以 launch 參數覆寫為 `qwen2.5:3b`（或修改預設值）。
- 工具呼叫成功率、中文品質、GPU 占用尚未實測，不應寫成既有結論。
