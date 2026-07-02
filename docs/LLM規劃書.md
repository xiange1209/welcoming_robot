# 智慧銀行 LLM 規劃書

更新日期：2026-05-26

本文件整理目前 repo 內 Qwen2.5、Gemma 3、Gemma 4 的定位、估計資源需求、現有支援狀態與建議測試順序。重要原則只有一個：區分已驗證、已配置但未驗證、以及純規劃中的內容，避免把估計值誤當成實測值。

## 狀態標記

- 已配置：repo 內已有對應 Modelfile 或程式碼支援路徑。
- 待實測：尚未在這個 workspace 直接留下可重現的延遲、tok/s、JSON 成功率紀錄。
- 已驗證：需在後續測試報告中補上命令、模型版本、硬體與結果；目前尚未達到這個標準。

## 模型比較總表

| 模型 | 對應檔案 | 估計模型/顯存需求 | RTX 3050 4GB 適配性 | 中文能力 | JSON 結構輸出 | thinking 支援 | 目前狀態 |
|------|----------|-------------------|---------------------|----------|---------------|---------------|----------|
| qwen2.5:3b | Modelfile_qwen | 約 1.9 GB 模型，約 2.2 GB GPU 佔用 | 最穩定 | 強 | 高 | 否 | 預設主模型，已配置，待端對端實測 |
| gemma3:4b | Modelfile_gemma3 | 約 3-4 GB 級別 | 可嘗試，但可能接近上限 | 中到高 | 高 | 否 | 已補配置，待實測 |
| gemma4:e2b | Modelfile_gemma4e2b | 約 1.5-2.0 GB 級別 | 適合 | 中到高 | 高 | 是 | 已配置，待實測 |
| gemma4:e4b | Modelfile_gemma4e4b | 約 2.5-3.5 GB 級別 | 可跑但接近上限 | 高 | 高 | 是 | 已配置，待實測 |

> 上表中的大小與顯存數字屬於目前規劃與使用經驗整理值，不是本 repo 內已完成的基準測試結果。

## 為什麼目前先用 qwen2.5:3b

- 它已經是 src/llm_server/main.py 的預設模型。
- 在 4GB VRAM 級距下，留給 FastAPI 與系統本身的空間比較充足。
- 中文理解與短 JSON 指令輸出是目前最需要的能力，而不是長篇生成。
- 若要給組員最快速重現，qwen2.5:3b 的失敗面最小。

## 為什麼還保留 Gemma 系列

- gemma3:4b 適合拿來比較中文理解與 JSON 約束的穩定度。
- gemma4:e2b / e4b 已被目前 llm_client.py 視為支援 thinking 串流的模型族群，可搭配聊天頁觀察推理過程。
- 若後續要展示模型切換能力，Gemma 系列比從零再建新設定快很多。

## 目前程式碼實際支援的點

- src/llm_server/main.py 可透過 --model 切換模型。
- src/llm_server/llm_client.py 在模型名稱包含 gemma4 時會開啟 think 參數。
- src/llm_server/static/chat.html 可透過 SSE 看 thinking / content 串流。
- FALLBACK 機制已存在，即使 Ollama 離線也能回傳基本規則式回應。

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

## 建議的實測欄位

正式比較模型時，至少補這五個數據，否則不要在簡報或 README 中宣稱是效能結論：

| 指標 | 說明 |
|------|------|
| 首 token 延遲 | 從送出請求到第一段有效內容出現 |
| 完整回應延遲 | 以 30-50 字 reply 的總耗時為準 |
| JSON 解析成功率 | 連跑 10 次，能被 parse_json_response 正確解析的比例 |
| 中文回覆品質 | 是否保持繁中、是否符合銀行場景 |
| GPU / CPU 占用 | 是否發生明顯 CPU 卸載或卡頓 |

## 建議的測試順序

1. 先測 qwen2.5:3b，取得基準線。
2. 再測 gemma4:e2b，確認 thinking 模式是否值得保留。
3. 再測 gemma3:4b，看中文與 JSON 是否有優勢。
4. 最後才測 gemma4:e4b，避免一開始就把時間浪費在接近 VRAM 上限的模型。

## 目前結論

- 若目標是讓組員最快重現並做展示，先用 qwen2.5:3b。
- 若目標是展示 thinking 與模型切換，再補測 gemma4:e2b。
- gemma3:4b 與 gemma4:e4b 目前都還是待驗證選項，不應在 README 裡寫成既有實測結論。
