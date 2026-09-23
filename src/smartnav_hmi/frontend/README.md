# SmartNav HMI 前端

平板上的操作介面。React + TypeScript + Tailwind CSS v4，由 Vite 打包。

## 建置

```bash
cd frontend
npm install
npm run build        # tsc -b && vite build
```

產物留在 Vite 預設的 `dist/`。`setup.py` 會把 `frontend/dist/**` 原樣裝進 share，
所以車上的路徑是 `share/smartnav_hmi/frontend/dist`：

| 產物                    | 後端怎麼提供                                  |
| ----------------------- | --------------------------------------------- |
| `frontend/dist/index.html` | `GET /`（帶 `Cache-Control: no-store`）    |
| `frontend/dist/assets/*`   | mount 在 `/assets`（檔名帶雜湊，可放心快取） |

`base` 與 `assetsDir` 都維持 Vite 預設，產出的網址剛好是 `/assets/<檔名>`，
跟後端 mount 的路徑對得起來，不需要任何特例。

改完之後要讓車上吃到新版：

```bash
npm run build                                   # 先產出 dist/
colcon build --packages-select smartnav_hmi     # 再裝進 share
```

**順序不能反。** `setup.py` 的檔案清單是在它執行的當下用 glob 決定的，
先 colcon 後 npm 會裝到上一版，而畫面看起來完全正常——只是改的東西沒生效。

> **用 `--symlink-install` 也一樣要重跑 colcon。**
> 符號連結讓 `index.html` 直接指向 `src/` 裡的即時檔案，看起來「npm build 就夠了」，
> 但 `assets/` 底下的檔名帶內容雜湊，每次建置都是新名字。舊的連結指向不存在的
> 舊雜湊、新檔案則還沒被連進 share——結果是 index.html 是新的、它要的 JS 卻 404，
> 畫面一片空白。

> `public/` 目錄刻意不用：它的內容會被複製到 `dist/` 根目錄，而後端只提供
> `/` 和 `/assets`，放在那裡的檔案一律 404。要跟著出貨的靜態資源請放
> `src/assets/` 並用相對路徑引用，讓它走資源管線進 `assets/`。

## 開發

```bash
npm run dev
```

`vite.config.ts` 已經把 `/api`、`/video`、`/ws` 轉給 `127.0.0.1:8080`。要對著真的
機器人改版面，把那三個 target 換成車上的 IP 即可（影像與狀態推播都會跟著過去）。

## 管理者分頁

八個分頁裡有四個要管理者登入才能進去：**地圖與導航、遙控建圖、系統開關、使用者管理**。
未登入時它們仍然顯示在導覽列上，只是標成鎖頭並且點下去會直接開登入框——
舊版是把它們整個 `display:none`，結果是功能看起來憑空消失了。

帳號與密碼由 `hmi_server_node` 的 `admin_username` / `admin_password` 參數決定，
預設值與覆寫方式見 `config/.env` 與該節點啟動時印出的提示。

要恢復成「未登入完全看不到」：把 `App.tsx` 裡的 `const tabs = TABS` 換成
`visibleTabs(admin)`（`lib/tabs.ts` 已經有這支）。

## 架構

```text
src/
  lib/        不依賴 React 的那一半：狀態容器、WebSocket、遙控、朗讀、地圖繪製
  hooks/      訂閱與計時器的共用邏輯
  components/ 共用 UI 與長駐的框架（頂部狀態列、底部導覽列、對話框）
  pages/      八個分頁，一頁一個檔
  styles/     Tailwind 進入點與設計語彙（顏色、圓角、動畫曲線）
```

### 高頻資料流

後端以 **10 Hz** 推播整份狀態快照（身分、電壓、車速、位姿、對話、作業…）。
天真的作法是把它放進 Context，但那樣每秒會把整棵樹重建十次，Pi 4 接的平板直接掉幀。
這裡的作法是：

1. **外部 store + selector 訂閱**（`lib/store.ts`、`hooks/useStore.ts`）
   每個元件只訂閱自己要的那一小塊，值沒變就不重繪。所以電壓變了不會連帶重繪對話，
   車速每秒跳十次也只有那一顆數字在動。
2. **影格合併**（`createStore` 的 `coalesce`）
   同一個動畫影格內的多次推播只觸發一次重繪；分頁被隱藏時 rAF 不跑，
   於是**一次都不重繪**（`get()` 永遠是最新值，回到前景畫面不會是舊的）。
3. **畫布完全繞過 React**（`components/MapCanvas.tsx`）
   地圖與車體位姿直接訂閱 store，用 rAF 重畫。
4. **對話只在真的變動時重建**
   後端在對話沒變時不帶 `messages` 欄位，前端沿用**同一個陣列參考**，
   所以 LLM 逐字串流的整段期間，已完成的訊息一則都不重繪，動的只有那顆 ghost 泡泡。
5. **遙控迴圈不進 React**（`lib/teleop.ts`）
   方向鍵是 10 Hz 的送出迴圈，走 state 會讓「放開」慢一拍——那一拍是車子多跑的距離。

### 按需開啟的資源

MJPEG 串流、地圖心跳、系統狀態輪詢都只在「正在看的那一頁而且分頁沒被隱藏」時才開。
螢幕關掉或切到別的 App 時 **不會** 觸發切頁事件，只有 `visibilitychange` 會——
漏掉這一道，後端就會持續搬運影格給沒有人在看的螢幕（實測影像那條約佔 22% CPU）。
