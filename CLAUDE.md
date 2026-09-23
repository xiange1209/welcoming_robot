# 專案目錄與工作區規範 (Project Directory Rules)

> ⚠️ 最高優先級路徑規則 (CRITICAL PATH RULE):
> 絕對禁止將源碼移至、建立於或引用於任何巢狀工作區目錄（例如 ./welcoming_robot_ws/src/... 或 ./smartnav_ws/src/...）。
> 所有源碼與 Package 必須「直接」放在根目錄下的 ./src/ 內。

---

## 1. 專案目錄結構標準

本專案的根目錄結構必須嚴格遵守以下佈局：

.  
├── src/                  <-- 所有 Package 必須直接放這裡！  
│   ├── package_A/  
│   └── package_B/  
├── README.md  
└── CLAUDE.md  

❌ 嚴禁使用的路徑 (Forbidden Paths)

* ./welcoming_robot_ws/src/...
* ./welcoming_robot_ws/src/smartnav_ws/src/...
* ./smartnav_ws/src/...
* 任何其他多餘的 *_ws/ 子目錄

---

## 2. ROS / ROS2 建置與指令規範

當生成 Bash 腳本、Dockerfile、CMakeLists.txt 或建置指令時：

1. 直接將目前根目錄視為唯一的 Workspace，或直接針對 ./src 進行編譯（例如 colcon build）。
2. 切勿生成會自動建立 welcoming_robot_ws 資料夾或複製 ./src 到子目錄的指令。
3. 請預設使用者已經在專案根目錄下操作。

---

## 3. Git 分支與版本控制嚴格規範 (Git Rules)

> ⚠️ 最高優先級 Git 規則:
>
> 1. 嚴禁刪除任何分支！尤其是 wheeltec 分支。
> 2. 嚴禁擅自將 wheeltec 分支合併 (Merge) 或 Rebase 到 master / main 分支。
> 3. 嚴禁執行任何包含 git branch -d、git branch -D 或 git push origin --delete 的指令。
> 4. 嚴禁使用中文命名 Git 分支！所有新建立的分支名稱必須完全使用全英文、數字、斜線 (/) 或連字號 (-)，例如：feature/navigation 或 fix/laser-scan。
>
>

當處理 Git 操作時：

1. 除非使用者明確指示「請幫我合併到 master」，否則絕對不可以執行切換到 master 並進行 merge 的操作。
2. 若需要建立新分支，請務必確認分支名稱完全沒有任何中文字元，且必須使用半形英文命名。
3. 若需要提供 Git 指令，僅提供 git commit 或 git push origin <當前分支>，切勿加上刪除或跨分支合併的指令。

---

## 4. AI 輸出前自檢清單 (Checklist)

每次生成程式碼、檔案路徑或 Bash 指令前，請確認：

* [ ] 所有原始碼路徑是否都以 ./src/ 開頭？
* [ ] 是否完全沒有包含任何 *_ws/src/ 前綴？
* [ ] 我剛才生成的指令或分支名稱中，是否有包含中文字元？
* [ ] 我剛才生成的指令中，是否有包含刪除分支 (-d/-D) 或合併至 master 的操作？
