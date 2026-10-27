#!/bin/bash
# 部署後修復執行權限 —— ★ 每次從筆電解壓部署包之後都要跑一次
#
# ## 為什麼需要這支
#
# 筆電端打包時 tar 記錄的模式是 -rw-rw-r--（無執行位元），
# 解壓後 maprun 底下所有 .sh 都變成不可執行。
#
# 症狀很難聯想到權限：
#   - 平板按「啟動感測器」沒反應（HMI 用 subprocess 直接執行腳本 -> Permission denied）
#   - 但在終端機用 `bash run_sensors_cc.sh` 卻完全正常
#   -> 「終端機測得過 ≠ 平板按得動」的典型案例
#
# 實際發生過兩次：2026-09-15 修復一次，同日晚間重新部署後又復發。
#
# 用法： ~/maprun/fix_perms_cc.sh
set -u
cd "$(dirname "$0")" || exit 1
n=0
while IFS= read -r f; do
  [ -x "$f" ] || { chmod +x "$f" && n=$((n+1)); }
done < <(find . -maxdepth 2 \( -name '*.sh' -o -name '*_cc.py' \))
echo "已補上執行權限：$n 個檔案"

# 順便檢查 CRLF —— 同一個打包路徑也會把行尾換成 CRLF，症狀是
# 「$'\r': command not found」，一樣不會聯想到部署
bad=0
while IFS= read -r f; do
  grep -q $'\r' "$f" 2>/dev/null && { echo "  ⚠ $f 含 CRLF，請跑 sed -i 's/\r$//' $f"; bad=$((bad+1)); }
done < <(find . -maxdepth 2 \( -name '*.sh' -o -name '*.py' \))
[ "$bad" = 0 ] && echo "CRLF 檢查：乾淨"
