#!/bin/bash
# 強制套用雙板環境（不管有沒有 ~/.smartnav/dualboard_role）。給 B 的 systemd 感知服務與手動測試用。
# 一般操作不需要它：標記檔存在時，~/maprun/env.sh 會自己套用雙板環境（見 overlay.sh）。
# 用法：source ~/maprun/dualboard/env_dualboard.sh
# ★ 2026-10-22 改成薄包裝：值全部在 overlay.sh（原本草稿直接寫在這裡，env.sh 又會把它們洗回單板的值）。
source /home/user/maprun/env.sh
source /home/user/maprun/dualboard/overlay.sh
