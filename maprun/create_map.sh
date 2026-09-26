#!/bin/bash
# ★ 舊版建圖入口，已停用（2026-08-25 加上護欄）。原檔在 舊版_勿用/create_map.sh。
#
# 這支特別危險：它與 create_map_cc.sh 只差三個字元，而 `create_map<TAB>`
# 補出來的是**這一支**（舊的排在前面）。舊版沒有帶 _cc 的參數，
# 建出來的圖給現行 _cc 鏈用會對不上。

cat >&2 <<'EOF'

  ✗ create_map.sh 已停用 —— 這是舊版 smartnav_navigation 的建圖入口。

    它沒有帶 _cc 的參數，建出來的圖與現行導航鏈對不上。
    ★ 注意 `create_map<TAB>` 會先補到這一支，不是你要的那支。

    現在該用的是：
      ~/maprun/create_map_cc.sh

    真的要跑舊版：
      bash ~/maprun/舊版_勿用/create_map.sh

EOF
exit 1
