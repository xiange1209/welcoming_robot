#!/bin/bash
# 切換雙板／單板模式（只管那個標記檔，不動網路、服務與任何既有檔案）。
#   用法：~/maprun/dualboard/dualboard_mode.sh A|B|off|status
#     A、B：寫入 ~/.smartnav/dualboard_role → 之後所有 source ~/maprun/env.sh 的腳本自動吃雙板環境
#     off ：刪掉標記檔 → 回到單板環境（domain 0、只在本機）
#     status：顯示標記檔與「現在 source env.sh 會得到的環境」
#   ★ 改完要重開所有 ROS 單元才會生效（平板先把單元都關掉；A 再 sudo systemctl restart smartnav-hmi；
#     B 再 sudo systemctl restart smartnav-perception）。已經開著的終端機要重開。
ROLE_FILE="${SMARTNAV_ROLE_FILE:-$HOME/.smartnav/dualboard_role}"
show() {
  echo "標記檔：$ROLE_FILE → $( [ -f "$ROLE_FILE" ] && cat "$ROLE_FILE" || echo '（沒有＝單板）' )"
  echo "source ~/maprun/env.sh 之後："
  (set +u; source "${ENV_SH:-/home/user/maprun/env.sh}" >/dev/null 2>&1
   echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE"
   echo "  CYCLONEDDS_URI=$CYCLONEDDS_URI"
   echo "  SMARTNAV_DUALBOARD=${SMARTNAV_DUALBOARD:-（沒設）}  SMARTNAV_BOARD=${SMARTNAV_BOARD:-（沒設）}")
}
case "${1:-}" in
  A|B) mkdir -p "$(dirname "$ROLE_FILE")" && printf '%s\n' "$1" > "$ROLE_FILE" && echo "已切到雙板模式：這塊是 $1"; show
       echo "★ 接著：平板關掉所有單元 → A: sudo systemctl restart smartnav-hmi　B: sudo systemctl restart smartnav-perception" ;;
  off) rm -f "$ROLE_FILE" && echo "已回到單板模式"; show
       echo "★ 接著：平板關掉所有單元 → sudo systemctl restart smartnav-hmi（還要拿掉 HMI 的 drop-in：README 第 5 節）" ;;
  status) show ;;
  *) echo "用法：$0 A|B|off|status"; exit 1 ;;
esac
