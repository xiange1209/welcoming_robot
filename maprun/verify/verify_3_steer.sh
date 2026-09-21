#!/usr/bin/env bash
# V3 A-3 舵機「量測」—— 注意是量測，不是校準。
#
# ★★ 期待值已下修（2026-09-18 翻 PWM 表後確認）：
#    右滿舵被 Servo_min 截斷只差 5.9 counts（值 1.1%），
#    但實測轉向能力缺 27~37% —— 差一個數量級。
#    而且電位器只搬中位、不增加行程（總行程 787.7 counts，限幅窗口 870 counts，本來就塞得下）。
#    → 所以「校完舵機就開得過門口」這個期待不成立。量到角度記下來就走，不要耗在這裡。
#
# 做法：把車架高讓前輪離地 → 打滿舵 → 用量角器量前輪實際轉角
#
# 用法：~/maprun/verify/verify_3_steer.sh

source "$(dirname "$0")/_lib.sh"

echo "V3 舵機行程量測（不是校準）   $(date '+%Y-%m-%d %H:%M:%S')"
echo
echo "  韌體理論值：左滿舵 +19.5 度（外輪）／右滿舵 -28.6 度（內輪）"
echo "  兩者換算後的迴轉半徑只差 0.0011 公尺，韌體層是對稱的。"
echo
echo "  ⚠ 前輪必須離地。車子還在地上就打滿舵會傷舵機。"
read -r -p "  前輪已離地？(y/N) " OK
[ "$OK" = y ] || { echo "  中止。"; exit 1; }

hdr "打滿舵並量測"
ask "左滿舵，前輪實際轉角（度，填正值）" L_DEG
ask "右滿舵，前輪實際轉角（度，填正值）" R_DEG

judge() { # judge <左|右> <實測> <理論>
  local side="$1" got="$2" want="$3"
  [ -z "$got" ] && { rec "${side}滿舵轉角" WARN "未填" ""; return; }
  local ratio
  ratio=$(awk -v g="$got" -v w="$want" 'BEGIN{printf "%.0f", 100*g/w}')
  if [ "$(awk -v r="$ratio" 'BEGIN{print (r>=90)?1:0}')" = 1 ]; then
    rec "${side}滿舵轉角" PASS "${got} 度（理論 ${want}，達 ${ratio}%）" "行程足夠"
  else
    rec "${side}滿舵轉角" FAIL "${got} 度（理論 ${want}，只有 ${ratio}%）" \
        "★ 行程不足，旋鈕救不了 —— 真因更可能在連桿幾何或舵機行程末端"
  fi
}
echo
judge 左 "$L_DEG" 19.5
judge 右 "$R_DEG" 28.6

# 順手記下目前的軟體補償值，之後對照用
TRIM="$HOME/.smartnav/steering_trim.json"
[ -f "$TRIM" ] && rec "steering_trim.json" INFO "$(tr -d '\n ' < "$TRIM")" "目前的軟體補償"

hdr "結論"
echo "  量到的角度記下來就好。★ 不要花時間調旋鈕想開過門口 ——"
echo "  缺 27~37% 的行程，而電位器只搬中位不增行程。"
echo "  對策是：錄製路徑時人示範三點轉向，選點位避開最窄處。"

banner_done
