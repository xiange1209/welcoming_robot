#!/bin/bash
# 通用的單一節點啟動器，給 HMI 的「系統開關」用。
#
# 用法： run_node_cc.sh <package> <executable> [log 檔名]
# 例：   run_node_cc.sh smartnav_vision face_embedding
#
# 為什麼要有這支：
# HMI 是 systemd 服務，它的環境沒有 source 過 ROS 與 env.sh，
# 直接 subprocess 執行 `ros2 run ...` 會找不到指令。這支負責補上環境。
#
# 每個節點各自寫一個 log，出問題時才知道是誰在噴什麼。

set +u
source /home/user/maprun/env.sh

CYCLONEDDS_FILE="${CYCLONEDDS_URI#file://}"
if [ -n "$CYCLONEDDS_FILE" ] && [ ! -f "$CYCLONEDDS_FILE" ]; then
  echo "[run_node_cc] 警告：找不到 $CYCLONEDDS_FILE，改用 CycloneDDS 預設設定" >&2
  unset CYCLONEDDS_URI
fi

PKG="$1"
EXE="$2"
LOGNAME="${3:-${PKG}_${EXE}}"

if [ -z "$PKG" ] || [ -z "$EXE" ]; then
  echo "用法: run_node_cc.sh <package> <executable> [log名稱]" >&2
  exit 2
fi

# ★★ 2026-08-14：第 4 個之後的參數原封傳給 ros2 run。★★
#
# 原本這行是寫死的 `ros2 run $PKG $EXE`，**任何參數都傳不進去**。
# 而 HMI 的系統面板走的就是這支腳本 —— 於是「從平板啟動的節點一律吃預設值」。
# 實際後果（8/14 稽核抓到，兩個都是靜默的）：
#   bank_reception  notify_backend 預設 "none" -> Telegram 通報變成只印 log
#   user_auth       recognition_threshold 0.8  -> 8/14 實測會漏掉 3/9
# 這與 ASR 那個坑是同一個病：終端機跑得動的方法，面板完全沒有採用。
# ★ 發表當天沒有鍵盤，面板路徑就是唯一路徑。
exec ros2 run "$PKG" "$EXE" "${@:4}" > "$LOGDIR/${LOGNAME}.log" 2>&1
