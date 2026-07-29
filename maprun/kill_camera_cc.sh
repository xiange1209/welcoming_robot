#!/bin/bash
# 安全地停掉相機相關行程。
#
# 為什麼要獨立成一支腳本：
# `pgrep -f astra_camera_node` 會連「含有這串字的那條 ros2 launch 指令」一起匹配到，
# 而如果我是在互動 shell 裡直接打 pkill，那條 shell 自己的指令列也含有這串字 ——
# 結果就是把自己的 shell 殺掉（實測 exit 144，已經發生三次）。
#
# 寫成獨立腳本後，pattern 只出現在這個檔案裡、不會出現在呼叫端的 cmdline；
# 再加上排除自己與父行程，就不會自殺。

SELF=$$
PARENT=$PPID

kill_pattern() {
    local pat="$1"
    local sig="$2"
    for pid in $(pgrep -f "$pat" 2>/dev/null); do
        # 排除自己、父行程、以及 pgrep 自身
        [ "$pid" = "$SELF" ] && continue
        [ "$pid" = "$PARENT" ] && continue
        kill "$sig" "$pid" 2>/dev/null && echo "  已送 $sig 給 $pid"
    done
}

echo "停止相機..."
kill_pattern "depth_obstacle_cc" -TERM
kill_pattern "astra_camera_node" -TERM
kill_pattern "camera_obstacle_cc.launch" -TERM
kill_pattern "astra.launch.xml" -TERM
sleep 4
kill_pattern "depth_obstacle_cc" -KILL
kill_pattern "astra_camera_node" -KILL
sleep 1

if pgrep -f astra_camera_node >/dev/null 2>&1; then
    echo "警告：仍有相機行程存活"
    exit 1
fi
echo "相機已完全停止"
exit 0
