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

# ★★ 2026-08-24：補上 republish 與 camera_face_only.launch ★★
#
# 在此之前這支只殺 astra_camera_node，**沒有殺 republish**（raw -> compressed），
# 而 camera_face_only.launch.py 一定會起一個。實測後果：
#     PID 4760  存活 2870 秒（48 分鐘）  CPU 17.9%
# 也就是說這支腳本印出「相機已完全停止」的同時，有一個孤兒 republish
# 在燒掉將近五分之一顆核心，而且會一直燒到重開機。
# ★ 最後的存活檢查也只看 astra_camera_node，所以它**回報成功**。
#
# camera_manager_cc_node 用的是 `os.killpg` 對整個 process group 送訊號，
# 它的註解就寫著「只 kill launch 的話底下的 astra_camera_node 與 republish
# 會變成孤兒繼續吃 CPU」—— 那正是這支腳本的毛病。
#
# 用 `image_transport/republish`（執行檔的完整路徑片段）而不是單純 `republish`：
# 後者太籠統，會匹配到任何提到這個字的東西。
echo "停止相機..."
kill_pattern "depth_obstacle_cc" -TERM
kill_pattern "astra_camera_node" -TERM
kill_pattern "image_transport/republish" -TERM
kill_pattern "camera_obstacle_cc.launch" -TERM
kill_pattern "camera_face_only.launch" -TERM
kill_pattern "astra.launch.xml" -TERM
sleep 4
kill_pattern "depth_obstacle_cc" -KILL
kill_pattern "astra_camera_node" -KILL
kill_pattern "image_transport/republish" -KILL
sleep 1

LEFT=0
pgrep -f astra_camera_node >/dev/null 2>&1 && LEFT=1
pgrep -f "image_transport/republish" >/dev/null 2>&1 && LEFT=1
if [ "$LEFT" = "1" ]; then
    echo "警告：仍有相機行程存活"
    pgrep -af "astra_camera_node|image_transport/republish" 2>/dev/null
    exit 1
fi
echo "相機已完全停止（含 republish）"
exit 0
