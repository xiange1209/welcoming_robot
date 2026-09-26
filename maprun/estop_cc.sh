#!/bin/bash
# 緊急停車：殺掉所有會發速度指令的行程，並補送零速。
#
# 韌體的指令逾時是 1.0 秒（balance.c:238-240），所以光是殺掉行程，
# 車子還會照最後一則指令跑滿 1 秒（0.15 m/s 下約 15 公分）。
# 這裡在殺掉之後**主動補送零速**，把那 1 秒縮到幾十毫秒。
#
# 用 /proc 比對 argv，不用 pkill -f（這個專案被字串比對誤判過七次）。
source /home/user/maprun/env.sh 2>/dev/null
python3 - <<'PY'
import os, signal
TARGETS = ("steering_trim_cc", "teleop_keyboard_cc", "path_teach_cc",
           "controller_server", "velocity_smoother", "collision_monitor")
def argv(p):
    try: return [a.decode() for a in open(f"/proc/{p}/cmdline","rb").read().split(b"\0") if a]
    except OSError: return None
def eff(a):
    return a[1] if os.path.basename(a[0]).startswith("python") and len(a)>1 else a[0]
killed=[]
for n in os.listdir("/proc"):
    if not n.isdigit(): continue
    a=argv(int(n))
    if not a: continue
    prog=eff(a)
    if os.path.basename(prog) in TARGETS and "/lib/" in prog:
        try: os.kill(int(n), signal.SIGKILL); killed.append(f"{os.path.basename(prog)}({n})")
        except OSError: pass
print("已強制結束:", ", ".join(killed) or "無")
PY
# 補送零速，不要等韌體那 1 秒逾時
timeout 3 ros2 topic pub -r 30 -t 30 /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1
echo "已補送零速指令"
