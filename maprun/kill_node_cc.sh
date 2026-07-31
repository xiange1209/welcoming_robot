#!/bin/bash
# 停掉單一節點，給 HMI 的「系統開關」用。
#
# 用法： kill_node_cc.sh <package> <executable>
# 例：   kill_node_cc.sh smartnav_vision face_embedding
#
# ── 為什麼不比對整條指令列 ────────────────────────────────
#
# 因為「指令列裡有這個字串」不等於「這是那個節點」。
# 這個專案被這件事咬過五次（每次都是 exit 144，shell 自己被殺掉）：
#
#   1~3 次  pkill -f <關鍵字> 匹配到呼叫端自己的 shell
#   第 4 次  改用 pgrep -f <節點名稱>，結果別的終端機在跑
#           `tail .../smartnav_vision_face_embedding.log`，那條也被殺
#   第 5 次  改成比對完整安裝路徑，結果診斷指令
#           `pgrep -f install/smartnav_vision/lib/.../face_embedding`
#           本身就含有那條路徑，還是被殺
#
# 只排除 $$ / $PPID 沒有用：那護得住自己的行程樹，
# 護不住任何「剛好提到這個名字」的無關行程。
#
# ── 正確做法：只看 argv[0] ───────────────────────────────
#
# argv[0] 是**實際被執行的程式**，不是參數。
#   真正的節點      argv[0] = .../install/<pkg>/lib/<pkg>/<exe>
#   ros2 run 包裝   argv[0] 結尾是 ros2，且 argv[1..3] = run <pkg> <exe>
#   提到名字的 shell argv[0] = /bin/bash        <- 不會被誤判
#
# 用 Python 掃 /proc 是因為 cmdline 是 NUL 分隔的，
# 在 shell 裡切乾淨很麻煩、容易再出錯。

PKG="$1"
EXE="$2"

if [ -z "$PKG" ] || [ -z "$EXE" ]; then
    echo "用法: kill_node_cc.sh <package> <executable>" >&2
    exit 2
fi

exec python3 - "$PKG" "$EXE" <<'PYEOF'
import os, signal, sys, time

pkg, exe = sys.argv[1], sys.argv[2]
node_suffix = f"install/{pkg}/lib/{pkg}/{exe}"
me = os.getpid()
parent = os.getppid()


def argv_of(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as fp:
            raw = fp.read()
    except OSError:
        return None
    if not raw:
        return None
    return [a.decode("utf-8", "replace") for a in raw.split(b"\0") if a]


def effective_program(argv):
    """取出「實際被執行的程式」，跳過直譯器。

    ROS 2 的 Python 節點都是 python3 執行腳本，所以 argv[0] 一律是
    /usr/bin/python3，真正的識別在 argv[1]：

        包裝層 ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'run', pkg, exe]
        節點   ['/usr/bin/python3', '.../install/<pkg>/lib/<pkg>/<exe>']

    只看 argv[0] 會把所有 python 行程混為一談；看整條指令列則會把
    「參數裡提到這個名字」的 shell 也算進去（已經因此殺錯五次）。
    """
    if not argv:
        return None, []
    a0 = argv[0]
    base = os.path.basename(a0)
    if base.startswith("python") and len(argv) > 1:
        return argv[1], argv[2:]
    return a0, argv[1:]


def is_target(argv):
    prog, rest = effective_program(argv)
    if not prog:
        return False
    # 直接執行安裝好的節點
    if prog.endswith(node_suffix):
        return True
    # ros2 run <pkg> <exe> 這層包裝
    if os.path.basename(prog) == "ros2" and rest[0:3] == ["run", pkg, exe]:
        return True
    return False


def collect():
    out = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid in (me, parent):
            continue
        if is_target(argv_of(pid)):
            out.append(pid)
    return out


targets = collect()
if not targets:
    print(f"沒有找到執行中的 {pkg}/{exe}")
    sys.exit(0)

for pid in targets:
    try:
        os.kill(pid, signal.SIGTERM)
        print(f"  已送 TERM 給 {pid}")
    except OSError:
        pass

time.sleep(3)
for pid in collect():
    try:
        os.kill(pid, signal.SIGKILL)
        print(f"  強制結束 {pid}")
    except OSError:
        pass

print(f"已停止 {pkg}/{exe}")
PYEOF
