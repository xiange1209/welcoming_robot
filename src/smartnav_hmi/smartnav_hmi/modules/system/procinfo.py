"""從 /proc 讀行程資訊。

★ 全部走 /proc，不開子行程：這些會被狀態頁輪詢，每次 fork 一個 ps
  在 Pi 4 上是不必要的負擔（跟 hardware.py 同一個理由）。

純函式，沒有 ROS 也沒有節點狀態，可以單獨測試。
"""

import os


def local_ros_processes() -> tuple:
    """掃 /proc 取得本機 ROS 行程的完整指令列與執行檔名

    ROS 2 的圖 API 不提供「節點在哪台機器」，只能反過來從本機行程推。
    回傳 (cmdline 字串集合, 執行檔基本名集合)。
    """
    cmdlines, exes = set(), set()
    try:
        pids = os.listdir("/proc")
    except OSError:
        return cmdlines, exes

    for pid in pids:
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as fh:
                parts = [p for p in fh.read().decode(errors="replace").split("\0") if p]
        except OSError:
            continue
        if not parts:
            continue
        joined = " ".join(parts)
        if "/opt/ros/" not in joined and "/install/" not in joined:
            continue
        cmdlines.add(joined)
        for token in parts[:4]:
            if token and not token.startswith("-"):
                base = os.path.basename(token)
                # python3／ros2 這種包裝行程的名字對判定沒有幫助
                if base not in ("python3", "python", "ros2", "sh", "bash"):
                    exes.add(base)
    return cmdlines, exes


def proc_cmdlines() -> list:
    """讀出所有行程的指令列

    用 /proc 而不是 pgrep：pgrep -f 會匹配到「指令列裡含有該關鍵字的
    呼叫端自己」，在這個專案已經造成過三次自殺（exit 144）。
    """
    out = []
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue
        try:
            with open(f"/proc/{entry.name}/cmdline", "rb") as fp:
                cmd = fp.read().replace(b"\0", b" ").decode("utf-8", "replace")
            if cmd.strip():
                out.append((int(entry.name), cmd))
        except (OSError, ValueError):
            continue
    return out
