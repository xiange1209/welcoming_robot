#!/usr/bin/env python3
"""量各節點的 CPU／記憶體占用，輸出報告用的表格（E7）。

★ 為什麼要自己寫而不是看 `top`
`top` 的第一次取樣是「自行程啟動以來的平均」，不是當下值 —— 直接讀會低估忙碌節點。
這支用兩次 /proc/<pid>/stat 的差值算真實區間占用，並把 ROS 節點的名字解析出來
（ROS 2 的 Python 節點 argv[0] 一律是 /usr/bin/python3，comm 欄看不出是誰）。

★ 這台是 Raspberry Pi 4，**4 核心 = 400%**。單一節點超過 100% 代表它多執行緒。
★ load average 要跟 vmstat 的 `r`（可執行執行緒數）一起看：
  2026-08-14 曾出現 CPU 只有 165% 但 load 37 的情況，那是**執行緒層級的排隊**，
  不是假象 —— vmstat 顯示 20+ 執行緒在等 4 顆核心。

用法：
    python3 cpu_report.py            # 量 5 秒
    python3 cpu_report.py 15         # 量 15 秒
"""
import os
import sys
import time
import glob

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
HZ = os.sysconf("SC_CLK_TCK")
PAGE = os.sysconf("SC_PAGE_SIZE")


def node_name(pid):
    """把 ROS 2 節點的真實身分解出來。"""
    try:
        argv = [a for a in open(f"/proc/{pid}/cmdline", "rb").read().split(b"\0") if a]
        argv = [a.decode(errors="replace") for a in argv]
    except Exception:
        return None
    if not argv:
        return None
    prog = argv[0]
    if os.path.basename(prog).startswith("python") and len(argv) > 1:
        prog = argv[1]
    base = os.path.basename(prog)
    # ros2 run 的包裝層
    if base == "ros2" and len(argv) > 3 and argv[2] == "run":
        return f"{argv[4]}"
    return base


def sample():
    out = {}
    for d in glob.glob("/proc/[0-9]*"):
        pid = d.rsplit("/", 1)[1]
        try:
            parts = open(f"{d}/stat").read().rsplit(") ", 1)[1].split()
            utime, stime = int(parts[11]), int(parts[12])
            rss = int(parts[21]) * PAGE
            out[pid] = (utime + stime, rss)
        except Exception:
            pass
    return out


def main():
    a = sample()
    t0 = time.time()
    time.sleep(DUR)
    b = sample()
    dt = time.time() - t0

    rows = []
    for pid, (tb, rss) in b.items():
        if pid not in a:
            continue
        cpu = (tb - a[pid][0]) / HZ / dt * 100.0
        if cpu < 0.5:
            continue
        name = node_name(pid)
        if not name:
            continue
        rows.append((cpu, rss / 1048576.0, pid, name))
    rows.sort(reverse=True)

    ncpu = os.cpu_count() or 4
    print(f"  量測 {dt:.1f} 秒　核心數 {ncpu}（滿載 = {ncpu*100}%）\n")
    print(f"  {'CPU%':>7} {'RSS MB':>8}  {'PID':>7}  節點")
    print("  " + "-" * 56)
    tot = 0.0
    totmem = 0.0
    for cpu, mem, pid, name in rows[:18]:
        tot += cpu
        totmem += mem
        print(f"  {cpu:7.1f} {mem:8.1f}  {pid:>7}  {name}")
    print("  " + "-" * 56)
    print(f"  {tot:7.1f} {totmem:8.1f}  合計（前 {min(18,len(rows))} 名）"
          f"　= 全機 {100*tot/(ncpu*100):.0f}% 的算力")

    try:
        la = os.getloadavg()
        print(f"\n  load average {la[0]:.2f} / {la[1]:.2f} / {la[2]:.2f}"
              f"　（> {ncpu} 就代表有執行緒在排隊）")
    except Exception:
        pass
    try:
        t = int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000.0
        print(f"  溫度 {t:.1f}°C　（Pi 4 到 80°C 才降頻）")
    except Exception:
        pass
    m = {}
    for line in open("/proc/meminfo"):
        k, v = line.split(":", 1)
        m[k] = int(v.strip().split()[0]) // 1024
    print(f"  記憶體 已用 {m['MemTotal']-m['MemAvailable']} / {m['MemTotal']} MB"
          f"（可用 {m['MemAvailable']} MB）")


if __name__ == "__main__":
    main()
