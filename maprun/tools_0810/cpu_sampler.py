#!/usr/bin/env python3
"""每秒採一次 CPU / 記憶體 / 溫度 / 各行程占用，寫成 CSV。"""
import csv, os, sys, time

DUR = int(sys.argv[1]) if len(sys.argv) > 1 else 300
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/user/cpu_e7.csv"
NCPU = os.cpu_count() or 4

def cpu_times():
    with open("/proc/stat") as f:
        p = f.readline().split()[1:]
    v = [int(x) for x in p]
    idle = v[3] + v[4]
    return sum(v), idle

def temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return None

def mem():
    d = {}
    with open("/proc/meminfo") as f:
        for line in f:
            k, v = line.split(":", 1)
            d[k] = int(v.split()[0])
    used = d["MemTotal"] - d["MemAvailable"]
    return used / 1024.0, d["MemTotal"] / 1024.0

def throttled():
    try:
        return os.popen("vcgencmd get_throttled 2>/dev/null").read().strip()
    except Exception:
        return ""

rows = []
pt, pi = cpu_times()
t0 = time.time()
while time.time() - t0 < DUR:
    time.sleep(1.0)
    nt, ni = cpu_times()
    dt, di = nt - pt, ni - pi
    pct = 100.0 * (1 - di / dt) if dt else 0.0
    pt, pi = nt, ni
    mu, mt = mem()
    la = os.getloadavg()[0]
    rows.append([f"{time.time()-t0:.1f}", f"{pct:.1f}", f"{la:.2f}",
                 f"{temp() or 0:.1f}", f"{mu:.0f}", f"{mt:.0f}", throttled()])

with open(OUT, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "cpu_pct", "load1", "temp_c", "mem_used_mb", "mem_total_mb", "throttled"])
    w.writerows(rows)

c = [float(r[1]) for r in rows]; l = [float(r[2]) for r in rows]
tp = [float(r[3]) for r in rows]; m = [float(r[4]) for r in rows]
print(f"  取樣 {len(rows)} 秒（{NCPU} 核）")
print(f"  CPU   平均 {sum(c)/len(c):5.1f}%   尖峰 {max(c):5.1f}%")
print(f"  負載  平均 {sum(l)/len(l):5.2f}    尖峰 {max(l):5.2f}   （> {NCPU} 就是排隊）")
print(f"  溫度  平均 {sum(tp)/len(tp):5.1f}°C  尖峰 {max(tp):5.1f}°C  （80°C 開始降頻）")
print(f"  記憶  尖峰 {max(m):.0f} / {rows[0][5]} MB")
th = [r[6] for r in rows if r[6] and r[6] != "throttled=0x0"]
print(f"  降頻  {'★ 有發生：' + th[0] if th else '沒有發生'}")
