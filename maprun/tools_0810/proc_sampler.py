#!/usr/bin/env python3
"""逐行程 CPU 採樣（E7）：回答「哪些該留在 Pi、哪些該搬到筆電」。

★ 不用 pgrep/pkill —— 直接讀 /proc/<pid>/cmdline 比對，
  這個專案禁止 pkill -f / pgrep -f（會誤殺呼叫端自己的 shell）。
"""
import csv, os, sys, time
from collections import defaultdict

DUR = int(sys.argv[1]) if len(sys.argv) > 1 else 240
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/user/cpu_e7_perproc.csv"
HZ = os.sysconf("SC_CLK_TCK")
NCPU = os.cpu_count() or 4

# 認得出來的角色。比對 cmdline 的關鍵字 -> 顯示名稱
ROLES = [
    ("nav2_controller/controller_server", "nav2 controller"),
    ("nav2_planner/planner_server",       "nav2 planner"),
    ("nav2_amcl/amcl",                    "AMCL 定位"),
    ("nav2_collision_monitor",            "collision_monitor"),
    ("nav2_map_server/map_server",        "map_server"),
    ("nav2_bt_navigator",                 "bt_navigator"),
    ("nav2_smoother",                     "smoother"),
    ("nav2_behaviors",                    "behaviors"),
    ("nav2_velocity_smoother",            "velocity_smoother"),
    ("nav2_waypoint_follower",            "waypoint_follower"),
    ("nav2_lifecycle_manager",            "lifecycle_manager"),
    ("slam_toolbox",                      "slam_toolbox"),
    ("path_teach_cc",                     "path_teach_cc ★循跡"),
    ("steering_trim_cc",                  "steering_trim_cc"),
    ("scan_filter_cc",                    "scan_filter_cc"),
    ("stuck_detector_cc",                 "stuck_detector_cc"),
    ("imu_bias_corrector_cc",             "imu_bias_corrector"),
    ("map_service_cc",                    "map_service_cc"),
    ("waypoint_service_cc",               "waypoint_service_cc"),
    ("navigation_action_cc",              "navigation_action_cc"),
    ("hmi_server",                        "HMI 伺服器"),
    ("llm_service",                       "LLM 客戶端"),
    ("robot_localization",                "EKF"),
    ("ekf_node",                          "EKF"),
    ("lslidar",                           "雷達驅動"),
    ("wheeltec_robot",                    "底盤驅動"),
    ("robot_state_publisher",             "robot_state_publisher"),
    ("face_embedding",                    "人臉推論"),
    ("camera_manager_cc",                 "相機管理"),
    ("depth_obstacle_cc",                 "深度避障"),
]


def scan():
    """回傳 {角色: (pid 集合, jiffies 總和, RSS MB)}"""
    out = defaultdict(lambda: [set(), 0, 0.0])
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as f:
                cl = f.read().replace(b"\0", b" ").decode("utf-8", "replace")
            if not cl.strip():
                continue
            role = None
            for key, name in ROLES:
                if key in cl:
                    role = name
                    break
            if role is None:
                continue
            with open(f"/proc/{d}/stat") as f:
                p = f.read().rsplit(")", 1)[1].split()
            jif = int(p[11]) + int(p[12])          # utime + stime
            rss = int(p[21]) * os.sysconf("SC_PAGE_SIZE") / 1048576.0
            e = out[role]
            e[0].add(d); e[1] += jif; e[2] += rss
        except (OSError, ValueError, IndexError):
            continue
    return out


prev = scan()
t0 = time.time()
acc = defaultdict(list)
rssmax = defaultdict(float)
while time.time() - t0 < DUR:
    time.sleep(2.0)
    cur = scan()
    for role, (pids, jif, rss) in cur.items():
        pj = prev.get(role, [set(), jif, 0.0])[1]
        pct = 100.0 * (jif - pj) / HZ / 2.0        # 一顆核心 = 100%
        if pct >= 0:
            acc[role].append(pct)
        rssmax[role] = max(rssmax[role], rss)
    prev = cur

rows = []
for role, vals in acc.items():
    if not vals:
        continue
    rows.append((sum(vals) / len(vals), max(vals), rssmax[role], role))
rows.sort(reverse=True)

with open(OUT, "w", newline="", encoding="utf-8") as fh:
    w = csv.writer(fh)
    w.writerow(["role", "cpu_avg_pct", "cpu_peak_pct", "rss_peak_mb"])
    for a, p, m, r in rows:
        w.writerow([r, f"{a:.1f}", f"{p:.1f}", f"{m:.0f}"])

print(f"  取樣 {int(time.time()-t0)} 秒（{NCPU} 核，100% = 吃滿一顆核心）")
print()
print(f"  {'角色':<24}{'平均':>8}{'尖峰':>9}{'記憶體':>10}")
print("  " + "-" * 52)
tot = 0.0
for a, p, m, r in rows:
    tot += a
    print(f"  {r:<24}{a:7.1f}%{p:8.1f}%{m:8.0f} MB")
print("  " + "-" * 52)
print(f"  {'合計':<24}{tot:7.1f}%          （400% = 四核全滿）")
