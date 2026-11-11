#!/usr/bin/env python3
"""localize_here.py --auto 的變形（11/10 車上 agent，繞過 HMI /api/localize/here 只等 0.5 秒的時間差）。

一般流程：發 /initialpose → 0.5 秒後 /align_pose。AMCL 剛換位姿時舊的 map→odom 還有效一陣子，
對齊讀到的是舊位姿，把全圖比對找到的正確位置蓋掉（11/04、11/10 都發生）。
這支：全圖比對 → 只設位姿（align=false）→ 等 SETTLE 秒 → 同一個位姿再設一次並對齊。車不會動。
用法：python3 localize_settle.py [等待秒數，預設 3]
"""
import math
import os
import subprocess
import sys
import time

T = '/home/user/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020'
sys.path.insert(0, T)
import localize_here as LH  # noqa: E402

SETTLE = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0


def main():
    LH.ensure_ros_env()
    env = subprocess.run(["systemctl", "show", "smartnav-hmi", "-p", "Environment", "--value"],
                         capture_output=True, text=True).stdout.split()
    kv = dict(x.split("=", 1) for x in env if "=" in x)
    st, js = LH.call("POST", "/api/login", {"username": "admin", "password": kv.get("SMARTNAV_ADMIN_PASSWORD", "")})
    if st != 200:
        print(f"✗ 登入 HMI 失敗（HTTP {st}）")
        return 1
    tok = js["token"]
    try:
        pose, why, cands = LH.auto_pose()
        if pose is None:
            print(f"⚠ 自動判斷不出來：{why}")
            return 2
        x, y, yaw = pose
        st, js = LH.call("POST", "/api/localize/here", {"x": x, "y": y, "yaw": yaw, "align": False}, token=tok)
        print(f"  只設位姿：{js.get('message')}（HTTP {st}）；等 {SETTLE:.0f} 秒讓 AMCL／TF 換過去…")
        time.sleep(SETTLE)
        return LH.post_and_verify(tok, {"x": x, "y": y, "yaw": yaw, "align": True},
                                  f"自動判斷的位置 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)", auto_cands=cands)
    finally:
        LH.call("POST", "/api/logout", {}, token=tok)


if __name__ == "__main__":
    sys.exit(main())
