#!/usr/bin/env python3
"""系統時鐘有沒有在跑的當下跳過 —— 證實或排除「牆鐘驅動的馬達迴圈」假說

## 為什麼需要這支（2026-08-25）

`_unwedge_leg`（楔住脫困的推進段）與 `steering_trim` 的量測迴圈原本用
`time.time()`（牆鐘）當逾時界限，而不是 `time.monotonic()`。已經改掉了，
但**改對了不代表它本來就是主因** —— 要確認它有沒有真的發生過。

假說：RPi4 沒有 RTC 電池，開機時系統時間是 `fake-hwclock` 回填的舊值，
等連上手機熱點之後 NTP 才把它**階躍**校正。如果那個階躍發生在車子正在跑的
當下，兩個馬達迴圈會：

    往前跳 -> 差值瞬間變大 -> 迴圈立刻結束 -> 這一段等於沒推
              -> 脫困回報失敗 -> 反覆脫困 -> 把 AMCL 轉丟
    往後跳 -> 差值變負 -> **永遠 < limit** -> 迴圈不因逾時結束，車子一直推

## 這支怎麼判

牆鐘與單調鐘的**差值**（wall - mono）在正常情況下是常數。
時鐘一被階躍校正，這個差值就會跳一大段。所以：

    差值跳動 = 時鐘被調過
    差值穩定 = 時鐘沒動

另外去 journal 撈「開機之後多久才校時」—— 那決定了風險窗有多寬。

## 用法

    python3 ~/maprun/tools_0825/clock_step_check.py            # 只看歷史，秒回
    python3 ~/maprun/tools_0825/clock_step_check.py --watch 300 # 再盯 300 秒

★ 建議在**剛開機、還沒開始跑**的時候用 --watch 盯著，那正是風險窗。
"""
import argparse
import os
import subprocess
import sys
import time


def sh(cmd):
    """跑一個指令，失敗就回 None（這支不該因為撈不到 log 就掛掉）。"""
    try:
        out = subprocess.run(cmd, shell=True, capture_output=True,
                             text=True, timeout=15)
        return out.stdout if out.returncode == 0 else None
    except Exception:
        return None


def uptime_sec():
    try:
        with open("/proc/uptime") as f:
            return float(f.read().split()[0])
    except Exception:
        return None


def section(title):
    print()
    print("=" * 64)
    print(title)
    print("=" * 64)


def check_history():
    section("一、這次開機到現在，時鐘被調過嗎")

    up = uptime_sec()
    if up is not None:
        print(f"已開機 {up:.0f} 秒（{up / 60:.1f} 分鐘）")

    # 核心在系統時間被設定時會記一筆。這是最直接的證據。
    hits = []
    for pat in ("Time has been changed",
                "System clock wrong",
                "Initial synchronization to time server",
                "Synchronized to time server",
                "System time before build time"):
        out = sh(f'journalctl -b --no-pager 2>/dev/null | grep -i "{pat}"')
        if out:
            for line in out.strip().splitlines():
                hits.append(line.strip())

    if hits is None or not hits:
        print()
        print("journal 裡找不到校時紀錄。可能是：")
        print("  (a) 這次開機時鐘真的沒被調過")
        print("  (b) 沒有權限讀 journal —— 試 `sudo journalctl -b | grep -i \"time has been changed\"`")
        print("  (c) 這台沒跑 systemd-timesyncd / chrony")
    else:
        print()
        print(f"★ 找到 {len(hits)} 筆校時紀錄：")
        for h in hits[:20]:
            print("   " + h[:160])
        print()
        print("★★ 有紀錄代表時鐘**確實會跳**。接著要看它是「開機多久之後」跳的 ——")
        print("    那個時間點如果落在你通常開始跑的時候，假說就成立。")

    # 時間同步服務現在的狀態
    out = sh("timedatectl 2>/dev/null")
    if out:
        print()
        print("--- timedatectl ---")
        for line in out.strip().splitlines():
            line = line.strip()
            if any(k in line for k in ("System clock synchronized",
                                       "NTP service", "Local time",
                                       "RTC time", "Universal time")):
                print("   " + line)
        if "RTC time: n/a" in out or "RTC time: n/a" in out.replace("  ", " "):
            print("   ★ RTC time: n/a —— 這台沒有硬體時鐘，符合假說前提")

    # 有沒有 fake-hwclock（沒有 RTC 的板子才會裝）
    if os.path.exists("/etc/fake-hwclock.data"):
        try:
            with open("/etc/fake-hwclock.data") as f:
                saved = f.read().strip()
            print()
            print(f"--- fake-hwclock ---")
            print(f"   上次關機時存的時間：{saved}")
            print("   ★ 有這個檔就代表**沒有 RTC 電池**，開機時間是回填的，"
                  "之後一定要靠 NTP 校正 —— 風險窗存在。")
        except Exception:
            pass
    else:
        print()
        print("--- fake-hwclock ---")
        print("   沒有 /etc/fake-hwclock.data")


def watch(seconds):
    section(f"二、接下來 {seconds:.0f} 秒盯著牆鐘與單調鐘的差值")
    print("差值 = time.time() - time.monotonic()。正常情況它是**常數**。")
    print("一旦跳動，就是時鐘被調了，跳多少就是階躍多大。")
    print()

    base = time.time() - time.monotonic()
    t0 = time.monotonic()
    worst = 0.0
    steps = []
    last_print = 0.0
    prev = base

    try:
        while time.monotonic() - t0 < seconds:
            time.sleep(0.25)
            cur = time.time() - time.monotonic()
            delta = cur - prev
            if abs(delta) > 0.5:        # 0.5 秒以上才算「階躍」，不是慢慢調
                el = time.monotonic() - t0
                steps.append((el, delta))
                print(f"★★ 第 {el:.1f} 秒：時鐘{'往前' if delta > 0 else '往後'}"
                      f"跳了 {abs(delta):.1f} 秒")
                if abs(delta) > abs(worst):
                    worst = delta
            prev = cur
            el = time.monotonic() - t0
            if el - last_print >= 30.0:
                last_print = el
                print(f"   （{el:.0f}/{seconds:.0f} 秒，累積偏移 "
                      f"{cur - base:+.3f} 秒）")
    except KeyboardInterrupt:
        print("\n（手動中斷）")

    print()
    total = (time.time() - time.monotonic()) - base
    if steps:
        print(f"★★ 判定：**時鐘在觀測期間跳了 {len(steps)} 次**，"
              f"最大一次 {worst:+.1f} 秒，累積 {total:+.3f} 秒。")
        print("   假說成立：牆鐘驅動的迴圈在這種時候會出事。")
        print("   → 已改成 monotonic 的地方不會再受影響；")
        print("     但如果你跑的是**還沒更新的舊版**，那就是不穩定的直接原因。")
        print("   → 另外建議：等 `timedatectl` 顯示 System clock synchronized: yes")
        print("     之後再開始跑，把風險窗關掉。")
    else:
        print(f"✓ 判定：觀測期間時鐘**沒有階躍**（累積偏移 {total:+.3f} 秒，"
              f"那是正常的緩調）。")
        print("   這一段時間內假說不成立。★ 但這只證明「這 "
              f"{seconds:.0f} 秒沒跳」——")
        print("   風險窗在**剛開機還沒連上熱點**的那幾分鐘，要在那時候量才算數。")


def main():
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("--watch", type=float, default=0.0,
                    metavar="秒", help="再盯著看幾秒（預設 0 = 只看歷史）")
    args = ap.parse_args()

    print("系統時鐘階躍檢查　—— 證實或排除「牆鐘驅動的馬達迴圈」假說")
    check_history()
    if args.watch > 0:
        watch(args.watch)
    else:
        section("下一步")
        print("要確認風險窗有多寬，請在**剛開機**時跑：")
        print("    python3 ~/maprun/tools_0825/clock_step_check.py --watch 300")
    print()


if __name__ == "__main__":
    sys.exit(main())
