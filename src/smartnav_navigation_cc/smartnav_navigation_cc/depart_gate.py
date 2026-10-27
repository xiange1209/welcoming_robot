"""出發前確認行進方向淨空（★ 2026-10-27 新增）。

為什麼：迎賓時客人講完「帶我去 X」通常就站在車子正前方。以前規劃完直接起跑 → path_teach 看到「前方障礙」
→ 車動不了 5 秒判定卡住 → 脫困（倒車、打方向）—— 對著人做脫困動作。10/27 上機 12:36 那趟就是這樣起步就卡。
而且規劃出來的路徑約三成是**先倒車**（存檔的第一點 d=-1），所以要看「第一段要走的方向」，不是只看前面。

做法：出發前抓雷射，看第一段行進方向左右 sector_deg 內、clear_m 以內有沒有東西（至少 min_points 點，避免雜訊）：
  - 淨空 → 立刻出發（不多等）
  - 有東西 → 用 /speech_text 請人讓開（平板會念），等它連續淨空 hold_sec 秒；10 秒還擋著再講一次；
    最多等 wait_max_sec，之後照常出發（剩下的交給 path_teach 原本的停車等待／繞行）。
  - 等的時候被取消（急停、新的導航）→ 立刻回 False，不出發。
只在這段期間訂閱 /scan，平常不吃 CPU。
"""
import json
import math
import os
import time

PATH_DB = os.path.expanduser("~/.smartnav/path_database/paths.json")


def first_direction(path_id, db_path=PATH_DB):
    """存檔路徑第一點的行進方向：+1 前進、-1 後退；讀不到回 None（當前進處理）"""
    try:
        with open(db_path, encoding="utf-8") as fh:
            pts = json.load(fh)["paths"][path_id]["points"]
        d = int(pts[0].get("d", 1))
        return -1 if d < 0 else 1
    except Exception:  # noqa: BLE001
        return None


def sector_block(ranges, angle_min, angle_inc, center, sector_rad, clear_m, min_range=0.25, min_points=3):
    """扇區內比 clear_m 近的點夠多 → 回最近距離；否則回 None（淨空）。角度以雷射座標系（0＝車頭）為準。"""
    near = []
    for i, r in enumerate(ranges):
        if not (min_range <= r < clear_m) or math.isnan(r):
            continue
        a = angle_min + i * angle_inc
        if abs(math.atan2(math.sin(a - center), math.cos(a - center))) <= sector_rad:
            near.append(r)
    return min(near) if len(near) >= min_points else None


def wait_until_clear(get_scan, is_cancelled, speak, log, *, direction, clear_m=0.9, sector_deg=25.0,
                     hold_sec=1.0, wait_max_sec=20.0, first_scan_sec=1.0, repeat_after_sec=10.0,
                     speech_front="不好意思，我要出發了，請先站到我旁邊一點。",
                     speech_rear="不好意思，我要先倒車，請後面的人先讓一下。",
                     sleep=time.sleep, now=time.monotonic):
    """回 True＝可以出發、False＝等的時候被取消。get_scan() 回最新的 LaserScan 或 None。"""
    center = math.pi if direction == -1 else 0.0
    where = "後方" if direction == -1 else "前方"
    sector = math.radians(sector_deg)
    t0 = now()
    while get_scan() is None:                 # 剛訂閱，等第一筆
        if is_cancelled():
            return False
        if now() - t0 > first_scan_sec:
            log(f"出發前檢查：{first_scan_sec:.0f} 秒內收不到雷射，略過檢查直接出發")
            return True
        sleep(0.05)
    spoke, blocked_once, clear_since = 0, False, None
    while True:
        if is_cancelled():
            return False
        s = get_scan()
        d = sector_block(s.ranges, s.angle_min, s.angle_increment, center, sector, clear_m)
        t = now()
        if d is None:
            if not blocked_once:
                return True                   # 一開始就淨空：立刻出發，不多等
            clear_since = clear_since or t
            if t - clear_since >= hold_sec:
                log(f"出發前檢查：{where}淨空了（等了 {t - t0:.1f} 秒）—— 出發")
                return True
        else:
            clear_since = None
            if not blocked_once:
                blocked_once = True
                log(f"出發前檢查：{where} {d:.2f} m 有人／東西（門檻 {clear_m:.2f} m）—— 請對方讓開、等淨空再走")
            if spoke == 0 or (spoke == 1 and t - t0 >= repeat_after_sec):
                speak(speech_rear if direction == -1 else speech_front)
                spoke += 1
        if t - t0 >= wait_max_sec:
            log(f"出發前檢查：等了 {wait_max_sec:.0f} 秒{where}還是有東西 —— 照常出發（交給循跡的停車等待／繞行）")
            return True
        sleep(0.1)
