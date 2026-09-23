"""到訪統計：讀 bank_reception_node 寫的 visit_log.db。

★ 這支是**唯讀**的，而且開在一個獨立的 sqlite 連線上。
  bank_reception_node 是寫入端，兩邊各自連線、各自關閉——sqlite 本身處理
  併發，不要試圖共用連線物件（跨行程共用不了，跨執行緒共用要
  check_same_thread=False，兩個都不必要）。

★ 全程 try/except 且失敗回空結果：統計頁看不到數字是小事，
  讓 HMI 因為讀不到一個選用的資料庫而噴 500 才是大事。

★ 這是阻塞 I/O，呼叫端要丟到 executor。
"""

import sqlite3
import time
from pathlib import Path
from typing import Any, Dict, Optional

VISIT_TYPE_LABEL = {"VIP": "貴賓", "GUEST": "訪客",
                    "ADMIN": "管理者", "BLACKLIST": "黑名單"}


def read_visit_stats(days: int = 7, db_path: Optional[str] = None) -> Dict[str, Any]:
    """回傳到訪統計。★ 這是阻塞 I/O，呼叫端要丟到 executor。"""
    out: Dict[str, Any] = {
        "available": False, "reason": "", "today": {}, "total": 0,
        "unique_people": 0, "by_type": {}, "by_hour": [0] * 24,
        "by_day": [], "recent": [],
    }
    path = db_path or str(Path.home() / ".smartnav" / "visit_log.db")
    out["path"] = path
    if not Path(path).exists():
        out["reason"] = ("尚未有任何到訪記錄。★ 這代表 bank_reception "
                         "還沒跑過、或它的 enable_visit_log 是 false")
        return out
    try:
        # uri=True + mode=ro：唯讀開啟，絕不可能動到寫入端的資料
        conn = sqlite3.connect(f"file:{path}?mode=ro", uri=True, timeout=2.0)
    except Exception as exc:                    # noqa: BLE001
        out["reason"] = f"開啟資料庫失敗：{exc}"
        return out
    try:
        cur = conn.cursor()
        now = int(time.time())
        # 今天的起點用本地時間的 00:00，不是 now-86400 ——
        # 展示時講的「今天接待幾位」指的是日曆上的今天。
        lt = time.localtime(now)
        day0 = int(time.mktime((lt.tm_year, lt.tm_mon, lt.tm_mday,
                                0, 0, 0, 0, 0, -1)))
        since = day0 - (days - 1) * 86400

        cur.execute("SELECT COUNT(*), COUNT(DISTINCT person_uuid) FROM visits")
        row = cur.fetchone() or (0, 0)
        out["total"], out["unique_people"] = int(row[0]), int(row[1] or 0)

        cur.execute("SELECT person_type, COUNT(*) FROM visits "
                    "WHERE ts >= ? GROUP BY person_type", (day0,))
        today_by_type = {str(k): int(v) for k, v in cur.fetchall()}
        out["today"] = {
            "count": sum(today_by_type.values()),
            "by_type": today_by_type,
            "since": day0,
        }

        cur.execute("SELECT person_type, COUNT(*) FROM visits GROUP BY person_type")
        out["by_type"] = {str(k): int(v) for k, v in cur.fetchall()}

        # 今日每小時分布 —— 展示時最有畫面的一張圖
        cur.execute("SELECT ts FROM visits WHERE ts >= ?", (day0,))
        for (ts,) in cur.fetchall():
            out["by_hour"][time.localtime(int(ts)).tm_hour] += 1

        # 近 N 天每日人次
        cur.execute("SELECT ts FROM visits WHERE ts >= ?", (since,))
        per_day: Dict[str, int] = {}
        for (ts,) in cur.fetchall():
            k = time.strftime("%m/%d", time.localtime(int(ts)))
            per_day[k] = per_day.get(k, 0) + 1
        out["by_day"] = [
            {"date": time.strftime("%m/%d", time.localtime(since + i * 86400)),
             "count": per_day.get(
                 time.strftime("%m/%d", time.localtime(since + i * 86400)), 0)}
            for i in range(days)
        ]

        cur.execute("SELECT ts, person_name, person_type, confidence "
                    "FROM visits ORDER BY ts DESC LIMIT 20")
        out["recent"] = [
            {"ts": int(r[0]),
             "time": time.strftime("%m/%d %H:%M:%S", time.localtime(int(r[0]))),
             "name": r[1] or "未知",
             "type": r[2] or "",
             "type_label": VISIT_TYPE_LABEL.get(str(r[2]), str(r[2] or "—")),
             "confidence": round(float(r[3] or 0.0), 3)}
            for r in cur.fetchall()
        ]
        out["available"] = True
    except Exception as exc:                    # noqa: BLE001
        out["reason"] = f"查詢失敗：{exc}"
    finally:
        try:
            conn.close()
        except Exception:                       # noqa: BLE001
            pass
    return out

