#!/usr/bin/env python3
"""打包要送上 Pi 的交付包。在 Windows 端的 車子/ 目錄執行。

    python pack_car_code.py            # 檔名用今天的日期
    python pack_car_code.py 20260901   # 指定日期

## 為什麼不直接用 tar

檔名裡的中文在 `tar -tzf` 的輸出會變成八進位跳脫，用 grep 驗證會得到**假陰性**
（2026-08-25 就因為這樣誤報過「驗證清單不在包裡」）。Python 的 tarfile
直接吐 unicode 檔名，驗證才可靠。

## 三個閘門（任何一個不過就不打包）

1. **腳本 CRLF** —— Pi 上 shebang 帶 ^M 會變成 `bad interpreter`。
   `.md` 帶 CRLF 無害，不擋。
2. **疑似機密** —— Telegram token、API key、硬編密碼。
   ★ 真正的機密放在 Pi 的 `~/.smartnav/secrets/`，本來就不該進包，
     這個掃描是防止不小心貼進程式碼。
3. **建置產物** —— `build/`、`install/`、`__pycache__` 一律排除。

## ★ 為什麼排除 logs/

`maprun/logs/` 裡的東西是**從 Pi 拿回來的**，包回去會覆蓋 Pi 上比較新的。
裡面的 `nav_trace_*.csv` 是專案鐵則第 9 條「每趟重播都存逐筆 CSV」的研究數據，
被舊副本蓋掉就沒了。

地圖（`.pgm` / `maps_backup/` / `*.json`）則**照舊收進來** —— 那是既有行為。
"""
import hashlib
import os
import pathlib
import re
import sys
import tarfile
import time

# ★ 2026-09-09：Windows 主控台預設是 CP950，印 "✓" 會丟 UnicodeEncodeError。
#   踩到的方式很惡劣：**包已經成功寫出來了**，卻在最後一行報告成功時崩掉，
#   看起來像打包失敗。今天（實驗室測試日）差點因此以為沒打包成功。
#   這裡在最前面就把輸出改成 UTF-8，errors="replace" 保證永遠不會因為
#   印字而讓打包看起來失敗。
for _s in (sys.stdout, sys.stderr):
    if hasattr(_s, "reconfigure"):
        try:
            _s.reconfigure(encoding="utf-8", errors="replace")
        except (OSError, ValueError):    # 被重導向到不支援的串流時忽略
            pass

SKIP_DIR = {"__pycache__", ".git", "build", "install", "log", ".pytest_cache",
            "node_modules", ".vscode", "logs", "logs_0814"}
SKIP_EXT = {".pyc", ".pyo", ".so", ".o", ".a"}
SKIP_NAME = {".gitignore"}

SECRET = re.compile(
    r"(bot\d{6,}:[A-Za-z0-9_-]{30,}"          # Telegram bot token
    r"|sk-[A-Za-z0-9]{20,}"                   # OpenAI 式金鑰
    r"|password\s*[:=]\s*['\"][^'\"]{4,}"
    r"|api[_-]?key\s*[:=]\s*['\"][^'\"]{8,}"
    r"|token\s*[:=]\s*['\"][A-Za-z0-9:_-]{20,})", re.I)

TEXT = (".py", ".sh", ".yaml", ".yml", ".md", ".xml", ".cfg", ".html", ".js")
EXEC_TEXT = (".py", ".sh", ".yaml", ".yml", ".xml", ".cfg")   # CRLF 對這些致命


def want(p: pathlib.Path) -> bool:
    if set(p.parts) & SKIP_DIR:
        return False
    if p.suffix.lower() in SKIP_EXT or p.name in SKIP_NAME:
        return False
    return not p.is_symlink()


def collect():
    targets = []
    for d in ("welcoming_robot_ws", "maprun"):
        if pathlib.Path(d).is_dir():
            targets += [p for p in pathlib.Path(d).rglob("*")
                        if p.is_file() and want(p)]
    targets += list(pathlib.Path(".").glob("*.md"))
    for extra in ("e1_results.csv",):
        if pathlib.Path(extra).exists():
            targets.append(pathlib.Path(extra))
    return sorted(set(targets))


def main() -> int:
    stamp = sys.argv[1] if len(sys.argv) > 1 else time.strftime("%Y%m%d")
    out = f"car_code_{stamp}.tar.gz"

    targets = collect()
    if not targets:
        print("✗ 找不到 welcoming_robot_ws/ 或 maprun/ —— 請在 車子/ 目錄執行")
        return 1

    hits, crlf, md_crlf = [], [], []
    for p in targets:
        try:
            b = p.read_bytes()
        except Exception:                       # noqa: BLE001
            continue
        if p.suffix in EXEC_TEXT and b"\r\n" in b:
            crlf.append(str(p))
        elif p.suffix in TEXT and b"\r\n" in b:
            md_crlf.append(str(p))
        if p.suffix in TEXT:
            m = SECRET.search(b.decode("utf-8", "ignore"))
            if m:
                hits.append((str(p), m.group(0)[:50]))

    if crlf:
        print("✗ 腳本帶 CRLF（Pi 上會 bad interpreter），先修再打包：")
        for f in crlf:
            print("   ", f)
        print("\n修法： python -c \"import pathlib;"
              "[p.write_bytes(p.read_bytes().replace(b'\\r\\n',b'\\n')) "
              "for p in map(pathlib.Path, ['檔名'])]\"")
        return 1
    if hits:
        print("✗ 疑似機密，不打包：")
        for f, h in hits:
            print("   ", f, "->", h)
        return 1

    with tarfile.open(out, "w:gz") as t:
        for p in targets:
            t.add(p, arcname=str(p).replace("\\", "/"))

    md5 = hashlib.md5(pathlib.Path(out).read_bytes()).hexdigest()
    print(f"✓ {out}")
    print(f"  檔案數 {len(targets)}")
    print(f"  大小   {os.path.getsize(out) / 1024 / 1024:.2f} MB")
    print(f"  MD5    {md5}")
    print(f"  腳本 CRLF 0 / 疑似機密 0")
    if md_crlf:
        print(f"  （.md 帶 CRLF {len(md_crlf)} 個，無害不處理）")
    print()
    print(f"  scp {out} user@192.168.137.106:~/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
