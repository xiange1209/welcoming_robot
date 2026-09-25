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
    # ★ 2026-09-24：原本收的是 welcoming_robot_ws/，但 9/23 目錄已攤平成 src/，
    #   照舊寫法會打出「只有 maprun、沒有任何 ROS 套件」的包，而且不報錯——
    #   上車 colcon build 建的是舊程式碼，卻以為已經更新了。
    #   ⚠ 本機可能還殘留舊的 welcoming_robot_ws/（被 gitignore 的快取與備份），
    #     這裡刻意不收它。
    targets = []
    for d in ("src", "maprun"):
        if pathlib.Path(d).is_dir():
            targets += [p for p in pathlib.Path(d).rglob("*")
                        if p.is_file() and want(p)]
    targets += list(pathlib.Path(".").glob("*.md"))
    # cyclonedds.xml：解壓在 ~ 之下就落在 ~/cyclonedds.xml，正是 env.sh 找的位置。
    # 2026-09-25 前沒收它 —— 換一片新 SD 卡照 README 部署，nav2 會因 participant 索引耗盡而崩
    for extra in ("e1_results.csv", "cyclonedds.xml"):
        if pathlib.Path(extra).exists():
            targets.append(pathlib.Path(extra))
    return sorted(set(targets))


def preflight() -> list:
    """打包前的硬性檢查。回傳錯誤訊息清單，空的代表可以打包。

    兩項都是「漏了不會報錯、上車才發現」的靜默失效，所以在這裡擋。
    """
    errs = []

    # 1) frontier 是 git 子模組。沒 init 的話 src/frontier_exploration_ros2/ 是空目錄，
    #    包照樣打得出來，但車上會少一整個套件。
    sub = pathlib.Path("src/frontier_exploration_ros2")
    if not (sub / "package.xml").exists():
        errs.append("frontier 子模組沒初始化（src/frontier_exploration_ros2/ 是空的）\n"
                    "      修法：git submodule update --init src/frontier_exploration_ros2")
    else:
        # ★ 子模組是獨立的 git repo，父 repo 的 .gitattributes（eol=lf）管不到它。
        #   它吃的是系統層 core.autocrlf，而 Git for Windows 預設是 true
        #   → 簽出後全變 CRLF（2026-09-24 實際發生）。
        #   ★ 不要用下面通用的「改寫位元組」修法：那會讓子模組變成有修改的髒狀態。
        #     要修 checkout 設定，讓它從上游的 LF blob 重新簽出。
        crlf = [p for p in sub.rglob("*")
                if p.is_file() and ".git" not in p.parts
                and p.suffix in EXEC_TEXT and b"\r\n" in p.read_bytes()]
        if crlf:
            errs.append(f"frontier 子模組有 {len(crlf)} 個檔是 CRLF（Windows 的 autocrlf 害的，上游本身是 LF）\n"
                        "      修法（只要做一次）：\n"
                        "        git -C src/frontier_exploration_ros2 config core.autocrlf false\n"
                        "        git -C src/frontier_exploration_ros2 rm -rq --cached .\n"
                        "        git -C src/frontier_exploration_ros2 reset -q --hard")

    # 2) 前端是 React，dist/ 被 gitignore。setup.py 把 frontend/dist/** 裝進 share，
    #    沒建置的話 HMI 後端照跑、平板打開是一片空白。
    fe = pathlib.Path("src/smartnav_hmi/frontend")
    dist_index = fe / "dist" / "index.html"
    if not dist_index.exists():
        errs.append("前端還沒建置（找不到 frontend/dist/index.html）\n"
                    "      修法：cd src/smartnav_hmi/frontend && npm ci && npm run build")
    else:
        # 改了 TSX 卻忘了重建 —— dist 比原始碼舊就擋下來
        srcs = [p for p in (fe / "src").rglob("*") if p.is_file()]
        newest = max((p.stat().st_mtime for p in srcs), default=0)
        if newest > dist_index.stat().st_mtime:
            stale = max(srcs, key=lambda p: p.stat().st_mtime)
            errs.append(f"前端 dist 比原始碼舊（{stale.relative_to(fe)} 在建置之後又改過）\n"
                        "      修法：cd src/smartnav_hmi/frontend && npm run build")
    return errs


def main() -> int:
    stamp = sys.argv[1] if len(sys.argv) > 1 else time.strftime("%Y%m%d")
    out = f"car_code_{stamp}.tar.gz"

    if not pathlib.Path("src").is_dir():
        print("✗ 找不到 src/ —— 請在 車子/ 目錄執行，並確認在 9/23 之後的分支上")
        return 1

    errs = preflight()
    if errs:
        print("✗ 打包前檢查沒過：")
        for e in errs:
            print("   ·", e)
        return 1

    targets = collect()
    if not targets:
        print("✗ 找不到 src/ 或 maprun/ —— 請在 車子/ 目錄執行")
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

    def _mode(ti: tarfile.TarInfo) -> tarfile.TarInfo:
        # ★ Windows 檔案系統沒有執行權限位元，tarfile 會把 .sh 記成 0644，
        #   解壓到車上就是 Permission denied。直接在包裡寫成 0755，
        #   不靠人記得上車後 chmod +x。
        ti.mode = 0o755 if ti.name.endswith(".sh") else 0o644
        return ti

    with tarfile.open(out, "w:gz") as t:
        for p in targets:
            t.add(p, arcname=str(p).replace("\\", "/"), filter=_mode)

    md5 = hashlib.md5(pathlib.Path(out).read_bytes()).hexdigest()
    print(f"✓ {out}")
    print(f"  檔案數 {len(targets)}")
    print(f"  大小   {os.path.getsize(out) / 1024 / 1024:.2f} MB")
    print(f"  MD5    {md5}")
    print(f"  腳本 CRLF 0 / 疑似機密 0")
    if md_crlf:
        print(f"  （.md 帶 CRLF {len(md_crlf)} 個，無害不處理）")
    print()
    print("── 筆電 ──")
    print(f"  scp {out} user@192.168.137.106:~/")
    print()
    print("── Pi ──（repo 的 src/ 對應 workspace 的 src/，所以分兩次解壓）")
    print(f"  1) cd ~ && tar -xzf {out} --exclude='src/*'   # 先拿 maprun（含遷移腳本）")
    print("  2) ~/maprun/verify/migrate_layout.sh            # ★ 只有第一次升結構要跑")
    print(f"  3) cd ~/welcoming_robot_ws && tar -xzf ~/{out} src")
    print("  4) colcon build --symlink-install && source install/setup.bash")
    print()
    print("  第 2 步漏掉的話，新舊兩份套件會同時在 src/ 底下，")
    print("  colcon 報 Duplicate package names。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
