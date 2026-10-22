# -*- coding: utf-8 -*-
"""2026-10-22：雙板環境覆蓋層（env.sh 的雙板區塊、overlay.sh、dualboard_mode.sh）的行為測試。

背景：車上 agent 的雙板草稿把雙板環境寫在 env_dualboard.sh，另外靠 HMI 的 systemd drop-in 設 domain 42。
但 HMI 的「定位＋導航」單元走 run_nav_cc.sh，那支一開頭就 source ~/maprun/env.sh，而 env.sh 把 ROS_DOMAIN_ID
設回 0、探索範圍設回 LOCALHOST → HMI（domain 42）看不到自己開出來的導航。12 支腳本都 source env.sh。
修法：env.sh 在 ~/.smartnav/dualboard_role 存在時自己 source overlay.sh；沒有標記檔＝完全是單板。

測什麼（真的用 bash 執行 env.sh 的雙板區塊與 overlay.sh，環境用暫存目錄）
  ✓ 沒有標記檔：domain 0、LOCALHOST、CYCLONEDDS_URI 都不變，SMARTNAV_DUALBOARD 沒設（＝單板行為完全不變）
  ✓ 標記檔 A／B：domain 42、SUBNET、CYCLONEDDS_URI 指到雙板 xml、SMARTNAV_DUALBOARD=1、SMARTNAV_BOARD 對
  ✓ 有標記檔但 overlay.sh 不見：不動環境（不會因為缺檔把車子環境弄壞）
  ✓ set -u 下不報錯（有些腳本會開 set -u 再 source）
  ✓ overlay 的值＝HMI systemd drop-in 的值（逐項，防兩邊漂移）
  ✓ dualboard_mode.sh：A／B 寫標記檔、off 刪掉、亂給參數回用法並非 0 結束
不測：真的拔網路線（V3，要在實機）。

用法：python -X utf8 maprun/tools_1021/test_dualboard_env.py
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

CAR = Path(os.environ.get("SMARTNAV_REPO") or Path(__file__).resolve().parents[2])
MAPRUN = CAR / "maprun"
DB = MAPRUN / "dualboard"
BASH = shutil.which("bash")
results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


if not BASH:
    print("  （沒有 bash，跳過）")
    sys.exit(0)

env_sh = (MAPRUN / "env.sh").read_text(encoding="utf-8")
marker = "# ★★ 2026-10-22 雙板模式 ★★"
check(marker in env_sh, "env.sh 有雙板區塊")
BLOCK = env_sh[env_sh.index(marker):]
TMP = Path(tempfile.mkdtemp(prefix="dualboard_env_"))
ROLE = TMP / "dualboard_role"
OVERLAY_DIR = TMP / "dualboard"
OVERLAY_DIR.mkdir()
shutil.copy(DB / "overlay.sh", OVERLAY_DIR / "overlay.sh")


def posix(p: Path) -> str:
    s = str(p).replace("\\", "/")
    return re.sub(r"^([A-Za-z]):", lambda m: "/" + m.group(1).lower(), s)   # C:/x → /c/x（Git Bash）


def run(script: str, extra_env=None, pre="", u=False):
    """先設單板的值（env.sh 前半段做的事），再跑雙板區塊，印出結果"""
    e = {**os.environ, "SMARTNAV_ROLE_FILE": posix(ROLE), "DUALBOARD_DIR": posix(OVERLAY_DIR)}
    e.pop("SMARTNAV_DUALBOARD", None)
    e.pop("SMARTNAV_BOARD", None)
    if extra_env:
        e.update(extra_env)
    body = f"""{'set -u' if u else ''}
export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST CYCLONEDDS_URI=file:///home/user/cyclonedds.xml
{pre}
{script}
echo "DOMAIN=$ROS_DOMAIN_ID"; echo "RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE"; echo "URI=$CYCLONEDDS_URI"
echo "DUAL=${{SMARTNAV_DUALBOARD:-}}"; echo "BOARD=${{SMARTNAV_BOARD:-}}"
"""
    r = subprocess.run([BASH, "-c", body], capture_output=True, text=True, encoding="utf-8", env=e)
    out = dict(l.split("=", 1) for l in r.stdout.splitlines() if "=" in l and l.split("=", 1)[0] in ("DOMAIN", "RANGE", "URI", "DUAL", "BOARD"))
    return r.returncode, out, r.stderr


print("── 1. 沒有標記檔：單板行為完全不變 ──")
ROLE.unlink(missing_ok=True)
rc, o, err = run(BLOCK)
check(rc == 0 and o["DOMAIN"] == "0" and o["RANGE"] == "LOCALHOST" and o["URI"] == "file:///home/user/cyclonedds.xml" and o["DUAL"] == "" and o["BOARD"] == "",
      f"domain 0、LOCALHOST、原本的 xml、SMARTNAV_DUALBOARD 沒設（{o}）")

print("── 2. 標記檔 A／B ──")
for role in ("A", "B"):
    ROLE.write_text(role + "\n", encoding="utf-8")
    rc, o, err = run(BLOCK)
    check(rc == 0 and o["DOMAIN"] == "42" and o["RANGE"] == "SUBNET" and o["DUAL"] == "1" and o["BOARD"] == role,
          f"標記 {role}：domain 42、SUBNET、DUALBOARD=1、BOARD={role}")
    check(o["URI"] == f"file://{posix(OVERLAY_DIR)}/cyclonedds_dualboard.xml", "CYCLONEDDS_URI 指到雙板 xml")

print("── 3. 邊界 ──")
ROLE.write_text("A\n", encoding="utf-8")
(OVERLAY_DIR / "overlay.sh").rename(OVERLAY_DIR / "overlay.sh.off")
rc, o, err = run(BLOCK)
check(rc == 0 and o["DOMAIN"] == "0" and o["RANGE"] == "LOCALHOST" and o["DUAL"] == "", "有標記檔但 overlay.sh 不見：環境不動（不會把車子環境弄壞）")
(OVERLAY_DIR / "overlay.sh.off").rename(OVERLAY_DIR / "overlay.sh")
rc, o, err = run(BLOCK, u=True)
check(rc == 0 and o["DOMAIN"] == "42" and "unbound" not in err, f"set -u 下不報錯（stderr：{err.strip()[:60] or '空'}）")
ROLE.unlink()
rc, o, err = run(BLOCK, u=True)
check(rc == 0 and o["DOMAIN"] == "0" and "unbound" not in err, "set -u、沒有標記檔：也不報錯")

print("── 4. overlay 的值＝HMI systemd drop-in 的值（防漂移） ──")
ROLE.write_text("A\n", encoding="utf-8")
rc, o, err = run(f'DUALBOARD_DIR=/home/user/maprun/dualboard; source "{posix(DB)}/overlay.sh"')
dropin = (DB / "systemd/smartnav-hmi.service.d/50-dualboard.conf").read_text(encoding="utf-8")
vals = dict(re.findall(r"^Environment=([A-Z_]+)=(.*)$", dropin, re.M))
check(vals.get("ROS_DOMAIN_ID") == o["DOMAIN"], f"ROS_DOMAIN_ID：drop-in {vals.get('ROS_DOMAIN_ID')} ＝ overlay {o['DOMAIN']}")
check(vals.get("ROS_AUTOMATIC_DISCOVERY_RANGE") == o["RANGE"], "ROS_AUTOMATIC_DISCOVERY_RANGE 一致")
check(vals.get("CYCLONEDDS_URI") == o["URI"], "CYCLONEDDS_URI 一致（路徑都是 /home/user/maprun/dualboard/…）")
check(vals.get("SMARTNAV_DUALBOARD") == o["DUAL"], "SMARTNAV_DUALBOARD 一致")
check(vals.get("SMARTNAV_BOARD") == "A", "drop-in 有 SMARTNAV_BOARD=A（HMI 在 A）")

print("── 5. dualboard_mode.sh ──")
fake_env = TMP / "env.sh"
fake_env.write_text("export ROS_DOMAIN_ID=0\nexport ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST\nexport CYCLONEDDS_URI=file:///home/user/cyclonedds.xml\n" + BLOCK, encoding="utf-8")
ROLE.unlink(missing_ok=True)


def mode(arg):
    e = {**os.environ, "SMARTNAV_ROLE_FILE": posix(ROLE), "DUALBOARD_DIR": posix(OVERLAY_DIR), "ENV_SH": posix(fake_env)}
    r = subprocess.run([BASH, str(DB / "dualboard_mode.sh"), arg], capture_output=True, text=True, encoding="utf-8", env=e)
    return r.returncode, r.stdout


rc, out = mode("A")
check(rc == 0 and ROLE.read_text().strip() == "A" and "ROS_DOMAIN_ID=42" in out, "A：寫標記檔、status 顯示 domain 42")
rc, out = mode("B")
check(rc == 0 and ROLE.read_text().strip() == "B" and "SMARTNAV_BOARD=B" in out, "B：改寫成 B")
rc, out = mode("off")
check(rc == 0 and not ROLE.exists() and "ROS_DOMAIN_ID=0" in out, "off：刪標記檔、status 回 domain 0")
rc, out = mode("status")
check(rc == 0 and "沒有＝單板" in out, "status：沒有標記檔時說「沒有＝單板」")
rc, out = mode("C")
check(rc != 0 and "用法" in out, "亂給參數：回用法、非 0 結束")

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
