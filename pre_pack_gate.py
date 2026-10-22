#!/usr/bin/env python3
"""打包前閘門：一個指令跑完所有「不需要 ROS」的檢查，全過才打包。在 車子/ 目錄執行。

    python pre_pack_gate.py             # 跑全部檢查
    python pre_pack_gate.py --quick     # 略過語法掃描（只跑測試）

為什麼要有這個：過去多次「功能靜默消失」（9/23 重構的 import 路徑、ASR 增益被改回、一鍵驗證缺 import…）
都是打包時沒人逐項跑測試。pack_car_code.py 只擋 CRLF／機密／建置產物，不跑行為測試。
本腳本把已有的替身測試與語法檢查串在一起，**任何一項失敗就以非 0 結束**。

檢查項目（找不到測試檔會標「缺」並視為失敗，避免因為檔案搬走而靜默變成「全過」）：
  1. 探索基準 206／審查基準 100／知識庫 122（測試檔放在 cloud_workspace/tests/nav_stub 或 專題/scripts/explore_watchdog）
  2. maprun/tools_1015 的測試（G5 同處卡住修補、num_ctx 接線、轉向診斷分析、曲率）與 tools_0930 的 scan_self_echo；
     tools_1021（10/21：帶位判斷、LLM 接線、導航修補、ASR 修補；10/22：雷射偏移、雙板、朗讀電話號碼、HMI 版本簽章）
  3. 全部 .py 語法（ast.parse）與 maprun 內全部 .sh 語法（bash -n）

⚠ 這些都是替身測試（沒有 ROS、沒有硬體）：全過 ≠ 上車一定正常，只代表「沒有已知的靜默失效回歸」。
"""
import ast
import os
import pathlib
import re
import shutil
import subprocess
import sys
import time

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (OSError, ValueError):
        pass

REPO = pathlib.Path(__file__).resolve().parent
TEST_HOMES = [REPO.parent / "cloud_workspace" / "tests" / "nav_stub",
              REPO.parent / "scripts" / "explore_watchdog"]


def find_test(name: str):
    for h in TEST_HOMES:
        if (h / name).is_file():
            return h / name
    return None


CHECKS = [
    ("探索基準（期望 206）", find_test("test_explore_fixes.py")),
    ("審查基準（期望 100）", find_test("test_review_fixes.py")),
    ("知識庫（期望 122）", find_test("test_knowledge_base.py")),
    ("G5 同處反覆卡住修補（16）", REPO / "maprun/tools_1015/test_g5_stall_site.py"),
    ("LLM num_ctx 接線（6）", REPO / "maprun/tools_1015/test_num_ctx.py"),
    ("轉向診斷分析（52）", REPO / "maprun/tools_1015/test_analyze.py"),
    ("曲率分析（車子版，第 8 節跳過）", REPO / "maprun/tools_1015/curvature/test_curvature.py"),
    ("scan_self_echo", REPO / "maprun/tools_0930/test_scan_self_echo.py"),
    # ★ 2026-10-21：10/20 實車之後的修補
    ("帶位判斷 nav_intent（56）", REPO / "maprun/tools_1021/test_nav_intent.py"),
    ("LLM 接線：常駐／FAQ 併入／工具精簡／帶位防呆（31）", REPO / "maprun/tools_1021/test_llm_wiring.py"),
    ("導航修補：硬停事件／重規劃重試／逐拍紀錄／每趟重讀的等待時間（28）", REPO / "maprun/tools_1021/test_nav_replan.py"),
    ("雷射左右偏移 laser_y_offset_m 的車身幾何（17）", REPO / "maprun/tools_1021/test_laser_offset.py"),
    ("雙板：A 的 HMI 不碰 B 的單元、狀態看節點圖（27）", REPO / "maprun/tools_1021/test_dualboard_hmi.py"),
    ("雙板：環境覆蓋層 env.sh／overlay／dualboard_mode（19）", REPO / "maprun/tools_1021/test_dualboard_env.py"),
    ("朗讀前的電話號碼整理 forSpeech（34，需要 Node 22.6+，沒有就跳過）", REPO / "maprun/tools_1021/test_tts_forspeech.py"),
    ("HMI 前端版本簽章 /api/version（13）", REPO / "maprun/tools_1021/test_hmi_version.py"),
    ("ASR 修補：觸發塊重送／收音緩衝／除錯錄音／launch", REPO / "maprun/tools_1021/test_asr_fixes.py"),
]


def run_test(path: pathlib.Path):
    env = {**os.environ, "SMARTNAV_REPO": str(REPO), "PYTHONUTF8": "1"}
    t0 = time.time()
    r = subprocess.run([sys.executable, "-X", "utf8", str(path)], capture_output=True, text=True,
                       encoding="utf-8", errors="replace", env=env, timeout=600)
    tail = [ln for ln in (r.stdout + r.stderr).splitlines() if ln.strip()]
    # 優先抓「N/M 通過」或「全部通過」那一行（有些測試最後一行是日誌）
    summ = [ln for ln in tail if re.search(r"\d+/\d+ 通過|全部通過", ln)]
    pick = summ[-1] if summ else (tail[-1] if tail else "（沒有輸出）")
    return r.returncode == 0, pick.strip(), time.time() - t0


def syntax_scan():
    bad = []
    n_py = n_sh = 0
    for d in ("src", "maprun"):
        for p in (REPO / d).rglob("*.py"):
            if set(p.parts) & {"node_modules", "__pycache__", "build", "install"}:
                continue
            if "frontier_exploration_ros2" in p.parts:
                continue
            n_py += 1
            try:
                ast.parse(p.read_text(encoding="utf-8"), filename=str(p))
            except (SyntaxError, UnicodeDecodeError) as e:
                bad.append(f"{p.relative_to(REPO)}: {e}")
    bash = shutil.which("bash")
    if bash:
        for p in (REPO / "maprun").rglob("*.sh"):
            n_sh += 1
            r = subprocess.run([bash, "-n", str(p)], capture_output=True, text=True, encoding="utf-8", errors="replace")
            if r.returncode != 0:
                bad.append(f"{p.relative_to(REPO)}: {r.stderr.strip()[:160]}")
    return bad, n_py, n_sh, bool(bash)


def main() -> int:
    quick = "--quick" in sys.argv
    print(f"打包前閘門｜{time.strftime('%F %T')}｜repo={REPO}")
    try:
        br = subprocess.run(["git", "-C", str(REPO), "rev-parse", "--abbrev-ref", "HEAD"], capture_output=True, text=True).stdout.strip()
        sh = subprocess.run(["git", "-C", str(REPO), "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
        print(f"分支 {br}（{sh}）")
    except OSError:
        pass
    print()
    failed = []
    for label, path in CHECKS:
        if path is None or not pathlib.Path(path).is_file():
            print(f"  ✗ 缺  {label}：找不到測試檔（找過 {', '.join(str(h) for h in TEST_HOMES)}）")
            failed.append(label)
            continue
        ok, last, dt = run_test(pathlib.Path(path))
        print(f"  {'✓' if ok else '✗'}  {label}：{last}（{dt:.1f} 秒）")
        if not ok:
            failed.append(label)
    if not quick:
        bad, n_py, n_sh, has_bash = syntax_scan()
        note = "" if has_bash else "（找不到 bash，略過 .sh 語法）"
        print(f"  {'✓' if not bad else '✗'}  語法：{n_py} 個 .py、{n_sh} 個 .sh{note}" + ("" if not bad else f"，{len(bad)} 個有問題"))
        for b in bad[:10]:
            print("       ·", b)
        if bad:
            failed.append("語法")
    print()
    if failed:
        print(f"✗ 閘門沒過（{len(failed)} 項）：{'、'.join(failed)}")
        print("  先修好再 python pack_car_code.py。")
        return 1
    print("✓ 閘門全過。下一步：python pack_car_code.py")
    print("  （替身測試全過 ≠ 上車一定正常；只代表沒有已知的靜默失效回歸）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
