# -*- coding: utf-8 -*-
"""2026-10-22：HMI 前端版本簽章（smartnav_hmi/core/version.py 的 assets_signature）的行為測試。

背景：iPad「加入主畫面」的 App 切走再切回來不會重新載入，10/22 就是朗讀修正部署了、平板還在跑 10/20 的舊 JS。
後端 /api/version 回報 index.html 引用的 /assets/ 檔名，前端（lib/version.ts）拿來和自己載入時的那一組比。
兩邊的取法必須一致，所以這裡測後端這一邊的規則，並用真的 build 出來的 index.html 驗一次。

測什麼
  ✓ 取 type="module" 的 script 與 rel="stylesheet" 的 link，只取 /assets/ 底下的檔名，排序後用 | 串起來
  ✓ 屬性順序不同（src 在 type 前面）、單雙引號、帶查詢字串（?v=…）都能處理
  ✓ 不該算進去的：modulepreload、icon、外部網址、不在 /assets/ 底下的檔案、一般（非 module）script
  ✓ 讀不到東西回空字串（前端把空字串視為「不知道」，不提示更新）
  ✓ 內容一變檔名就變 → 簽章就變（舊版 index.html 與新版 index.html 的簽章不同）
  ✓ 目前 build 出來的 frontend/dist/index.html 取得到簽章，且含 .js 與 .css 各一個
  ✓ api/main.py 真的有 /api/version 路由並使用這個函式（防止只改了函式、路由沒接上）
不測：瀏覽器端 DOM 的取法（lib/version.ts 的 ownSignature，要在瀏覽器裡跑）。

用法：python -X utf8 maprun/tools_1021/test_hmi_version.py
"""
import importlib.util
import os
import sys
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

CAR = Path(os.environ.get("SMARTNAV_REPO") or Path(__file__).resolve().parents[2])
HMI = CAR / "src/smartnav_hmi"
results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


spec = importlib.util.spec_from_file_location("hmi_version", HMI / "smartnav_hmi/core/version.py")
ver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ver)
sig = ver.assets_signature

print("── 1. 一般的 Vite 輸出 ──")
html = (
    '<!doctype html><html><head><meta charset="UTF-8" />'
    '<link rel="icon" type="image/svg+xml" href="/favicon.svg" />'
    '<script type="module" crossorigin src="/assets/index-AbC123.js"></script>'
    '<link rel="stylesheet" crossorigin href="/assets/index-XyZ789.css">'
    '</head><body><div id="root"></div></body></html>'
)
check(sig(html) == "index-AbC123.js|index-XyZ789.css", f"JS＋CSS 取到並排序：{sig(html)}")

print("── 2. 寫法不同 ──")
check(sig("<script src='/assets/a-1.js' type='module'></script>") == "a-1.js", "src 在 type 前面、單引號")
check(sig('<script type="module" src="/assets/a-1.js?v=20261020"></script>') == "a-1.js", "帶查詢字串（10/20 車上手改過的 ?v=…）")
check(sig('<LINK REL="stylesheet" HREF="/assets/s-2.css">') == "s-2.css", "大寫標籤與屬性")

print("── 3. 不該算進去的 ──")
check(sig('<link rel="modulepreload" href="/assets/vendor-1.js">') == "", "modulepreload 不算")
check(sig('<script src="/assets/legacy.js"></script>') == "", "沒有 type=module 的 script 不算")
check(sig('<script type="module" src="https://cdn.example.com/x.js"></script>') == "", "外部網址（不在 /assets/）不算")
check(sig('<link rel="icon" href="/assets/logo.svg">') == "", "icon 不算")
check(sig("") == "" and sig("<html></html>") == "", "沒有東西 → 空字串（前端視為不知道）")

print("── 4. 版本一換簽章就換 ──")
old = html
new = html.replace("index-AbC123.js", "index-NEW999.js")
check(sig(old) != sig(new), "JS 檔名變了 → 簽章變")
new_css = html.replace("index-XyZ789.css", "index-CSS000.css")
check(sig(old) != sig(new_css), "只有 CSS 檔名變了 → 簽章也變（JS 沒動的樣式更新也抓得到）")

print("── 5. 真的 build 出來的 dist ──")
dist = HMI / "frontend/dist/index.html"
if dist.is_file():
    real = sig(dist.read_text(encoding="utf-8"))
    parts = real.split("|") if real else []
    check(any(p.endswith(".js") for p in parts) and any(p.endswith(".css") for p in parts),
          f"dist/index.html 取得到 JS＋CSS：{real}")
else:
    print("  （frontend/dist 不存在，略過；打包前會先 build）")

print("── 6. 路由有接上 ──")
main_py = (HMI / "smartnav_hmi/api/main.py").read_text(encoding="utf-8")
check('"/api/version"' in main_py and "assets_signature" in main_py, "api/main.py 有 /api/version 並使用 assets_signature")

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
