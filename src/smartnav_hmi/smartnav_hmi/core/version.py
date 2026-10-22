"""前端版本簽章。

為什麼要有：iPad 用 Chrome「加入主畫面」開成獨立 App，切走再切回來只是恢復、
**不會重新載入**。2026-10-20 載入的舊 JS 會一直跑，部署了新版也看不到
（10/22 就是朗讀修正部署了，平板還在跑舊版，把「165」唸成一百六十五）。

做法：後端回報目前部署的 index.html 引用了哪些 /assets/ 檔案（Vite 把內容雜湊
放在檔名裡，內容一變檔名就變），前端拿來和自己載入時的那一組比，不一樣就提示更新。
前端的對應邏輯在 frontend/src/lib/version.ts（兩邊的取法要一致：只看
type="module" 的 script 與 rel="stylesheet" 的 link、只取 /assets/ 底下的檔名）。

★ 純標準函式庫、不碰 ROS，測試直接 import：maprun/tools_1021/test_hmi_version.py。
"""

import re

_TAG_RE = re.compile(r"<(script|link)\b([^>]*)>", re.IGNORECASE)


def _attr(attrs: str, name: str) -> str:
    m = re.search(r"""\b%s\s*=\s*["']([^"']+)["']""" % name, attrs, re.IGNORECASE)
    return m.group(1) if m else ""


def _basename(url: str) -> str:
    """'/assets/index-AbC123.js?v=2#x' -> 'index-AbC123.js'；不在 /assets/ 底下回空字串"""
    path = url.split("?", 1)[0].split("#", 1)[0]
    if "/assets/" not in path:
        return ""
    return path.rsplit("/", 1)[-1]


def assets_signature(html: str) -> str:
    """index.html 引用的 /assets/ 檔名（JS 入口＋樣式表），排序後用 | 串起來。讀不到回空字串"""
    names = set()
    for m in _TAG_RE.finditer(html):
        tag, attrs = m.group(1).lower(), m.group(2)
        if tag == "script" and re.search(r"""\btype\s*=\s*["']module["']""", attrs, re.IGNORECASE):
            name = _basename(_attr(attrs, "src"))
        elif tag == "link" and re.search(r"""\brel\s*=\s*["']stylesheet["']""", attrs, re.IGNORECASE):
            name = _basename(_attr(attrs, "href"))
        else:
            continue
        if name:
            names.add(name)
    return "|".join(sorted(names))
