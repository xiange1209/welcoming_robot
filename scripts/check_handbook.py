#!/usr/bin/env python
"""制度檔健康檢查（Windows 上請用 `python scripts/check_handbook.py` 執行）

檢查四件事，全部通過 exit 0，任一 FAIL exit 1（WARN 不影響 exit code）：
1. 行數上限：CLAUDE.md ≤150、.claude/handbook/ 編號檔各 ≤120、memory/MEMORY.md ≤40
2. 最小結構：規則檔必須以「# 」標題開頭且 ≥15 行（防「內容蒸發成空檔」仍通過檢查）
3. 禁用語句：規則檔內不得出現模糊留白詞；「」『』"" 引號內的出現視為引用範例，豁免
4. 路由完整性：CLAUDE.md 的「⚡ 每個 session 先讀」區塊必須提及 handbook 每一份編號檔，
   且路由區提到的編號檔都實際存在

audit-*.md（審計報告）與 backups/ 不在檢查範圍；HANDOFF.md 只做禁語檢查。
MEMORY.md 缺席時記 WARN（可能是別台機器 clone，非制度損壞）。
"""

import sys
import re
from pathlib import Path

# Windows console 預設 CP950，中文輸出會變亂碼——強制 UTF-8（本專案實測教訓）
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

ROOT = Path(__file__).resolve().parent.parent
HANDBOOK = ROOT / ".claude" / "handbook"
MEMORY_DIR = Path.home() / ".claude" / "projects" / "c--Users-----Desktop---" / "memory"
MEMORY_MD = MEMORY_DIR / "MEMORY.md"

FORBIDDEN = ["視情況", "用你的判斷", "保持高品質", "適當地", "注意平衡"]

failures = []
warnings = []


def read_utf8(path):
    """讀檔；非 UTF-8 給可讀的 FAIL 而不是 traceback（PowerShell 5.1 預設 UTF-16 是已知坑）"""
    try:
        return path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        failures.append(f"[編碼] {path.name} 不是 UTF-8（可能被 PowerShell 以 UTF-16 覆寫）")
        return None


def numbered_handbook_files():
    return sorted(p for p in HANDBOOK.glob("[0-9]*.md"))


def check_lines_and_structure():
    limits = {ROOT / "CLAUDE.md": 150}
    for p in numbered_handbook_files():
        limits[p] = 120
    if MEMORY_MD.exists():
        limits[MEMORY_MD] = 40
    else:
        warnings.append(f"[WARN] MEMORY.md 不存在（{MEMORY_MD}）——若在原機器上出現代表記憶遺失")
    for path, limit in limits.items():
        if not path.exists():
            failures.append(f"[行數] 檔案不存在: {path}")
            continue
        text = read_utf8(path)
        if text is None:
            continue
        lines = text.splitlines()
        if len(lines) > limit:
            failures.append(f"[行數] {path.name}: {len(lines)} 行，超過上限 {limit}")
        # MEMORY.md 是索引檔，短是健康的；15 行下限只適用於規則檔
        min_lines = 3 if path == MEMORY_MD else 15
        if len(lines) < min_lines or not text.lstrip().startswith("# "):
            failures.append(f"[結構] {path.name}: 少於 {min_lines} 行或缺「# 」標題——內容可能被清空或截斷")


def is_quoted(line, word):
    """該行中 word 的每一次出現都被引號（「」『』"）緊鄰包住才算引用範例"""
    quoted = re.findall(r"[「『\"]([^「『」』\"]*)[」』\"]", line)
    return line.count(word) <= sum(q.count(word) for q in quoted)


def check_forbidden():
    targets = [ROOT / "CLAUDE.md", *numbered_handbook_files(), HANDBOOK / "HANDOFF.md"]
    if MEMORY_DIR.exists():
        targets += sorted(MEMORY_DIR.glob("*.md"))
    for path in targets:
        if not path.exists():
            continue
        text = read_utf8(path)
        if text is None:
            continue
        for i, line in enumerate(text.splitlines(), 1):
            for word in FORBIDDEN:
                if word in line and not is_quoted(line, word):
                    failures.append(f"[禁語] {path.name}:{i} 出現「{word}」（未加引號，非引用範例）")


def check_routing():
    claude_md = ROOT / "CLAUDE.md"
    text = read_utf8(claude_md) if claude_md.exists() else None
    if text is None:
        failures.append("[路由] CLAUDE.md 不存在或不可讀")
        return
    m = re.search(r"## ⚡[^\n]*\n(.*?)(?=\n## )", text, re.S)
    if not m:
        failures.append("[路由] CLAUDE.md 找不到「## ⚡ 每個 session 先讀」區塊——路由入口被刪")
        return
    section = m.group(1)
    expected = numbered_handbook_files()
    if not expected:
        failures.append(f"[路由] {HANDBOOK} 下找不到任何編號規則檔")
    for p in expected:
        if p.name not in section:
            failures.append(f"[路由] 路由區未提及 {p.name}（新檔要加進路由區）")
    for name in re.findall(r"\d\d-[a-z-]+\.md", section):
        if not (HANDBOOK / name).exists():
            failures.append(f"[路由] 路由區提到 {name} 但檔案不存在")


check_lines_and_structure()
check_forbidden()
check_routing()

for w in warnings:
    print(w)
if failures:
    print("FAIL：")
    for f in failures:
        print("  " + f)
    sys.exit(1)
print("OK：行數/結構/禁語/路由 全部通過")
