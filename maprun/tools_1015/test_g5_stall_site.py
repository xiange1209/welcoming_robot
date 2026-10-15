# -*- coding: utf-8 -*-
"""T2（2026-10-15）：同一處反覆卡住 -> 放棄這條路徑、交回導航層重新規劃 的行為測試。

背景：docs/cloud/reports/T2_g5_regression.md。G5 在走廊盡頭掉頭時，脫困成功但路徑追蹤又在同一個索引卡住，
每輪約 28 秒一輪接一輪到逾時；成功的試驗多半是靠「失敗 -> 重新規劃」才通。

測什麼（以及不測什麼）
  ✓ _stall_site_exhausted／_stall_site_msg 的行為（用 G5 日誌裡實際的卡住索引序列當輸入）
  ✓ 兩個進入脫困的位置（卡住判定、停車等待後脫困）在脫困**之前**都會問這個函式（AST 檢查接線）
  ✗ 沒有跑完整的 _follow_loop（它需要 TF／掃描／action 的完整替身），所以「接線」只證明呼叫存在且順序正確，
    迴圈裡 abort 之後的行為要靠本機模擬（驗證計畫見報告）。
  ✗ 煞車／停車判定完全沒被測，因為完全沒被改（git diff 可證，見報告）。
"""
import ast
import importlib.abc
import importlib.machinery
import importlib.util
import os
import sys
import types
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # noqa: BLE001
    pass

CAR = Path(os.environ.get("SMARTNAV_REPO") or Path(__file__).resolve().parents[2])
PT_PATH = CAR / "src/smartnav_navigation_cc/smartnav_navigation_cc/path_teach_cc_node.py"

STUB_PREFIXES = (
    "rclpy", "rcl_interfaces", "std_msgs", "nav_msgs", "geometry_msgs", "sensor_msgs",
    "nav2_msgs", "lifecycle_msgs", "tf2_ros", "tf2_msgs", "tf2_geometry_msgs", "smartnav_msgs",
    "action_msgs", "std_srvs", "builtin_interfaces", "tf_transformations", "ament_index_python",
    "visualization_msgs",
)


class StubModule(types.ModuleType):
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        cls = type(name, (), {"__init__": lambda self, *a, **k: None, "__call__": lambda self, *a, **k: self})
        setattr(self, name, cls)
        return cls


class StubFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def find_spec(self, fullname, path, target=None):
        if fullname.split(".")[0] in STUB_PREFIXES:
            return importlib.machinery.ModuleSpec(fullname, self, is_package=True)
        return None

    def create_module(self, spec):
        m = StubModule(spec.name)
        m.__path__ = []
        return m

    def exec_module(self, module):
        pass


sys.meta_path.insert(0, StubFinder())
sys.path.insert(0, str(CAR / "src/smartnav_navigation_cc"))
spec = importlib.util.spec_from_file_location("pt_t2", PT_PATH)
pt = importlib.util.module_from_spec(spec)
sys.modules["pt_t2"] = pt
spec.loader.exec_module(pt)
PT = pt.PathTeachNode
SRC = PT_PATH.read_text(encoding="utf-8")

results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


def node(limit=4, tol=2):
    o = PT.__new__(PT)
    o.stall_site_limit, o.stall_site_idx_tol = limit, tol
    return o


def feed(o, idxs):
    """依序餵『判定卡住』的路徑索引，回傳第一次被要求放棄時是第幾次（沒有 = None）"""
    site = {"idx": None, "n": 0}
    for k, i in enumerate(idxs, 1):
        if o._stall_site_exhausted(site, i):
            return k, site
    return None, site


print("── 1. 用 G5 日誌的真實卡住索引序列 ──")
# TRI_HEAD 試驗3：卡住索引 7/88 連續 5 次（日誌 45.3、108.7、135.9、164.3、192.2 秒）
n, _ = feed(node(), [7, 7, 7, 7, 7])
check(n == 4, f"HEAD 試驗3（索引 7 連續卡住）：第 4 次判定卡住時放棄（實際第 {n} 次），日誌裡是拖到第 6+ 次還在原處")
# TRI_BASE 試驗3：索引 11/92 共 3 次後自己脫出
n, _ = feed(node(), [11, 11, 11])
check(n is None, "BASE 試驗3（索引 11 卡 3 次後自己脫出）：不受影響（BASE 最多 3 次就脫出的路徑不會被誤殺）")
# TRI_BASE 試驗1：索引 11 共 4 次
n, _ = feed(node(), [11, 11, 11, 11])
check(n == 4, "BASE 試驗1（索引 11 卡 4 次，日誌最後靠朝向偏離中止才換路徑）：第 4 次（日誌 106 秒）放棄 -> 比日誌的 133 秒早交回重新規劃")

print("── 2. 『同一處』的定義 ──")
n, _ = feed(node(), [7, 8, 9, 9])
check(n == 4, "索引在 ±2 內漂動（7,8,9,9）算同一處")
n, _ = feed(node(), [7, 7, 7, 12, 12, 12])
check(n is None, "中途換到別處（索引差 5）-> 重新計數，不累計（7,7,7 然後 12,12,12 都只有 3 次）")
n, _ = feed(node(), [7, 7, 12, 7, 7])
check(n is None, "離開再回來也重新計數（保守：只在『連續』卡在同一處才放棄）")

print("── 3. 關閉與邊界 ──")
n, _ = feed(node(limit=0), [7] * 20)
check(n is None, "stall_site_replan_after=0 -> 永不放棄（回到只受 max_escapes 限制的舊行為）")
n, _ = feed(node(limit=1), [7])
check(n == 1, "limit=1 -> 第一次就放棄")

print("── 4. 放棄訊息要走『一般失敗』路徑 ──")
n, site = feed(node(), [7, 7, 7, 7])
msg = node()._stall_site_msg(site, 7, 88)
check(not msg.startswith(pt.NEW_OBSTACLE_TAG), "不以 [新障礙] 開頭（否則導航層會不計重試地一直等通道）")
check("車身外緣持續不足" not in msg, "不含『車身外緣持續不足』（否則導航層會先呼叫 /unwedge 推車）")
check("7/88" in msg and "4 次" in msg and "重新規劃" in msg, f"訊息講清楚在哪、幾次、接下來做什麼：{msg}")

print("── 5. 接線（AST）：兩個進入脫困的位置都先問、且在 escapes += 1 之前 ──")
tree = ast.parse(SRC)
loop = next(fn for cls in tree.body if isinstance(cls, ast.ClassDef) and cls.name == "PathTeachNode"
            for fn in cls.body if isinstance(fn, ast.FunctionDef) and fn.name == "_follow_loop")


def is_call(n, name):
    return isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == name


def is_escapes_inc(n):
    return isinstance(n, ast.AugAssign) and isinstance(n.target, ast.Name) and n.target.id == "escapes"


call_lines = sorted(c.lineno for c in ast.walk(loop) if is_call(c, "_stall_site_exhausted"))
inc_lines = sorted(n.lineno for n in ast.walk(loop) if is_escapes_inc(n))
check(len(call_lines) == 2, f"_follow_loop 內恰有 2 個 _stall_site_exhausted 呼叫（卡住判定、停車等待後脫困）（找到 {len(call_lines)}）")
nxt = [next((l for l in inc_lines if l > c), None) for c in call_lines]
check(all(n is not None and n - c <= 14 for c, n in zip(call_lines, nxt)),
      f"每個呼叫之後 14 行內才是 escapes += 1（先問、後脫困）：呼叫 {call_lines} -> 遞增 {nxt}")
check(sum(1 for l in inc_lines if l < call_lines[0]) == 0, "第一個呼叫之前沒有任何 escapes += 1（沒有漏接的脫困入口）")

print("── 6. 預設值 ──")
check('declare_parameter("stall_site_replan_after", 4)' in SRC, "預設 stall_site_replan_after = 4（=最多已脫困 3 次；BASE 的 3/3 成功試驗最多卡 3 次）")
check('declare_parameter("stall_site_idx_tol", 2)' in SRC, "預設 stall_site_idx_tol = 2")

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
