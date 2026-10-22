# -*- coding: utf-8 -*-
"""2026-10-22：雷射左右偏移（laser_y_offset_m）的幾何測試。

背景：10/20 三次自身回波量測，兩側鋁擠條死角不對稱（左 29.1°／右 20.8°，中心鏡像誤差 2°）→ 推算雷射偏左約 2.5 cm；
使用者 10/22 說「好像的確光達位置不是正中間」。廠商 robot_model.yaml 寫左右只偏 0.67 mm，path_teach 的所有
車身矩形原本也都假設雷射在中線上。偏左 2.5 cm 的後果：左側淨空被低估、**右側被高估 2.5 cm**。

測什麼（用替身物件呼叫真正的方法，不需要 ROS）
  ✓ laser_y = 0（預設）時，每個函式的結果與舊行為完全相同（同一批掃描點、同一個數字）
  ✓ laser_y = +0.025 時：右側的點離車身「更近」2.5 cm、左側「更遠」2.5 cm（_body_margin、_side_clearance、_end_gaps）
  ✓ _arc_clearance、_forward_clearance 的車寬帶：邊緣的點因偏移而進／出
  ✓ 宣告預設值 0.0、類別預設 0.0、map_service 同名參數
不測：偏移的數值本身（要用尺量；見 分析_實車20261021.md §六）。

用法：python -X utf8 maprun/tools_1021/test_laser_offset.py
"""
import ast
import importlib.abc
import importlib.machinery
import importlib.util
import math
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
MS_PATH = CAR / "src/smartnav_navigation_cc/smartnav_navigation_cc/map_service_cc_node.py"

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
spec = importlib.util.spec_from_file_location("pt_laser", PT_PATH)
pt = importlib.util.module_from_spec(spec)
sys.modules["pt_laser"] = pt
spec.loader.exec_module(pt)
PT = pt.PathTeachNode
results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


class Scan:
    """只放一個點的雷射：angle_min＝該點方向、只有 1 束"""

    def __init__(self, r, deg):
        self.angle_min = math.radians(deg)
        self.angle_increment = 0.01
        self.ranges = [r]
        self.range_min = 0.05
        self.range_max = 50.0


def obj(laser_y=None, scan=None):
    o = object.__new__(PT)
    o.laser_x = 0.089
    o.obs_half_w = 0.22
    if laser_y is not None:
        o.laser_y = laser_y
    o._get_scan = lambda: scan
    return o


def point_at(x, y):
    """雷射座標的 (x, y) → (r, deg)"""
    return math.hypot(x, y), math.degrees(math.atan2(y, x))


print("── 1. 預設值與接線 ──")
check(PT.laser_y == 0.0, "類別預設 laser_y = 0.0（沒經過 __init__ 的替身物件＝舊行為）")
tree = ast.parse(PT_PATH.read_text(encoding="utf-8"))
decl = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
        and n.func.attr == "declare_parameter" and n.args and isinstance(n.args[0], ast.Constant)
        and n.args[0].value == "laser_y_offset_m"]
check(len(decl) == 1 and decl[0].args[1].value == 0.0, "path_teach 宣告 laser_y_offset_m，預設 0.0")
ms = ast.parse(MS_PATH.read_text(encoding="utf-8"))
decl2 = [n for n in ast.walk(ms) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
         and n.func.attr == "declare_parameter" and n.args and isinstance(n.args[0], ast.Constant)
         and n.args[0].value == "laser_y_offset_m"]
check(len(decl2) == 1 and decl2[0].args[1].value == 0.0, "map_service 也宣告 laser_y_offset_m，預設 0.0")

print("── 2. _body_margin：右側的牆更近、左側更遠 ──")
# 一堵牆在雷射座標 y = ∓0.20、x = 0.10（車身中段）：舊模型離車身 1.5 cm
for side, y in (("右", -0.20), ("左", +0.20)):
    r, deg = point_at(0.10, y)
    sc = Scan(r, deg)
    _, any0 = PT._body_margin(obj(0.0, sc), 1)
    _, anyn = PT._body_margin(obj(None, sc), 1)     # 沒設 laser_y（類別預設）
    _, any1 = PT._body_margin(obj(0.025, sc), 1)
    check(close(any0, 0.015, 1e-6) and close(anyn, any0), f"{side}側點（y={y:+.2f}）：laser_y=0 → 淨空 1.5 cm，且與沒設時完全相同")
    want = 0.0 if side == "右" else 0.040
    check(close(any1, want, 1e-6), f"{side}側點：雷射偏左 2.5 cm → 淨空 {want*100:.1f} cm（右＝已壓到車身、左＝多 2.5 cm）")

print("── 3. _side_clearance／_end_gaps ──")
for side, y in (("右", -0.30), ("左", +0.30)):
    # 側面：x 在車身縱向範圍內（base 座標 -0.09~0.40；laser_x = 0.089 → 雷射 x -0.179~0.311）
    r, deg = point_at(0.05, y)
    sc = Scan(r, deg)
    l0, r0 = PT._side_clearance(obj(0.0, sc))
    l1, r1 = PT._side_clearance(obj(0.025, sc))
    got0, got1 = (r0, r1) if side == "右" else (l0, l1)
    check(close(got0, 0.115, 1e-6), f"{side}側牆 y=0.30：laser_y=0 → 空隙 11.5 cm")
    check(close(got1, 0.115 - 0.025 if side == "右" else 0.115 + 0.025, 1e-6),
          f"{side}側牆：雷射偏左 2.5 cm → 空隙 {'9.0' if side == '右' else '14.0'} cm")
# 車頭前方偏右 / 偏左一點的點：_end_gaps 的 oy
for side, y in (("右", -0.22), ("左", +0.22)):
    r, deg = point_at(0.40, y)                 # 雷射座標 x=0.40 → base x=0.489 > 車頭 0.40
    sc = Scan(r, deg)
    f0, _ = PT._end_gaps(obj(0.0, sc))
    f1, _ = PT._end_gaps(obj(0.025, sc))
    check(f0 < float("inf") and f1 < float("inf") and (f1 < f0 if side == "右" else f1 > f0),
          f"車頭前方{side}側點：雷射偏左時，{'右' if side == '右' else '左'}邊的距離{'變近' if side == '右' else '變遠'}（{f0:.3f}→{f1:.3f} m）")

print("── 4. _arc_clearance／_forward_clearance 的車寬帶（obs_half_w = 0.22） ──")
# 正前方 0.5 m、側向 0.23：舊模型在帶外（> 0.22）；雷射偏左 2.5 cm → 右側點變成 −0.205（帶內）、左側點變 0.255（帶外更遠）
for side, y in (("右", -0.23), ("左", +0.23)):
    sc = Scan(*point_at(0.5, y))
    a0 = PT._arc_clearance(obj(0.0, sc), 1, 0.0, max_dist=1.2)
    a1 = PT._arc_clearance(obj(0.025, sc), 1, 0.0, max_dist=1.2)
    f0 = PT._forward_clearance(obj(0.0, sc), 1) if hasattr(PT, "_forward_clearance") else None
    check(a0 == 1.2 and (a1 < 1.2 if side == "右" else a1 == 1.2),
          f"_arc_clearance {side}側 {abs(y):.2f} m：laser_y=0 帶外（{a0:.2f}）→ 偏左 2.5 cm 後{'帶內、會擋（%.2f m）' % a1 if side == '右' else '仍帶外（%.2f）' % a1}")
    if f0 is not None:
        g0 = f0
        g1 = PT._forward_clearance(obj(0.025, sc), 1)
        check((g0 == float("inf") and g1 < float("inf")) if side == "右" else (g0 == float("inf") and g1 == float("inf")),
              f"_forward_clearance {side}側：同上（{g0} → {g1}）")

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
