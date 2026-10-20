# -*- coding: utf-8 -*-
"""2026-10-21：導航修補（commit 5aa79cf，車上 agent 10/20 提案 A–D）的行為與接線測試。

背景：10/20 14:50:17~14:52:58 門口→起點，161 秒內 1,040 個控制週期處於外緣硬停，硬停逾時一次都沒觸發
（「一拍放行就歸零」），之後脫困把車帶離路徑 1.2 m 楔死，重新規劃第一次失敗就整趟放棄。

測什麼
  ✓ _margin_release（硬停事件的解除條件，純函式）：舊行為等價（兩個 resume 參數都 0）、遲滯、中斷重算、推進解除
  ✓ _refresh_margin_params：margin_stop_timeout_sec <= 0 時沿用 no_progress × 2；負值夾到 0
  ✓ 接線（AST）：逐拍紀錄開／寫／關各一處、「一拍放行就歸零」那行已拿掉、事件中不脫困有「最後 3 點除外」、
    navigation_action 每次導航重讀 replan_fail_retry_sec
  ✗ 沒跑完整 _follow_loop（要 TF／掃描／action 替身），迴圈內的行為靠 Gazebo（backups/研究數據/模擬_導航修補_20261021/）與實車

用法：python -X utf8 maprun/tools_1021/test_nav_replan.py
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
NA_PATH = CAR / "src/smartnav_navigation_cc/smartnav_navigation_cc/navigation_action_cc_node.py"

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
spec = importlib.util.spec_from_file_location("pt_1021", PT_PATH)
pt = importlib.util.module_from_spec(spec)
sys.modules["pt_1021"] = pt
spec.loader.exec_module(pt)
PT = pt.PathTeachNode
rel = PT._margin_release
results = []


def check(cond, msg):
    results.append(bool(cond))
    print(("  ✓ " if cond else "  ✗ ") + msg)


STOP = 0.06
print("── 1. 舊行為等價（margin_resume_m=0、margin_resume_sec=0） ──")
clear, why = rel(100.0, 0.0, 0.07, STOP, 0.0, 0.0, 0.0, 0.10)
check(bool(why), f"第一拍 margin 7 cm > 6 cm 就解除（{why}）")
clear, why = rel(100.0, 0.0, 0.06, STOP, 0.0, 0.0, 0.0, 0.10)
check(not why and clear == 0.0, "margin 剛好 6 cm（= 門檻）不解除")

print("── 2. 遲滯（預設 +3 cm、連續 1.0 s） ──")
c, w = rel(10.0, 0.0, 0.08, STOP, 0.03, 1.0, 0.0, 0.10)
check(not w and c == 0.0, "8 cm（< 9 cm）不算放行，連續起點不起算")
c, w = rel(10.0, 0.0, 0.10, STOP, 0.03, 1.0, 0.0, 0.10)
check(not w and c == 10.0, "10 cm：記下連續起點 10.0，還不解除")
c2, w = rel(10.5, c, 0.10, STOP, 0.03, 1.0, 0.0, 0.10)
check(not w and c2 == 10.0, "0.5 s 後仍不解除")
c3, w = rel(11.0, c2, 0.10, STOP, 0.03, 1.0, 0.0, 0.10)
check(bool(w), f"滿 1.0 s 解除（{w}）")

print("── 3. 中間掉下去要重算（10/20 是「硬停／放行逐拍交錯」） ──")
c, _ = rel(20.0, 0.0, 0.10, STOP, 0.03, 1.0, 0.0, 0.10)
c, w = rel(20.6, c, 0.08, STOP, 0.03, 1.0, 0.0, 0.10)
check(c == 0.0 and not w, "0.6 s 時掉到 8 cm → 連續起點歸零")
c, w = rel(20.7, c, 0.10, STOP, 0.03, 1.0, 0.0, 0.10)
c, w = rel(21.6, c, 0.10, STOP, 0.03, 1.0, 0.0, 0.10)
check(not w, "從 20.7 重新起算，21.6 還不到 1 秒")
c, w = rel(21.7, c, 0.10, STOP, 0.03, 1.0, 0.0, 0.10)
check(bool(w), "21.7 滿 1 秒才解除")

print("── 4. 沿路徑推進也算恢復 ──")
c, w = rel(30.0, 0.0, 0.08, STOP, 0.03, 1.0, 0.12, 0.10)
check("推進" in w, f"推進 12 cm ≥ 10 cm → 解除（{w}）")
c, w = rel(30.0, 0.0, 0.08, STOP, 0.03, 1.0, 0.50, 0.0)
check(not w, "need_adv = 0 時不用推進這條")

print("── 5. _refresh_margin_params ──")


class _P:
    def __init__(self, v):
        self.value = v


def fake(values, no_progress=5.0):
    o = types.SimpleNamespace(no_progress=no_progress)
    o.get_parameter = lambda n: _P(values[n])
    PT._refresh_margin_params(o)
    return o


o = fake({"margin_resume_m": 0.03, "margin_resume_sec": 1.0, "margin_stop_timeout_sec": 0.0,
          "margin_block_suppress_stuck": True})
check(o.margin_stop_timeout == 10.0, f"逾時 0 → 沿用 no_progress × 2 = 10 s（實際 {o.margin_stop_timeout}）")
o = fake({"margin_resume_m": -1.0, "margin_resume_sec": -2.0, "margin_stop_timeout_sec": 6.0,
          "margin_block_suppress_stuck": False})
check(o.margin_resume_m == 0.0 and o.margin_resume_sec == 0.0, "負值夾到 0")
check(o.margin_stop_timeout == 6.0 and o.margin_block_suppress_stuck is False, "其餘照讀")

print("── 6. 接線（AST／文字） ──")
SRC = PT_PATH.read_text(encoding="utf-8")
tree = ast.parse(SRC)


def fn(name):
    return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)


def calls_in(f, attr):
    return [n for n in ast.walk(f) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == attr]


fl, ex = fn("_follow_loop"), fn("_execute_follow")
check(len(calls_in(fl, "_trace_row")) == 1, "_follow_loop 每拍寫一行逐拍紀錄（一處）")
check(len(calls_in(ex, "_trace_open")) == 1 and len(calls_in(ex, "_trace_close")) == 1, "_execute_follow 開檔一次、finally 關檔一次")
check(len(calls_in(ex, "_refresh_margin_params")) == 1, "每趟重播開始重讀硬停參數（ros2 param set 不必重啟）")
import re  # noqa: E402

check(not re.search(r"^\s*margin_stop_t = 0\.0\s+# 恢復了", SRC, re.M), "「一拍放行就歸零」那行已拿掉")
check("idx < len(pts) - 3" in ast.get_source_segment(SRC, fl), "事件中不脫困：最後 3 點除外（保留放寬容差收下）")
check("車身外緣持續不足" in SRC, "交回訊息保留「車身外緣持續不足」（navigation_action 靠這幾個字認）")
NA = NA_PATH.read_text(encoding="utf-8")
check('declare_parameter("replan_fail_retry_sec", 4.0)' in NA, "navigation_action 宣告 replan_fail_retry_sec=4.0")
check(NA.count("self.replan_fail_retry_sec = max(") == 2, "初始化一次＋每次導航重讀一次")
check("已試 {attempt}/{self.max_replan_attempts} 次" in NA, "最後的失敗訊息帶「已試 N/M 次」")

print(f"\n{sum(results)}/{len(results)} 通過")
sys.exit(0 if all(results) else 1)
