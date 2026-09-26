"""akm_realism 的換算規則（純函式，不需要 ROS 就能跑：python -m pytest test/）"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:
    from smartnav_sim.akm_realism_node import shape_command
except ImportError:  # 沒有 rclpy 的環境（Windows）：只載入純函式
    import importlib.util
    import types

    for mod in ("rclpy", "rclpy.node", "geometry_msgs", "geometry_msgs.msg"):
        sys.modules.setdefault(mod, types.ModuleType(mod))
    sys.modules["rclpy.node"].Node = object
    sys.modules["geometry_msgs.msg"].Twist = object
    spec = importlib.util.spec_from_file_location(
        "akm", os.path.join(os.path.dirname(__file__), "..", "smartnav_sim", "akm_realism_node.py"))
    akm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(akm)
    shape_command = akm.shape_command

REAL = dict(deadzone=0.085, r_fw=0.750, bias=0.235, r_left=1.030, r_right=1.183)
IDEAL = dict(deadzone=0.0, r_fw=0.750, bias=0.235, r_left=0.750, r_right=0.750)
TRIM = -0.235   # steering_trim_cc 的預設補償（rad/m）


def run(v, w, cfg=REAL):
    return shape_command(v, w, **cfg)


def test_deadzone_stops_the_car():
    # 8/24 實測：平均指令 +0.080 m/s 時車子不動
    assert run(0.080, 0.1) == (0.0, 0.0, "deadzone")
    assert run(-0.080, 0.0) == (0.0, 0.0, "deadzone")


def test_zero_command_is_not_counted_as_deadzone():
    assert run(0.0, 0.0) == (0.0, 0.0, None)
    assert run(0.0, 0.3, IDEAL) == (0.0, 0.0, None)   # 阿克曼：v=0 不能原地轉


def test_just_above_deadzone_moves():
    # cmd_vel_floor_cc 把速度抬到 0.095 就是為了過這個死區
    v, _, ev = run(0.095, 0.0)
    assert v == 0.095 and ev is None


def test_mech_bias_drifts_left_without_trim():
    v, w, _ = run(0.2, 0.0)
    assert w == pytest.approx(0.2 * 0.235)            # 下 0 卻往左偏


def test_steering_trim_cancels_mech_bias():
    # 導航鏈 steering_trim 送 ω += −0.235·v；經過底盤偏移 +0.235 後應該剛好走直線
    v = 0.2
    _, w, _ = run(v, TRIM * v)
    assert w == pytest.approx(0.0, abs=1e-12)


def test_reverse_flips_yaw_sign():
    # 8/29 實測：前進 +13.46 度/m、倒車 −15.70 度/m（反號）
    _, w_fwd, _ = run(0.2, 0.0)
    _, w_rev, _ = run(-0.2, 0.0)
    assert w_fwd > 0 and w_rev < 0


def test_left_turn_clamped_to_measured_radius():
    v = 0.2
    _, w, ev = run(v, v / 0.80)            # 規劃器用 0.80 m 半徑
    assert ev == "clamp_left"
    assert v / w == pytest.approx(1.030)   # 實車左轉最小 1.030


def test_right_turn_clamped_to_measured_radius():
    v = 0.2
    _, w, ev = run(v, -v / 0.80)
    assert ev == "clamp_right"
    assert v / w == pytest.approx(-1.183)


def test_right_turn_limit_is_wider_than_left():
    # 左右不對稱是實測重點：右轉比左轉更轉不過去
    v = 0.2
    _, wl, _ = run(v, v / 0.5)
    _, wr, _ = run(v, -v / 0.5)
    assert abs(v / wr) > abs(v / wl)


def test_gentle_turn_passes_with_bias_only():
    v = 0.2
    _, w, ev = run(v, v / 3.0)             # 半徑 3 m，遠大於極限
    assert ev is None
    assert w / v == pytest.approx(1 / 3.0 + 0.235)


def test_ideal_mode_keeps_bias_but_symmetric_firmware_limit():
    v = 0.2
    _, w, ev = run(v, v / 0.5, IDEAL)
    # 韌體先截到 1/0.75，再加偏移，再被 1/0.75 的機構極限截住
    assert ev == "clamp_left"
    assert v / w == pytest.approx(0.750)
    v2, _, _ = run(0.01, 0.0, IDEAL)       # 理想車沒有死區
    assert v2 == 0.01


def test_reverse_turn_uses_steering_side_not_yaw_side():
    # 倒車時舵往左打（κ>0）一樣受「左轉」極限限制，雖然車身是往右轉
    v = -0.2
    _, w, ev = run(v, v / 0.80)            # κ = +1.25
    assert ev == "clamp_left"
    assert w / v == pytest.approx(1 / 1.030)
    assert w < 0


def test_curvature_preserved_scaling():
    # ω/v 只由舵角決定：同一個曲率、不同速度，輸出曲率一樣
    _, w1, _ = run(0.15, 0.15 * 0.5)
    _, w2, _ = run(0.40, 0.40 * 0.5)
    assert w1 / 0.15 == pytest.approx(w2 / 0.40)
    assert math.isfinite(w1) and math.isfinite(w2)
