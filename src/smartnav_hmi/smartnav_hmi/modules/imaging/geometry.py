"""平面幾何小工具。

全部是純函式，沒有 ROS 節點相依，可以單獨測試。
"""

import math
from typing import Dict

from geometry_msgs.msg import Pose


def yaw_to_quaternion(yaw: float) -> Dict[str, float]:
    """把平面角度轉成四元數（只繞 Z 軸）"""
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}


def normalize_angle(a: float) -> float:
    """把角度收斂到 (-pi, pi]。

    ★ 2026-08-24：位姿外推用。不加這個的話 yaw 相減會在 ±pi 交界處
      跳出 2pi 的假位移 —— 車子在地圖上會瞬間轉半圈。
    """
    return math.atan2(math.sin(a), math.cos(a))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """從四元數取出平面角度（只看 Z 軸分量）"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def make_pose(x: float, y: float, yaw: float) -> Pose:
    """由平面座標與角度組出 geometry_msgs/Pose"""
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = 0.0
    q = yaw_to_quaternion(float(yaw))
    pose.orientation.x = q["x"]
    pose.orientation.y = q["y"]
    pose.orientation.z = q["z"]
    pose.orientation.w = q["w"]
    return pose
