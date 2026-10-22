"""雙板 B：相機（人臉模式）＋人臉＋認證＋語音輸入，一次開齊。

★ 完全沿用 HMI 的 system_control.launch.py（同樣的節點名稱、參數與 launch 檔），
  只是把 HMI 平常一個一個開的四個單元，在 B 上依序開起來 —— 不另外抄一份參數，避免 A、B 兩邊漂移。
  順序照「完整迎賓故事線」情境：相機先起（Astra S 要時間），再人臉、認證，最後語音。

用法（B 上）：
    source ~/maprun/dualboard/env_dualboard.sh
    ros2 launch ~/maprun/dualboard/launch/perception_b.launch.py
平常由 systemd 的 smartnav-perception.service 開機自動執行。

★ 每個 include 都包在 GroupAction(scoped=True) 裡：system_control.launch.py 用 OpaqueFunction 在執行當下才讀
  launch 參數 unit／variant；不隔離的話，後面的 include 會蓋掉前面設的值，四個 include 全變成最後那個單元。
★ 人臉推論間隔沿用節點預設（2 秒）。10/20 實測改成 4 秒能省約 65 點 CPU，要改由筆電端決定（README 第 6 節）。
★ 2026-10-22 車上 agent 起草，只做過 py_compile 與 launch 解析檢查，尚未在雙板實機驗證。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

# (單元, 變體, 第幾秒開)
UNITS = [
    ("camera", "face", 0.0),
    ("face", "", 6.0),
    ("user_auth", "", 6.0),
    ("asr_chain", "", 10.0),
]


def generate_launch_description():
    system_control = os.path.join(
        get_package_share_directory("smartnav_hmi"), "launch", "system_control.launch.py")
    actions = []
    for unit, variant, delay in UNITS:
        group = GroupAction(
            scoped=True,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(system_control),
                launch_arguments={"unit": unit, "variant": variant}.items(),
            )],
        )
        actions.append(TimerAction(period=delay, actions=[group]) if delay > 0 else group)
    return LaunchDescription(actions)
