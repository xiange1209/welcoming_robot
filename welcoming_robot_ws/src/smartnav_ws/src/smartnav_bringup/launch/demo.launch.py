#!/usr/bin/env python3
"""迎賓機器人統一啟動（★ 發表當天用這一支）

## 為什麼需要這支

在此之前要開四個終端機分別跑 `~/maprun/` 底下的腳本，順序錯了就出事。
發表當天沒有第二次機會，也沒有時間慢慢排錯。

## ★★ 啟動順序是有理由的，不要改 ★★

    感測器 ──(8s)──> 導航 ──(6s)──> 視覺/大腦 ──(4s)──> 語音 ──(3s)──> LLM/HMI

1. **感測器一定要在導航之前**。反過來會進入救不回來的 mapping 死結
   （專案硬性規則第 5 條）。8 秒是等底盤串口、光達轉速穩定、TF 樹建起來。
2. **導航要在視覺之前**：`user_auth` 啟動時會去連資料庫，而 nav2 的
   lifecycle 轉換很吃 CPU，兩個搶會讓 nav2 的 activate 逾時。
3. **LLM 最後**：它啟動時會等 `create_map` / `navigate` 兩個 action server
   （見 `llm_service_node._wait_for_services`），那兩個要 nav2 起來才有。

## 用法

    ros2 launch smartnav_bringup demo.launch.py                 # 全部
    ros2 launch smartnav_bringup demo.launch.py audio:=false    # 不要語音
    ros2 launch smartnav_bringup demo.launch.py start_mode:=mapping   # 建圖模式

★ 想單獨重啟某個節點，仍然用 `~/maprun/kill_node_cc.sh` + `run_node_cc.sh`，
  **不要**把整支 launch 殺掉重來（會連感測器一起重啟，等 8 秒）。
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _inc(pkg, rel, args=None, cond=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), rel])),
        launch_arguments=list((args or {}).items()),
        condition=cond,
    )


def generate_launch_description():
    args = [
        DeclareLaunchArgument("sensors", default_value="true"),
        DeclareLaunchArgument("nav", default_value="true"),
        DeclareLaunchArgument("vision", default_value="true"),
        DeclareLaunchArgument("brain", default_value="true"),
        DeclareLaunchArgument("audio", default_value="true"),
        DeclareLaunchArgument("llm", default_value="true"),
        DeclareLaunchArgument("hmi", default_value="true"),
        DeclareLaunchArgument("with_camera", default_value="true"),
        DeclareLaunchArgument(
            "start_mode", default_value="auto",
            description="auto / mapping / localization"),
        DeclareLaunchArgument(
            "use_exploration", default_value="false",
            description="★ 兩個探索參數在下游要一起設，這裡只暴露一個避免設出矛盾組合"),
    ]

    sensors = LaunchConfiguration("sensors")
    nav = LaunchConfiguration("nav")
    start_mode = LaunchConfiguration("start_mode")
    use_expl = LaunchConfiguration("use_exploration")

    # ── 0 s：感測器（底盤、光達、相機、TF）──
    stage_sensors = GroupAction([
        LogInfo(msg="[bringup] 1/5 感測器啟動中…（導航要等它 8 秒）"),
        _inc("smartnav_navigation_cc", "launch/sensors_cc.launch.py",
             {"with_camera": LaunchConfiguration("with_camera")},
             IfCondition(sensors)),
    ])

    # ── 8 s：導航堆疊 ──
    stage_nav = TimerAction(period=8.0, actions=[
        LogInfo(msg="[bringup] 2/5 導航堆疊啟動中…"),
        _inc("smartnav_navigation_cc", "launch/nav_bringup_cc.launch.py",
             {"use_sim_time": "false", "use_rviz": "false",
              "start_mode": start_mode,
              # ★ 這兩個一定要一起給。只給 use_exploration 的話 explorer 不啟動，
              #   但 map_service_cc 仍以為要自動探索，去呼叫不存在的服務，
              #   15 秒後 create_map 失敗而且畫面上什麼都看不到。
              "use_exploration": use_expl,
              "auto_start_exploration": use_expl},
             IfCondition(nav)),
    ])

    # ── 14 s：視覺 + 大腦 ──
    stage_brain = TimerAction(period=14.0, actions=[
        LogInfo(msg="[bringup] 3/5 人臉辨識與迎賓劇本…"),
        Node(package="smartnav_vision", executable="face_embedding",
             name="face_embedding_node", output="screen",
             condition=IfCondition(LaunchConfiguration("vision"))),
        Node(package="smartnav_brain", executable="user_auth",
             name="user_auth_node", output="screen",
             condition=IfCondition(LaunchConfiguration("brain"))),
        Node(package="smartnav_brain", executable="bank_reception",
             name="bank_reception_node", output="screen",
             condition=IfCondition(LaunchConfiguration("brain"))),
    ])

    # ── 18 s：語音 ──
    # ★ voice_trigger 要先於 speech_recognizer：前者開麥克風，
    #   後者只是訂閱它發出來的音訊。反過來不會壞，只是 log 會先噴一堆等待。
    stage_audio = TimerAction(period=18.0, actions=[
        LogInfo(msg="[bringup] 4/5 語音（雙麥克風 cancel 模式）…"),
        Node(package="smartnav_audio", executable="voice_trigger",
             name="voice_trigger_node", output="screen",
             condition=IfCondition(LaunchConfiguration("audio"))),
        Node(package="smartnav_audio", executable="speech_recognizer",
             name="speech_recognizer_node", output="screen",
             condition=IfCondition(LaunchConfiguration("audio"))),
    ])

    # ── 21 s：LLM + HMI ──
    stage_top = TimerAction(period=21.0, actions=[
        LogInfo(msg="[bringup] 5/5 LLM 與平板介面…（HMI 在 :8080）"),
        Node(package="smartnav_llm", executable="llm_service",
             name="llm_service_node", output="screen",
             condition=IfCondition(LaunchConfiguration("llm"))),
        _inc("smartnav_hmi", "launch/hmi.launch.py", {},
             IfCondition(LaunchConfiguration("hmi"))),
        LogInfo(msg="[bringup] ✓ 全部啟動完成。平板開 http://<pi-ip>:8080"),
    ])

    return LaunchDescription(args + [stage_sensors, stage_nav,
                                     stage_brain, stage_audio, stage_top])
