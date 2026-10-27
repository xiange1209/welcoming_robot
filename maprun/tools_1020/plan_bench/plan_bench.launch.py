"""離線規劃基準：只開 map_server ＋ planner_server（車上同一份 nav2_senior_akm_cc.yaml），不開控制器、不動車。

  - 地圖：~/.smartnav/map_database/data/<map>.yaml（預設 nav_state.json 的 last_active_map）
  - 全域成本地圖拿掉 obstacle_layer（沒有雷達；它等不到 /scan_slam 會讓規劃器一直判「成本地圖過期」）
  - TF：map → odom_combined → base_footprint 用靜態轉換頂著（規劃時用 use_start 指定起點，這個位置只是讓成本地圖起得來）
用法：ros2 launch ~/maprun/tools_1020/plan_bench/plan_bench.launch.py [map:=map_xxx] [base_tf:=false（端到端測試用）]
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def _setup(context):
    home = os.path.expanduser("~/.smartnav")
    map_id = LaunchConfiguration("map").perform(context) or json.load(open(f"{home}/nav_state.json"))["last_active_map"]
    params = os.path.join(get_package_share_directory("smartnav_navigation_cc"), "config", "nav2_senior_akm_cc.yaml")
    overlay = os.path.join(HERE, "no_obstacle_layer.yaml")
    return [
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[params, {"yaml_filename": f"{home}/map_database/data/{map_id}.yaml", "use_sim_time": False}]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
             parameters=[params, overlay]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_plan_bench",
             output="screen",
             parameters=[{"autostart": True, "node_names": ["map_server", "planner_server"], "bond_timeout": 0.0}]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="plan_bench_tf_map_odom",
             arguments=["--frame-id", "map", "--child-frame-id", "odom_combined"]),
    ] + ([] if LaunchConfiguration("base_tf").perform(context).lower() in ("false", "0") else [
        # ★ 2026-10-24：端到端測試（track_sim/e2e_fake_car.py）由假車自己發 odom_combined -> base_footprint，帶 base_tf:=false
        Node(package="tf2_ros", executable="static_transform_publisher", name="plan_bench_tf_odom_base",
             arguments=["--frame-id", "odom_combined", "--child-frame-id", "base_footprint"]),
    ])


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("map", default_value=""),
                              DeclareLaunchArgument("base_tf", default_value="true"),
                              OpaqueFunction(function=_setup)])
