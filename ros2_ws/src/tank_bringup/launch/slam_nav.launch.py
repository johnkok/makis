"""
slam_nav.launch.py

Starts slam_toolbox (online async mapping) and Nav2 on top of the running
hardware bringup. Start bringup.launch.py first, then this launch file.

Usage:
  ros2 launch tank_bringup slam_nav.launch.py
  ros2 launch tank_bringup slam_nav.launch.py map_dir:=/maps  mode:=localization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('tank_bringup')
    nav2_dir    = get_package_share_directory('nav2_bringup')

    mode     = LaunchConfiguration('mode')
    map_dir  = LaunchConfiguration('map_dir')

    return LaunchDescription([
        DeclareLaunchArgument('mode',    default_value='mapping',
                              description='mapping | localization'),
        DeclareLaunchArgument('map_dir', default_value='/maps'),

        # ── SLAM Toolbox ───────────────────────────────────
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                os.path.join(bringup_dir, 'config', 'slam_params.yaml'),
                {'mode': mode},
            ],
            output='screen',
        ),

        # ── Nav2 ───────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
            }.items(),
        ),
    ])
