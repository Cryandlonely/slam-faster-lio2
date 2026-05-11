#!/usr/bin/env python3
"""
调试用启动文件 —— 同时启动 RTK 节点、导航节点和 RViz

用法:
  ros2 launch nav_planner slam_nav_debug.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg     = get_package_share_directory('nav_planner')
    pkg_rtk = get_package_share_directory('rtk')

    nav_config  = os.path.join(pkg,     'config', 'slam_nav_params.yaml')
    rviz_cfg    = os.path.join(pkg,     'rviz',   'nav_debug.rviz')

    # 通过 include rtk_launch.py 启动 RTK 节点, 复用其内置的 USB 串口动态检测
    rtk_launch_file = os.path.join(pkg_rtk, 'launch', 'rtk_launch.py')
    rtk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtk_launch_file),
    )

    slam_nav_node = Node(
        package='nav_planner',
        executable='slam_nav_node',
        name='slam_nav_node',
        output='screen',
        parameters=[nav_config],
    )

    static_tf_body_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.4', '0', '0', '0', '0', '0', 'body', 'base_link'],
        output='screen',
    )

    static_tf_body_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'livox_frame'],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
    )

    return LaunchDescription([
        static_tf_body_to_base,
        static_tf_body_to_livox,
        rtk_launch,
        slam_nav_node,
        rviz_node,
    ])
