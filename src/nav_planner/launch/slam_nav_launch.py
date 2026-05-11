#!/usr/bin/env python3
"""
SLAM 导航模块启动文件

启动:
  1. rtk_node      - RTK 串口读取 + 发布 /outdoor/odom
  2. slam_nav_node  - 导航规划+跟踪节点 (GPS模式直接订阅 /outdoor/odom)
  3. static_transform - body → base_link / body → livox_frame 静态TF
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav_config = os.path.join(
        get_package_share_directory('nav_planner'),
        'config',
        'slam_nav_params.yaml')

    # 通过 include rtk_launch.py 启动 RTK 节点, 复用其内置的 USB 串口动态检测
    rtk_launch_file = os.path.join(
        get_package_share_directory('rtk'),
        'launch',
        'rtk_launch.py')

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

    return LaunchDescription([
        static_tf_body_to_base,
        static_tf_body_to_livox,
        rtk_launch,
        slam_nav_node,
    ])
