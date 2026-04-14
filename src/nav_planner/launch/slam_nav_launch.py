#!/usr/bin/env python3
"""
SLAM 导航模块启动文件

启动:
  1. slam_nav_node - 导航规划+跟踪节点 (订阅 SLAM 位姿, 发布 /cmd_vel)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('nav_planner'),
        'config',
        'slam_nav_params.yaml')

    slam_nav_node = Node(
        package='nav_planner',
        executable='slam_nav_node',
        name='slam_nav_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        slam_nav_node,
    ])
