#!/usr/bin/env python3
"""
SLAM 导航模块启动文件

启动:
  1. slam_nav_node - 导航规划+跟踪节点 (通过 TF 获取底盘位姿, 发布 /cmd_vel)
  2. static_transform - body → base_link 静态TF (雷达到底盘中心的安装偏移补偿)

TF 链:
  map → camera_init (transform_fusion.py 发布)
    → body            (FAST_LIO 发布)
      → base_link     (本文件的静态 TF)

nav_planner 通过 TF 查询 map → base_link 得到底盘中心在地图中的位姿。
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

    # 雷达(body)安装在底盘中心(base_link)前方 0.4m 处
    # body → base_link: base_link 在 body 的后方 0.4m, 即 x=-0.4
    static_tf_body_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.4', '0', '0', '0', '0', '0', 'body', 'base_link'],
        output='screen',
    )

    return LaunchDescription([
        static_tf_body_to_base,
        slam_nav_node,
    ])
