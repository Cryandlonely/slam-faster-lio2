from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chassis',
            executable='chassis_driver_node',
            name='chassis_driver_node',
            output='screen',
        ),
    ])
