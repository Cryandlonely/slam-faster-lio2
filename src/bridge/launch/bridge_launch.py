import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bridge'),
        'config',
        'bridge_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='bridge',
            executable='bridge_node',
            name='bridge_node',
            parameters=[config],
            output='screen',
        ),
    ])
