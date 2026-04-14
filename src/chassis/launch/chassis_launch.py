import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('chassis'),
        'config',
        'chassis_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='chassis',
            executable='chassis_node',
            name='chassis_node',
            parameters=[config],
            output='screen',
        ),
    ])
