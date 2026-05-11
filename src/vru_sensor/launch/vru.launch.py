from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('vru_sensor'),
        'config', 'vru_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='vru_sensor',
            executable='vru_node',
            name='vru_node',
            output='screen',
            parameters=[config],
        )
    ])
