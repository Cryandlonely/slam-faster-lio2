import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 配置文件路径 (所有参数从文件读取)
    config_file = os.path.join(
        get_package_share_directory('rtk'),
        'config',
        'rtk_params.yaml')

    # 以下 launch 参数可在命令行覆盖配置文件中的值
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='',
        description='RTK 串口设备路径 (留空则使用配置文件值)'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='',
        description='串口波特率 (留空则使用配置文件值)'
    )

    # RTK 节点: 先加载配置文件, 再按需覆盖串口参数
    rtk_node = Node(
        package='rtk',
        executable='rtk_node',
        name='rtk_node',
        output='screen',
        parameters=[
            config_file,          # 基础参数来自配置文件
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        rtk_node,
    ])
