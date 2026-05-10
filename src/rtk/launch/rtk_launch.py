import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


# RTK 设备 USB 标识 (与脚本 scripts 中的检测规则一致)
RTK_USB_VID = 0x19D1
RTK_USB_PID = 0x0001
RTK_USB_LOCATION_HINT = ".6"   # USB 物理位置子串 (区分多个相同 VID:PID 设备)


def detect_rtk_serial_port():
    """按 VID/PID + USB location 动态查找 RTK 串口设备。

    返回 (device_path, info_msg)；未找到时 device_path 为 None。
    """
    try:
        import serial.tools.list_ports
    except ImportError:
        return None, "[RTK 串口检测] 未安装 pyserial, 跳过动态检测"

    ports = serial.tools.list_ports.comports()
    if not ports:
        return None, "[RTK 串口检测] 系统未发现任何串口设备"

    candidates = []
    for p in ports:
        if (p.vid == RTK_USB_VID and p.pid == RTK_USB_PID
                and p.location and RTK_USB_LOCATION_HINT in p.location):
            candidates.append(p)

    if not candidates:
        def _fmt(p):
            vid = f"{p.vid:04X}" if p.vid else "----"
            pid = f"{p.pid:04X}" if p.pid else "----"
            return f"{p.device}(VID={vid},PID={pid},loc={p.location})"
        listed = ", ".join(_fmt(p) for p in ports)
        return None, (
            f"[RTK 串口检测] 未找到匹配设备 "
            f"(VID={RTK_USB_VID:04X} PID={RTK_USB_PID:04X} "
            f"loc 含 '{RTK_USB_LOCATION_HINT}'); 现有: {listed}"
        )

    target = candidates[0]
    return target.device, f"[RTK 串口检测] 命中设备: {target.device} " \
                          f"(loc={target.location})"


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
        description='RTK 串口设备路径 (留空则自动检测, 检测失败回退到配置文件值)'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='',
        description='串口波特率 (留空则使用配置文件值)'
    )

    # ---- 动态串口检测 ----
    detected_port, detect_msg = detect_rtk_serial_port()

    # 组装节点参数: 基础来自 yaml; 检测到端口则追加 override
    node_parameters = [config_file]
    if detected_port:
        node_parameters.append({'serial_port': detected_port})

    rtk_node = Node(
        package='rtk',
        executable='rtk_node',
        name='rtk_node',
        output='screen',
        parameters=node_parameters,
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        LogInfo(msg=detect_msg),
        rtk_node,
    ])
