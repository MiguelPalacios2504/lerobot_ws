"""
LeRobot líder + bridge hacia Piper.

Arranca el brazo LeRobot en modo pasivo y publica /piper/joint_commands para Piper.
El feedback del Piper queda en /piper/joint_states (no usar /joint_states global).
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_PKG_CTRL = get_package_share_directory('lerobot_controller')
_LAUNCH_DIR = os.path.join(_PKG_CTRL, 'launch')
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from controller_robot_stack import make_robot_stack  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    leader_ns = LaunchConfiguration('leader_ns').perform(context)
    leader_port = LaunchConfiguration('leader_uart_port').perform(context)
    mapping_file = LaunchConfiguration('mapping_config').perform(context)
    bridge_delay = float(LaunchConfiguration('bridge_delay_sec').perform(context))

    pkg_ctrl = get_package_share_directory('lerobot_controller')
    fastdds_xml = os.path.join(pkg_ctrl, 'config', 'fastdds_no_shm.xml')

    bridge = Node(
        package='lerobot_teleoperation',
        executable='lerobot_to_piper_bridge',
        name='lerobot_to_piper_bridge',
        output='screen',
        parameters=[mapping_file],
    )

    actions = [
        SetEnvironmentVariable('ROS_DISABLE_DAEMON', '1'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_xml),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', f'{leader_ns}/world'],
        ),
    ]
    actions.extend(make_robot_stack(
        ns=leader_ns,
        uart_port=leader_port,
        is_sim='false',
        leader_only='true',
        teleop_follower='false',
    ))
    actions.append(TimerAction(period=bridge_delay, actions=[bridge]))
    return actions


def generate_launch_description():
    pkg_teleop = get_package_share_directory('lerobot_teleoperation')
    default_mapping = os.path.join(pkg_teleop, 'config', 'lerobot_to_piper_mapping.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('leader_uart_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('leader_ns', default_value='leader'),
        DeclareLaunchArgument('mapping_config', default_value=default_mapping),
        DeclareLaunchArgument('bridge_delay_sec', default_value='2.5'),
        OpaqueFunction(function=_launch_setup),
    ])
