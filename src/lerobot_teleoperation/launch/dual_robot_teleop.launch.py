"""
Teleoperación leader/follower con dos brazos SO101.

  leader   -> mueves este brazo a mano (solo lectura de joint_states)
  follower -> imita las posiciones del leader (comando directo, baja latencia)
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_PKG_CTRL = get_package_share_directory("lerobot_controller")
_LAUNCH_DIR = os.path.join(_PKG_CTRL, "launch")
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from controller_robot_stack import make_robot_stack  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    leader_ns = LaunchConfiguration("leader_ns").perform(context)
    follower_ns = LaunchConfiguration("follower_ns").perform(context)
    leader_port = LaunchConfiguration("leader_uart_port").perform(context)
    follower_port = LaunchConfiguration("follower_uart_port").perform(context)
    offset_y = LaunchConfiguration("follower_offset_y").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)

    pkg_teleop = get_package_share_directory("lerobot_teleoperation")
    mirror = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_teleop, "launch", "teleop_mirror.launch.py")
        ),
        launch_arguments={
            "source_ns": leader_ns,
            "target_ns": follower_ns,
            "command_mode": "forward",
            "publish_deadband": "0.002",
            "smoothing_alpha": "1.0",
        }.items(),
    )

    actions = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", f"{leader_ns}/world"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", offset_y, "0", "0", "0", "0", "world", f"{follower_ns}/world"],
        ),
    ]
    actions.extend(make_robot_stack(
        ns=leader_ns,
        uart_port=leader_port,
        is_sim="false",
        leader_only="true",
        teleop_follower="false",
    ))
    actions.extend(make_robot_stack(
        ns=follower_ns,
        uart_port=follower_port,
        is_sim="false",
        leader_only="false",
        teleop_follower="true",
    ))
    actions.append(TimerAction(period=2.5, actions=[mirror]))

    if use_rviz.lower() == "true":
        rviz_config = os.path.join(pkg_teleop, "rviz", "teleop_dual.rviz")
        actions.append(TimerAction(period=3.0, actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="teleop_dual_rviz",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "leader_uart_port",
            default_value="/dev/ttyACM1",
            description="Puerto serial del brazo líder (controller)",
        ),
        DeclareLaunchArgument(
            "follower_uart_port",
            default_value="/dev/ttyACM0",
            description="Puerto serial del brazo imitador",
        ),
        DeclareLaunchArgument("leader_ns", default_value="leader"),
        DeclareLaunchArgument("follower_ns", default_value="follower"),
        DeclareLaunchArgument(
            "follower_offset_y",
            default_value="0.35",
            description="Separación visual del follower en RViz (metros, eje Y)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Abrir RViz con ambos brazos",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
