import os
import sys

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_LAUNCH_DIR = os.path.join(get_package_share_directory("lerobot_moveit"), "launch")
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

_DESC_LAUNCH_DIR = os.path.join(get_package_share_directory("lerobot_description"), "launch")
if _DESC_LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _DESC_LAUNCH_DIR)

from jazzy_compat import ignition_xacro_arg  # noqa: E402
from moveit_config_loader import move_group_parameters, rviz_moveit_parameters  # noqa: E402


def generate_launch_description():
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="false",
        description="true para simulación (usa /clock), false para hardware real",
    )
    is_sim = LaunchConfiguration("is_sim")
    is_ignition = ignition_xacro_arg()

    pkg_moveit = get_package_share_directory("lerobot_moveit")
    controllers_hw_path = os.path.join(pkg_moveit, "config", "moveit_controllers.yaml")
    controllers_sim_path = os.path.join(pkg_moveit, "config", "moveit_controllers_sim.yaml")

    with open(controllers_hw_path, "r", encoding="utf-8") as handle:
        moveit_controllers_hw = yaml.safe_load(handle)
    with open(controllers_sim_path, "r", encoding="utf-8") as handle:
        moveit_controllers_sim = yaml.safe_load(handle)

    common_move_group_params = move_group_parameters(is_sim, is_ignition) + [
        {"planning_scene_monitor.use_robot_state_topic": True},
        {"planning_scene_monitor.publish_robot_description": True},
        {"planning_scene_monitor.publish_planning_scene": True},
        {"allow_trajectory_execution": True},
    ]

    move_group_hw = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common_move_group_params + [moveit_controllers_hw, {"use_sim_time": False}],
        arguments=["--ros-args", "--log-level", "info"],
        condition=UnlessCondition(is_sim),
    )

    move_group_sim = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common_move_group_params + [moveit_controllers_sim, {"use_sim_time": True}],
        arguments=["--ros-args", "--log-level", "info"],
        condition=IfCondition(is_sim),
    )

    rviz_config_hw = os.path.join(pkg_moveit, "config", "moveit.rviz")
    rviz_config_sim = os.path.join(pkg_moveit, "config", "moveit_sim.rviz")
    common_rviz_params = rviz_moveit_parameters(is_sim, is_ignition)

    rviz_hw = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_hw],
        parameters=common_rviz_params + [{"use_sim_time": False}],
        condition=UnlessCondition(is_sim),
    )

    rviz_sim = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_sim],
        parameters=common_rviz_params + [{"use_sim_time": True}],
        condition=IfCondition(is_sim),
    )

    delayed_move_group_sim = TimerAction(
        period=3.0,
        actions=[move_group_sim],
        condition=IfCondition(is_sim),
    )

    return LaunchDescription([
        is_sim_arg,
        move_group_hw,
        delayed_move_group_sim,
        rviz_hw,
        rviz_sim,
    ])
