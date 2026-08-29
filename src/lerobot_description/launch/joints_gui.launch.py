"""RViz + joint_state_publisher_gui: calibración del gemelo virtual.

Usa la misma convención de ángulos que MoveIt y el controlador real
(invert_joint_sign:=true en joints 2-5).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("lerobot_description")

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="false",
        description="false = URDF hardware; true = URDF simulación",
    )
    is_sim = LaunchConfiguration("is_sim")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(pkg, "urdf", "lerobot.urdf.xacro"),
            " is_sim:=", is_sim,
            " invert_joint_sign:=true",
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz_config = os.path.join(pkg, "rviz", "joints_gui.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"robot_description": robot_description}],
    )

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        is_sim_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])
