import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_LAUNCH_DIR = os.path.join(get_package_share_directory("lerobot_moveit"), "launch")
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from moveit_config_loader import moveit_cpp_parameters  # noqa: E402


def generate_launch_description():
    is_sim_arg = DeclareLaunchArgument("is_sim", default_value="true")
    is_sim = LaunchConfiguration("is_sim")

    moveit_params = moveit_cpp_parameters(is_sim, "false")

    return LaunchDescription([
        is_sim_arg,
        Node(
            package="lerobot_cpp_examples",
            executable="simple_moveit_interface",
            parameters=[moveit_params, {"use_sim_time": is_sim}],
            output="screen",
        ),
    ])
