import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# Permite importar controller_robot_stack desde el directorio launch instalado.
_LAUNCH_DIR = os.path.join(get_package_share_directory("lerobot_controller"), "launch")
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from controller_robot_stack import make_robot_stack  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    return make_robot_stack(
        ns=LaunchConfiguration("ns").perform(context),
        uart_port=LaunchConfiguration("uart_port").perform(context),
        is_sim=LaunchConfiguration("is_sim").perform(context),
        leader_only=LaunchConfiguration("leader_only").perform(context),
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("is_sim", default_value="false"),
        DeclareLaunchArgument("ns", default_value=""),
        DeclareLaunchArgument("uart_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument(
            "leader_only",
            default_value="false",
            description="Solo joint_state_broadcaster (brazo líder en teleoperación)",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
