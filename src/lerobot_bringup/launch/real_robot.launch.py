import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_ctrl = get_package_share_directory("lerobot_controller")
    pkg_moveit = get_package_share_directory("lerobot_moveit")
    pkg_remoto = get_package_share_directory("lerobot_remoto")

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ctrl, "launch", "controller.launch.py")
        ),
        launch_arguments={"is_sim": "false"}.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, "launch", "moveit.launch.py")
        ),
        launch_arguments={"is_sim": "false"}.items(),
    )

    remote_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_remoto, "launch", "remote_interface.launch.py")
        ),
        launch_arguments={"is_sim": "false"}.items(),
    )

    return LaunchDescription([
        controller,
        moveit,
        remote_interface,
    ])
