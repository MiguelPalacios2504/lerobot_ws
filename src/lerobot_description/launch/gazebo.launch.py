import os
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_LAUNCH_DIR = os.path.join(get_package_share_directory("lerobot_description"), "launch")
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from jazzy_compat import gz_physics_engine_args, ignition_xacro_arg  # noqa: E402


def generate_launch_description():
    pkg_desc = get_package_share_directory("lerobot_description")
    pkg_ros_gz = get_package_share_directory("ros_gz_sim")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(pkg_desc, "urdf", "lerobot.urdf.xacro"),
        description="Path to the robot URDF/Xacro file",
    )

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",
        description="Must be true for Gazebo simulation",
    )

    sim_controllers_config_arg = DeclareLaunchArgument(
        "sim_controllers_config",
        default_value="lerobot_controllers_sim.yaml",
        description="YAML de ros2_control cargado por el plugin de Gazebo",
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(pkg_desc).parent.resolve())],
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_sim:=", LaunchConfiguration("is_sim"),
            " is_ignition:=", ignition_xacro_arg(),
            " sim_controllers_config:=", LaunchConfiguration("sim_controllers_config"),
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 4 empty.sdf {gz_physics_engine_args()}"}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "lerobot"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]",
        ],
    )

    return LaunchDescription([
        model_arg,
        is_sim_arg,
        sim_controllers_config_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])
