import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc = get_package_share_directory("lerobot_description")
    pkg_ros_gz = get_package_share_directory("ros_gz_sim")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(pkg_desc, "urdf", "lerobot.urdf.xacro"),
        description="Path to the robot URDF/Xacro file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(pkg_desc).parent.resolve())]
    )

    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    is_ignition = "true" if ros_distro == "humble" else "false"
    physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-bullet-featherstone-plugin"

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model"), " is_ignition:=", is_ignition]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 4 empty.sdf {physics_engine}"}.items(),
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
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"  # ← corregido
        ],
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])
