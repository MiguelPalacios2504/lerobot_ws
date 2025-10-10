from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("lerobot", package_name="lerobot_moveit")
        .to_moveit_configs()
    )
    node = Node(
        package="lerobot_cpp_examples",
        executable="simple_moveit_interface",
        parameters=[moveit_config.to_dict()],   # robot_description, srdf, kinematics, etc.
        output="screen",
    )
    return LaunchDescription([node])
