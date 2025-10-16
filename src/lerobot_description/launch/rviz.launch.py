import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",
        description="True para simular con Gazebo (usa /clock y /joint_states)"
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("lerobot_description"),
            "urdf",
            "lerobot.urdf.xacro"
        ),
        description="Ruta del archivo URDF/Xacro del robot"
    )

    is_sim = LaunchConfiguration("is_sim")

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True}
        ],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("lerobot_description"),
        "rviz",
        "gazebo_sync.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": True}],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        condition=IfCondition(is_sim),
    )

    return LaunchDescription([
        is_sim_arg,
        model_arg,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[gz_bridge]),
        rviz_node,
    ])
