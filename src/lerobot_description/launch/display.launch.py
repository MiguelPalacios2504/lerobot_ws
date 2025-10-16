import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",
        description="True para simulación (RViz/Gazebo), False para robot real"
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("lerobot_description"),
            "urdf",
            "lerobot.urdf.xacro"
        ),
        description="Ruta al URDF o Xacro del robot"
    )

    is_sim = LaunchConfiguration("is_sim")

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("lerobot_description"),
        "rviz",
        "display.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    # SImulation
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(is_sim),
        output="screen",
    )

    gui_to_controller_bridge = Node(
        package="lerobot_controller",
        executable="gui_to_controller_bridge",
        name="gui_to_controller_bridge",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(is_sim),
        output="screen",
    )

    delayed_bridge = TimerAction(period=2.0, actions=[gui_to_controller_bridge])

    sim_group = GroupAction([
        joint_state_publisher_gui,
        delayed_bridge
    ])

    # Real bot
    controllers_file_hw = os.path.join(
        get_package_share_directory("lerobot_controller"),
        "config",
        "lerobot_controllers.yaml"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_file_hw,
        ],
        condition=UnlessCondition(is_sim),
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(is_sim),
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(is_sim),
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(is_sim),
        output="screen",
    )

    hardware_group = GroupAction([
        controller_manager,
        joint_state_broadcaster,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])

    return LaunchDescription([
        is_sim_arg,
        model_arg,
        robot_state_publisher,
        rviz_node,
        sim_group,
        hardware_group,
    ])
