import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # === Argumentos ===
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="False",
        description="Launch in simulation mode if True"
    )

    is_sim = LaunchConfiguration("is_sim")

    # === URDF / Xacro ===
    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(
                get_package_share_directory("lerobot_description"),
                "urdf",
                "lerobot.urdf.xacro"
            ),
            " is_sim:=", is_sim
        ]),
        value_type=str
    )

    # === Nodo: Robot State Publisher ===
    # Se usa en ambos modos, pero puede omitirse en hardware si quieres
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # === Nodo: ros2_control_node (solo hardware) ===
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": is_sim},
            os.path.join(
                get_package_share_directory("lerobot_controller"),
                "config",
                "lerobot_controllers.yaml"
            ),
        ],
        condition=UnlessCondition(is_sim),  # 🔸 Solo se ejecuta si NO es simulación
    )

    # === Spawners (solo hardware) ===
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=UnlessCondition(is_sim),
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=UnlessCondition(is_sim),
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=UnlessCondition(is_sim),
    )

    # === Descripción del lanzamiento ===
    return LaunchDescription([
        is_sim_arg,
        robot_state_publisher_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])
