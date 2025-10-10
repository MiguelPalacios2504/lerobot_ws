from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Argumentos
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True",
        description="Set to False to use the real robot hardware"
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("lerobot_description"),
            "urdf",
            "lerobot.urdf.xacro"
        ),
        description="Path to the robot URDF/Xacro file"
    )

    is_sim = LaunchConfiguration("is_sim")

    # Carga el URDF procesando los xacro
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    # Nodo robot_state_publisher (necesario en ambos modos)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # GUI de sliders — solo si estamos en simulación
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(is_sim)
    )

    # Nodo de control físico — solo si is_sim = False
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        condition=UnlessCondition(is_sim),
        parameters=[
            {"robot_description": robot_description},
            os.path.join(
                get_package_share_directory("lerobot_controller"),
                "config",
                "lerobot_controllers.yaml",
            ),
        ],
        output="screen",
    )

    # Spawners para los controladores físicos
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(is_sim),
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(is_sim),
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(is_sim),
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("lerobot_description"),
            "rviz",
            "display.rviz"
        )],
    )

    return LaunchDescription([
        is_sim_arg,
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        controller_manager,
        joint_state_broadcaster,
        arm_controller_spawner,
        gripper_controller_spawner,
        rviz_node,
    ])
