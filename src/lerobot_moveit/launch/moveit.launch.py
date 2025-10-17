import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def generate_launch_description():
    # Argumento para ejecución real o simulada
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="false"
    )

    # === Construcción de la configuración MoveIt ===
    moveit_config = (
        MoveItConfigsBuilder("lerobot", package_name="lerobot_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("lerobot_description"),
            "urdf", "lerobot.urdf.xacro"
        ))
        .robot_description_semantic(file_path="config/lerobot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(moveit_manage_controllers=True)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # === Cargar manualmente moveit_controllers.yaml ===
    controller_path = os.path.join(
        get_package_share_directory("lerobot_moveit"),
        "config",
        "moveit_controllers.yaml"
    )
    with open(controller_path, "r") as f:
        moveit_controllers = yaml.safe_load(f)

    # === Nodo Move Group ===
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_controllers,  # ✅ Se pasa el YAML como parámetro
            {"planning_scene_monitor.use_robot_state_topic": True},
            {"planning_scene_monitor.publish_robot_description": True},
            {"planning_scene_monitor.publish_planning_scene": True},
            {"allow_trajectory_execution": True},
            {"moveit_manage_controllers": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # === RViz ===
    rviz_config = os.path.join(
        get_package_share_directory("lerobot_moveit"),
        "config",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # === Delay para asegurar que ros2_control esté listo ===
    delayed_move_group = TimerAction(period=3.0, actions=[move_group_node])

    return LaunchDescription([
        is_sim_arg,
        delayed_move_group,
        rviz_node,
    ])
