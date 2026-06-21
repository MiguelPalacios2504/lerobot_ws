import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # === Args ===
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="false",
        description="true para simulación (usa /clock), false para hardware real",
    )
    is_sim = LaunchConfiguration("is_sim")
    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    is_ignition = "true" if ros_distro == "humble" else "false"

    # === MoveIt config (URDF/SRDF/kinematics/limits) ===
    moveit_config = (
        MoveItConfigsBuilder("lerobot", package_name="lerobot_moveit")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("lerobot_description"),
                "urdf", "lerobot.urdf.xacro"
            ),
            mappings={
                "is_sim": is_sim,
                "is_ignition": is_ignition,
            },
        )
        .robot_description_semantic(file_path="config/lerobot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(moveit_manage_controllers=True)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # === Rutas de controladores MoveIt (HW y SIM separados) ===
    pkg_moveit = get_package_share_directory("lerobot_moveit")
    controllers_hw_path = os.path.join(pkg_moveit, "config", "moveit_controllers.yaml")
    controllers_sim_path = os.path.join(pkg_moveit, "config", "moveit_controllers_sim.yaml")

    with open(controllers_hw_path, "r") as f:
        moveit_controllers_hw = yaml.safe_load(f)
    with open(controllers_sim_path, "r") as f:
        moveit_controllers_sim = yaml.safe_load(f)

    # === move_group (dos variantes, cada una con su YAML y use_sim_time literal) ===
    common_move_group_params = [
        moveit_config.to_dict(),
        {"planning_scene_monitor.use_robot_state_topic": True},
        {"planning_scene_monitor.publish_robot_description": True},
        {"planning_scene_monitor.publish_planning_scene": True},
        {"allow_trajectory_execution": True},
        {"moveit_manage_controllers": True},
    ]

    move_group_hw = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common_move_group_params + [moveit_controllers_hw, {"use_sim_time": False}],
        arguments=["--ros-args", "--log-level", "info"],
        condition=UnlessCondition(is_sim),
    )

    move_group_sim = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common_move_group_params + [moveit_controllers_sim, {"use_sim_time": True}],
        arguments=["--ros-args", "--log-level", "info"],
        condition=IfCondition(is_sim),
    )

    # === RViz (HW vs SIM: en sim hay que usar moveit_sim.rviz con Use Sim Time=true) ===
    rviz_config_hw = os.path.join(pkg_moveit, "config", "moveit.rviz")
    rviz_config_sim = os.path.join(pkg_moveit, "config", "moveit_sim.rviz")

    common_rviz_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    rviz_hw = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_hw],
        parameters=common_rviz_params + [{"use_sim_time": False}],
        condition=UnlessCondition(is_sim),
    )

    rviz_sim = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_sim],
        parameters=common_rviz_params + [{"use_sim_time": True}],
        condition=IfCondition(is_sim),
    )

    # En SIM damos un pequeño margen a que se activen los controladores de Gazebo
    delayed_move_group_sim = TimerAction(
        period=3.0,
        actions=[move_group_sim],
        condition=IfCondition(is_sim),
    )

    return LaunchDescription([
        is_sim_arg,
        move_group_hw,
        delayed_move_group_sim,
        rviz_hw,
        rviz_sim,
    ])
