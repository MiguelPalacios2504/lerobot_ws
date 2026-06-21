import os
import sys

from launch.actions import LogInfo, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

_LAUNCH_DIR = os.path.join(get_package_share_directory("lerobot_description"), "launch")
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from jazzy_compat import ignition_xacro_arg  # noqa: E402


def _robot_namespace(ns: str):
    return None if not ns else ns


def _frame_prefix(ns: str):
    return f"{ns}/" if ns else ""


def make_robot_stack(
    ns: str,
    uart_port: str,
    is_sim: str,
    leader_only: str,
    teleop_follower: str = "false",
):
    """Crea los nodos de un brazo (RSP + ros2_control + spawners)."""
    use_sim = is_sim.lower() == "true"
    leader_mode = leader_only.lower() == "true"
    teleop_mode = teleop_follower.lower() == "true"
    spawn_hw_controllers = (not use_sim) and (not leader_mode)
    robot_ns = _robot_namespace(ns)
    frame_prefix = _frame_prefix(ns) if ns else ""

    pkg_desc = get_package_share_directory("lerobot_description")
    pkg_ctrl = get_package_share_directory("lerobot_controller")
    controllers_hw_path = os.path.join(pkg_ctrl, "config", "lerobot_controllers.yaml")
    controllers_teleop_path = os.path.join(pkg_ctrl, "config", "lerobot_controllers_teleop.yaml")
    controllers_sim_path = os.path.join(pkg_ctrl, "config", "lerobot_controllers_sim.yaml")
    controllers_sim_teleop_path = os.path.join(
        pkg_ctrl, "config", "lerobot_controllers_sim_teleop.yaml"
    )
    if use_sim:
        controllers_path = controllers_sim_teleop_path if teleop_mode else controllers_sim_path
    else:
        controllers_path = controllers_teleop_path if teleop_mode else controllers_hw_path
    move_time_ms = "40" if teleop_mode else "300"
    command_deadband = "0.004" if teleop_mode else "0.002"

    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(pkg_desc, "urdf", "lerobot.urdf.xacro"),
            " is_sim:=", is_sim,
            " is_ignition:=", ignition_xacro_arg(),
            " serial_port:=", uart_port,
            " passive_mode:=", "true" if leader_mode else "false",
            " move_time_ms:=", move_time_ms,
            " command_deadband:=", command_deadband,
        ]),
        value_type=str,
    )

    label = ns or "default"
    profile = "teleop" if teleop_mode else "default"
    actions = [
        LogInfo(
            msg=(
                f"[lerobot_controller] brazo '{label}' uart={uart_port} "
                f"leader_only={leader_only} profile={profile}"
            )
        ),
    ]

    if use_sim:
        actions.extend([
            TimerAction(period=5.0, actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "30",
                    "--switch-timeout", "30",
                ],
                output="screen",
            )]),
            TimerAction(period=6.0, actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_controller",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "30",
                    "--switch-timeout", "30",
                    "--param-file", controllers_path,
                ],
                output="screen",
            )]),
            TimerAction(period=6.5, actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "gripper_controller",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "30",
                    "--switch-timeout", "30",
                    "--param-file", controllers_path,
                ],
                output="screen",
            )]),
        ])
        return actions

    rsp_params = [
        {"robot_description": robot_description},
        {"use_sim_time": False},
    ]
    if frame_prefix:
        rsp_params.append({"frame_prefix": frame_prefix})

    actions.extend([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot_ns,
            parameters=rsp_params,
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=robot_ns,
            parameters=[
                {"robot_description": robot_description},
                controllers_path,
            ],
            output="screen",
        ),
        TimerAction(period=0.8, actions=[Node(
            package="controller_manager",
            executable="spawner",
            namespace=robot_ns,
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "controller_manager",
            ],
            output="screen",
        )]),
    ])

    if spawn_hw_controllers:
        actions.extend([
            TimerAction(period=1.2, actions=[Node(
                package="controller_manager",
                executable="spawner",
                namespace=robot_ns,
                arguments=[
                    "arm_controller",
                    "--controller-manager", "controller_manager",
                    "--controller-manager-timeout", "10",
                    "--param-file", controllers_path,
                ],
                output="screen",
            )]),
            TimerAction(period=1.4, actions=[Node(
                package="controller_manager",
                executable="spawner",
                namespace=robot_ns,
                arguments=[
                    "gripper_controller",
                    "--controller-manager", "controller_manager",
                    "--controller-manager-timeout", "10",
                    "--param-file", controllers_path,
                ],
                output="screen",
            )]),
        ])

    return actions
