import os

from launch.actions import LogInfo, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _robot_namespace(ns: str):
    return None if not ns else ns


def _frame_prefix(ns: str):
    return f"{ns}/" if ns else ""


def _node_kwargs(robot_ns, **extra):
    if robot_ns:
        extra["namespace"] = robot_ns
    return extra


def _spawner_node(robot_ns, arguments):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=arguments,
        output="screen",
        **_node_kwargs(robot_ns),
    )


def _spawner_args(controller_name, controllers_path, robot_ns=None):
    cm = f"/{robot_ns}/controller_manager" if robot_ns else "/controller_manager"
    return [
        controller_name,
        "--controller-manager", cm,
        "--controller-manager-timeout", "30",
        "--switch-timeout", "30",
        "--param-file", controllers_path,
    ]


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
    controllers_path = controllers_teleop_path if teleop_mode else controllers_hw_path
    move_time_ms = "40" if teleop_mode else "300"
    command_deadband = "0.004" if teleop_mode else "0.002"

    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(pkg_desc, "urdf", "lerobot.urdf.xacro"),
            " is_sim:=", is_sim,
            " serial_port:=", uart_port,
            " passive_mode:=", "true" if leader_mode else "false",
            " move_time_ms:=", move_time_ms,
            " command_deadband:=", command_deadband,
            " invert_joint_sign:=true",
        ]),
        value_type=str,
    )

    label = ns or "default"
    profile = "teleop" if teleop_mode else "default"
    fastdds_xml = os.path.join(pkg_ctrl, "config", "fastdds_no_shm.xml")
    actions = []
    if not os.environ.get("RMW_IMPLEMENTATION"):
        actions.extend([
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_xml),
        ])
    actions.append(LogInfo(
            msg=(
                f"[lerobot_controller] brazo '{label}' uart={uart_port} "
                f"leader_only={leader_only} profile={profile}"
            )
        ))

    if use_sim:
        jsb_sim = Node(
            package="controller_manager",
            executable="spawner",
            arguments=_spawner_args("joint_state_broadcaster", controllers_sim_path, robot_ns),
            output="screen",
        )
        arm_sim = Node(
            package="controller_manager",
            executable="spawner",
            arguments=_spawner_args("arm_controller", controllers_sim_path, robot_ns),
            output="screen",
        )
        grip_sim = Node(
            package="controller_manager",
            executable="spawner",
            arguments=_spawner_args("gripper_controller", controllers_sim_path, robot_ns),
            output="screen",
        )
        actions.extend([
            TimerAction(period=5.0, actions=[jsb_sim]),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=jsb_sim,
                    on_exit=[arm_sim],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=arm_sim,
                    on_exit=[grip_sim],
                )
            ),
        ])
        return actions

    rsp_params = [
        {"robot_description": robot_description},
        {"use_sim_time": False},
    ]
    if frame_prefix:
        rsp_params.append({"frame_prefix": frame_prefix})

    cm_params = [
        {"robot_description": robot_description},
        controllers_path,
    ]

    jsb_spawner = _spawner_node(
        robot_ns,
        _spawner_args("joint_state_broadcaster", controllers_path, robot_ns),
    )

    actions.extend([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=rsp_params,
            output="screen",
            **_node_kwargs(robot_ns),
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=cm_params,
            output="screen",
            **_node_kwargs(robot_ns),
        ),
        TimerAction(period=2.0, actions=[jsb_spawner]),
    ])

    if spawn_hw_controllers:
        arm_spawner = _spawner_node(
            robot_ns,
            _spawner_args("arm_controller", controllers_path, robot_ns),
        )
        grip_spawner = _spawner_node(
            robot_ns,
            _spawner_args("gripper_controller", controllers_path, robot_ns),
        )
        actions.extend([
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=jsb_spawner,
                    on_exit=[arm_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=arm_spawner,
                    on_exit=[grip_spawner],
                )
            ),
        ])

    return actions
