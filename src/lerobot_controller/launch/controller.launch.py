import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ============================================================
    # 🧩 ARGUMENTOS
    # ============================================================
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="false",
        description="true → simulación (Gazebo/RViz) | false → hardware real",
    )

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="moveit",
        description="Modo de simulación: 'gui' (sliders en RViz) o 'moveit'",
    )

    is_sim = LaunchConfiguration("is_sim")
    mode = LaunchConfiguration("mode")

    # ============================================================
    # 📦 DIRECTORIOS
    # ============================================================
    pkg_desc = get_package_share_directory("lerobot_description")
    pkg_ctrl = get_package_share_directory("lerobot_controller")
    pkg_moveit = get_package_share_directory("lerobot_moveit")

    controllers_hw = os.path.join(pkg_ctrl, "config", "lerobot_controllers.yaml")
    controllers_sim_gui = os.path.join(pkg_ctrl, "config", "lerobot_controllers_gui.yaml")
    controllers_sim_moveit = os.path.join(pkg_moveit, "config", "moveit_controllers.yaml")

    # ============================================================
    # 🤖 DESCRIPCIÓN DEL ROBOT (URDF → XML)
    # ============================================================
    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(pkg_desc, "urdf", "lerobot.urdf.xacro"),
            " is_sim:=", is_sim  # 👈 importante para cargar plugin correcto
        ]),
        value_type=str,
    )

    # ============================================================
    # 📡 NODO COMÚN: robot_state_publisher
    # ============================================================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # ============================================================
    # 🔌 HARDWARE REAL
    # ============================================================
    controller_manager_hw = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_hw,
        ],
        condition=UnlessCondition(is_sim),
        output="screen",
    )

    spawners_hw = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[ctrl, "--controller-manager", "/controller_manager"],
            condition=UnlessCondition(is_sim),
            output="screen",
        )
        for ctrl in ["joint_state_broadcaster", "arm_controller", "gripper_controller"]
    ]

    hardware_group = GroupAction([controller_manager_hw] + spawners_hw)

    # ============================================================
    # 🧩 SIMULACIÓN (modo GUI)
    # ============================================================
    controller_manager_sim_gui = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True},
            controllers_sim_gui,
        ],
        condition=IfCondition(
            PythonExpression([
                "'", is_sim,
                "' == 'true' and '", mode,
                "' == 'gui'"
            ])
        ),
        output="screen",
    )

    spawners_sim_gui = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[ctrl, "--controller-manager", "/controller_manager"],
            condition=IfCondition(
                PythonExpression([
                    "'", is_sim,
                    "' == 'true' and '", mode,
                    "' == 'gui'"
                ])
            ),
            output="screen",
        )
        for ctrl in ["joint_state_broadcaster", "arm_controller", "gripper_controller"]
    ]

    gui_bridge = Node(
        package="lerobot_controller",
        executable="gui_to_controller_bridge",
        name="gui_to_controller_bridge",
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'", is_sim,
                "' == 'true' and '", mode,
                "' == 'gui'"
            ])
        ),
    )

    sim_gui_group = GroupAction([
        controller_manager_sim_gui,
        *spawners_sim_gui,
        TimerAction(period=2.0, actions=[gui_bridge])
    ])

    # ============================================================
    # 🤖 SIMULACIÓN (modo MoveIt)
    # ============================================================
    controller_manager_sim_moveit = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True},
            controllers_sim_moveit,
        ],
        condition=IfCondition(
            PythonExpression([
                "'", is_sim,
                "' == 'true' and '", mode,
                "' == 'moveit'"
            ])
        ),
        output="screen",
    )

    spawners_sim_moveit = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[ctrl, "--controller-manager", "/controller_manager"],
            condition=IfCondition(
                PythonExpression([
                    "'", is_sim,
                    "' == 'true' and '", mode,
                    "' == 'moveit'"
                ])
            ),
            output="screen",
        )
        for ctrl in ["joint_state_broadcaster", "arm_controller", "gripper_controller"]
    ]

    sim_moveit_group = GroupAction([controller_manager_sim_moveit] + spawners_sim_moveit)

    # ============================================================
    # 🚀 DESCRIPCIÓN FINAL DEL LANZAMIENTO
    # ============================================================
    return LaunchDescription([
        is_sim_arg,
        mode_arg,
        robot_state_publisher,
        hardware_group,
        sim_gui_group,
        sim_moveit_group,
    ])
