"""Load MoveIt parameters for ROS 2 Jazzy."""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def _pkg_share(package: str) -> str:
    return get_package_share_directory(package)


def _load_yaml_path(path: str) -> dict:
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _load_pkg_yaml(package: str, relative_path: str) -> dict:
    return _load_yaml_path(os.path.join(_pkg_share(package), relative_path))


def _load_text_pkg(package: str, relative_path: str) -> str:
    path = os.path.join(_pkg_share(package), relative_path)
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def _jazzy_ompl_pipeline() -> dict:
    share = _pkg_share("moveit_configs_utils")
    ompl = _load_yaml_path(os.path.join(share, "default_configs", "ompl_planning.yaml"))
    ompl.update(_load_yaml_path(os.path.join(share, "default_configs", "ompl_defaults.yaml")))
    ompl["arm"] = {
        "default_planner_config": "RRTConnect",
        "planner_configs": ["RRTConnect"],
    }
    ompl["gripper"] = {
        "default_planner_config": "RRTConnect",
        "planner_configs": ["RRTConnect"],
    }
    return ompl


def _jazzy_pilz_pipeline() -> dict:
    share = _pkg_share("moveit_configs_utils")
    return _load_yaml_path(
        os.path.join(share, "default_configs", "pilz_industrial_motion_planner_planning.yaml")
    )


def robot_description_parameter(is_sim, is_ignition: str) -> dict:
    xacro_path = os.path.join(_pkg_share("lerobot_description"), "urdf", "lerobot.urdf.xacro")
    return {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                xacro_path,
                " is_sim:=", is_sim,
                " is_ignition:=", is_ignition,
            ]),
            value_type=str,
        ),
    }


def _robot_description_planning() -> dict:
    planning = _load_pkg_yaml("lerobot_moveit", "config/joint_limits.yaml")
    planning.update(_load_pkg_yaml("lerobot_moveit", "config/pilz_cartesian_limits.yaml"))
    return planning


def static_moveit_parameters() -> dict:
    return {
        "robot_description_semantic": _load_text_pkg("lerobot_moveit", "config/lerobot.srdf"),
        "robot_description_kinematics": _load_pkg_yaml("lerobot_moveit", "config/kinematics.yaml"),
        "robot_description_planning": _robot_description_planning(),
        "planning_pipelines": ["ompl", "pilz_industrial_motion_planner"],
        "default_planning_pipeline": "ompl",
        "ompl": _jazzy_ompl_pipeline(),
        "pilz_industrial_motion_planner": _jazzy_pilz_pipeline(),
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 1.2,
            "allowed_goal_duration_margin": 0.5,
            "allowed_start_tolerance": 0.01,
            "execution_duration_monitoring": True,
        },
        "moveit_manage_controllers": True,
    }


def move_group_parameters(is_sim, is_ignition: str) -> list:
    return [
        robot_description_parameter(is_sim, is_ignition),
        static_moveit_parameters(),
    ]


def rviz_moveit_parameters(is_sim, is_ignition: str) -> list:
    # RViz needs the same MoveIt params as move_group for IK / interactive markers.
    return move_group_parameters(is_sim, is_ignition)


def moveit_cpp_parameters(is_sim, is_ignition: str) -> dict:
    merged = {}
    merged.update(robot_description_parameter(is_sim, is_ignition))
    merged.update(static_moveit_parameters())
    merged.update(_load_pkg_yaml("lerobot_moveit", "config/planning_python_api.yaml"))
    return merged
