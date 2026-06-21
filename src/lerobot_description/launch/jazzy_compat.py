import os


def is_ignition_backend() -> bool:
    """Humble uses Ignition Gazebo; Jazzy and newer use Gazebo Sim (gz)."""
    return os.environ.get("ROS_DISTRO", "jazzy") == "humble"


def ignition_xacro_arg() -> str:
    return "true" if is_ignition_backend() else "false"


def gz_physics_engine_args() -> str:
    if is_ignition_backend():
        return ""
    return "--physics-engine gz-physics-bullet-featherstone-plugin"
