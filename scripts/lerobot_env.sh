#!/usr/bin/env bash
# Entorno aislado (sin overlay de tesis_ws). Uso normal: source install/setup.bash
set +u
_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset CYCLONEDDS_URI RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source "${_WS_ROOT}/install/setup.bash"
ros2 daemon stop >/dev/null 2>&1 || true
