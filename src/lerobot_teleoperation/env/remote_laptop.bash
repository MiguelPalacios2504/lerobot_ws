#!/usr/bin/env bash
# Source en la LAPTOP antes de lanzar leader + relay.
#   source install/setup.bash
#   source src/lerobot_teleoperation/env/remote_laptop.bash
#
# Requiere: sudo apt install ros-jazzy-rmw-cyclonedds-cpp

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

_ros_root="/opt/ros/${ROS_DISTRO:-jazzy}"
_cyclone_lib="${_ros_root}/lib/librmw_cyclonedds_cpp.so"
if [[ ! -f "$_cyclone_lib" ]]; then
  _cyclone_prefix="$(ros2 pkg prefix rmw_cyclonedds_cpp 2>/dev/null || true)"
  if [[ -n "$_cyclone_prefix" && -f "${_cyclone_prefix}/lib/librmw_cyclonedds_cpp.so" ]]; then
    _cyclone_lib="${_cyclone_prefix}/lib/librmw_cyclonedds_cpp.so"
  else
    echo "[remote_laptop] ERROR: falta rmw_cyclonedds_cpp."
    echo "  sudo apt install ros-jazzy-rmw-cyclonedds-cpp"
    echo "  (o desactiva el entorno remoto: unset RMW_IMPLEMENTATION CYCLONEDDS_URI)"
    return 1 2>/dev/null || exit 1
  fi
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

_PKG_SHARE="$(ros2 pkg prefix lerobot_teleoperation 2>/dev/null)/share/lerobot_teleoperation"
if [[ -z "$_PKG_SHARE" || ! -d "$_PKG_SHARE" ]]; then
  _WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
  _PKG_SHARE="$_WS_ROOT/src/lerobot_teleoperation"
fi

export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${_PKG_SHARE}/config/cyclonedds_laptop.xml}"

echo "[remote_laptop] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[remote_laptop] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[remote_laptop] CYCLONEDDS_URI=$CYCLONEDDS_URI"
