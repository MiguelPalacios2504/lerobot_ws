#!/usr/bin/env bash
# Instala dependencias apt para compilar TODO lerobot_ws en ROS 2 Jazzy (Ubuntu 24.04).
# Uso en Raspberry / laptop:
#   cd ~/lerobot_ws
#   bash scripts/install_dependencies_jazzy.sh
#
# Opcional (solo follower remoto, sin MoveIt/Gazebo):
#   MINIMAL=1 bash scripts/install_dependencies_jazzy.sh

set -euo pipefail

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: ROS 2 Jazzy no encontrado en /opt/ros/jazzy"
  echo "Instala primero ROS 2 Jazzy: https://docs.ros.org/en/jazzy/Installation.html"
  exit 1
fi

echo "==> Actualizando apt..."
sudo apt update

COMMON_PKGS=(
  build-essential
  git
  python3-colcon-common-extensions
  python3-rosdep
  python3-vcstool
  libserial-dev
  python3-transforms3d
  # ros2_control + controladores
  ros-jazzy-ros2-control
  ros-jazzy-hardware-interface
  ros-jazzy-controller-manager
  ros-jazzy-joint-state-broadcaster
  ros-jazzy-joint-trajectory-controller
  ros-jazzy-forward-command-controller
  # descripción / launches
  ros-jazzy-robot-state-publisher
  ros-jazzy-xacro
  ros-jazzy-joint-state-publisher-gui
  ros-jazzy-tf2-ros
  ros-jazzy-tf-transformations
  # twin remoto
  ros-jazzy-rmw-cyclonedds-cpp
)

MINIMAL_PKGS=(
  ros-jazzy-ros-base
)

FULL_PKGS=(
  ros-jazzy-desktop
  ros-jazzy-ros-gz
  ros-jazzy-gz-ros2-control
  ros-jazzy-moveit
  ros-jazzy-moveit-ros-planning-interface
  ros-jazzy-moveit-ros-move-group
  ros-jazzy-moveit-ros-visualization
  ros-jazzy-moveit-planners-ompl
  ros-jazzy-pilz-industrial-motion-planner
)

if [[ "${MINIMAL:-0}" == "1" ]]; then
  echo "==> Modo MINIMAL (follower remoto, sin MoveIt/Gazebo)"
  PKGS=("${COMMON_PKGS[@]}" "${MINIMAL_PKGS[@]}")
else
  echo "==> Modo FULL (workspace completo)"
  PKGS=("${COMMON_PKGS[@]}" "${FULL_PKGS[@]}")
fi

echo "==> Instalando paquetes (${#PKGS[@]})..."
sudo apt install -y "${PKGS[@]}"

# moveit_configs_utils: opcional en Jazzy (el repo usa moveit_config_loader.py)
if [[ "${MINIMAL:-0}" != "1" ]]; then
  if apt-cache show ros-jazzy-moveit-configs-utils &>/dev/null; then
    echo "==> Instalando moveit_configs_utils (opcional)..."
    sudo apt install -y ros-jazzy-moveit-configs-utils || true
  else
    echo "==> moveit_configs_utils no está en apt; se omite (OK en esta rama)."
  fi
fi

# rosdep para dependencias declaradas en package.xml
if [[ -d src ]]; then
  echo "==> rosdep (dependencias de package.xml)..."
  if ! sudo rosdep init 2>/dev/null; then
    true
  fi
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y || true
fi

echo ""
echo "Listo. Siguiente:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  cd ~/lerobot_ws"
if [[ "${MINIMAL:-0}" == "1" ]]; then
  echo "  colcon build --symlink-install --packages-select \\"
  echo "    lerobot_msgs lerobot_controller lerobot_description lerobot_teleoperation"
else
  echo "  colcon build --symlink-install"
fi
echo "  source install/setup.bash"
