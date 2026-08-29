#!/usr/bin/env bash
# Comprueba que la CLI ve el mismo grafo ROS que el controlador.
set -euo pipefail
source "$(dirname "$0")/../install/setup.bash"

echo "=== ros2 node list ==="
timeout 5 ros2 node list || echo "(timeout: no nodes visible)"

echo
echo "=== ros2 control list_controllers ==="
timeout 5 ros2 control list_controllers || echo "(timeout: controller_manager not reachable)"

echo
echo "=== ros2 topic info /joint_states -v ==="
ros2 topic info /joint_states -v 2>/dev/null | sed -n '1,20p'
