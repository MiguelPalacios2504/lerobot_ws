#!/usr/bin/env bash
# Entorno ROS común para teleop LeRobot -> Piper (todas las terminales).
# Uso: source /path/to/teleop_ros_env.bash

export ROS_DOMAIN_ID=0
export ROS_DISABLE_DAEMON=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${LEROBOT_WS:-$HOME/Documents/GIthub/lerobot_ws}/src/lerobot_controller/config/fastdds_no_shm.xml"

unset ROS_LOCALHOST_ONLY
unset CYCLONEDDS_URI

echo "[teleop] ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION"
