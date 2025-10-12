
Hallo! 👋  
I'm **Miguel**, and this is an **open-source ROS 2 project** for the low-cost arm robot **SO101**.  
It’s still under development, but I believe it’s time to share it with the community 💪  

---

## Overview

This repository provides the **ROS 2 workspace** for controlling and simulating the SO101 arm.  
It includes:
- URDF/Xacro models for visualization
- Basic kinematics and control nodes (C++/Python)
- Launch files for bringing up the robot and testing movement
- Example configurations for MoveIt and Gazebo (coming soon)

The mechanical frame is based on the work of [AntoBrandi](https://github.com/AntoBrandi), whose open-source designs inspired me to learn by building.

---

## Requirements

Tested on:
- **Ubuntu 22.04**
- **ROS 2 Humble Hawksbill**
- **colcon**, **rviz2**, **gazebo_ros_pkgs**
- **rclcpp**, **geometry_msgs**, **sensor_msgs**

Install the basic dependencies:
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs python3-colcon-common-extensions



ros2 launch lerobot_bringup simulated_robot.launch.py

./ngrok http 5000
