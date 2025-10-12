# LeRobot SO101 – Low-Cost 6-DOF Arm

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

## ⚙️ Servo Settings

Before running the robot, set the servo **internal offsets** using the original Feetech software:  
👉 [FeetechRC Software (FD1.9.8.3)](https://www.feetechrc.com/software.html)

Set the servos to the **neutral position**:

```
Joint angles: 0°, 90°, -90°, 0°, 0°, 0°
```

> ⚠️ These offsets are important for correct inverse kinematics and feedback readings.

🖼️ *Add image here showing the correct servo positions.*

---

## 🧩 Requirements

Tested on:
- **Ubuntu 22.04**
- **ROS 2 Humble Hawksbill**
- **colcon**, **rviz2**, **gazebo_ros_pkgs**
- **rclcpp**, **geometry_msgs**, **sensor_msgs**

Install the basic dependencies:
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs python3-colcon-common-extensions
```

---

## How to Run

### Simulation with Gazebo
```bash
ros2 launch lerobot_controller sim_controller.launch.py
ros2 launch lerobot_description gazebo.launch.py
```

### Visualization with RViz + Gazebo
```bash
ros2 launch lerobot_description gazebo.launch.py
```

---

### Real Robot Controller

For running the **real robot** (connected via `/dev/ttyACM0`, baudrate `1000000`):

```bash
ros2 launch lerobot_controller controller.launch.py sim:=False
```

---

### Motion Planning with MoveIt

**Real robot:**
```bash
ros2 launch lerobot_moveit moveit.launch.py sim:=False
```

**Simulated robot:**
```bash
ros2 launch lerobot_moveit moveit.launch.py
```

🖼️ *Add image or GIF of MoveIt planning here.*

---

## Under Development

Some modules are still being integrated:
```bash
ros2 launch lerobot_bringup simulated_robot.launch.py
./ngrok http 5000

lerobot_cpp_examples/scripts/lero_ik
lerobot_cpp_exmaples/scripts/lero_dk

```

---

## 🧱 Repository Structure

```
lerobot_ws/
├── src/
│   ├── lerobot_description/     # URDF, meshes, RViz launch
│   ├── lerobot_controller/      # Nodes for motion control
│   ├── lerobot_cpp_examples/    # C++ kinematics examples
│   └── ...
├── README.md
└── .gitignore
```

---

## Credits

Frame design and inspiration: [AntoBrandi](https://github.com/AntoBrandi)  
Development and adaptation: **Miguel Encarnacion**

---

## 📬 Contact

If you’d like to collaborate, test, or improve this project:  
**GitHub:** [MiguelPalacios2504](https://github.com/MiguelPalacios2504)
