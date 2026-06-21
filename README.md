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
- MoveIt 2 motion planning and Gazebo Sim (gz) integration

The mechanical frame is based on the work of [AntoBrandi](https://github.com/AntoBrandi), whose open-source designs inspired me to learn by building.

---

## ⚙️ Servo Settings

Before running the robot, set the servo **internal offsets** using the original Feetech software:  
👉 [FeetechRC Software (FD1.9.8.3)](https://www.feetechrc.com/software.html)

Set the servos to the **neutral position**:

```
Joint angles: 0°, 90°, -90°, 0°, 0°, 0°
```
<p align="center">
  <img src="images/seting.jpeg" alt="LeRobot setting position" width="600"/>
</p>

Change the offset so you can get in this position the next values, otherwise you will have to do extra setting actions.

```
Default values: 2179, 3594, 345, 2354, 2165, 2275
```

<p align="center">
  <img src="images/feetech.jpeg" alt="Feetech software" width="600"/>
</p>

> 

---

## 🧩 Requirements

> **Rama `jazzy`:** Ubuntu 24.04 + ROS 2 Jazzy  
> **Rama `main`:** Ubuntu 22.04 + ROS 2 Humble

Tested on (rama `jazzy`):
- **Ubuntu 24.04 Noble**
- **ROS 2 Jazzy Jalisco**
- **colcon**, **rviz2**, **ros_gz_sim**, **gz_ros2_control**, **MoveIt 2**

Install dependencies:
```bash
sudo apt update
sudo apt install \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-moveit \
  ros-jazzy-moveit-planners-ompl \
  ros-jazzy-pilz-industrial-motion-planner \
  ros-jazzy-moveit-ros-visualization \
  ros-jazzy-ros2-control \
  ros-jazzy-xacro \
  python3-colcon-common-extensions
```

> Si `ros-jazzy-moveit-configs-utils` no aparece en apt, no es necesario: esta rama carga la config MoveIt desde `lerobot_moveit/launch/moveit_config_loader.py`.

Build the workspace:
```bash
source /opt/ros/jazzy/setup.bash
cd lerobot_ws
colcon build --symlink-install
source install/setup.bash
```

---

## How to Run

### Demo Robot Limits and joint visualization
```bash
ros2 launch lerobot_description display.launch.py is_sim:=true

```
<p align="center">
  <img src="images/demo.png" alt="LeRobot demo" width="600"/>
</p>
---

### Simulation with Gazebo Sim (gz)
```bash
ros2 launch lerobot_description gazebo.launch.py is_sim:=true
ros2 launch lerobot_controller controller.launch.py is_sim:=true
```
test movement

```ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['limb1_to_base_link', 'limb2_to_limb1', 'limb3_to_limb2', 'limb4_to_limb3', 'limb5_to_limb4'],
  points: [{
    positions: [0.3, 0.4, -0.2, 0.1, 0.0],
    time_from_start: {sec: 3}
  }]
}"
```
<p align="center">
  <img src="images/gazebo.png" alt="LeRobot in Gazebo" width="600"/>
</p>

### Simulation Gazebo Sim and RViz


```bash
ros2 launch lerobot_description gazebo.launch.py is_sim:=true
ros2 launch lerobot_controller controller.launch.py is_sim:=true
ros2 launch lerobot_description rviz.launch.py is_sim:=true
```

### Full simulated stack (Gazebo + control + MoveIt + remote)

```bash
ros2 launch lerobot_bringup simulated_robot.launch.py
```

---


### Real Robot Controller

For running the **real robot** (connected via `/dev/ttyACM0`, baudrate `1000000`):

```bash
ros2 launch lerobot_controller controller.launch.py is_sim:=false
```
<p align="center">
  <img src="images/rviz_real.png" alt="LeRobot real with rviz" width="600"/>
</p>
---

### Motion Planning with MoveIt

**Real robot:** it depends in which mode you would like to work is_sim:true (simulation) is_sim: false (real robot)
```bash
ros2 launch lerobot_moveit moveit.launch.py
ros2 launch lerobot_moveit moveit.launch.py is_sim:=true
```
<p align="center">
  <img src="images/moveit.png" alt="LeRobot with moveit" width="600"/>
</p>

<p align="center">
  <img src="images/moveit_real.jpg" alt="LeRobot real with moveit" width="600"/>
</p>

## Under Development

Some modules are still being integrated:
```bash
ros2 launch lerobot_bringup simulated_robot.launch.py
./ngrok http 5000

lerobot_cpp_examples/scripts/lero_ik
lerobot_cpp_exmaples/scripts/lero_dk

ros2 launch lerobot_description gazebo.launch.py \
  sim_controllers_config:=lerobot_controllers_sim_teleop.yaml
  
ros2 launch lerobot_controller controller.launch.py \
  is_sim:=true teleop_follower:=true

ros2 launch lerobot_controller controller.launch.py \
  is_sim:=false ns:=leader leader_only:=true uart_port:=/dev/ttyACM0

ros2 launch lerobot_teleoperation teleop_mirror.launch.py \
  source_ns:=leader \
  target_ns:=/ \
  command_mode:=forward \
  publish_deadband:=0.002 \
  smoothing_alpha:=1.0

```

---

## 🧱 Repository Structure

```
lerobot_ws/
├── src/
│   ├── lerobot_description/     # URDF, meshes, RViz/Gazebo launch
│   ├── lerobot_controller/      # ros2_control + Feetech hardware interface
│   ├── lerobot_moveit/          # MoveIt 2 motion planning
│   ├── lerobot_bringup/         # Full-stack launch files
│   ├── lerobot_cpp_examples/    # C++ kinematics and MoveIt examples
│   ├── lerobot_teleoperation/   # Dual-arm teleoperation
│   ├── lerobot_remoto/          # Remote / Alexa interface
│   ├── lerobot_msgs/            # Custom messages and actions
│   └── lerobot_utils/           # Shared utilities
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
