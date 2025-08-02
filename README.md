# CityLab Project – TurtleBot3 Autonomous Patrol 🚀

**Goal:**  
A self-contained ROS 2 C++ package where a TurtleBot3 robot patrols an environment by scanning the frontal 180° with a LiDAR sensor, detecting obstacles under 35 cm, computing the angle toward the farthest safe ray, and patrolling indefinitely—first in Gazebo simulation, then on a live TurtleBot3 accessed remotely.

---

## 📌 Project Overview

The core logic, encapsulated in the `Patrol` class within `patrol.cpp`, works as follows:

- Subscribes to `/scan` and processes only the frontal ±90° sector.
- Within that range, ignores infinite readings, identifies the maximum non-infinite ray, and stores its orientation in `direction_`.
- Utilises a 10 Hz control loop to publish `geometry_msgs/msg/Twist` messages: `linear.x = 0.1 m/s`; `angular.z = direction_ / 2`.
  
This combination enables the robot to move in straight lines until it approaches an obstacle (< 0.35 m), then steer toward the clearest direction.  
✅ Obstacle avoidance behavior is achieved using reactive LiDAR-only measurements.

---

## ✨ Features

- Modular ROS 2 C++ node (`Patrol`) with separate subscriber callback and control loop.
- Safety logic using only ±π/2 frontal LiDAR—minimal dependencies, intuitive behavior.
- Two launch files for convenience:
  - `main_turtlebot3_lab.launch.xml`: Spawns Gazebo world with TurtleBot3.
  - `start_patrolling.launch.py`: Launches the patrol C++ node.
- Package name: `robot_patrol`, ready to clone and build inside any ROS 2 workspace (`ros2_ws`).
- Optionally supports RViz: displays `/scan`, `/odom`, and TF frames (`fixed frame = odom`).

---

## ⚙️ Prerequisites

Before running anything, ensure:

- **Ubuntu 22.04** (or compatible with **ROS 2 Humble** or newer).
- Install TurtleBot3 and Gazebo simulation packages:
  ```bash
  sudo apt update
  sudo apt install \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-gazebo \
    ros-humble-rviz2 \
    python3-colcon-common-extensions


## 🧪 How to Run the Robot Simulation

1. Build & Source (once per terminal)
```bash
  cd ~/ros2_ws
  colcon build --symlink-install
  source install/setup.bash
  source /opt/ros/humble/setup.bash

```
2. Launch Gazebo (with all sim nodes)
```bash
  ros2 launch turtlebot3_gazebo main_turtlebot3_lab.launch.xml
```
Wait ~30 seconds for Gazebo and TurtleBot3 to appear.

3. Launch Patrol Node (in another terminal)

```bash
  cd ~/ros2_ws
  source install/setup.bash
  ros2 launch robot_patrol start_patrolling.launch.py
```
<!-- 
## 📁 File Hierarchy

citylab_project/
└── robot_patrol/
    ├── src/patrol.cpp
    ├── launch/
    │    └── start_patrolling.launch.py
    ├── patrol.rviz           # (Optional) RViz config
    ├── CMakeLists.txt
    └── package.xml

-->



