

# M.A.T.O

> A low cost mapping robot

![Badge C++](https://img.shields.io/badge/C++-17-blue.svg?style=flat&logo=c%2B%2B) 
![ROS 2](https://img.shields.io/badge/ROS_2-22314E?style=flat&logo=c%2B%2B)

# About

**M.A.T.O** is a low cost mapping robot that uses ROS2, Slam_Toolbox, micro-ros and a ESP-32. Using only three low cost sensors, lidar, encoder, giroscope

## Installation

### 1. Prerequisites
* **ROS 2 Humble** (Desktop Install recommended)
* **Micro-ROS Agent** (For ESP32 communication)
* An functional ros2_ws

### 2. Setup 
Create a workspace and clone the repository:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone [https://github.com/YOUR_USERNAME/mapping_robot.git](https://github.com/YOUR_USERNAME/mapping_robot.git)

cd ~/ros2_ws
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
```

## Usage
