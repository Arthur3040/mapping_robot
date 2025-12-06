

# Mr (Mapping Robot)

> A low cost mapping robot

![ROS 2](https://img.shields.io/badge/ROS_2-22314E?style=flat&logo=c%2B%2B)
![micro-ROS on ESP32](https://img.shields.io/badge/micro--ROS_on-ESP32-ffb300?style=flat&logo=espressif)
![Badge C++](https://img.shields.io/badge/C++-17-blue.svg?style=flat&logo=c%2B%2B) 
![Built with CMake](https://img.shields.io/badge/Built_with-CMake-064F8C?style=flat&logo=cmake)



# About

**Mr** is a low cost mapping robot that uses ROS2, Slam_Toolbox, micro-ros and a ESP-32. Using only three low cost sensors, lidar, encoder, giroscope

## Hardware
Segue informacoes do robo de baixo custo que foi modelado.

### Components List
*
*
*
### Eletric Circuit/Schematic
IMAGEM circuito eletrico;

## Software Installation

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
```
Installing dependecies
```bash
cd ~/ros2_ws
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
```

## Usage

you can use the simulator in gz, or the code for our real life robot model!

### Simulation
open 3 terminals, source them and go inside the ros2_ws
```bash
ros2 launch mapping_robot robot_state_publish.py
ros2 run teleop
ros2 run slam_toolbox
```
your gz should be looking similar to this:
IMAGE

### Physical robot
Open 3 terminals, source them and go inside the ros2_ws, in a wife that both the computer and the ESP-32 are connected, configure the name and password of the code inside the micro_ros.cpp
```bash
ros2 run connect micro-ros server
ros2 launch mapping_robot real_bridge.py
ros2 run teleop
```
Your rviz should look similar to this:
IMAGE


