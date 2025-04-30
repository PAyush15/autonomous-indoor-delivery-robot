# Autonomous-delivery-robot-with-real-time-obstacle-avoidance

Table of Contents:

  - [Overview](#overview)
  - [Demo](#demo)
  - [Features](#features)
  - [Project Structure](#projectstructure)
  - [Installation](#installation)
  - [Usage](#usage)
  - [Hardware Details](#hardwaredetails)
  - [License](#license)
  - [Acknowledgments](#acknowledgments)

## Overview

Autonomous Delivery Robot with Real-Time Obstacle Avoidance is a ROS-based indoor robot designed to automate goods delivery in places like offices, malls, and hospitals. It navigates autonomously using LiDAR and the ROS Navigation Stack, avoids static and dynamic obstacles, and delivers items securely using QR code-based authentication.

Built with a 3D-printed chassis and powered by Jetson Nano, the robot includes dual delivery compartments, a live camera feed, and multiple control modes—manual (teleop), scripted, and fully autonomous. Mapping and localization are handled via Cartographer SLAM, and the full system was tested in Gazebo and RViz before real-world deployment.


## Demo

https://github.com/user-attachments/assets/71b35f18-758d-4af4-b2c4-f475b9f0dad2


## Features

- **Autonomous Indoor Navigation** using ROS Navigation Stack and Cartographer SLAM

- **Real-Time Obstacle Avoidance** with 2D LiDAR and ultrasonic sensors

- **Secure Delivery** with QR code-based compartment unlocking

- **Live Video Streaming** via Raspberry Pi camera

- **Multiple Control Modes**: teleoperation, scripted navigation, autonomous mode

- **Modular 3D-Printed Design** with dual delivery compartments

- **URDF Model Integration** for simulation and parameter tuning

- **Tested in Simulation** before deployment

- **Tested with robot** in real-time



## Project Structure

├── 📂 autonomous-indoor-delivery-robot-main/
  ├── 📜 README.md\
  ├── 📜 qrcode_scan.py                  # QR code detection script (Jetson Nano + Pi Cam)\
  ├── 📜 rosserial.ino                   # Arduino code for servo/buzzer control\
  ├── 📂 adbot_description/             # URDF and package description for adbot\
    ├── 📂 config/               # ROS configuration and parameter files\
    ├── 📂 launch/               # Launch files for simulation and real-world runs\
    ├── 📂 maps/               # Launch files for simulation and real-world runs\
    ├── 📂 meshes/               # STL files for 3D components\
    ├── 📂 params/               # Parameters for Navigation (Ex. global_costmap_params.yaml)\
    ├── 📂 scripts/               # ROS nodes for the robot\
    ├── 📂 urdf/               # URDF files of the robot\
    ├── 📂 worlds/               # Gazebo worlds files\
    ├── 📜 CMakeLists.txt        # Build instructions (if using ROS2 or C++)\
    ├── 📜 package.xml           # ROS package definition\
  ├── 📂 gbot_core/\
  ├── 📂 rmp_bot_description/          # URDF and package description for rmp_bot\
    ├── 📜 CMakeLists.txt\
    ├── 📜 LICENSE\
    ├── 📜 package.xml\
  ├── 📂 ros_controllers-melodic-devel/ # External ROS control package fork\
    ├── 📜 .gitignore\
    ├── 📜 .travis.yml\
    ├── 📜 README.md\



## Installation

Prerequisites
Ensure you have ROS Noetic installed on Ubuntu 20.04. If not, install it:
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
  ```
Initialize rosdep and set up your environment:

```bash
sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
  ```

Install Required ROS Packages

```bash
sudo apt install -y \
  ros-noetic-navigation \
  ros-noetic-slam-gmapping \
  ros-noetic-teleop-twist-keyboard \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher-gui \
  ros-noetic-xacro \
  ros-noetic-gazebo-ros \
  ros-noetic-map-server \
  ros-noetic-amcl \
  ros-noetic-rviz \
  python3-rosdep \
  python3-rosinstall \
  python3-vcstools \
  python3-catkin-tools
  ```
  
Clone and Build the Package

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone the repository
git clone https://github.com/PAyush15/autonomous-indoor-delivery-robot

# Build the workspace
cd ~/catkin_ws
catkin build

# Source the setup file
source devel/setup.bash
```

## Usage

Run QR Code Authentication System
```rosrun autonomous-indoor-delivery-robot-main qrcode_scan.py```

This script (for Jetson Nano + Raspberry Pi Camera):

Captures QR/aruco markers\
Validates user delivery code\
Sends a signal to the Arduino (via serial) to unlock the correct compartment\

Run Arduino Firmware
1. Upload the following code to your Arduino Uno:\
2. Open Arduino IDE\
3. Connect your Arduino board\
4. Open the file: autonomous-indoor-delivery-robot-main/rosserial.ino\
5. Select board & port\
6. Click Upload

The Arduino script controls:

Servo motor (for locking mechanism)\
Buzzer (for unauthorized access)

Launch URDF (example for RViz test)
Then, to launch the robot in the gazebo world:\
```roslaunch adbot_description gazebo.launch```\

Then, to start SLAM for the robot, launch:
```roslaunch adbot_description slam.launch map:=maps/trs_lab_thin_b.yaml use_sim_time:=true```

Finally, launch the navigation launch file with the parameter file path
```ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true```

## Hardware Details

### 🔩 Hardware Components

| Component              | Description                                                              | Quantity |
|------------------------|---------------------------------------------------------------------------|----------|
| Jetson Nano            | Main onboard processor for navigation, vision, and QR decoding           | 1        |
| RP Lidar A1            | 2D LiDAR for mapping and obstacle detection                              | 1        |
| Raspberry Pi Camera V2 | Camera module for scanning QR/aruco codes                                | 1        |
| SG90 Micro Servo       | Controls locking of delivery compartments                                | 1        |
| Arduino Uno            | Handles servo, LED, and buzzer control based on Jetson serial commands   | 1        |
| L298N Motor Driver     | Motor driver to control BO motors                                         | 1        |
| 60 RPM BO Motors       | DC geared motors for robot mobility                                      | 4        |
| 18650 Li-ion Cells     | Power supply for Jetson Nano and Arduino                                 | 4        |
| XL4015 Buck Converter  | Steps down voltage to power Jetson Nano (5V/4A)                          | 1        |
| 3D Printed Chassis     | Custom modular body for the robot built with ABS                         | -        |
| Misc. Electronics      | LEDs, buzzers, connectors, jacks, holders, etc.                          | -        |


## License

This project is licensed under the MIT License.
Feel free to use, modify, and distribute — just give credit where it’s due!

## Acknowledgments

Thanks to BVM Alumni Association for project funding

Special thanks to Prof. Dr. Vinay Patel for mentorship and guidance


