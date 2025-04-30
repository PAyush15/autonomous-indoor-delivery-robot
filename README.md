# Autonomous-delivery-robot-with-real-time-obstacle-avoidance

Table of Contents:

  - [Overview](#overview)
  - [Demo](#demo)
  - [Features](#features)
  - [Project Structure](#projectstructure)
  - [Installation](#installation)
  - [Usage](#usage)
  - [Hardware Details](#hardwaredetails)
  - [Model Files](#modelfiles)
  - [Contributing](#contributing)
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

📦 autonomous-delivery-robot/\
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
└── 📜 README.md


## Installation

📦 Prerequisites
Ensure you have ROS Noetic installed on Ubuntu 20.04. If not, install it:
'''
sudo apt update
sudo apt install ros-noetic-desktop-full
'''
Initialize rosdep and set up your environment:

'''
sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
'''

Install Required ROS Packages

'''sudo apt install -y \
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
  '''
  
Clone and Build the Package

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone the repository
git clone https://github.com/yourusername/autonomous-delivery-robot.git

# Build the workspace
cd ~/catkin_ws
catkin build

# Source the setup file
source devel/setup.bash
```

## Usage

## Hardware Details

## Model Files

## Contributing

## License

## Acknowledgments


