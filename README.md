# Autonomous-delivery-robot-with-real-time-obstacle-avoidance

Table of Contents:

  [Overview](#overview)\
  [Demo](#demo)\
  [Features](#features)\
  [Project Structure](#projectstructure)\
  [Installation](#installation)\
  [Usage](#usage)\
  [Hardware Details](#hardwaredetails)\
  [Model Files](#modelfiles)\
  [Contributing](#contributing)\
  [License](#license)\
  [Acknowledgments](#acknowledgments)

## Overview

Autonomous Delivery Robot with Real-Time Obstacle Avoidance is a ROS-based indoor robot designed to automate goods delivery in places like offices, malls, and hospitals. It navigates autonomously using LiDAR and the ROS Navigation Stack, avoids static and dynamic obstacles, and delivers items securely using QR code-based authentication.

Built with a 3D-printed chassis and powered by Jetson Nano, the robot includes dual delivery compartments, a live camera feed, and multiple control modes—manual (teleop), scripted, and fully autonomous. Mapping and localization are handled via Cartographer SLAM, and the full system was tested in Gazebo and RViz before real-world deployment.


## Demo

https://github.com/user-attachments/assets/71b35f18-758d-4af4-b2c4-f475b9f0dad2


## Features

Autonomous Indoor Navigation using ROS Navigation Stack and Cartographer SLAM

Real-Time Obstacle Avoidance with 2D LiDAR and ultrasonic sensors

Secure Delivery with QR code-based compartment unlocking

Live Video Streaming via Raspberry Pi camera

Multiple Control Modes: teleoperation, scripted navigation, autonomous mode

Modular 3D-Printed Design with dual delivery compartments

URDF Model Integration for simulation and parameter tuning

Tested in Simulation (Gazebo + RViz) before deployment



## Project Structure

📦 autonomous-delivery-robot/
├── 📂 config/               # ROS configuration and parameter files
├── 📂 launch/               # Launch files for simulation and real-world runs
├── 📂 maps/               # Launch files for simulation and real-world runs
├── 📂 meshes/               # STL files for 3D components
├── 📂 params/               # Parameters for Navigation (Ex. global_costmap_params.yaml)
├── 📂 scripts/               # ROS nodes for the robot
├── 📂 urdf/               # URDF files of the robot
├── 📂 worlds/               # Gazebo worlds files
├── 📜 CMakeLists.txt        # Build instructions (if using ROS2 or C++)
├── 📜 package.xml           # ROS package definition
└── 📜 README.md


## Installation

## Usage

## Hardware Details

## Model Files

## Contributing

## License

## Acknowledgments


