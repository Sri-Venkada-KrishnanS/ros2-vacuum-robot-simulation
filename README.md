# ros2-vacuum-robot-simulation
ROS 2 Humble–based vacuum cleaning robot simulation featuring LiDAR-driven obstacle avoidance, path tracking, and autonomous navigation in Gazebo.

Milestone 1: Environment Setup and obstacle avoidance.

# Overview
This project implements a autonomous vacuum cleaning robot that navigates through a grid environment, avoids obstacles using simulated sensors, and tracks its path in real time using standard ROS topics and RViz visualization.

# Setup
- ROS 2: Humble Hawksbill
- Simulator: Gazebo Fortress

# Build Instructions
```
colcon build --packages-select vacuum_bot
source install/setup.bash

# This command starts Gazebo Fortress, spawns the robot, and sets up the communication bridge:
source install/setup.bash
ros2 launch vacuum_bot sim.launch.py

# This command runs the Python node that processes LiDAR data to avoid obstacles (obstacles is defined in sdf file which automatically opens when executed sim.launch.py):
source install/setup.bash
ros2 run vacuum_bot vacuum_logic.py
```
