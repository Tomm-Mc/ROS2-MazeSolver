# Autonomous Maze Navigation and Path Optimization with ROS 2 Jazzy and Gazebo Harmonic

## Overview
This project involves a differential drive robot navigating and solving a maze in a simulated environment. The robot maps the maze using LIDAR, calculates the optimal path to the exit using the A* algorithm, and visualizes the process in RViz. The entire system runs within a Docker container for portability and consistency.

## Key Features
1. **Simulated Robot in Gazebo Harmonic**  
   - Differential drive robot with accurate kinematics.  
   - LIDAR sensor with a 160° field of view (-1.396263 to 1.396263 radians).  

2. **Maze Exploration and Mapping**  
   - Real-time occupancy grid mapping with LIDAR data.  
   - Visualization of the maze and the robot's position in RViz.  

3. **Path Optimization with A***  
   - After mapping, the A* algorithm computes the shortest path from start to exit.  

4. **Containerized Setup**  
   - Docker ensures reproducibility with pre-configured ROS 2 Jazzy and Gazebo Harmonic.  

## Installation
1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-folder>
2. Indicate the system where to look the ROS instalation:
   ```bash
   source /opt/ros/jazzy/setup.bash
3. Define env variable regarding the Gazebo version:
   ```bash
   export GZ_VERSION=harmonic
4. Access the project directory
   ```bash
   cd root/template_ws
   source install/local_setup.bash
5. Run the simulation:
   ```bash
   ros2 launch ros_gz_example_bringup diff_drive.launch.py
6. After running the simulation run the best_path.py script to get the best path based on the generated map:
   ```bash
   python3 best_path.py
