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

------------------------------------------------------------------------------------
# This project was developed based in the ros_gz_project_template
A template project integrating ROS 2 and Gazebo simulator.

## Included packages

* `ros_gz_example_description` - holds the sdf description of the simulated system and any other assets.

* `ros_gz_example_gazebo` - holds gazebo specific code and configurations. Namely this is where systems end up.

* `ros_gz_example_application` - holds ros2 specific code and configurations.

* `ros_gz_example_bringup` - holds launch files and high level utilities.


## Install

For using the template with Gazebo Fortress switch to the `fortress` branch of this repository, otherwise use the default branch `main` for Gazebo Harmonic onwards.

### Requirements

1. Choose a ROS and Gazebo combination https://gazebosim.org/docs/latest/ros_installation

   Note: If you're using a specific and unsupported Gazebo version with ROS 2, you might need to set the `GZ_VERSION` environment variable, for example:

    ```bash
    export GZ_VERSION=harmonic
    ```
    Also need to build [`ros_gz`](https://github.com/gazebosim/ros_gz) and [`sdformat_urdf`](https://github.com/ros/sdformat_urdf) from source if binaries are not available for your chosen combination.

1. Install necessary tools

    ```bash
    sudo apt install python3-vcstool python3-colcon-common-extensions git wget
    ```

### Use as template
Directly `Use this template` and create your project repository on Github.

Or start by creating a workspace and cloning the template repository:

   ```bash
   mkdir -p ~/template_ws/src
   cd ~/template_ws/src
   git clone https://github.com/gazebosim/ros_gz_project_template.git
   ```

## Usage

1. Install dependencies

    ```bash
    cd ~/template_ws
    source /opt/ros/$ROS_DISTRO/setup.bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro <ROS_DISTRO>
    ```

1. Build the project

    ```bash
    colcon build --cmake-args -DBUILD_TESTING=ON
    ```

1. Source the workspace

    ```bash
    . ~/template_ws/install/setup.sh
    ```

1. Launch the simulation

    ```bash
    ros2 launch ros_gz_example_bringup diff_drive.launch.py
    ```

For a more detailed guide on using this template see [documentation](https://gazebosim.org/docs/latest/ros_gz_project_template_guide).

