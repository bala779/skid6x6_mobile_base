# 🦾 Skid6x6 Mobile Robot - ROS 2 Package

## Overview

This project contains a complete **ROS 2 (Humble/Foxy)** setup for a **6x6 skid-steer mobile robot** — including URDF modeling, sensor integration, SLAM, and Navigation Stack (Nav2).  
It simulates and navigates a 6-wheel differential-drive robot in Gazebo and RViz.

The system integrates:
- URDF and 3D model description
- Odometry and sensor fusion (EKF)
- SLAM for mapping (using LiDAR)
- Nav2 stack for autonomous navigation
- World environments for simulation

## 🧩 Repository Structure

edge_force_ws/

    - src
    
    - skid6x6_description/
    
    - skid6x6_navigation/

## 📦 Packages Description

### 1️⃣ skid6x6_description
This package contains the **robot’s physical and visual description**, Gazebo world files, and launch files for simulation and visualization.

## ⚙️ Key Features
- Full 6-wheel skid-steer URDF model  
- Modular xacro-based design  
- LiDAR, and sensor plugin integration  
- Ready for Gazebo and RViz simulation  
- Differential drive controller configuration

### 2️⃣ skid6x6_navigation
This package provides **localization, mapping, and navigation** functionality using Nav2 and SLAM Toolbox.

## ⚙️ Key Features
- SLAM Toolbox integration for real-time mapping  
- EKF-based odometry
- Preconfigured RViz setup for visualization  
- Map saving and loading utilities
- Navigation2 (Nav2) stack for autonomous path planning  - In Progress
  

## 🚀 Build & Run Instructions

**1️⃣ Build the Workspace**

This project uses two additional packages for 3D mapping and scan merging:

lidarslam_ros2 – for LiDAR-based 3D SLAM and mapping.

scan merger    – for merging multiple LiDAR scans into a single point cloud.

## 🧩 Steps to Build: 

  - #### Clone the repositories

      - git clone https://github.com/rsasaki0109/lidarslam_ros2.git

      - git clone https://github.com/LCAS/scans_merger.git

  - #### Build the workspace

      - colcon build

  - #### Source the setup file

      - source install/setup.bash


2️⃣ **Launch simulation**

ros2 launch skid6x6_description skid6x6_launch.py

3️⃣ **Run SLAM Mapping**

ros2 launch skid6x6_navigation skid6x6_mapping_launch.py

4️⃣ **Run Navigation Stack**

ros2 launch skid6x6_navigation skid6x6_nav2_launch.py

5️⃣ **Save the generated map**

ros2 service call /map_save std_srvs/srv/Empty {}

