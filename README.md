# MRAC-robotic-spatial-analysis
## Overview
This is the repository for MRAC 24/25 Workshop 2.2, which focuses on advanced 3D scanning and spatial analysis of building environments using a combination of photogrammetry, 3D LiDAR, robot platform, robot dog, and drone. This project emphasizes the use of robotic systems to autonomously or via teleoperation gather and analyze spatial data in different environments. Each system contributes to separate parts of the scanning and analysis process, providing a comprehensive approach to spatial data collection and 3D modeling.


## Table of Contents

1. [Overview](#overview)
2. [Key Technologies](#key-technologies)
3. [Project Structure](#project-structure)
4. [Installation](#installation)
   - [Required Operating Systems](#required-operating-systems)
   - [Foxglove Studio Installation](#foxglove-studio-installation)
   - [SSH Server Setup](#ssh-server-setup)
   - [PCL Tools](#pcl-tools)
5. [Usage](#usage)
   - [Husky Connection](#husky-connection)
   - [Launching ROS2 Nodes](#launching-ros2-nodes)
   - [Foxglove Bridge Setup](#foxglove-bridge-setup)

## Key Technologies

- **Photogrammetry**: High-resolution 3D model generation from photographs.
- **3D LiDAR**: Laser-based scanning for precise spatial mapping.
- **Robot Platforms (Husky A200)**: Ground-based robotic platforms equipped for 3D scanning and data collection.
- **Robot Dog(unitree GO2)**: Agile robots designed for scanning in challenging environments.
- **Drone**: Aerial data collection to complement ground-based scans.
- **ROS (Robot Operating System)**: Used for communication and control of the rover and robot dog. 

## Project Structure

- **Rover and Robot Dog**: Teleoperated robotic platforms, using ROS for communication and control, designed for navigating and scanning different environments.
- **Photogrammetry & LiDAR Systems**: These technologies operate independently to gather detailed spatial data from different areas.
- **Data Analysis**: Spatial data is analyzed separately for each scanning method, enabling a comprehensive understanding of the environment.

## Installation
**Required Operating Systems:** Ubuntu 22.04, Windows10/11

**Foxglove Studio Installation**  
```
sudo snap install foxglove-studio
```

**SSH Server**
```
sudo apt install openssh-server
```
**PCL Tools**
```
sudo apt install pcl-tools
```


## Usage
## Husky Connection

1. Connect to the iaac_husky hotspot (WIFI password: EnterIaac22@)
2. Open a terminal in your computer and ssh to the husky (user password: iaac)
   ```
   ssh iaac@10.42.0.1
   ```
3. Use tmux as the terminal on husky robot
   ```
   tmux new -s husky
   ```
4. launch the ros2 nodes for mobile base (first tmux terminal)
   ```
   ros2 launch /etc/clearpath/platform/launch/platform-service.launch.py
   ```
5. launch the ros2 nodes for lidar (second tmux terminal)
   ```
   ros2 launch livox_ros_driver2 msg_MID360_launch.py
   ```
6. launch the mapping node (third tmux terminal)
   ```
   ros2 launch fast_lio mapping.launch.py
   ```
7. launch the usb_cam node to bring up the camera (fourth tmux terminal)
   ```
   ros2 launch usb_cam camera.launch.py
   ```
8. launch the foxglove bridge for visualization (fifth tmux terminal)
   ```
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml
   ```
9. Open foxglove-studio and change the web address to ``10.42.0.1``
10. Load the panel from this repository

## Unitree Go2 
1. Clone this repository
2. Build the docker image from the go2_robot package
```
cd MRAC-robot-spatial-analysis/go2_robot
.docker/build_image.sh
```
3. Run the image 
```
.docker/run_user.sh
```
Or run the image with Nvidia graphic card
```
.docker/run_user_nvidia.sh
```
4. Change the ownership of the folder
```
sudo chown -R YOUR_USER_NAME /dev_ws
```
5. Bringup the robot dog
```
ros2 launch go2_bringup go2.launch.py
```
6. Build the voxel map from 4D lidar
```
ros2 run go2_voxelmap voxelmap_node

```
