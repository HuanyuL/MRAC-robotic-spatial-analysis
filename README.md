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

### Husky Connection

1. **Connect to the Husky robot**:
   - Connect to the **iaac_husky** hotspot (WIFI password: `EnterIaac22@`).
   - Open a terminal on your computer and SSH into the Husky robot (user password: `iaac`):
     ```bash
     ssh iaac@10.42.0.1
     ```

2. **Using tmux for terminal management**:
   - Start a new tmux session:
     ```bash
     tmux new -s husky
     ```
     This command will start a new tmux session named `husky`.

3. **Split the tmux terminal**:
   - Once inside tmux, you can split the terminal window to launch multiple processes concurrently. To split the tmux terminal:
     - Press `Ctrl + B`, then release both keys and press `%` to split the terminal vertically.
     - To split horizontally, press `Ctrl + B`, then release both keys and press `"` (double quote).

4. **Launch the ROS2 nodes in separate tmux panes**:

   **Terminal 1 (Mobile Base)**:
   - In the first tmux pane, launch the mobile base:
     ```bash
     ros2 launch /etc/clearpath/platform/launch/platform-service.launch.py
     ```

   **Terminal 2 (LiDAR)**:
   - In the second tmux pane, launch the LiDAR:
     ```bash
     ros2 launch livox_ros_driver2 msg_MID360_launch.py
     ```

   **Terminal 3 (Mapping)**:
   - In the third tmux pane, launch the mapping node:
     ```bash
     ros2 launch fast_lio mapping.launch.py
     ```

   **Terminal 4 (Foxglove Bridge)**:
   - In the fourth tmux pane, launch the Foxglove bridge:
     ```bash
     ros2 launch foxglove_bridge foxglove_bridge_launch.xml
     ```

5. **Open Foxglove Studio**:
   - Open **Foxglove Studio** and set the web address to `10.42.0.1`.
   - Load the panel from this repository to visualize the data.

### Unitree Go2 Robot

1. **Clone the repository**:
   - Clone the MRAC-robot-spatial-analysis repository to your system.

2. **Build the Docker image**:
   - **Note**: You only need to build the image when using the Unitree Go2 robot.
   - Navigate to the `go2_robot` package directory and build the Docker image:
     ```bash
     cd MRAC-robot-spatial-analysis/go2_robot
     .docker/build_image.sh
     ```

3. **Run the Docker image**:
   - To run the image, use the following command:
     ```bash
     .docker/run_user.sh
     ```
   - Or, if you're using an Nvidia graphic card, run the image with the Nvidia runtime:
     ```bash
     .docker/run_user_nvidia.sh
     ```

4. **Change the folder ownership**:
   - To avoid permission issues, change the ownership of the workspace folder:
     ```bash
     sudo chown -R YOUR_USER_NAME /dev_ws
     ```

5. **Terminal Setup for Go2 Robot**:

   **Terminal 1 (Go2 Bringup)**:
   - In the first terminal, bring up the robot dog:
     ```bash
     ros2 launch go2_bringup go2.launch.py
     ```

   **Terminal 2 (RQT Visualization)**:
   - In the second terminal, use RQT to visualize the camera image or other sensor data:
     ```bash
     rqt
     ```
     You can use `rqt_image_view` to see the camera feed or other visualization plugins for other sensors.

   **Terminal 3 (Teleoperation)**:
   - In the third terminal, use the keyboard teleoperation for controlling the robot:
     ```bash
     ros2 run teleop_twist_keyboard teleop_twist_keyboard
     ```
     This will allow you to control the Unitree Go2 robot using the keyboard.

   **Terminal 4 (Map Saver Service)**:
   - In the fourth terminal, use the map saver service to save the generated voxel map:
     ```bash
     ros2 run go2_interfaces voxel_map_saver
     ```

   **OPTIONAL Terminal 5 (Rosbag Recording)**:
   - If you want to record the data to a rosbag, use the following command to record the selected topics:
     ```bash
     ros2 bag record /tf /pointcloud /utlidar/robot_odom /camera/compressed
     ```
     **Warning**: Make sure your computer has enough space for recording, and **do not record for too long** to avoid filling up the disk. You can stop the recording by pressing `Ctrl + C` and you need to download it from the container.

   **OPTIONAL Terminal 6 (save the map)**
   - **Important**: You must wait for the scanning to finish before saving the map. **AND PLEASE CHANGE THE PATH!!!!!!!!!!!!!!!!**
   - Once the scanning is complete, you can save the voxel map using the following service call:
     ```bash
     ros2 service call /save_voxel_cloud go2_interfaces/srv/SaveVoxelCloud "{filename: '/YOUR FILE PATH/export.pcd'}"
     ```

