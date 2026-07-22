# ROS 2 Jazzy 3D Mapping & SLAM Stack

This repository contains setup guides for configuring a 3D mapping and SLAM pipeline using ROS 2 Jazzy on Ubuntu 24.04. 

The hardware stack utilizes an Intel RealSense D455 and a YDLidar X2, operating without wheel encoders by relying on visual odometry.

## Documentation

### 1. Hardware & Sensor Setup
**File:** [`RealSense-D455_YDLidar-X2_Setup_Guide.md`](./RealSense-D455_YDLidar-X2_Setup_Guide.md)

This guide covers the driver installation and configuration for the sensors, including:
* Building `librealsense2` and the `realsense-ros` wrapper from source on Ubuntu 24.04.
* Building the `YDLidar-SDK` and configuring the `ydlidar_ros2_driver` specifically for the X2 model (115200 baudrate, single channel).
* Managing USB bandwidth contention when running both devices on a single USB hub.

### 2. 3D Mapping & SLAM Layer
**File:** [`3D-Mapping-SLAM-Layer_RTABMap-FastMapping_Setup_Guide.md`](./3D-Mapping-SLAM-Layer_RTABMap-FastMapping_Setup_Guide.md)

This guide covers the visual SLAM and voxel map generation pipeline:
* Publishing the static TF between the camera and lidar.
* Configuring RTAB-Map for visual odometry (`rgbd_odometry`) and lidar-assisted SLAM fusion.
* Installing and configuring Intel's FastMapping package for 3D OctoMap generation.
* Visualizing the complete pipeline (PointClouds, OccupancyGrids, and MarkerArrays) in RViz2.

### 2. Robot 3D Model

The robot's 3D printable components are provided as STL files:

- [`Base.stl`](./Base.stl) – Main chassis of the robot.
- [`Roof.stl`](./Roof.stl) – Top layer base for lidar 
- [`Piece3.2.STL`](./Piece3.2.STL) – Additional mounting or structural component to connect Lidar. 

These files can be opened in any CAD software or slicer (e.g., FreeCAD, Cura, PrusaSlicer, Bambu Studio) for viewing, modification, or 3D printing.
