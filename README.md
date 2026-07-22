# ROS 2 Jazzy 3D Mapping & SLAM Stack

This repository contains setup guides for configuring a 3D mapping and SLAM pipeline using ROS 2 Jazzy on Ubuntu 24.04. 

The hardware stack utilizes an Intel RealSense D455 and a YDLidar X2, operating without wheel encoders by relying on visual odometry. It also includes the underlying firmware, calibration utilities, and teleoperation scripts for the robot's Arduino-controlled mecanum drive base.

**File:** [`Physical Robot Image`](./Robot.jpeg)

## Documentation & Code

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

### 3. Mecanum Drive Base & Teleoperation

This section contains the pinout configurations, Arduino firmware, and python scripts for driving the mecanum base.

**Files:**
* [`Pinout_Reference.md`](./Pinout_Reference.md) – Documents the Rev.F pin map for the Mecanum Drive and Encoders wired to an Arduino Mega 2560. Details the hardware connections for the Cytron MDDRC10 drivers and JGB37-520 motors.
* [`encoder_calibration.ino`](./encoder_calibration.ino) – An Arduino utility to measure `TICKS_PER_REV` and derive `TICKS_PER_METER` for the four mecanum wheels It safely holds the motor drivers at neutral while wheels are turned manually for calibration.
* [`teleop_keyboard.py`](./teleop_keyboard.py) – A hold-to-move keyboard teleoperation script utilizing `pynput`. It repeats the motion command at 20 Hz to feed the firmware's 200 ms watchdog timeout, immediately halting the robot when the key is released.
* [`teleop_ssh.py`](./teleop_ssh.py) – An SSH-friendly keyboard teleoperation script that reads the terminal directly in raw mode, making it work without a graphical display. It uses latched control where tapping a direction keeps the robot moving and tapping space or 's' stops it.

### 4. Robot 3D Model

The robot's 3D printable components are provided as STL files:

- [`Base.stl`](./Base.stl) – Main chassis of the robot.
- [`Roof.stl`](./Roof.stl) – Top layer base for lidar.
- [`Piece3.2.STL`](./Piece3.2.STL) – Additional mounting or structural component to connect the Lidar. 

These files can be opened in any CAD software or slicer (e.g., FreeCAD, Cura, PrusaSlicer, Bambu Studio) for viewing, modification, or 3D printing.
