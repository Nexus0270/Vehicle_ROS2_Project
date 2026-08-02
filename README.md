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

### 2. 3D Mapping & SLAM Layer (Development Guide - Not Used in Final Release)
**File:** [`3D-Mapping-SLAM-Layer_RTABMap-FastMapping_Setup_Guide.md`](./3D-Mapping-SLAM-Layer_RTABMap-FastMapping_Setup_Guide.md)

> ⚠️ **Note:** This guide was created during development but was **not used in the final release**. The final release uses a different approach as documented in the section below.

This guide covers the visual SLAM and voxel map generation pipeline:
* Publishing the static TF between the camera and lidar.
* Configuring RTAB-Map for visual odometry (`rgbd_odometry`) and lidar-assisted SLAM fusion.
* Installing and configuring Intel's FastMapping package for 3D OctoMap generation.
* Visualizing the complete pipeline (PointClouds, OccupancyGrids, and MarkerArrays) in RViz2.

### 3. Final Release - Guarded Teleoperation + Lidar SLAM

> 📄 **For complete documentation, see [`HOW_TO_RUN.txt`](./HOW_TO_RUN.txt)** — this file contains the full operational guide including SSH access, headless testing, and troubleshooting.

**Current Mapping & SLAM Implementation:**

The final release uses a **lidar-based SLAM approach** rather than the RTAB-Map + FastMapping pipeline documented in Section 2. Key differences:

- **SLAM Method**: Lidar ICP (scan matching) for odometry — works by pushing the robot by hand
- **Camera Role**: Depth camera is used for **obstacle detection cross-check** (lidar vs camera distance agreement), not for visual odometry
- **RTAB-Map**: Still used, but primarily for building the 2D occupancy grid map from lidar scans
- **3D Mapping**: Not used in final release — the camera runs at 424x240 @ 15 Hz for faster keyframe capture, but the depth data feeds the safety guard's cross-check rather than 3D reconstruction

**Quick Start (one command, does everything):**
```bash
cd ~/ARS_Project
./run_teleop_slam.sh
```

This starts the whole stack and then hands you the keyboard. Press Ctrl-C to quit - it stops the robot and shuts the stack down.

The camera runs 424x240 @ 15 Hz by DEFAULT. That is deliberate and is the better mapping mode: at 640x480 the colour stream only manages 3.4 Hz on USB2, and RTAB-Map syncs colour with depth, so the whole pipeline was capped at 3.4 Hz keyframes. 424x240 gives 15 Hz on both - 4.4x more keyframes.

**Options:**
- `./run_teleop_slam.sh --no-camera` - lidar only (camera unplugged)
- `./run_teleop_slam.sh --no-slam` - guarded teleop only, much lighter
- `./run_teleop_slam.sh --no-rviz` - no visualisation window
- `./run_teleop_slam.sh --full-res` - 640x480 camera (SLOWER mapping)
- `./run_teleop_slam.sh --bench` - close-quarters thresholds (0.15 m)
- `./run_teleop_slam.sh --wheel-odom` - encoders instead of lidar ICP

**Manual Start:**
```bash
# Terminal 1 - the stack
source /opt/ros/jazzy/setup.bash
source ~/ARS_Project/ydlidar_ros2_ws/install/setup.bash
source ~/ARS_Project/ars_ws/install/setup.bash
ros2 launch ars_base teleop_slam.launch.py

# Terminal 2 - teleop keyboard
source /opt/ros/jazzy/setup.bash
source ~/ARS_Project/ars_ws/install/setup.bash
xset r rate 200 40
ros2 run ars_base teleop_keys
```

**Teleop Controls (hold to move, release to stop):**
- `w` - forward
- `s` - backward
- `a` - strafe LEFT (pure sideways - mecanum, no rotation)
- `d` - strafe RIGHT
- `q` - rotate left
- `e` - rotate right
- `space` - stop now
- `+ / -` - speed up / down
- `Ctrl-C` - quit (publishes a stop first)

**Obstacle Auto-Stop:**
The safety guard sits between teleop and the wheels. Motion TOWARD anything closer than stop_distance is refused, even if you hold the key.

Defaults: hard stop < 0.40 m, slow zone 0.40 - 0.80 m

Change on the fly:
```bash
ros2 param set /safety_guard stop_distance 0.30
ros2 param set /safety_guard slow_distance 0.60
```

Or at launch:
```bash
ros2 launch ars_base teleop_slam.launch.py stop_distance:=0.5
```

**Watching It Work:**
```bash
ros2 topic echo /safety/blocked            # true the moment it refuses
ros2 topic echo /safety/min_range          # nearest obstacle, metres
ros2 topic echo /obstacle_check/summary    # lidar vs camera agreement
ros2 topic echo /odom --field pose.pose    # wheel odometry
```

**RViz - Seeing the Map:**
```bash
ros2 run rviz2 rviz2 -d ~/ARS_Project/ars_ws/install/ars_base/share/ars_base/config/slam.rviz
```

Shows one xyz triad on the robot, the occupancy grid, live laser points, the robot model, and a cyan ICP odometry arrow. Drive around and the map fills in. SLAM tracks motion from the LIDAR (ICP scan matching), so it works even if you push the robot by hand.

To use wheel encoders for odometry instead:
```bash
ros2 launch ars_base teleop_slam.launch.py odom_source:=wheels
```

**Stopping / Cleaning Up:**
```bash
pkill -9 -f "install/ars_base/lib|ydlidar_ros2_driver_node|realsense2_camera|rtabmap|rviz2"
```

### 4. Mecanum Drive Base & Teleoperation


This section contains the pinout configurations, Arduino firmware, and python scripts for driving the mecanum base.

**Files:**
* [`Pinout_Reference.md`](./Pinout_Reference.md) – Documents the Rev.F pin map for the Mecanum Drive and Encoders wired to an Arduino Mega 2560. Details the hardware connections for the Cytron MDDRC10 drivers and JGB37-520 motors.
* [`encoder_calibration.ino`](./encoder_calibration.ino) – An Arduino utility to measure `TICKS_PER_REV` and derive `TICKS_PER_METER` for the four mecanum wheels It safely holds the motor drivers at neutral while wheels are turned manually for calibration.
* [`teleop_keyboard.py`](./teleop_keyboard.py) – A hold-to-move keyboard teleoperation script utilizing `pynput`. It repeats the motion command at 20 Hz to feed the firmware's 200 ms watchdog timeout, immediately halting the robot when the key is released.
* [`teleop_ssh.py`](./teleop_ssh.py) – An SSH-friendly keyboard teleoperation script that reads the terminal directly in raw mode, making it work without a graphical display. It uses latched control where tapping a direction keeps the robot moving and tapping space or 's' stops it.

### 5. Robot 3D Model


The robot's 3D printable components are provided as STL files:

- [`Base.stl`](./Base.stl) – Main chassis of the robot.
- [`Roof.stl`](./Roof.stl) – Top layer base for lidar.
- [`Piece3.2.STL`](./Piece3.2.STL) – Additional mounting or structural component to connect the Lidar. 

These files can be opened in any CAD software or slicer (e.g., FreeCAD, Cura, PrusaSlicer, Bambu Studio) for viewing, modification, or 3D printing.
