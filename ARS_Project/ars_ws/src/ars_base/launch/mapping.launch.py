#!/usr/bin/env python3
"""
3D mapping layer: RealSense D455 + RTAB-Map, on top of bringup.launch.py.

    ros2 launch ars_base bringup.launch.py        # terminal 1
    ros2 launch ars_base mapping.launch.py        # terminal 2

WHAT CHANGED VS THE GUIDE'S PART D, AND WHY
  1. visual_odometry is now FALSE by default; RTAB-Map consumes the wheel
     odometry from serial_bridge instead. The log evidence for this: in the
     05:50 session rgbd_odometry lost tracking for 371 CONSECUTIVE frames
     (~25 s at 15 fps) with quality=0 -- the "crash when moving fast". Wheel
     encoders do not motion-blur, so this failure mode disappears. RTAB-Map
     still corrects the encoders' drift through loop closure and the lidar,
     via the map->odom transform.
     Set visual_odometry:=true to go back to the old behaviour.

  2. frame_id is base_link, not camera_link. With a real URDF the robot has
     a proper body frame, so the camera is no longer pretending to be one.

  3. The camera runs at full resolution by default. The 424x240x15 profile
     existed only to stop the lidar's serial link being starved on a shared
     USB2 hub -- but both USB3 buses on this machine are empty, so putting
     the camera on its own USB3 port removes that constraint entirely, and
     the extra resolution/framerate is exactly what visual tracking wants.
     If the camera has to share a hub again, pass low_bandwidth:=true.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    rs_share = get_package_share_directory('realsense2_camera')
    rtab_share = get_package_share_directory('rtabmap_launch')

    low_bw = LaunchConfiguration('low_bandwidth')
    vis_odom = LaunchConfiguration('visual_odometry')

    args = [
        DeclareLaunchArgument('low_bandwidth', default_value='false',
                              description='reduced camera profile for a shared USB hub'),
        DeclareLaunchArgument('visual_odometry', default_value='false',
                              description='true = RGB-D odometry, false = wheel odometry'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('delete_db', default_value='true',
                              description='start a fresh map instead of extending ~/.ros/rtabmap.db'),
    ]

    # Flat topic names (camera_namespace:=/) so they match RTAB-Map's own
    # defaults -- avoids the doubled /camera/camera/... namespace.
    camera_common = {
        'enable_color': 'true',
        'enable_depth': 'true',
        'align_depth.enable': 'true',
        'pointcloud.enable': 'false',   # heaviest stream; RTAB-Map builds its own
        'camera_namespace': '/',
        'camera_name': 'camera',
    }

    use_cam = LaunchConfiguration('use_camera')

    # Two mutually exclusive camera blocks. Both conditions have to test
    # use_camera AND low_bandwidth together -- launch has no boolean operator
    # on Condition objects, so the logic goes inside a PythonExpression.
    def cam_condition(want_low_bw):
        return IfCondition(PythonExpression([
            "'", use_cam, "' == 'true' and '", low_bw, "' == ",
            "'true'" if want_low_bw else "'false'",
        ]))

    camera_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rs_share, 'launch', 'rs_launch.py')),
        launch_arguments=camera_common.items(),
        condition=cam_condition(False),
    )

    camera_low = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rs_share, 'launch', 'rs_launch.py')),
        launch_arguments={**camera_common,
                          'rgb_camera.color_profile': '424x240x15',
                          'depth_module.depth_profile': '480x270x15'}.items(),
        condition=cam_condition(True),
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rtab_share, 'launch', 'rtabmap.launch.py')),
        launch_arguments={
            'rgb_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'frame_id': 'base_link',
            'odom_topic': '/odom',
            'visual_odometry': vis_odom,
            'subscribe_scan': 'true',
            'scan_topic': '/scan',
            'approx_sync': 'true',
            'qos': '2',
            'rtabmap_viz': 'false',
            'rviz': 'false',
            # Only meaningful when visual_odometry:=true. Default is 0, which
            # means "stay lost forever"; 1 reinitialises after a single failed
            # frame, so a blur-induced dropout self-heals in ~70 ms instead of
            # freezing the map until you restart the node.
            'odom_args': '--Odom/ResetCountdown 1',
        }.items(),
    )

    return LaunchDescription(args + [camera_full, camera_low, rtabmap])
