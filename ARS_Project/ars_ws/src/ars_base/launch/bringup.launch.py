#!/usr/bin/env python3
"""
Robot bringup: description + TF tree, the Arduino serial bridge, and the
YDLidar X2.

    ros2 launch ars_base bringup.launch.py

NOTE ON THE LIDAR
  This starts the ydlidar DRIVER NODE directly instead of calling
  ydlidar_launch.py, because that launch file also spawns a static
  base_link -> laser_frame transform which conflicts with the URDF's own
  laser_joint. Same driver, same params file, just without the extra TF.

Teleop from another terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg = get_package_share_directory('ars_base')
    default_params = os.path.join(pkg, 'config', 'ars_base.yaml')

    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')
    lidar_params = os.path.join(ydlidar_share, 'params', 'ydlidar.yaml')

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='serial_bridge parameters'),
        DeclareLaunchArgument('use_lidar', default_value='true',
                              description='start the YDLidar X2 driver'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='run the bridge with no Arduino attached'),
    ]

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'description.launch.py')))

    bridge = Node(
        package='ars_base',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {'dry_run': LaunchConfiguration('dry_run')},
        ],
    )

    lidar = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace='/',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_params],
        condition=IfCondition(LaunchConfiguration('use_lidar')),
    )

    return LaunchDescription(args + [description, bridge, lidar])
