#!/usr/bin/env python3
"""
Publishes the robot description and the whole static TF tree beneath
base_link (wheels, laser_frame, camera_link).

This REPLACES both of the ad-hoc static_transform_publisher calls that used
to fight over laser_frame's parent:
  * the guide's "Terminal 1"  camera_link -> laser_frame
  * ydlidar_launch.py's       base_link  -> laser_frame
Do not run either of those alongside this.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('ars_base')
    default_model = os.path.join(pkg, 'urdf', 'ars_robot.urdf.xacro')

    model_arg = DeclareLaunchArgument(
        'model', default_value=default_model,
        description='Absolute path to the robot xacro/urdf')

    # ParameterValue(..., value_type=str) is required: without it the xacro
    # output gets reinterpreted (it starts with "<?xml", which the parameter
    # layer will happily mangle) and robot_state_publisher rejects it.
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    return LaunchDescription([
        model_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
    ])
