#!/usr/bin/env python3
"""
Nav2 autonomous navigation for the ARS mecanum robot.

FULL STACK, one terminal each:
    ros2 launch ars_base bringup.launch.py     # description + bridge + lidar
    ros2 launch ars_base mapping.launch.py     # camera + RTAB-Map (SLAM)
    ros2 launch ars_base nav2.launch.py        # this file
    rviz2

Then in RViz set Fixed Frame to "map", add the Nav2 panel, and use the
"2D Goal Pose" tool to send a target.

WHY THERE IS NO map_server OR amcl HERE
  RTAB-Map already publishes the occupancy grid (/rtabmap/map) AND the
  map->odom transform. Running AMCL as well would put two nodes in a fight
  over map->odom, and the TF tree would jitter between their two answers.
  So mapping.launch.py must be running before this.

  If you later want to navigate a PRE-SAVED map instead of live SLAM, that
  is when you add map_server + amcl and stop running RTAB-Map.

The nodes are listed explicitly rather than pulled in from nav2_bringup's
navigation_launch.py, so the node graph in your report matches this file
one-for-one.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Brought up in this order by the lifecycle manager. Order matters: the
# costmaps inside planner/controller need TF and the map to already exist.
LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
]


def generate_launch_description():
    pkg = get_package_share_directory('ars_base')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 parameter file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='automatically configure+activate the stack'),
    ]

    common = [params_file, {'use_sim_time': use_sim_time}]

    nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=common,
             # The controller publishes to cmd_vel_nav; velocity_smoother
             # takes that and emits the final cmd_vel the bridge consumes.
             # Without this pair of remaps the smoother is bypassed and the
             # robot gets raw, unsmoothed velocity steps.
             remappings=[('cmd_vel', 'cmd_vel_nav')]),

        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen', parameters=common),

        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=common),

        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=common,
             remappings=[('cmd_vel', 'cmd_vel_nav')]),

        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=common),

        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=common),

        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=common,
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')]),

        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time},
                         {'autostart': autostart},
                         {'node_names': LIFECYCLE_NODES}]),
    ]

    return LaunchDescription(args + nodes)
