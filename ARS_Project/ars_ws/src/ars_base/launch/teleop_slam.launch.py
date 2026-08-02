#!/usr/bin/env python3
"""
THE ONE COMMAND. Guarded teleoperation + lidar SLAM + camera 3D mapping.

    ros2 launch ars_base teleop_slam.launch.py

Then, in a second terminal (the keyboard node needs its own stdin, so it
cannot live inside a launch file):

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r cmd_vel:=/cmd_vel_raw

Or just run ./run_teleop_slam.sh, which does both.

DATA FLOW

    teleop ──> /cmd_vel_raw ──> safety_guard ──> /cmd_vel ──> serial_bridge ──> wheels
                                    ^
                                    │ blocks motion toward obstacles
                              /scan │
    YDLidar ──────────────────┴─────┴──> RTAB-Map ──> map, 3D cloud
    D455 ─────> depth + rgb ────────────────┘   │
                    └──> obstacle_verifier <────┘  cross-checks lidar vs depth

WHY TELEOP PUBLISHES TO /cmd_vel_raw
  Nothing reaches the wheels without passing through the guard. The bridge
  listens only to /cmd_vel, and only the guard publishes there, so there is no
  way to accidentally bypass the obstacle check -- including later, when Nav2
  becomes another velocity source.

ARGUMENTS
  use_camera:=false     lidar-only, if the camera is unplugged
  use_rtabmap:=false    skip SLAM, keep guarded teleop (much lighter)
  low_bandwidth:=true   reduced camera profile for a shared USB hub
  stop_distance:=0.5    metres; hard stop threshold
  use_rviz:=false       skip the visualisation window
  odom_source:=wheels   use encoders instead of lidar ICP
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('ars_base')
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')
    rs_share = get_package_share_directory('realsense2_camera')
    rtab_share = get_package_share_directory('rtabmap_launch')

    use_camera = LaunchConfiguration('use_camera')
    use_rtabmap = LaunchConfiguration('use_rtabmap')
    low_bw = LaunchConfiguration('low_bandwidth')

    odom_source = LaunchConfiguration('odom_source')
    # 'lidar' -> icp_odometry owns odom->base_link and the SLAM tracks motion
    # from scan matching alone, so it works even when the robot is pushed by
    # hand and needs none of the unmeasured wheel geometry.
    # 'wheels' -> serial_bridge owns odom->base_link from the encoders.
    # Exactly one of them may publish that transform; two publishers for one
    # child frame is a startup race, which is the bug that silently dropped
    # 1246 scans in an earlier session.
    is_lidar_odom = PythonExpression(["'", odom_source, "' == 'lidar'"])
    is_wheel_odom = PythonExpression(["'", odom_source, "' == 'wheels'"])

    args = [
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_rtabmap', default_value='true'),
        # DEFAULT TRUE despite the name -- this is measurably the BETTER
        # mapping mode on this hardware, not a degraded fallback.
        #   false: colour 640x480 @ 3.4 Hz, depth 640x480 @ 12.7 Hz
        #   true : colour 424x240 @ 15.0 Hz, depth 424x240 @ 15.0 Hz
        # RTAB-Map syncs colour with depth, so the 3.4 Hz colour throttled the
        # whole pipeline to 3.4 Hz keyframes; depth frames were discarded
        # waiting for a colour partner. At 424x240 both streams fit USB2
        # comfortably and mapping runs 4.4x faster. Since align_depth aligns
        # depth TO colour, the higher colour resolution was setting the depth
        # resolution anyway -- so little real detail is lost, and far more
        # viewpoints are gained, which is what actually fills a reconstruction.
        DeclareLaunchArgument('low_bandwidth', default_value='true',
                              description='424x240@15 (recommended). false = 640x480 but only 3.4 Hz'),
        DeclareLaunchArgument('odom_source', default_value='lidar',
                              description="'lidar' (ICP scan matching) or 'wheels' (encoders)"),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg, 'config', 'ars_base.yaml')),
        DeclareLaunchArgument('stop_distance', default_value='0.40'),
        DeclareLaunchArgument('slow_distance', default_value='0.80'),
        DeclareLaunchArgument('record', default_value='true',
                              description='write a black-box log to ~/ARS_Project/logs'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='open RViz showing the map and camera'),
    ]

    # ---- visualisation ----------------------------------------------------
    # Shows the lidar SLAM map, the camera's 3D cloud, and both live camera
    # images in one window.
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(pkg, 'config', 'slam.rviz')],
        output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # ---- robot description + TF tree -------------------------------------
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'description.launch.py')))

    # ---- Arduino bridge ---------------------------------------------------
    bridge = Node(
        package='ars_base', executable='serial_bridge', name='serial_bridge',
        output='screen', emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            # Yields TF ownership to icp_odometry when the lidar is the odom
            # source. /odom is still published either way, so wheel odometry
            # remains available for comparison and drift checks.
            {'publish_tf': ParameterValue(is_wheel_odom, value_type=bool)},
        ],
    )

    # ---- lidar ICP odometry ----------------------------------------------
    # Derives motion by matching consecutive scans, so SLAM tracks the robot
    # whether it is driven, pushed, or carried.
    icp_odom = Node(
        package='rtabmap_odom', executable='icp_odometry', name='icp_odometry',
        output='screen', emulate_tty=True,
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'wait_for_transform': 0.2,
            'qos': 2,                     # /scan is best-effort sensor QoS
            'approx_sync': True,
            # 2D lidar ICP. PointToPlane is for 3D clouds and degrades badly
            # on a single scan line, so point-to-point is correct here.
            'Icp/PointToPlane': 'false',
            'Icp/VoxelSize': '0.05',
            'Icp/RangeMax': '8.0',
            # Loosened from 0.15: /scan runs at only ~3.8 Hz on the shared USB
            # hub, so consecutive scans are far apart and a tight
            # correspondence window makes matching fail on ordinary motion.
            'Icp/MaxCorrespondenceDistance': '0.30',
            'Icp/Iterations': '30',
            'Icp/Epsilon': '0.001',
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'true',
            # Kalman-filter the pose output. Without this the raw ICP solution
            # is republished every scan, so its 10-40 mm frame-to-frame noise
            # shakes everything drawn in the map frame -- most visibly the live
            # point cloud, which is re-transformed through this pose on every
            # frame and therefore jitters as a whole. Filtering costs a little
            # responsiveness and buys a great deal of visual stability.
            # 0 = none, 1 = Kalman, 2 = particle.
            'Odom/FilteringStrategy': '1',
            'Odom/KalmanProcessNoise': '0.001',
            'Odom/KalmanMeasurementNoise': '0.01',
            # Do NOT emit a null (0,0,0) pose when tracking is lost. With this
            # true, every lost frame published the origin, which downstream
            # looks identical to the robot teleporting -- that is what the
            # "7 m jump while stationary" measurement was actually seeing.
            # Holding silence is honest; publishing a wrong pose is not.
            'publish_null_when_lost': False,
            # Both extremes are wrong here:
            #   1 -> re-initialises on a single failed match, so ordinary
            #        transient failures reset the pose.
            #   0 -> never recovers; one lost lock leaves odometry dead for
            #        the rest of the session (observed: 272 consecutive
            #        zero-ratio updates that never came back).
            # A few frames of grace lets brief failures ride out while still
            # recovering from a genuine loss.
            'Odom/ResetCountdown': '5',
            'Reg/Strategy': '1',          # ICP registration
        }],
        remappings=[('scan', '/scan'), ('odom', '/odom_icp')],
        condition=IfCondition(is_lidar_odom),
    )

    # ---- lidar ------------------------------------------------------------
    # Driver node directly, not ydlidar_launch.py, so its static
    # base_link->laser_frame TF cannot fight the URDF's laser_joint.
    lidar = LifecycleNode(
        package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node', namespace='/', output='screen',
        parameters=[os.path.join(ydlidar_share, 'params', 'ydlidar.yaml')],
    )

    # ---- safety guard -----------------------------------------------------
    guard = Node(
        package='ars_base', executable='safety_guard', name='safety_guard',
        output='screen', emulate_tty=True,
        parameters=[{
            'stop_distance': LaunchConfiguration('stop_distance'),
            'slow_distance': LaunchConfiguration('slow_distance'),
        }],
    )

    # ---- camera -----------------------------------------------------------
    cam_common = {
        'enable_color': 'true', 'enable_depth': 'true',
        'align_depth.enable': 'true',
        # LIVE colour point cloud on /camera/depth/color/points, published
        # every frame. This is what reacts instantly to movement in front of
        # the camera -- /rtabmap/cloud_map is the accumulated MAP and only
        # updates on new keyframes, so a waving hand never appears in it.
        # Note this is computed on the HOST by librealsense from the depth and
        # colour images already being received; it is NOT an extra USB stream,
        # so it costs CPU rather than bandwidth.
        'pointcloud.enable': 'true',
        'camera_namespace': '/', 'camera_name': 'camera',
        # Depth post-processing filters are left OFF, having been TRIED AND
        # REVERTED on 2026-07-23. Enabling spatial + temporal measured WORSE:
        # valid pixels 79.4% -> 71.6%, temporal noise 13.6 -> 34.5 mm.
        # Caveat: that test was not scene-controlled, so treat it as "no
        # demonstrated benefit" rather than proof of harm. There is a sound
        # reason to expect little gain though -- temporal filtering averages
        # across frames, which suits a FIXED camera; on a moving robot the
        # viewpoint changes every frame so it smears instead of denoising.
        # If revisiting, test with the robot stationary AND moving, and change
        # one filter at a time.
        # hole_filling should stay off regardless: it INVENTS depth the sensor
        # never measured, which is fine for a pretty picture but dishonest for
        # anything the obstacle checker reads.
    }

    def cam_cond(want_low):
        return IfCondition(PythonExpression([
            "'", use_camera, "' == 'true' and '", low_bw, "' == ",
            "'true'" if want_low else "'false'"]))

    camera_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rs_share, 'launch', 'rs_launch.py')),
        launch_arguments=cam_common.items(), condition=cam_cond(False))

    camera_low = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rs_share, 'launch', 'rs_launch.py')),
        launch_arguments={**cam_common,
                          'rgb_camera.color_profile': '424x240x15',
                          'depth_module.depth_profile': '480x270x15'}.items(),
        condition=cam_cond(True))

    # ---- camera/lidar cross-check ----------------------------------------
    verifier = Node(
        package='ars_base', executable='obstacle_verifier',
        name='obstacle_verifier', output='screen', emulate_tty=True,
        remappings=[
            ('depth_image', '/camera/aligned_depth_to_color/image_raw'),
            # Intrinsics for the ALIGNED depth stream. Must be the aligned
            # camera_info, not the raw depth module's -- alignment reprojects
            # depth into the colour sensor's frame, so the raw intrinsics
            # would put every point in the wrong place.
            ('depth_info', '/camera/aligned_depth_to_color/camera_info'),
        ],
        condition=IfCondition(use_camera),
    )

    # ---- black-box recorder ----------------------------------------------
    # Essential for headless runs: with no monitor there is no other way to
    # confirm afterwards that the auto-stop actually fired.
    logger = Node(
        package='ars_base', executable='event_logger', name='event_logger',
        output='screen', emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('record')),
    )

    # ---- SLAM -------------------------------------------------------------
    # Wheel odometry rather than visual: encoders do not motion-blur, which is
    # what caused the 25 s tracking blackout during fast motion.
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rtab_share, 'launch', 'rtabmap.launch.py')),
        launch_arguments={
            'rgb_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'frame_id': 'base_link',
            # Follows odom_source, so RTAB-Map always consumes whichever node
            # actually owns the odom->base_link transform.
            'odom_topic': PythonExpression(
                ["'/odom_icp' if '", odom_source, "' == 'lidar' else '/odom'"]),
            'visual_odometry': 'false',
            'subscribe_scan': 'true',
            'scan_topic': '/scan',
            'approx_sync': 'true',
            'qos': '2',
            'rtabmap_viz': 'false',
            'rviz': 'false',
            # Grid/Sensor: 0=laser only, 1=depth camera only, 2=BOTH.
            # RTAB-Map silently forces 0 whenever subscribe_scan is true, which
            # is why /rtabmap/cloud_map was a colourless 2,950-point lidar
            # slice. With 2 the 2D occupancy map still comes from the 360-deg
            # lidar (far better coverage than the camera's 86-deg FOV) while
            # the 3D cloud gains the D455's dense RGB points.
            #
            # DepthDecimation 2 halves the depth resolution used for the cloud
            # -- full resolution is ~300k points per keyframe and will bog the
            # machine down. RangeMax 4.0 discards far depth, which is where the
            # D455's error grows worst.
            # Density: DepthDecimation 1 uses EVERY depth pixel (4x the points
            # of 2), CellSize 0.015 merges points only within 15 mm instead of
            # 30 (roughly 2x more survive the voxel filter), RangeMax 5.0
            # keeps another metre of room. Together ~8x denser than the first
            # conservative guess, which measured only 28k points.
            # If this ever bogs down, DepthDecimation is the dial to turn
            # first -- it is the cheapest large reduction.
            # THE keyframe-rate control. Default 1 Hz means RTAB-Map throws
            # away ~13 of every 14 camera frames, so raising camera fps beyond
            # ~2 Hz does nothing for the map -- this is the parameter that
            # actually governs map density and how often loop closure gets a
            # chance to fire. 2 Hz doubles keyframes; going much higher grows
            # the graph (and the map cloud) fast for diminishing return.
            'args': ('--delete_db_on_start '
                     '--Rtabmap/DetectionRate 2 '
                     '--Grid/Sensor 2 '
                     '--Grid/DepthDecimation 1 '
                     '--Grid/RangeMax 5.0 '
                     # 20 mm voxels. DO NOT lower this chasing detail without
                     # watching the MESSAGE SIZE, not the CPU load:
                     #   15 mm ->   344,770 pts =  5.5 MB   ok
                     #   10 mm -> 1,063,101 pts = 34.0 MB   RViz stalls,
                     #            republishing every ~1.1 s with 10 s gaps
                     # CPU load stays low either way because the cost falls on
                     # serialisation, DDS transport and RViz rendering rather
                     # than on computation -- so "spare cores" is the WRONG
                     # budget to spend here.
                     # Worse, the cloud grows with EXPLORED AREA and never
                     # shrinks: it went 681k -> 1.06M points while the robot
                     # moved 21 cm. A whole-floor map at 10 mm would be
                     # unusable. 20 mm keeps headroom for a real map.
                     '--Grid/CellSize 0.02 '
                     '--Grid/ClusterRadius 0.1 '
                     '--Grid/NormalsSegmentation false '
                     '--Grid/MaxGroundHeight 0.05 '
                     '--Grid/MaxObstacleHeight 1.8'),
        }.items(),
        # RTAB-Map needs the camera, so both flags must be on.
        condition=IfCondition(PythonExpression([
            "'", use_rtabmap, "' == 'true' and '", use_camera, "' == 'true'"])),
    )

    return LaunchDescription(
        args + [description, bridge, lidar, icp_odom, guard,
                camera_full, camera_low, verifier, rtabmap, rviz, logger])
