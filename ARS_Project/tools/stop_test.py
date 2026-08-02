#!/usr/bin/env python3
"""
Measures how accurately the robot actually stops for an obstacle.

The safety guard promises "no motion when something is closer than
stop_distance in the sector being driven toward". This measures what that is
worth on real hardware, where the promise is degraded by three things the
threshold does not know about:

    detection latency  scan period (80 ms at 12.5 Hz) + guard processing
    command latency    20 Hz TX to the Arduino + its own loop
    mechanical coast   the chassis does not stop the instant throttle is zero

    STOP ERROR = the gap when the guard fires, minus the gap at rest.

A NEGATIVE final gap margin (rest gap < stop_distance) is the number that
matters: it is how far INSIDE its own safety threshold the robot ends up.

    ./stop_test.py                          # 3 trials at 0.15 m/s
    ./stop_test.py --speed 0.30 --trials 5
    ./stop_test.py --axis y                 # strafe left into the obstacle
    ./stop_test.py --speed 0.10,0.20,0.30   # sweep: latency shows as slope

WHAT TO PUT IN FRONT OF THE ROBOT
  A flat vertical panel wider than the robot -- a cardboard box, a book bin, a
  wall. NOT a chair leg or a table edge: a thin object can fall between the
  1.5-degree spokes of a 12.5 Hz scan, and a table top overhangs above the
  lidar plane entirely, which is a real failure mode worth testing separately
  but not what this measures.

The camera cross-check is logged alongside (/obstacle_check/*), so the same run
tells you whether the D455 saw the obstacle at the same range the lidar did.
Only the LIDAR gates motion -- obstacle_verifier is advisory -- so a camera
disagreement here is a finding, not a stop-accuracy error.

Output goes to logs/stop_test_<timestamp>.csv, one row per sample.
"""

import argparse
import csv
import math
import os
import statistics
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

LOG_DIR = os.path.expanduser('~/ARS_Project/logs')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StopTest(Node):

    def __init__(self, half_angle_deg):
        super().__init__('stop_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.half_angle = math.radians(half_angle_deg)

        self.scan = None
        self.scan_stamp = 0.0
        self.laser_yaw = None
        self.cmd_out = (0.0, 0.0, 0.0)     # guard OUTPUT, i.e. what the base gets
        self.blocked = False
        self.wheel = None
        self.icp = None
        self.cam_range = float('nan')
        self.lidar_range = float('nan')
        self.agree = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(Bool, '/safety/blocked', self._on_blocked, 10)
        self.create_subscription(Odometry, '/odom', self._on_wheel, 10)
        self.create_subscription(Odometry, '/odom_icp', self._on_icp,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32, '/obstacle_check/camera_range',
                                 self._on_cam, 10)
        self.create_subscription(Float32, '/obstacle_check/lidar_range',
                                 self._on_lidar, 10)
        self.create_subscription(Bool, '/obstacle_check/agree', self._on_agree, 10)

    # ------------------------------------------------------------ callbacks
    def _on_scan(self, m):
        self.scan = m
        self.scan_stamp = time.time()

    def _on_cmd(self, m):
        self.cmd_out = (m.linear.x, m.linear.y, m.angular.z)

    def _on_blocked(self, m):
        self.blocked = m.data

    def _on_wheel(self, m):
        p = m.pose.pose
        self.wheel = (p.position.x, p.position.y, yaw_of(p.orientation),
                      m.twist.twist.linear.x, m.twist.twist.linear.y)

    def _on_icp(self, m):
        p = m.pose.pose
        self.icp = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_cam(self, m):
        self.cam_range = m.data

    def _on_lidar(self, m):
        self.lidar_range = m.data

    def _on_agree(self, m):
        self.agree = m.data

    # --------------------------------------------------------------- ranges
    def _yaw(self):
        """Lidar yaw in base_link from TF -- same source the guard uses, so a
        mounting change can never make this disagree with what gates motion."""
        if self.laser_yaw is not None:
            return self.laser_yaw
        frame = self.scan.header.frame_id if self.scan else 'laser_frame'
        try:
            tf = self.tf_buffer.lookup_transform('base_link', frame,
                                                 rclpy.time.Time())
        except Exception:
            return None
        self.laser_yaw = yaw_of(tf.transform.rotation)
        return self.laser_yaw

    def sector_range(self, centre):
        """Closest valid return within +/-half_angle of `centre` in base_link."""
        s = self.scan
        if s is None:
            return float('inf')
        yaw = self._yaw()
        if yaw is None:
            return float('inf')
        best = float('inf')
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < max(s.range_min, 0.05) or r > s.range_max:
                continue
            ang = s.angle_min + i * s.angle_increment + yaw
            d = math.atan2(math.sin(ang - centre), math.cos(ang - centre))
            if abs(d) <= self.half_angle and r < best:
                best = r
        return best

    # ----------------------------------------------------------------- move
    def stop(self):
        for _ in range(10):
            self.pub.publish(Twist())
            time.sleep(0.02)

    def wait_ready(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.scan is not None and self.wheel is not None:
                return True
            time.sleep(0.1)
        return False


def run_trial(node, speed, heading, timeout, writer, trial):
    """Drive toward the obstacle until the guard stops us, sampling at 50 Hz."""
    vx = speed * math.cos(heading)
    vy = speed * math.sin(heading)

    node.stop()
    time.sleep(0.5)

    start_gap = node.sector_range(heading)
    start_pose = node.wheel
    start_icp = node.icp
    t0 = time.time()

    samples = []
    fire_t = None          # first moment the guard zeroed the linear command
    fire_gap = None
    fire_pose = None
    fire_icp = None
    still_since = None
    cmd = Twist()

    while True:
        now = time.time() - t0
        gap = node.sector_range(heading)
        moving_cmd = math.hypot(node.cmd_out[0], node.cmd_out[1]) > 1e-3
        wheel = node.wheel or (0, 0, 0, 0, 0)
        measured_v = math.hypot(wheel[3], wheel[4])

        samples.append(dict(
            trial=trial, t=round(now, 3), cmd_speed=round(speed, 3),
            gap=round(gap, 4) if math.isfinite(gap) else '',
            guard_vx=round(node.cmd_out[0], 4),
            guard_vy=round(node.cmd_out[1], 4),
            blocked=int(node.blocked),
            odom_x=round(wheel[0], 4), odom_y=round(wheel[1], 4),
            odom_v=round(measured_v, 4),
            icp_x=round(node.icp[0], 4) if node.icp else '',
            icp_y=round(node.icp[1], 4) if node.icp else '',
            lidar_range=round(node.lidar_range, 3)
            if math.isfinite(node.lidar_range) else '',
            camera_range=round(node.cam_range, 3)
            if math.isfinite(node.cam_range) else '',
            agree='' if node.agree is None else int(node.agree),
        ))

        # The guard zeroing the linear command IS the trigger event. Only
        # count it once we have actually started moving, so the standing start
        # (guard output still zero) is not mistaken for a stop.
        if fire_t is None and now > 0.5 and not moving_cmd:
            fire_t, fire_gap = now, gap
            fire_pose, fire_icp = node.wheel, node.icp

        if fire_t is not None:
            if measured_v < 0.005:
                still_since = still_since or now
                if now - still_since > 0.7:
                    break
            else:
                still_since = None

        if now > timeout:
            print('  timed out -- the guard never stopped us. '
                  'Is the obstacle in the sector?')
            break

        cmd.linear.x, cmd.linear.y = vx, vy
        node.pub.publish(cmd)
        time.sleep(0.02)

    node.stop()
    time.sleep(1.0)                    # let odom and the scan settle
    rest_gap = node.sector_range(heading)
    rest_pose = node.wheel
    rest_icp = node.icp

    for s in samples:
        writer.writerow(s)

    def travel(a, b):
        if a is None or b is None:
            return float('nan')
        return math.hypot(b[0] - a[0], b[1] - a[1])

    return dict(
        start_gap=start_gap,
        fire_gap=fire_gap if fire_gap is not None else float('nan'),
        rest_gap=rest_gap,
        coast_gap=(fire_gap - rest_gap) if fire_gap is not None else float('nan'),
        coast_odom=travel(fire_pose, rest_pose),
        coast_icp=travel(fire_icp, rest_icp),
        total_odom=travel(start_pose, rest_pose),
        total_icp=travel(start_icp, rest_icp),
        cam_at_rest=node.cam_range,
        lidar_at_rest=node.lidar_range,
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--speed', default='0.15',
                    help='m/s, or a comma list to sweep (e.g. 0.10,0.20,0.30)')
    ap.add_argument('--trials', type=int, default=3, help='trials per speed')
    ap.add_argument('--axis', choices=['x', 'y', '-x', '-y'], default='x',
                    help='x forward, y strafe left, -x backward, -y strafe right')
    ap.add_argument('--stop-distance', type=float, default=0.40,
                    help="the guard's stop_distance, for the margin column")
    ap.add_argument('--sector-half-angle', type=float, default=40.0,
                    help="must match the guard's sector_half_angle_deg")
    ap.add_argument('--timeout', type=float, default=25.0)
    args = ap.parse_args()

    heading = {'x': 0.0, 'y': math.pi / 2, '-x': math.pi,
               '-y': -math.pi / 2}[args.axis]
    speeds = [float(s) for s in args.speed.split(',')]

    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(
        LOG_DIR, f'stop_test_{datetime.now():%Y%m%d_%H%M%S}.csv')

    rclpy.init()
    node = StopTest(args.sector_half_angle)
    # See odom_calib.py: the spin thread must be stopped before the node is
    # destroyed, or teardown aborts noisily after the summary has printed.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    results = []
    try:
        if not node.wait_ready():
            print('no /scan or /odom in 10 s -- is the stack running?')
            return 1

        print(f'Axis {args.axis}, guard stop_distance {args.stop_distance} m, '
              f'sector +/-{args.sector_half_angle:.0f} deg')
        print(f'Logging samples to {path}')
        print('Place a flat panel in the path, well beyond the stop threshold.')

        with open(path, 'w', newline='') as f:
            writer = None
            trial = 0
            for speed in speeds:
                for k in range(args.trials):
                    trial += 1
                    gap = node.sector_range(heading)
                    print(f'\n--- trial {trial}: {speed:.2f} m/s '
                          f'(current gap {gap:.2f} m)')
                    if gap < args.stop_distance * 2:
                        print('  gap is small; back the robot up so it has room '
                              'to reach speed before the guard fires.')
                    print('  Hands clear. ENTER to run, Ctrl-C to stop.')
                    input()
                    if writer is None:
                        writer = csv.DictWriter(f, fieldnames=[
                            'trial', 't', 'cmd_speed', 'gap', 'guard_vx',
                            'guard_vy', 'blocked', 'odom_x', 'odom_y', 'odom_v',
                            'icp_x', 'icp_y', 'lidar_range', 'camera_range',
                            'agree'])
                        writer.writeheader()
                    r = run_trial(node, speed, heading, args.timeout,
                                  writer, trial)
                    r['speed'] = speed
                    r['trial'] = trial
                    results.append(r)
                    f.flush()
                    print(f'  fired at {r["fire_gap"]:.3f} m, '
                          f'rest gap {r["rest_gap"]:.3f} m, '
                          f'coast {r["coast_gap"]:.3f} m '
                          f'(odom {r["coast_odom"]:.3f} m)')
    except KeyboardInterrupt:
        print('\naborted')
    finally:
        node.stop()

    if results:
        summarise(results, args)
    executor.shutdown()
    spin.join(timeout=2.0)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print(f'\nSamples: {path}')
    return 0


def summarise(results, args):
    print()
    print('=' * 78)
    print('STOPPING ACCURACY')
    print('=' * 78)
    hdr = (f'{"trial":>5}{"v (m/s)":>9}{"fire gap":>10}{"rest gap":>10}'
           f'{"coast":>8}{"odom":>8}{"margin":>9}{"cam@rest":>10}')
    print(hdr)
    print('-' * 78)
    for r in results:
        margin = r['rest_gap'] - args.stop_distance
        cam = (f'{r["cam_at_rest"]:.2f}'
               if math.isfinite(r['cam_at_rest']) else '--')
        print(f'{r["trial"]:>5}{r["speed"]:>9.2f}{r["fire_gap"]:>10.3f}'
              f'{r["rest_gap"]:>10.3f}{r["coast_gap"]:>8.3f}'
              f'{r["coast_odom"]:>8.3f}{margin:>+9.3f}{cam:>10}')
    print('-' * 78)

    by_speed = {}
    for r in results:
        by_speed.setdefault(r['speed'], []).append(r)
    for speed, rs in sorted(by_speed.items()):
        rest = [r['rest_gap'] for r in rs if math.isfinite(r['rest_gap'])]
        coast = [r['coast_gap'] for r in rs if math.isfinite(r['coast_gap'])]
        if not rest:
            continue
        spread = (max(rest) - min(rest))
        print(f'{speed:.2f} m/s  rest gap mean {statistics.fmean(rest):.3f} m, '
              f'spread {spread:.3f} m, '
              f'coast mean {statistics.fmean(coast):.3f} m')
        print(f'          margin vs stop_distance '
              f'{statistics.fmean(rest) - args.stop_distance:+.3f} m'
              + ('   <-- ENDS INSIDE ITS OWN THRESHOLD'
                 if statistics.fmean(rest) < args.stop_distance else ''))

    if len(by_speed) > 1:
        print()
        print('Coast rising with speed is stopping distance (latency x speed +')
        print('mechanical coast). If it is FLAT, the limit is scan latency, and')
        print('the fix is a higher scan rate, not a bigger stop_distance.')
    print()
    print('If the mean rest gap sits inside stop_distance, raise stop_distance')
    print('by that shortfall, or cap teleop speed. Both are one-line changes.')


if __name__ == '__main__':
    sys.exit(main())
