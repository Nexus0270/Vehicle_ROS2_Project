#!/usr/bin/env python3
"""
Tests the safety guard with the robot SUSPENDED, wheels free.

THE SETUP: the robot sits on a box, wheels off the ground and spinning freely.
A command is held continuously while you walk an object in toward the lidar.
When the object crosses the guard's threshold the wheels must stop; when you
withdraw it they must resume.

WHAT THIS MEASURES -- and what it does not
  MEASURES   the trigger RANGE, the full lidar -> guard -> bridge -> motor
             chain, the sector logic (is the right direction protected), the
             recovery path, and the camera cross-check at the same instant.
  DOES NOT   measure stopping DISTANCE. Suspended wheels carry no chassis
             mass, so there is no coast to measure. That number needs floor.

  Reporting the trigger range as a "stopping distance" would overstate the
  result: the real robot keeps moving after the wheels are commanded to zero.

THE LIDAR PLANE MOVES UP WITH THE ROBOT
  lidar_z is 0.09 m above base_link, which normally sits wheel-radius (0.0475
  m) off the floor. Propped on a box, add the box height. The test object must
  be TALL enough to cut that plane -- an object that passes underneath reads as
  "no obstacle", which is indistinguishable from a broken guard.

USAGE
    ./guard_trigger_test.py                      # forward, 30 s
    ./guard_trigger_test.py --axis y             # strafe left; approach LEFT
    ./guard_trigger_test.py --axis rot           # rotation, uses rotate_clear
    ./guard_trigger_test.py --duration 45

  Hold the command, walk the object in slowly, hold it, then withdraw it. The
  script reports every block and unblock with the range at that moment.

THE OFF-AXIS TEST THAT ACTUALLY PROVES ORIENTATION
  An object in FRONT cannot tell a 180 deg mount error from a mirrored scan --
  both put it at the same place. Run `--axis y` and approach from the robot's
  LEFT. If the guard blocks a LEFT strafe for an object on the LEFT, the
  sector geometry is genuinely right.
"""

import argparse
import csv
import math
import os
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


class GuardTest(Node):

    def __init__(self, half_angle_deg):
        super().__init__('guard_trigger_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.half_angle = math.radians(half_angle_deg)

        self.scan = None
        self.laser_yaw = None
        self.cmd_out = (0.0, 0.0, 0.0)
        self.blocked = False
        self.wheel_v = 0.0          # encoder-derived speed, the ground truth
        self.wheel_w = 0.0
        self.cam_range = float('nan')
        self.lidar_range = float('nan')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(Bool, '/safety/blocked', self._on_blocked, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Float32, '/obstacle_check/camera_range',
                                 self._on_cam, 10)
        self.create_subscription(Float32, '/obstacle_check/lidar_range',
                                 self._on_lidar, 10)

    def _on_scan(self, m):
        self.scan = m

    def _on_cmd(self, m):
        self.cmd_out = (m.linear.x, m.linear.y, m.angular.z)

    def _on_blocked(self, m):
        self.blocked = m.data

    def _on_odom(self, m):
        self.wheel_v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        self.wheel_w = abs(m.twist.twist.angular.z)

    def _on_cam(self, m):
        self.cam_range = m.data

    def _on_lidar(self, m):
        self.lidar_range = m.data

    def _yaw(self):
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
        """Nearest valid return within +/-half_angle of `centre`, in base_link."""
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

    def nearest_any(self):
        """Nearest return in ANY direction -- what gates rotation."""
        s = self.scan
        if s is None:
            return float('inf')
        best = float('inf')
        for r in s.ranges:
            if math.isfinite(r) and max(s.range_min, 0.05) <= r <= s.range_max:
                best = min(best, r)
        return best

    def stop(self):
        for _ in range(10):
            self.pub.publish(Twist())
            time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--axis', choices=['x', 'y', '-x', '-y', 'rot'], default='x',
                    help='x forward, y strafe LEFT, -y strafe RIGHT, rot spin')
    ap.add_argument('--speed', type=float, default=0.15, help='m/s for linear')
    ap.add_argument('--rate', type=float, default=1.0, help='rad/s for rot')
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--sector-half-angle', type=float, default=40.0,
                    help="must match the guard's sector_half_angle_deg")
    args = ap.parse_args()

    rot = args.axis == 'rot'
    heading = {'x': 0.0, 'y': math.pi / 2, '-x': math.pi,
               '-y': -math.pi / 2}.get(args.axis, 0.0)

    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR,
                        f'guard_trigger_{datetime.now():%Y%m%d_%H%M%S}.csv')

    rclpy.init()
    node = GuardTest(args.sector_half_angle)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    t0 = time.time()
    while time.time() - t0 < 10 and node.scan is None:
        time.sleep(0.1)
    if node.scan is None:
        print('no /scan in 10 s -- is the stack running?')
        return 1

    what = (f'ROTATE at {args.rate} rad/s' if rot
            else f'{args.axis} at {args.speed} m/s')
    print(f'Commanding {what} for {args.duration:.0f} s.')
    print('WHEELS MUST BE OFF THE GROUND. Walk the object in, hold, withdraw.')
    print()

    cmd = Twist()
    if rot:
        cmd.angular.z = args.rate
    else:
        cmd.linear.x = args.speed * math.cos(heading)
        cmd.linear.y = args.speed * math.sin(heading)

    events = []
    rows = []
    # Key the event on the guard's OWN decision (/safety/blocked), not on
    # encoder motion. Test A showed why: in the slow-distance taper the
    # command drops below the motor deadband, so the wheels sit still even
    # though the guard is NOT blocking -- an encoder-based detector then
    # chatters STOPPED/RESUMED across the whole taper zone. The guard's
    # blocked flag is the clean signal: it is exactly what we are testing.
    # wheel_speed is still recorded every row as the actuation ground truth.
    was_blocked = None
    t0 = time.time()

    try:
        while True:
            now = time.time() - t0
            if now > args.duration:
                break

            node.pub.publish(cmd)

            rng = node.nearest_any() if rot else node.sector_range(heading)
            moving = (node.wheel_w if rot else node.wheel_v) > 0.01
            guard_cmd = (abs(node.cmd_out[2]) if rot
                         else math.hypot(node.cmd_out[0], node.cmd_out[1]))

            rows.append(dict(
                t=round(now, 3),
                range=round(rng, 4) if math.isfinite(rng) else '',
                guard_cmd=round(guard_cmd, 4),
                blocked=int(node.blocked),
                wheel_speed=round(node.wheel_v, 4),
                wheel_omega=round(node.wheel_w, 4),
                moving=int(moving),
                lidar_range=round(node.lidar_range, 3)
                if math.isfinite(node.lidar_range) else '',
                camera_range=round(node.cam_range, 3)
                if math.isfinite(node.cam_range) else '',
            ))

            if was_blocked is None:
                was_blocked = node.blocked
            elif node.blocked != was_blocked:
                was_blocked = node.blocked
                events.append(dict(
                    t=now, kind='BLOCKED' if node.blocked else 'CLEARED',
                    range=rng, lidar=node.lidar_range,
                    camera=node.cam_range, moving=moving,
                    guard_cmd=guard_cmd))
                print(f'  [{now:5.1f}s] guard '
                      f'{"BLOCKED " if node.blocked else "CLEARED "}  '
                      f'range={rng:.3f} m  '
                      f'guard_out={guard_cmd:.3f}  '
                      f'wheels_moving={moving}  '
                      f'camera={node.cam_range:.2f} m')
            time.sleep(0.02)
    except KeyboardInterrupt:
        print('\naborted')
    finally:
        node.stop()

    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ['t'])
        w.writeheader()
        w.writerows(rows)

    print()
    print('=' * 68)
    print('GUARD TRIGGER TEST')
    print('=' * 68)
    blocks = [e for e in events if e['kind'] == 'BLOCKED']
    clears = [e for e in events if e['kind'] == 'CLEARED']
    if not events:
        print('The guard never changed state.')
        print('  If it stayed CLEAR the whole time: the object may be passing')
        print('  UNDER the lidar plane. Remember the plane rose with the box.')
        print('  If it stayed BLOCKED: something was in the sector from the')
        print('  start -- check the range column in the CSV.')
    for e in events:
        print(f'  {e["t"]:5.1f}s  {e["kind"]:8}  '
              f'range {e["range"]:.3f} m   '
              f'guard_out {e["guard_cmd"]:.3f}   '
              f'wheels_moving {e["moving"]}   '
              f'camera {e["camera"]:.3f} m')
    if blocks:
        rs = [e['range'] for e in blocks if math.isfinite(e['range'])]
        if rs:
            print()
            print(f'  BLOCK triggered at {sum(rs)/len(rs):.3f} m mean over '
                  f'{len(rs)} event(s)'
                  + (f', spread {max(rs)-min(rs):.3f} m' if len(rs) > 1 else ''))
            print(f'  compare with the threshold under test '
                  f'({"rotate_min_clearance ~0.18" if rot else "stop_distance 0.40"} m).')
        # Did the wheels actually reach standstill DURING each blocked
        # interval? Checking the transition instant is wrong -- the wheels are
        # still coasting then; free (unloaded) wheels take ~0.5 s to spin down.
        # So walk the rows: for each contiguous blocked run, did wheel_speed
        # drop below 0.01 before it cleared?
        intervals, cur = [], []
        for r in rows:
            if r['blocked'] == 1:
                cur.append(r)
            elif cur:
                intervals.append(cur); cur = []
        if cur:
            intervals.append(cur)
        failed = 0
        for run in intervals:
            ws = [r['wheel_speed'] if isinstance(r['wheel_speed'], float)
                  else 999.0 for r in run]
            dur = run[-1]['t'] - run[0]['t']
            # Only judge intervals long enough for the wheels to spin down.
            if dur > 0.7 and min(ws) >= 0.01:
                failed += 1
        if failed:
            print(f'  !! WARNING: wheels did NOT reach standstill in {failed} '
                  f'blocked interval(s) -- investigate.')
        else:
            print(f'  wheels reached standstill in every blocked interval '
                  f'long enough to judge -- guard flag and actuation agree.')
    if clears:
        print(f'  recovery: guard CLEARED {len(clears)} time(s) -- the '
              f'unblock path works.')
    print()
    print('  REMINDER: this is a trigger RANGE, not a stopping DISTANCE.')
    print(f'\nSamples: {path}')

    executor.shutdown()
    spin.join(timeout=2.0)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
