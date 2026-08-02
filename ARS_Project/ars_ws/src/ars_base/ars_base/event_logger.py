#!/usr/bin/env python3
"""
Black-box recorder for headless test runs.

WHY
  When the robot is driven with no monitor attached there is no way to see
  whether the guard fired, how close it let you get, or whether the camera
  agreed with the lidar. This writes everything to disk so the run can be
  reviewed afterwards, instead of relying on memory of what the robot
  seemed to do.

  Audible feedback would be the obvious alternative, but every audio output
  on this machine is HDMI -- it dies at exactly the moment the display is
  unplugged, which is precisely when it would be needed.

WHAT IT WRITES  (~/ARS_Project/logs/)
  run_<timestamp>.csv     5 Hz sample of every relevant signal
  run_<timestamp>.log     human-readable line per BLOCK / CLEAR event

  The CSV is for plotting or scanning after the fact; the .log is for
  answering "did the auto-stop work?" at a glance.
"""

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String


class EventLogger(Node):

    def __init__(self):
        super().__init__('event_logger')
        self.declare_parameter('log_dir', os.path.expanduser('~/ARS_Project/logs'))
        self.declare_parameter('rate', 5.0)

        log_dir = self.get_parameter('log_dir').value
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(log_dir, f'run_{stamp}.csv')
        self.log_path = os.path.join(log_dir, f'run_{stamp}.log')

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['t', 'min_range', 'blocked',
                           'lidar_range', 'camera_range', 'agree',
                           'cmd_in_vx', 'cmd_in_vy', 'cmd_in_wz',
                           'cmd_out_vx', 'cmd_out_vy', 'cmd_out_wz',
                           'odom_x', 'odom_y'])
        self.log = open(self.log_path, 'w')

        self.d = {'min_range': float('nan'), 'blocked': False,
                  'lidar': float('nan'), 'camera': float('nan'), 'agree': False,
                  'in': (0.0, 0.0, 0.0), 'out': (0.0, 0.0, 0.0),
                  'odom': (0.0, 0.0)}
        self.prev_blocked = False
        self.t0 = time.time()
        self.block_count = 0

        be = QoSProfile(depth=10)
        be.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(Float32, 'safety/min_range',
                                 lambda m: self.d.update(min_range=m.data), 10)
        self.create_subscription(Bool, 'safety/blocked',
                                 self.on_blocked, 10)
        self.create_subscription(Float32, 'obstacle_check/lidar_range',
                                 lambda m: self.d.update(lidar=m.data), 10)
        self.create_subscription(Float32, 'obstacle_check/camera_range',
                                 lambda m: self.d.update(camera=m.data), 10)
        self.create_subscription(Bool, 'obstacle_check/agree',
                                 lambda m: self.d.update(agree=m.data), 10)
        self.create_subscription(String, 'obstacle_check/summary',
                                 self.on_summary, 10)
        self.create_subscription(Twist, 'cmd_vel_raw', lambda m: self.d.update(
            **{'in': (m.linear.x, m.linear.y, m.angular.z)}), 10)
        self.create_subscription(Twist, 'cmd_vel', lambda m: self.d.update(
            out=(m.linear.x, m.linear.y, m.angular.z)), 10)
        self.create_subscription(Odometry, 'odom_icp', lambda m: self.d.update(
            odom=(m.pose.pose.position.x, m.pose.pose.position.y)), be)

        self.create_timer(1.0 / self.get_parameter('rate').value, self.sample)
        self.write(f'=== run started {stamp} ===')
        self.get_logger().info(f'recording to {self.log_path}')

    def write(self, text):
        line = f'[{time.time() - self.t0:7.1f}s] {text}'
        self.log.write(line + '\n')
        self.log.flush()          # flush every line: a hard power-off during
                                  # a test must not lose the evidence
        self.get_logger().info(text)

    @staticmethod
    def _direction(vx, vy, wz):
        """Plain-language name for the motion that was blocked, for the log."""
        if abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) > 1e-3:
            return 'rotating ' + ('left' if wz > 0 else 'right')
        parts = []
        if vx > 1e-3:
            parts.append('driving forward')
        elif vx < -1e-3:
            parts.append('driving backward')
        if vy > 1e-3:
            parts.append('strafing left')
        elif vy < -1e-3:
            parts.append('strafing right')
        return ' + '.join(parts) if parts else 'moving'

    def on_blocked(self, msg):
        self.d['blocked'] = msg.data
        if msg.data and not self.prev_blocked:
            # An obstacle just entered the guard's path and the wheels were
            # cut. Spell it out: this line is the whole point of the black-box
            # log when the run is reviewed without a display.
            self.block_count += 1
            r = self.d['min_range']
            cam = self.d['camera']
            where = self._direction(*self.d['in'])
            rng = f'{r:.2f} m' if math.isfinite(r) else 'unknown range'
            extra = f', camera cross-check {cam:.2f} m' if math.isfinite(cam) else ''
            self.write(f'*** OBSTACLE DETECTED - ROBOT AUTO-STOPPED *** '
                       f'(stop #{self.block_count}) obstacle at {rng} '
                       f'while {where}{extra}')
        elif self.prev_blocked and not msg.data:
            self.write('--- path clear - robot allowed to move again ---')
        self.prev_blocked = msg.data

    def on_summary(self, msg):
        # Only record the interesting case: the camera seeing something the
        # lidar does not. Logging every sample would bury it.
        if 'CAMERA SEES CLOSER' in msg.data:
            self.write(f'CAMERA/LIDAR DISAGREE - {msg.data}')

    def sample(self):
        d = self.d
        self.csv.writerow([
            f'{time.time() - self.t0:.2f}',
            f'{d["min_range"]:.3f}', int(d['blocked']),
            f'{d["lidar"]:.3f}', f'{d["camera"]:.3f}', int(d['agree']),
            f'{d["in"][0]:.3f}', f'{d["in"][1]:.3f}', f'{d["in"][2]:.3f}',
            f'{d["out"][0]:.3f}', f'{d["out"][1]:.3f}', f'{d["out"][2]:.3f}',
            f'{d["odom"][0]:.3f}', f'{d["odom"][1]:.3f}'])
        self.csv_file.flush()

    def close(self):
        self.write(f'=== run ended - {self.block_count} auto-stop events ===')
        try:
            self.csv_file.close()
            self.log.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = EventLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
