#!/usr/bin/env python3
"""
Lidar obstacle guard: the last thing between any velocity source and the wheels.

    teleop / Nav2  ->  /cmd_vel_raw  ->  [safety_guard]  ->  /cmd_vel  ->  bridge

WHY IT SITS HERE
  Filtering inside the teleop script would only protect teleop. Filtering here
  protects every future velocity source too -- Nav2, waypoint following, a
  joystick -- because they all have to come through /cmd_vel_raw to reach the
  wheels.

HOLONOMIC-AWARE
  A mecanum base can drive sideways, so "is something in front?" is the wrong
  question. This checks the sector the robot is actually travelling toward,
  derived from atan2(vy, vx). Strafing left is checked against the left
  sector, diagonal motion against the diagonal, and so on. A differential-
  drive guard would happily strafe into a wall.

ESCAPING A BLOCK
  Only the direction of travel is tested, so once blocked you can always
  reverse out: the opposite heading looks at a different, clear sector and is
  allowed through. That matters -- a guard you cannot escape from leaves the
  robot stuck against a wall with no way back.

FAIL-SAFE
  If /scan goes stale (lidar unplugged, driver crashed, USB drop) the guard
  publishes zero rather than passing commands through. Silence from a safety
  sensor must never read as "all clear".
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

import tf2_ros


class SafetyGuard(Node):

    def __init__(self):
        super().__init__('safety_guard')

        p = self.declare_parameter
        # Hard stop: no translation toward anything closer than this.
        p('stop_distance', 0.40)
        # Between stop_distance and slow_distance speed is scaled down
        # linearly, so the robot eases up instead of slamming to a halt.
        p('slow_distance', 0.80)
        # Half-width of the checked sector. Wide enough to cover the robot's
        # body plus a margin -- too narrow and a corner clips an obstacle the
        # guard never looked at.
        p('sector_half_angle_deg', 40.0)
        # Rotation is allowed unless something is practically touching, since
        # spinning in place is how you get out of a corner.
        p('rotate_min_clearance', 0.18)
        p('scan_timeout', 0.5)          # seconds before /scan counts as stale
        # Yaw of laser_frame relative to base_link. 0.0 matches the URDF; set
        # this if the lidar is ever mounted rotated.
        p('laser_yaw_offset', 0.0)
        p('min_valid_range', 0.05)      # ignore returns closer than this

        g = lambda n: self.get_parameter(n).value
        self.stop_d = g('stop_distance')
        self.slow_d = g('slow_distance')
        self.half_angle = math.radians(g('sector_half_angle_deg'))
        self.rot_clear = g('rotate_min_clearance')
        self.scan_timeout = g('scan_timeout')
        self.yaw_off = g('laser_yaw_offset')
        self.min_valid = g('min_valid_range')

        # The lidar's yaw relative to base_link is taken from TF (i.e. from the
        # URDF) rather than from the laser_yaw_offset parameter, so the mounting
        # angle lives in exactly one place. On this robot the X2 is mounted 180
        # deg backwards; with the offset duplicated in two files it is only a
        # matter of time before they disagree and the guard protects the wrong
        # end of the robot. The parameter remains as a fallback for when TF is
        # unavailable.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.yaw_from_tf = None

        self.scan = None
        self.scan_time = None
        # Recent nearest-obstacle readings. The rotation check compares a
        # MEDIAN of these rather than the latest single value: with an
        # obstacle sitting close to the threshold, scan noise flips a
        # single-sample comparison frame to frame, so rotating one way was
        # allowed and the other way refused purely by timing.
        self._recent_near = []

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.blocked_pub = self.create_publisher(Bool, 'safety/blocked', 10)
        self.range_pub = self.create_publisher(Float32, 'safety/min_range', 10)

        self.create_subscription(LaserScan, 'scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel_raw', self.on_cmd, 10)

        # Independent of incoming commands: if the velocity source itself dies
        # while the robot is moving, this still emits a stop.
        self.create_timer(0.1, self.on_timer)
        self.last_blocked = False

        # Without this the thresholds are frozen at startup: `ros2 param set`
        # would update the parameter server while the guard kept using its
        # cached copies, silently ignoring every retune.
        self.add_on_set_parameters_callback(self.on_params)

        self.get_logger().info(
            f'safety_guard: stop<{self.stop_d} m, slow<{self.slow_d} m, '
            f'sector +/-{math.degrees(self.half_angle):.0f} deg')

    # ------------------------------------------------------------- live tune
    def on_params(self, params):
        """Apply threshold changes immediately, so retuning needs no restart."""
        pending = {p.name: p.value for p in params}
        stop = pending.get('stop_distance', self.stop_d)
        slow = pending.get('slow_distance', self.slow_d)

        # A slow zone at or below the stop distance would make the taper
        # divide by zero (or invert), so refuse rather than accept a config
        # that silently breaks the speed ramp.
        if slow <= stop:
            return SetParametersResult(
                successful=False,
                reason=f'slow_distance ({slow}) must exceed stop_distance ({stop})')

        for name, value in pending.items():
            if name == 'stop_distance':
                self.stop_d = value
            elif name == 'slow_distance':
                self.slow_d = value
            elif name == 'rotate_min_clearance':
                self.rot_clear = value
            elif name == 'sector_half_angle_deg':
                self.half_angle = math.radians(value)
            elif name == 'scan_timeout':
                self.scan_timeout = value
            elif name == 'min_valid_range':
                self.min_valid = value
            elif name == 'laser_yaw_offset':
                self.yaw_off = value

        self.get_logger().info(
            f'thresholds updated: stop<{self.stop_d} m, slow<{self.slow_d} m, '
            f'rotate>{self.rot_clear} m, sector +/-{math.degrees(self.half_angle):.0f} deg')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ scan
    def on_scan(self, msg: LaserScan):
        self.scan = msg
        self.scan_time = self.get_clock().now()
        near = self.nearest_any()
        if math.isfinite(near):
            self._recent_near.append(near)
            del self._recent_near[:-5]      # keep the last 5

    def stable_nearest(self):
        """Median of recent nearest readings -- immune to single-frame noise."""
        if not self._recent_near:
            return float('inf')
        return sorted(self._recent_near)[len(self._recent_near) // 2]

    def laser_yaw(self):
        """Lidar yaw in base_link, from TF; falls back to the parameter."""
        if self.yaw_from_tf is not None:
            return self.yaw_from_tf
        frame = self.scan.header.frame_id if self.scan else 'laser_frame'
        try:
            tf = self.tf_buffer.lookup_transform('base_link', frame,
                                                 rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return self.yaw_off          # parameter fallback
        q = tf.transform.rotation
        self.yaw_from_tf = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.get_logger().info(
            f'lidar yaw from TF: {math.degrees(self.yaw_from_tf):+.1f} deg '
            f'(base_link <- {frame})')
        return self.yaw_from_tf

    def scan_stale(self):
        if self.scan is None or self.scan_time is None:
            return True
        age = (self.get_clock().now() - self.scan_time).nanoseconds / 1e9
        return age > self.scan_timeout

    def min_range_in_sector(self, centre):
        """Closest valid return within +/-half_angle of `centre` (base_link)."""
        s = self.scan
        yaw = self.laser_yaw()
        best = float('inf')
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < self.min_valid:
                continue
            if r < s.range_min or r > s.range_max:
                continue
            ang = s.angle_min + i * s.angle_increment + yaw
            # Wrap the difference into [-pi, pi] so sectors spanning the
            # +/-pi seam (i.e. driving straight backwards) work correctly.
            d = math.atan2(math.sin(ang - centre), math.cos(ang - centre))
            if abs(d) <= self.half_angle and r < best:
                best = r
        return best

    def nearest_any(self):
        s = self.scan
        best = float('inf')
        for r in s.ranges:
            if math.isfinite(r) and self.min_valid <= r <= s.range_max and r < best:
                best = r
        return best

    # --------------------------------------------------------------- command
    def on_cmd(self, msg: Twist):
        self.publish(self.filter(msg))

    def on_timer(self):
        if self.scan is not None and not self.scan_stale():
            self.range_pub.publish(Float32(data=float(self.nearest_any())))

    def filter(self, cmd: Twist) -> Twist:
        out = Twist()

        if self.scan_stale():
            # Fail closed. Never treat missing sensor data as clear road.
            self.note_block(True, 'no recent /scan - blocking all motion')
            return out

        vx, vy, wz = cmd.linear.x, cmd.linear.y, cmd.angular.z
        speed = math.hypot(vx, vy)
        blocked = False
        why = None

        if speed > 1e-3:
            heading = math.atan2(vy, vx)
            ahead = self.min_range_in_sector(heading)

            if ahead < self.stop_d:
                vx = vy = 0.0
                blocked = True
                why = (f'obstacle {ahead:.2f} m at bearing '
                       f'{math.degrees(heading):+.0f} deg (stop < {self.stop_d:.2f})')
            elif ahead < self.slow_d:
                # Linear taper from full speed at slow_d to zero at stop_d.
                scale = (ahead - self.stop_d) / (self.slow_d - self.stop_d)
                scale = max(0.0, min(1.0, scale))
                vx *= scale
                vy *= scale

        # Uses the debounced median, so rotation is allowed or refused
        # consistently instead of flickering with scan noise.
        if abs(wz) > 1e-3:
            near = self.stable_nearest()
            if near < self.rot_clear:
                wz = 0.0
                blocked = True
                why = why or (f'obstacle {near:.2f} m, too close to rotate '
                              f'(needs > {self.rot_clear:.2f})')

        out.linear.x, out.linear.y, out.angular.z = vx, vy, wz
        self.note_block(blocked, why)
        return out

    def note_block(self, blocked, reason=None):
        if blocked and not self.last_blocked:
            self.get_logger().warn(reason or 'obstacle - motion blocked')
        elif self.last_blocked and not blocked:
            self.get_logger().info('path clear')
        self.last_blocked = blocked
        self.blocked_pub.publish(Bool(data=bool(blocked)))

    def publish(self, twist):
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish(Twist())      # leave the robot stopped
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
