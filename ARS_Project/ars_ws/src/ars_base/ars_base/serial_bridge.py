#!/usr/bin/env python3
"""
ROS2 <-> Arduino serial bridge for the mecanum base.

WHAT IT DOES
  cmd_vel  ->  mecanum inverse kinematics  ->  "M fl fr rl rr" lines at 20 Hz
  "E fl fr rl rr" encoder lines  ->  forward kinematics  ->  /odom + TF + /joint_states

WIRE PROTOCOL (both directions defined by mecanum_direction_control.ino)
  TX  "M <fl> <fr> <rl> <rr>\\n"   four absolute throttles, -1.0 .. +1.0
  RX  "E <fl> <fr> <rl> <rr>"      four CUMULATIVE tick counts, signed longs

  The firmware's ENC_DIR_* table already normalises tick sign, so "wheel
  rolling forward" is positive on all four regardless of how the motors are
  mounted. This node therefore treats all four identically.

SIGN CONVENTIONS (ROS REP-103: x forward, y LEFT, z up, yaw CCW-positive)
  These were derived from the firmware's own direction table rather than
  assumed, so they are guaranteed to match the hardware:
      'w' forward     -> +1 +1 +1 +1     (all wheels forward)
      'a' strafe left -> -1 +1 +1 -1     => +vy
      'j' turn left   -> -1 +1 -1 +1     => +wz
  which is exactly the standard 45-degree-roller mecanum matrix below.

SAFETY
  Two independent layers, so no single failure leaves the robot driving:
    1. This node zeroes the throttles if /cmd_vel goes stale (cmd_timeout).
    2. The firmware stops on its own if M lines stop arriving for 200 ms.
  The TX timer runs unconditionally, so a stale cmd_vel still produces
  "M 0 0 0 0" rather than silence -- an explicit stop beats waiting out a
  watchdog.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS

from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:
    serial = None


WHEELS = ('fl', 'fr', 'rl', 'rr')


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        # ------------------------------------------------------------ params
        p = self.declare_parameter
        p('port', '/dev/ttyACM0')
        p('baud', 115200)
        p('dry_run', False)          # exercise the whole node with no hardware

        # Physical constants. wheel_diameter and ticks_per_rev are the values
        # measured by encoder_calibration.ino -- keep them in sync with the
        # sketch. The two separations MUST be measured on the real chassis.
        p('wheel_diameter', 0.095)
        p('ticks_per_rev', 70.6)
        p('wheel_separation_x', 0.20)   # front axle to rear axle, metres
        p('wheel_separation_y', 0.20)   # left wheel to right wheel, metres

        # Throttle is unitless (-1..1) and the base is open loop, so we need
        # to know what full throttle actually means in m/s to turn a cmd_vel
        # into a throttle. Measure it: full throttle on a straight run, read
        # the m/s column of the firmware's telemetry.
        p('max_wheel_speed', 0.60)

        # Set true if Nav2's controller_server is configured with
        # enable_stamped_cmd_vel:=true. Keep both ends in agreement.
        p('use_stamped_cmd_vel', False)

        p('cmd_timeout', 0.5)        # seconds of /cmd_vel silence -> stop
        p('tx_rate', 20.0)           # must beat the firmware's 200 ms watchdog
        p('publish_tf', True)
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')

        g = lambda n: self.get_parameter(n).value
        self.port = g('port')
        self.baud = g('baud')
        self.dry_run = g('dry_run')
        self.ticks_per_rev = g('ticks_per_rev')
        self.wheel_diameter = g('wheel_diameter')
        self.max_wheel_speed = g('max_wheel_speed')
        self.cmd_timeout = g('cmd_timeout')
        self.publish_tf = g('publish_tf')
        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')

        # Ticks per metre of wheel travel, and the mecanum lever arm.
        self.ticks_per_meter = self.ticks_per_rev / (math.pi * self.wheel_diameter)
        self.lever = (g('wheel_separation_x') + g('wheel_separation_y')) / 2.0

        if self.max_wheel_speed <= 0.0:
            raise ValueError('max_wheel_speed must be > 0')

        # ------------------------------------------------------------- state
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_th = 0.0
        self.wheel_angle = dict.fromkeys(WHEELS, 0.0)   # radians, for RViz

        self.prev_ticks = None       # None until the first E line arrives
        self.prev_stamp = None
        self.last_twist = (0.0, 0.0, 0.0)
        self.last_cmd_time = self.get_clock().now()
        self.ser = None
        self._lock = threading.Lock()
        self._running = True

        # ------------------------------------------------------------- ROS io
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_bc = TransformBroadcaster(self)

        # Nav2 switched cmd_vel from Twist to TwistStamped partway through its
        # life, and which one you get depends on the controller_server's
        # enable_stamped_cmd_vel setting. A type mismatch here is silent --
        # the topic simply never connects -- so accept either.
        if self.get_parameter('use_stamped_cmd_vel').value:
            self.create_subscription(TwistStamped, 'cmd_vel',
                                     lambda m: self.on_cmd_vel(m.twist), 10)
            self.get_logger().info('cmd_vel type: TwistStamped')
        else:
            self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.open_serial()
        self.create_timer(1.0 / g('tx_rate'), self.on_tx_timer)

        self.get_logger().info(
            f'serial_bridge up: {self.ticks_per_meter:.1f} ticks/m, '
            f'lever={self.lever:.3f} m, max_wheel_speed={self.max_wheel_speed} m/s'
            + ('  [DRY RUN - no serial]' if self.dry_run else f'  on {self.port}'))

    # ==================================================================== io
    def open_serial(self):
        if self.dry_run:
            self.get_logger().warn(
                'dry_run: not opening a serial port. cmd_vel is accepted and '
                'the M lines that WOULD be sent are logged; odometry stays at '
                'zero because no encoder data can arrive.')
            return

        if serial is None:
            raise RuntimeError('pyserial missing.  sudo apt install python3-serial')

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except serial.SerialException as e:
            raise RuntimeError(
                f'could not open {self.port}: {e}\n'
                f'Check "ls /dev/ttyACM*", that you are in the dialout group, '
                f'and that no Serial Monitor or teleop script holds the port.')

        # Opening the port toggles DTR, which resets the Mega. Wait it out,
        # then drop the boot banner so the parser never sees a partial line.
        self.get_logger().info('waiting for Arduino reset...')
        time.sleep(2.5)
        self.ser.reset_input_buffer()

        # Telemetry is OFF after a reset and 't' toggles it, so exactly one
        # 't' turns it on. 'r' zeroes the counters so odometry starts clean.
        # (This relies on the DTR reset above having actually happened -- if
        # you disable auto-reset on the board, send 't' by hand instead.)
        self.ser.write(b'r')
        self.ser.write(b't')

        threading.Thread(target=self.reader_loop, daemon=True).start()

    def reader_loop(self):
        """Parse 'E fl fr rl rr' lines. Anything else is passed to the log."""
        while self._running and rclpy.ok():
            try:
                raw = self.ser.readline()
            except Exception as e:                      # port yanked mid-run
                self.get_logger().error(f'serial read failed: {e}')
                break
            if not raw:
                continue

            line = raw.decode(errors='replace').strip()
            if not line.startswith('E '):
                # RPM/dist telemetry and the firmware's banner land here.
                if line and not line.startswith(('R ', 'RPM ')):
                    self.get_logger().debug(f'arduino: {line}')
                continue

            parts = line.split()
            if len(parts) != 5:
                continue
            try:
                ticks = tuple(int(v) for v in parts[1:])
            except ValueError:
                continue                                # corrupted line, skip

            self.on_ticks(ticks, self.get_clock().now())

    # ============================================================ kinematics
    def on_cmd_vel(self, msg: Twist):
        with self._lock:
            self.last_twist = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.last_cmd_time = self.get_clock().now()

    def on_tx_timer(self):
        """Stream throttles at a fixed rate, stopping if cmd_vel goes stale."""
        with self._lock:
            vx, vy, wz = self.last_twist
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / S_TO_NS

        if age > self.cmd_timeout:
            vx = vy = wz = 0.0

        # Mecanum inverse kinematics -> wheel linear speeds (m/s).
        L = self.lever
        v = (vx - vy - L * wz,      # fl
             vx + vy + L * wz,      # fr
             vx + vy - L * wz,      # rl
             vx - vy + L * wz)      # rr

        # Open loop, so m/s maps to throttle by a single measured scale. If a
        # request exceeds full throttle, scale all four together rather than
        # clipping individually -- clipping each wheel separately would distort
        # the commanded direction, which is far worse than arriving slowly.
        t = [s / self.max_wheel_speed for s in v]
        peak = max(abs(x) for x in t)
        if peak > 1.0:
            t = [x / peak for x in t]

        self.send_throttles(t)

    def send_throttles(self, t):
        line = 'M %.3f %.3f %.3f %.3f\n' % tuple(t)
        if self.dry_run:
            if any(abs(x) > 1e-6 for x in t):
                self.get_logger().info(f'[dry_run] would send: {line.strip()}')
            return
        try:
            self.ser.write(line.encode())
        except Exception as e:
            self.get_logger().error(f'serial write failed: {e}')

    def on_ticks(self, ticks, stamp):
        """Integrate cumulative encoder counts into an odometry pose."""
        # First sample only establishes the baseline -- integrating against a
        # zero "previous" would inject the whole startup count as one jump.
        if self.prev_ticks is None:
            self.prev_ticks = ticks
            self.prev_stamp = stamp
            return

        dt = (stamp - self.prev_stamp).nanoseconds / S_TO_NS
        if dt <= 0.0:
            return

        # Per-wheel distance travelled since the last sample, in metres.
        d = [(n - p) / self.ticks_per_meter
             for n, p in zip(ticks, self.prev_ticks)]
        self.prev_ticks = ticks
        self.prev_stamp = stamp

        dfl, dfr, drl, drr = d

        # Mecanum forward kinematics -- the exact inverse of the matrix above.
        dx = (dfl + dfr + drl + drr) / 4.0
        dy = (-dfl + dfr + drl - drr) / 4.0
        dth = (-dfl + dfr - drl + drr) / (4.0 * self.lever)

        # Integrate in the odom frame using the MIDPOINT heading. Using the
        # start heading instead makes every turn cut the corner, which shows
        # up as steady heading drift over a long run.
        th_mid = self.pose_th + dth / 2.0
        self.pose_x += dx * math.cos(th_mid) - dy * math.sin(th_mid)
        self.pose_y += dx * math.sin(th_mid) + dy * math.cos(th_mid)
        self.pose_th = self.wrap_angle(self.pose_th + dth)

        # Wheel spin for RViz. Publishing this from encoders means no
        # joint_state_publisher is needed anywhere in the stack.
        circumference = math.pi * self.wheel_diameter
        for name, dist in zip(WHEELS, d):
            self.wheel_angle[name] = self.wrap_angle(
                self.wheel_angle[name] + (dist / circumference) * 2.0 * math.pi)

        self.publish(stamp, dx / dt, dy / dt, dth / dt)

    @staticmethod
    def wrap_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    # =============================================================== publish
    def publish(self, stamp, vx, vy, wz):
        now = stamp.to_msg()
        qz = math.sin(self.pose_th / 2.0)
        qw = math.cos(self.pose_th / 2.0)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Rough diagonal covariance. Open-loop mecanum odometry slips badly
        # (the rollers scrub sideways), so these are deliberately pessimistic
        # -- it tells Nav2 and any EKF to trust the lidar/camera over these
        # numbers, which is exactly what you want.
        for i, var in ((0, 0.02), (7, 0.02), (35, 0.05)):
            odom.pose.covariance[i] = var
            odom.twist.covariance[i] = var
        for i in (14, 21, 28):
            odom.pose.covariance[i] = 1e6      # z, roll, pitch: unobservable
            odom.twist.covariance[i] = 1e6
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.pose_x
            tf.transform.translation.y = self.pose_y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_bc.sendTransform(tf)

        js = JointState()
        js.header.stamp = now
        js.name = [f'{w}_wheel_joint' for w in WHEELS]
        js.position = [self.wheel_angle[w] for w in WHEELS]
        self.joint_pub.publish(js)

    # =============================================================== cleanup
    def shutdown(self):
        """Always leave the robot stopped, even on an exception path."""
        self._running = False
        if self.ser is not None:
            try:
                self.ser.write(b'M 0 0 0 0\n')
                self.ser.write(b's')
                self.ser.flush()
                self.ser.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
