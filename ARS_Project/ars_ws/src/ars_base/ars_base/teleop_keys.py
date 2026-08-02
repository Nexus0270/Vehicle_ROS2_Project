#!/usr/bin/env python3
"""
Hold-to-move keyboard teleop. Press a key to move, release it to stop.

    ros2 run ars_base teleop_keys

Publishes geometry_msgs/Twist on /cmd_vel_raw, so everything it sends passes
through safety_guard before reaching the wheels.

HOW "RELEASE" IS DETECTED
  A terminal cannot report key RELEASE -- it only delivers characters. So
  release is inferred from silence: while a key is physically held the
  terminal auto-repeats it, and when nothing has arrived for key_timeout the
  node publishes zero.

  This is why key_timeout interacts with your keyboard's auto-repeat DELAY
  (the pause before repeats begin, typically ~500 ms). If key_timeout is
  shorter than that delay, holding a key produces move-stop-move stutter at
  the start of every press. Two ways to fix it, in order of preference:

    xset r rate 200 40      # repeat delay 200 ms, then 40/s  <- do this
    ros2 run ars_base teleop_keys --ros-args -p key_timeout:=0.6

  The first keeps stopping responsive. The second makes stopping sluggish,
  because the robot keeps moving for key_timeout after you let go.

TWO INDEPENDENT SAFETY LAYERS
  1. This node zeroes on key release, and on exit publishes an explicit stop.
  2. The firmware's own watchdog halts the robot if M lines stop arriving for
     200 ms -- so if this node is killed, the terminal closes, or SSH drops,
     the robot still stops without anyone telling it to.

  Neither layer depends on the other, which is the point.

CONTROLS
    w / s     forward / backward
    a / d     strafe left / right        (mecanum -- no rotation)
    q / e     rotate left / right
    space     stop now
    + / -     speed up / down
    Ctrl-C    quit (publishes a stop first)
"""

import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


# key -> (vx, vy, wz) as unit directions, scaled by the current speed
BINDINGS = {
    'w': (+1.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0),
    'a': (0.0, +1.0, 0.0),      # +y is LEFT (REP-103)
    'd': (0.0, -1.0, 0.0),
    'q': (0.0, 0.0, +1.0),      # +yaw is CCW
    'e': (0.0, 0.0, -1.0),
}

HELP = """
  w/s forward/back   a/d strafe   q/e rotate
  space stop   +/- speed   Ctrl-C quit
"""


class TeleopKeys(Node):

    def __init__(self):
        super().__init__('teleop_keys')
        p = self.declare_parameter
        p('linear_speed', 0.20)      # m/s
        # Rotation needs a MUCH larger number than translation to reach the
        # same throttle, because each wheel only sees L * wz, and L is
        # (sep_x + sep_y)/2 = 0.20 m here. At wz = 1.0 rad/s the wheels get
        # 0.2 m/s -> throttle 0.33, barely above the driver's deadband, which
        # is why the robot could not turn under its own weight. To saturate,
        # wz must reach max_wheel_speed / L = 0.60 / 0.20 = 3.0 rad/s.
        p('angular_speed', 3.00)     # rad/s
        p('speed_step', 0.05)        # per +/- press
        p('max_linear_speed', 0.60)  # matches max_wheel_speed: full throttle
        # Rotation is capped SEPARATELY, because scaling it with linear speed
        # ran it up to 9 rad/s (515 deg/s). At the lidar's ~3.8 Hz that is 135
        # degrees of turn between consecutive scans, which ICP cannot match --
        # scan matching then failed continuously and the SLAM pose jumped
        # metres at a time. 3.0 rad/s already saturates wheel throttle
        # (L * wz = max_wheel_speed), so this costs no torque at all: it only
        # removes commands the lidar could never track.
        p('max_angular_speed', 3.00)
        p('key_timeout', 0.35)       # silence before we call it a release
        p('publish_rate', 20.0)      # must beat the firmware's 200 ms watchdog

        self.lin = self.get_parameter('linear_speed').value
        self.ang = self.get_parameter('angular_speed').value
        self.step = self.get_parameter('speed_step').value
        self.lin_max = self.get_parameter('max_linear_speed').value
        self.ang_max = self.get_parameter('max_angular_speed').value
        # +/- scales rotation in step with translation, so "faster" means
        # faster at everything rather than only in a straight line.
        self.ang_per_lin = self.ang / max(self.lin, 1e-6)
        self.timeout = self.get_parameter('key_timeout').value
        self.period = 1.0 / self.get_parameter('publish_rate').value

        self.pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.vec = (0.0, 0.0, 0.0)
        self.last_key_time = 0.0

        # Mirror the guard's decision to THIS terminal. While you drive you
        # SEE the auto-stop the instant it fires -- not just afterwards in the
        # black-box log. event_logger records the same event to disk; this is
        # the live view for whoever is holding the keys (including over SSH).
        self._blocked = False
        self._min_range = float('nan')
        self.create_subscription(Float32, 'safety/min_range',
                                 lambda m: setattr(self, '_min_range', m.data), 10)
        self.create_subscription(Bool, 'safety/blocked', self._on_blocked, 10)

    def _on_blocked(self, msg):
        # Printed with \r\n because the terminal is in raw mode here.
        if msg.data and not self._blocked:
            r = self._min_range
            rng = f'{r:.2f} m' if math.isfinite(r) else 'very close'
            sys.stdout.write('\r\n*** OBSTACLE DETECTED - ROBOT AUTO-STOPPED '
                             f'(obstacle at {rng}) ***\r\n')
            sys.stdout.flush()
        elif self._blocked and not msg.data:
            sys.stdout.write('\r\n--- path clear - you can move again ---\r\n')
            sys.stdout.flush()
        self._blocked = msg.data

    def twist(self):
        vx, vy, wz = self.vec
        t = Twist()
        t.linear.x = vx * self.lin
        t.linear.y = vy * self.lin
        t.angular.z = wz * self.ang
        return t

    def set_speed(self, value):
        """Clamp and apply a new speed, scaling rotation with it."""
        self.lin = max(self.step, min(value, self.lin_max))
        self.ang = min(self.lin * self.ang_per_lin, self.ang_max)
        pct = 100.0 * self.lin / self.lin_max
        bar = '#' * int(pct / 5)
        sys.stdout.write(f'\r  speed {self.lin:.2f} m/s  ({pct:3.0f}% of full '
                         f'throttle) {bar:<20}\r\n')
        sys.stdout.flush()

    def stop(self):
        self.vec = (0.0, 0.0, 0.0)
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeys()

    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        # Raw-mode key reading needs a real terminal. Without this check the
        # failure is a bare termios traceback, which reads like a crash rather
        # than "you launched me in the wrong place".
        print('teleop_keys needs an interactive terminal.\n'
              'Run it directly in a terminal window:\n'
              '    ros2 run ars_base teleop_keys\n'
              'It cannot be started from inside a launch file, over a pipe,\n'
              'or with redirected stdin.')
        node.destroy_node()
        rclpy.shutdown()
        return

    old = termios.tcgetattr(fd)

    print(__doc__.split('CONTROLS')[0])
    print(HELP)
    print(f"  speed: {node.lin:.2f} m/s   key_timeout: {node.timeout:.2f} s")
    if os.environ.get('DISPLAY'):
        print("  tip: run  xset r rate 200 40  for snappier hold-to-move\n")

    try:
        tty.setraw(fd)
        next_pub = time.time()
        while rclpy.ok():
            # Poll stdin without blocking so the publish loop keeps running
            # even while no key is arriving -- the stream must not stall or
            # the firmware watchdog trips mid-press.
            if select.select([sys.stdin], [], [], 0.01)[0]:
                ch = sys.stdin.read(1)
                if ch == '\x03':                     # Ctrl-C
                    break
                elif ch in BINDINGS:
                    node.vec = BINDINGS[ch]
                    node.last_key_time = time.time()
                elif ch == ' ':
                    node.vec = (0.0, 0.0, 0.0)
                    node.last_key_time = 0.0
                elif ch in ('+', '='):
                    node.set_speed(node.lin + node.step)
                elif ch == '-':
                    node.set_speed(node.lin - node.step)

            # Release = silence for longer than key_timeout.
            if node.vec != (0.0, 0.0, 0.0) and \
                    time.time() - node.last_key_time > node.timeout:
                node.vec = (0.0, 0.0, 0.0)

            now = time.time()
            if now >= next_pub:
                next_pub = now + node.period
                node.pub.publish(node.twist())

            rclpy.spin_once(node, timeout_sec=0.0)

    except Exception as e:
        sys.stdout.write(f'error: {e}\r\n')
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        try:
            # Explicit stop rather than relying on the firmware watchdog to
            # time out -- belt and braces on the way out.
            for _ in range(3):
                node.stop()
                time.sleep(0.02)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('\nstopped.')


if __name__ == '__main__':
    main()
