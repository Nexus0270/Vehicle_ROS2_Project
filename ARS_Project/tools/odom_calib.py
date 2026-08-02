#!/usr/bin/env python3
"""
Wheel-odometry calibration for the mecanum base.

Drives a known open-loop move through the safety guard, then compares what the
WHEEL odometry (/odom, from encoder ticks) reported against what the LIDAR ICP
odometry (/odom_icp) reported and against a tape measure. From the tape number
it works out the corrected parameter to put in config/ars_base.yaml.

    ./odom_calib.py straight --distance 1.0        # fixes wheel_diameter
    ./odom_calib.py rotate   --turns 1.0           # fixes wheel_separation_*
    ./odom_calib.py strafe   --distance 1.0        # roller-slip diagnostic
    ./odom_calib.py record                         # passive, drive by teleop

WHICH KNOB EACH TEST TURNS
  straight  distance = ticks / ticks_per_meter, and ticks_per_meter is
            ticks_per_rev / (pi * wheel_diameter). Only the product matters, so
            we correct wheel_diameter and leave ticks_per_rev at the value the
            calibration sketch measured.
  rotate    dth = (-fl + fr - rl + rr) / (4 * lever), lever being the mean of
            the two separations. Reported yaw scales as 1/lever, so a robot
            that thinks it turned too far has its separations set too small.
  strafe    Mecanum rollers scrub sideways, so /odom always over-reports strafe
            distance. There is no separate parameter for it -- this run just
            tells you how much to distrust sideways odometry.

RUN ORDER MATTERS: straight first. Rotation uses wheel distance, so a wrong
wheel_diameter would be absorbed into the lever correction and both end up
wrong.

Commands go to /cmd_vel_raw, so the safety guard still applies and an obstacle
appearing mid-run stops the robot exactly as it would under teleop. The script
always publishes a stop on the way out, including on Ctrl-C.
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Calib(Node):

    def __init__(self):
        super().__init__('odom_calib')
        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        sensor_qos = QoSProfile(depth=20,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.wheel = None      # (x, y, yaw) latest from /odom
        self.icp = None        # (x, y, yaw) latest from /odom_icp
        self.blocked = False
        self.block_seen = False

        self.create_subscription(Odometry, '/odom', self._on_wheel, 10)
        self.create_subscription(Odometry, '/odom_icp', self._on_icp, sensor_qos)
        self.create_subscription(Bool, '/safety/blocked', self._on_blocked, 10)

    def _on_wheel(self, m):
        p = m.pose.pose
        self.wheel = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_icp(self, m):
        p = m.pose.pose
        self.icp = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_blocked(self, m):
        self.blocked = m.data
        if m.data:
            self.block_seen = True

    # ------------------------------------------------------------------ move
    def stop(self):
        """Publish repeatedly -- one dropped stop message is one runaway."""
        for _ in range(10):
            self.pub.publish(Twist())
            time.sleep(0.02)

    def drive(self, vx, vy, wz, duration, ramp=0.3):
        """Open-loop move with a ramp at both ends so the wheels do not slip."""
        t = Twist()
        rate = 50.0
        n = int(duration * rate)
        t0 = time.time()
        for i in range(n):
            elapsed = i / rate
            # Trapezoid: ease in over `ramp`, hold, ease out over `ramp`.
            if elapsed < ramp:
                s = elapsed / ramp
            elif elapsed > duration - ramp:
                s = max(0.0, (duration - elapsed) / ramp)
            else:
                s = 1.0
            t.linear.x, t.linear.y, t.angular.z = vx * s, vy * s, wz * s
            self.pub.publish(t)
            time.sleep(max(0.0, t0 + (i + 1) / rate - time.time()))
        self.stop()

    def wait_for_data(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.wheel is not None:
                return True
            time.sleep(0.1)
        return False

    def settle(self, seconds=1.5):
        """Let both estimators finish integrating before reading the pose."""
        time.sleep(seconds)


def delta(before, after):
    """Translation magnitude and yaw change between two (x, y, yaw) poses."""
    if before is None or after is None:
        return None
    dx, dy = after[0] - before[0], after[1] - before[1]
    return dx, dy, math.hypot(dx, dy), wrap(after[2] - before[2])


def ask_float(prompt):
    while True:
        try:
            s = input(prompt).strip()
        except EOFError:            # stdin closed / piped input exhausted
            return None
        if not s:
            return None
        try:
            return float(s)
        except ValueError:
            print('  not a number, try again (or blank to skip)')


def report(kind, node, before_w, before_i, cmd_value, args):
    node.settle()
    dw = delta(before_w, node.wheel)
    di = delta(before_i, node.icp)

    print()
    print('=' * 62)
    print(f'{kind} run: commanded {cmd_value}')
    if node.block_seen:
        print('  !! the safety guard fired during this run -- the robot was')
        print('     slowed or stopped, so treat the numbers with care')
    print('-' * 62)
    print(f'{"":10}{"dx (m)":>10}{"dy (m)":>10}{"dist (m)":>10}{"dyaw (deg)":>12}')
    for name, d in (('wheel', dw), ('lidar ICP', di)):
        if d is None:
            print(f'{name:10}{"  (no data)":>42}')
            continue
        print(f'{name:10}{d[0]:>10.4f}{d[1]:>10.4f}{d[2]:>10.4f}'
              f'{math.degrees(d[3]):>12.2f}')
    print('=' * 62)

    if dw is None:
        print('no wheel odometry received -- is serial_bridge running?')
        return

    if kind == 'record':
        # A free teleop drive has no known ground truth, so there is nothing
        # to solve for. The value is the wheel-vs-lidar DISAGREEMENT above:
        # they start from the same pose, so any divergence is accumulated
        # wheel slip (or ICP drift, which the stationary tests bound).
        if di is not None and dw[2] > 0.05:
            print(f'  wheel/lidar distance disagreement: '
                  f'{100 * (dw[2] / di[2] - 1):+.1f}%' if di[2] > 0.05 else '')
            print(f'  wheel/lidar heading disagreement:  '
                  f'{math.degrees(wrap(dw[3] - di[3])):+.2f} deg')
        return

    if kind == 'rotate':
        reported = abs(math.degrees(dw[3]))
        actual = ask_float('Tape/protractor: how far did it ACTUALLY turn, '
                           'in degrees?  (blank to skip) > ')
        if actual is None or actual == 0:
            return
        lever_old = (args.sep_x + args.sep_y) / 2.0
        lever_new = lever_old * reported / actual
        scale = lever_new / lever_old
        print()
        print(f'  reported {reported:.2f} deg vs actual {actual:.2f} deg'
              f'   (error {100 * (reported / actual - 1):+.1f}%)')
        print(f'  lever  {lever_old:.4f} -> {lever_new:.4f} m')
        print(f'  Put these in config/ars_base.yaml AND urdf/ars_robot.urdf.xacro:')
        print(f'    wheel_separation_x: {args.sep_x * scale:.4f}    (sep_x)')
        print(f'    wheel_separation_y: {args.sep_y * scale:.4f}    (sep_y)')
        print(f'  NOTE: only their SUM is observable from a spin test. Keep the')
        print(f'        measured ratio between them; this scales both equally.')
        return

    reported = abs(dw[0]) if kind == 'straight' else abs(dw[1])
    actual = ask_float('Tape measure: how far did it ACTUALLY travel, in '
                       'metres?  (blank to skip) > ')
    if actual is None or actual == 0:
        return
    err = 100 * (reported / actual - 1)
    print()
    print(f'  reported {reported:.4f} m vs actual {actual:.4f} m'
          f'   (error {err:+.1f}%)')

    if kind == 'strafe':
        print(f'  Sideways odometry scale factor: {reported / actual:.3f}')
        print('  This is roller scrub. Expect over-reporting (>1.0). There is')
        print('  no parameter to fix it -- it is why the strafe covariance is')
        print('  set pessimistic and why lidar odom stays the TF source.')
        return

    d_new = args.wheel_diameter * actual / reported
    print(f'  wheel_diameter  {args.wheel_diameter:.4f} -> {d_new:.4f} m')
    print(f'  (equivalently ticks_per_meter '
          f'{args.ticks_per_rev / (math.pi * args.wheel_diameter):.1f} -> '
          f'{args.ticks_per_rev / (math.pi * d_new):.1f})')
    print('  Put wheel_diameter in config/ars_base.yaml, then rebuild or')
    print('  restart the stack, and re-run this test to confirm it lands <2%.')


def measure_max_speed(node, args):
    """Measure the real m/s at throttle 1.0, straight from the encoders.

    max_wheel_speed is the ONLY number converting a cmd_vel into a throttle:
    the bridge sends throttle = wheel_speed / max_wheel_speed. So commanding
    exactly max_wheel_speed saturates the throttle at 1.0, and whatever the
    encoders then report IS the true value. No firmware telemetry needed.

    This is also the honest test of the recorded supply-sag problem, because
    all four motors draw at once -- exactly the case that collapsed the pack
    from 113 to 40 ticks/s. A measured speed far below the single-motor
    figure is that sag, not a calibration error.
    """
    v = args.speed_max
    print(f'This drives FORWARD at full throttle (commanding {v} m/s, the')
    print('current max_wheel_speed, which saturates the throttle at 1.0).')
    print('NEEDS A LONG CLEAR RUN -- several metres. It accelerates hard.')
    print('The safety guard still applies, but do not rely on it at speed:')
    print('that is the very thing the stopping test is there to measure.')
    print('\nHands clear. ENTER to go, Ctrl-C to abort.')
    input()

    node.block_seen = False
    samples = []
    t = Twist()
    t.linear.x = v
    t0 = time.time()
    # Hold full throttle for 3 s, sampling the encoder-reported speed. The
    # first second is acceleration, so only the tail is the steady state.
    while time.time() - t0 < 3.0:
        node.pub.publish(t)
        if node.wheel is not None:
            samples.append((time.time() - t0, node.wheel[0]))
        time.sleep(0.05)
    node.stop()

    if len(samples) < 10:
        print('not enough odometry samples')
        return 1

    # Differentiate position over the steady-state window rather than trusting
    # the twist field, which is noisy at a 10 Hz tick rate.
    steady = [s for s in samples if s[0] > 1.2]
    if len(steady) < 5:
        print('run too short to find a steady state')
        return 1
    dt = steady[-1][0] - steady[0][0]
    dx = steady[-1][1] - steady[0][1]
    measured = dx / dt if dt > 0 else 0.0

    print()
    print('=' * 62)
    print(f'steady-state speed at full throttle: {measured:.3f} m/s')
    print(f'  (over {dt:.2f} s, {dx:.3f} m, from encoders)')
    if node.block_seen:
        print('  !! the guard fired -- the robot was tapered or stopped, so')
        print('     this is NOT a full-throttle number. Get a longer runway.')
    print('=' * 62)
    print(f'  config says max_wheel_speed: {v}')
    print(f'  measured                   : {measured:.3f}')
    if measured < v * 0.75:
        print()
        print('  Measured is well under the configured value. Two candidates:')
        print('    - supply sag: four motors at once collapsing the pack')
        print('      (single/dual motors were fine, four were not)')
        print('    - wheel_diameter uncalibrated, inflating or deflating this')
        print('      reading -- run "straight" FIRST if you have not')
    print('  Set max_wheel_speed to the measured value in ars_base.yaml. It')
    print('  does NOT affect odometry accuracy (that comes from encoders); it')
    print('  makes Nav2 stop consistently over- or under-shooting.')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('test', choices=['straight', 'strafe', 'rotate', 'record',
                                     'maxspeed'])
    ap.add_argument('--distance', type=float, default=1.0,
                    help='metres to travel (straight/strafe)')
    ap.add_argument('--turns', type=float, default=1.0,
                    help='full revolutions to spin (rotate)')
    ap.add_argument('--speed', type=float, default=0.15,
                    help='m/s for straight/strafe')
    ap.add_argument('--rate', type=float, default=1.0,
                    help='rad/s for rotate')
    ap.add_argument('--speed-max', type=float, default=0.60,
                    help='current max_wheel_speed, i.e. what saturates throttle')
    # Current values, so the script can print a corrected one. Override if
    # ars_base.yaml has moved on.
    ap.add_argument('--wheel-diameter', type=float, default=0.095)
    ap.add_argument('--ticks-per-rev', type=float, default=70.6)
    ap.add_argument('--sep-x', type=float, default=0.20)
    ap.add_argument('--sep-y', type=float, default=0.20)
    args = ap.parse_args()

    rclpy.init()
    node = Calib()
    # Explicit executor rather than a bare rclpy.spin thread: the thread must
    # be stopped BEFORE the node is destroyed, or teardown races the running
    # spin and aborts with "terminate called without an active exception"
    # after the results have printed -- alarming and meaningless.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    try:
        if not node.wait_for_data():
            print('no /odom in 10 s -- is the stack running?')
            return 1

        if args.test == 'record':
            print('Recording. Drive with teleop, then press ENTER to compare.')
            before_w, before_i = node.wheel, node.icp
            input()
            report('record', node, before_w, before_i, 'teleop', args)
            return 0

        if args.test == 'maxspeed':
            return measure_max_speed(node, args)

        if args.test == 'rotate':
            duration = abs(args.turns) * 2 * math.pi / args.rate
            vx = vy = 0.0
            wz = math.copysign(args.rate, args.turns)
            what = f'{args.turns} turn(s) at {args.rate} rad/s'
            print('SETUP: clear space all round. Put a tape strip on the floor')
            print('       under a marked point on the chassis so you can read')
            print('       the final heading error.')
        else:
            duration = args.distance / args.speed
            wz = 0.0
            vx = args.speed if args.test == 'straight' else 0.0
            vy = args.speed if args.test == 'strafe' else 0.0
            what = f'{args.distance} m at {args.speed} m/s'
            print('SETUP: lay a tape measure along the path. Mark the floor at')
            print('       a fixed reference point on the chassis before and')
            print('       after -- same point both times.')

        # The ramp adds travel at both ends; the open-loop duration is only a
        # rough aim anyway, since the tape measure is the ground truth.
        print(f'\nAbout to drive: {what}  (~{duration + 0.3:.1f} s)')
        print('Hands clear. ENTER to go, Ctrl-C to abort.')
        input()

        node.block_seen = False
        before_w, before_i = node.wheel, node.icp
        node.drive(vx, vy, wz, duration)
        report(args.test, node, before_w, before_i, what, args)
        return 0

    except KeyboardInterrupt:
        print('\naborted')
        return 130
    finally:
        node.stop()
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
