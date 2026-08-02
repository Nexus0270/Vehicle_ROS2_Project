#!/usr/bin/env python3
"""
Works out what is producing the obstacle_verifier's constant camera range.

THE SYMPTOM this exists for: /obstacle_check/camera_range pinned at one value
(0.42 m in the 2026-07-23 runs) no matter what the lidar reports, so every
comparison logs "CAMERA SEES CLOSER". A reading that does not change is not an
obstacle -- it is a fixed structure being let through the height filter, and
the two candidates are the FLOOR and the robot's OWN CHASSIS.

It reproduces the verifier's exact pipeline -- same ROI, stride, percentile and
TF path -- but instead of one number it prints where the near points ARE:
their height distribution in base_link and their image row. Then it solves for
the camera_z that would put the floor back below min_height.

    ./camera_height_check.py                    # point at open floor
    ./camera_height_check.py --clear-view       # assert nothing within 2 m

READ IT LIKE THIS
  near points low in the image AND clustered at one height  -> floor
  near points at a constant height across all rows          -> own chassis
  near points spread over many heights                      -> a real obstacle
                                                               (move it away
                                                               and re-run)
"""

import argparse
import math
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image
import tf2_ros


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


class Checker(Node):

    def __init__(self, args):
        super().__init__('camera_height_check')
        self.args = args
        self.K = None
        self.depth = None
        self.frame = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo,
                                 '/camera/aligned_depth_to_color/camera_info',
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image,
                                 '/camera/aligned_depth_to_color/image_raw',
                                 self._on_depth, qos_profile_sensor_data)

    def _on_info(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _on_depth(self, m):
        self.depth = m
        self.frame = m.header.frame_id


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--roi-width-frac', type=float, default=0.6)
    ap.add_argument('--roi-height-frac', type=float, default=0.5)
    ap.add_argument('--stride', type=int, default=4)
    ap.add_argument('--min-depth', type=float, default=0.2)
    ap.add_argument('--max-depth', type=float, default=6.0)
    ap.add_argument('--min-height', type=float, default=-0.05)
    ap.add_argument('--max-height', type=float, default=1.50)
    ap.add_argument('--percentile', type=float, default=5.0)
    ap.add_argument('--clear-view', action='store_true',
                    help='you assert nothing real is within 2 m')
    args = ap.parse_args()

    rclpy.init()
    node = Checker(args)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    import time
    t0 = time.time()
    while time.time() - t0 < 10 and (node.depth is None or node.K is None):
        time.sleep(0.1)
    if node.depth is None or node.K is None:
        print('no depth image or camera_info in 10 s -- is the camera running?')
        return 1

    m = node.depth
    fx, fy, cx, cy = node.K

    # Depth arrives as uint16 millimetres from the RealSense.
    buf = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)
    depth_m = buf.astype(np.float32) / 1000.0

    h, w = depth_m.shape
    r0 = int(h * (1 - args.roi_height_frac) / 2)
    r1 = int(h * (1 + args.roi_height_frac) / 2)
    c0 = int(w * (1 - args.roi_width_frac) / 2)
    c1 = int(w * (1 + args.roi_width_frac) / 2)

    rows = np.arange(r0, r1, args.stride)
    cols = np.arange(c0, c1, args.stride)
    cc, rr = np.meshgrid(cols, rows)
    z = depth_m[rr, cc]

    valid = (z > args.min_depth) & (z < args.max_depth)
    if not valid.any():
        print('no valid depth pixels in the ROI at all.')
        return 1

    # Pinhole back-projection into the OPTICAL frame (x right, y down, z fwd).
    zv = z[valid]
    xv = (cc[valid] - cx) * zv / fx
    yv = (rr[valid] - cy) * zv / fy
    rowv = rr[valid]
    pts_opt = np.stack([xv, yv, zv], axis=1)

    # The TF listener needs a moment to fill after subscribing -- looking up
    # immediately fails with "frame does not exist" even though it is fine.
    tf = None
    t0 = time.time()
    while time.time() - t0 < 5.0:
        try:
            tf = node.tf_buffer.lookup_transform('base_link', node.frame,
                                                 rclpy.time.Time())
            break
        except Exception as e:
            err = e
            time.sleep(0.2)
    if tf is None:
        print(f'TF base_link <- {node.frame} unavailable: {err}')
        return 1

    R = quat_to_matrix(tf.transform.rotation)
    t = np.array([tf.transform.translation.x,
                  tf.transform.translation.y,
                  tf.transform.translation.z])
    pts = pts_opt @ R.T + t

    print(f'depth frame       {node.frame}  {w}x{h}')
    print(f'TF base_link <- camera   x={t[0]:+.3f}  y={t[1]:+.3f}  z={t[2]:+.3f}')
    print(f'valid ROI pixels  {len(pts)}')
    print()

    heights = pts[:, 2]
    inband = (heights > args.min_height) & (heights < args.max_height)
    dist = np.hypot(pts[:, 0], pts[:, 1])

    print(f'height filter [{args.min_height}, {args.max_height}] keeps '
          f'{inband.sum()} of {len(pts)} points')
    if not inband.any():
        print('nothing passes the filter -- camera_range would be NaN.')
        return 0

    kept = pts[inband]
    kept_d = dist[inband]
    kept_rows = rowv[inband]
    kept_h = heights[inband]

    reported = float(np.percentile(kept_d, args.percentile))
    print(f'camera_range the verifier would publish: {reported:.3f} m')
    print()

    # The nearest decile is what actually sets the published range.
    order = np.argsort(kept_d)
    n_near = max(1, len(order) // 10)
    near = order[:n_near]
    print(f'--- the nearest {n_near} points (these set the reading) ---')
    print(f'  distance   {kept_d[near].min():.3f} .. {kept_d[near].max():.3f} m')
    print(f'  height     {kept_h[near].min():+.3f} .. {kept_h[near].max():+.3f} m'
          f'   (median {np.median(kept_h[near]):+.3f})')
    print(f'  image row  {kept_rows[near].min()} .. {kept_rows[near].max()}'
          f'   of {h}   (bottom of ROI is row {r1})')
    print()

    # ---------------------------------------------------------- plane fit
    # The height filter TRUNCATES the distribution, so fitting only filtered
    # points would hide exactly the error we are hunting. Fit the unfiltered
    # cloud instead: a floor is a plane, and its tilt in base_link is a
    # direct readout of camera mounting error the URDF does not know about.
    #
    # A flat floor should come out as z = -wheel_radius everywhere, with a
    # normal straight up. Slope in x is a PITCH error, slope in y is ROLL.
    print('--- ground plane fit (unfiltered, points below +0.10 m) ---')
    ground = pts[pts[:, 2] < 0.10]
    if len(ground) < 50:
        print('  too few low points to fit a plane; skipping')
        pitch_deg = roll_deg = plane_z0 = None
    else:
        # Least squares z = a*x + b*y + c over the low points.
        A = np.column_stack([ground[:, 0], ground[:, 1],
                             np.ones(len(ground))])
        (a, b, c), *_ = np.linalg.lstsq(A, ground[:, 2], rcond=None)
        resid = ground[:, 2] - A @ np.array([a, b, c])
        rms = float(np.sqrt(np.mean(resid ** 2)))
        pitch_deg = math.degrees(math.atan(a))
        roll_deg = math.degrees(math.atan(b))
        plane_z0 = c
        print(f'  {len(ground)} points, plane z = {a:+.4f}x {b:+.4f}y '
              f'{c:+.4f},  rms {rms:.4f} m')
        print(f'  height under base_link origin   {c:+.4f} m '
              f'(should be {-0.0475:+.4f})')
        print(f'  slope forward (pitch error)     {pitch_deg:+.2f} deg')
        print(f'  slope sideways (roll error)     {roll_deg:+.2f} deg')
        if rms > 0.03:
            print('  rms is high -- this may not be a single flat surface.')
    print()

    h_spread = kept_h[near].max() - kept_h[near].min()
    row_frac = np.mean(kept_rows[near] > (r0 + r1) / 2)

    print('--- verdict ---')
    if row_frac > 0.8 and h_spread < 0.15:
        print('  FLOOR. The near points are all in the lower half of the ROI')
        print('  and at one height. The camera is looking down at the ground')
        print('  and the height filter is letting it through.')
        # Floor points should sit at -camera_z relative to base_link, i.e. at
        # the ground. If they land above min_height, camera_z is set too low.
        median_h = float(np.median(kept_h[near]))
        needed = t[2] - median_h - 0.02      # 2 cm below the true ground plane
        print()
        print(f'  Those points sit at z = {median_h:+.3f} m in base_link. The')
        print(f'  ground cannot be above the wheel axle, so camera_z is too')
        print(f'  small by roughly {median_h - (-0.0475):.3f} m.')
        print(f'  MEASURE the real height of the D455 lens above the floor,')
        print(f'  subtract the wheel radius (0.0475 m, base_link sits on the')
        print(f'  axle), and put THAT in camera_z. Current TF says {t[2]:.3f} m;')
        print(f'  this data implies about {needed:.3f} m.')
    elif h_spread < 0.05 and row_frac < 0.5:
        print('  OWN CHASSIS or a fixed mount. Constant height, not in the')
        print('  lower ROI. Something on the robot is in frame -- crop it out')
        print('  with roi_height_frac / roi_width_frac.')
    elif args.clear_view:
        print('  Points are spread in height with a clear view asserted --')
        print('  the camera is seeing something real that you did not expect.')
        print('  Check what is within 2 m of the lens.')
    else:
        print('  Looks like a genuine obstacle in view. Clear the space in')
        print('  front of the robot and re-run with --clear-view.')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
