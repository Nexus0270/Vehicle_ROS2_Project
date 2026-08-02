#!/usr/bin/env python3
"""
Cross-checks the lidar's obstacle distance against the depth camera's.

WHY
  The safety guard trusts one sensor. A lidar can miss glass, dark matt
  surfaces, and anything above or below its single scan plane -- a table top,
  an open drawer, a low step. The D455 sees a full 3D volume, so comparing the
  two tells you whether the guard's distance is trustworthy right now, and
  flags the cases where the lidar alone would drive you into something.

BOTH READINGS ARE EXPRESSED IN base_link
  This matters more than it sounds. Two naive comparisons are both wrong:

    * Comparing raw sensor distances ignores that the camera and lidar sit at
      different places on the robot. If the camera is 10 cm forward of the
      lidar, they report distances 10 cm apart for the same wall and the node
      cries "disagreement" forever.

    * Comparing a lidar RANGE against a depth-image Z is comparing different
      quantities. Depth images store perpendicular distance along the optical
      axis; a lidar reports radial distance. For a flat wall the lidar's own
      readings vary as d/cos(theta) across its sweep while every camera pixel
      reads d. They only agree dead ahead.

  So both are converted into actual 3D points, transformed into base_link via
  TF, and reduced to "distance from the robot's origin to the nearest obstacle
  point". That stays correct if you remount either sensor -- the URDF is the
  single source of truth, exactly as it is for the rest of the stack.

WHAT IT PUBLISHES
  /obstacle_check/lidar_range   Float32  nearest lidar point, from base_link
  /obstacle_check/camera_range  Float32  nearest depth point, from base_link
  /obstacle_check/agree         Bool     within tolerance of each other
  /obstacle_check/summary       String   human-readable, for `ros2 topic echo`
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, Float32, String

import tf2_ros


def quat_to_matrix(q):
    """Rotation matrix from a geometry_msgs Quaternion (no transforms3d dep)."""
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


class ObstacleVerifier(Node):

    def __init__(self):
        super().__init__('obstacle_verifier')

        p = self.declare_parameter
        p('base_frame', 'base_link')
        # Wedge, measured in base_link, over which the two are compared.
        # ~86 deg matches the D455's depth FOV.
        p('compare_fov_deg', 86.0)
        p('agree_tolerance', 0.20)     # metres before they count as disagreeing
        p('depth_percentile', 5.0)     # robust stand-in for "nearest"
        p('roi_width_frac', 0.6)       # central fraction of the image used
        p('roi_height_frac', 0.5)
        p('pixel_stride', 4)           # subsample; full ROI is ~90k points
        p('min_depth', 0.2)
        p('max_depth', 6.0)
        # Heights are in base_link z, and base_link sits on the WHEEL AXLE,
        # so the floor is at -wheel_diameter/2 = -0.0475 m (see the
        # base_footprint joint in the xacro). The old -0.05 default therefore
        # cleared the ground by 2.5 mm, which is not a margin at all: depth
        # noise, an unmeasured camera_z, a degree of camera pitch or an uneven
        # floor all put ground points above it. The symptom is
        # /obstacle_check/camera_range pinned at a constant ~0.42 m while the
        # lidar varies, i.e. every frame logging "CAMERA SEES CLOSER".
        # 0.02 clears the ground by 6.75 cm.
        #
        # COST OF RAISING IT: obstacles shorter than 6.75 cm are invisible to
        # this check. That is acceptable because the lidar plane is at
        # lidar_z = 0.09 m, so the camera still sees lower than the sensor it
        # is cross-checking. Do not raise it past the lidar height or the
        # cross-check stops adding anything.
        p('min_height', 0.02)          # ignore the floor, in base_link z
        p('max_height', 1.50)          # ignore the ceiling
        p('report_period', 0.5)

        g = lambda n: self.get_parameter(n).value
        self.base = g('base_frame')
        self.half_fov = math.radians(g('compare_fov_deg')) / 2.0
        self.tol = g('agree_tolerance')
        self.pct = g('depth_percentile')
        self.wfrac, self.hfrac = g('roi_width_frac'), g('roi_height_frac')
        self.stride = max(1, int(g('pixel_stride')))
        self.dmin, self.dmax = g('min_depth'), g('max_depth')
        self.zmin, self.zmax = g('min_height'), g('max_height')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._tf_cache = {}

        self.K = None                  # fx, fy, cx, cy
        self.lidar_r = float('nan')
        self.cam_r = float('nan')

        self.pub_l = self.create_publisher(Float32, 'obstacle_check/lidar_range', 10)
        self.pub_c = self.create_publisher(Float32, 'obstacle_check/camera_range', 10)
        self.pub_a = self.create_publisher(Bool, 'obstacle_check/agree', 10)
        self.pub_s = self.create_publisher(String, 'obstacle_check/summary', 10)

        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Image, 'depth_image', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'depth_info', self.on_info, qos_profile_sensor_data)
        self.create_timer(g('report_period'), self.report)

        self.get_logger().info(
            f'obstacle_verifier: comparing in {self.base} over the central '
            f'+/-{math.degrees(self.half_fov):.0f} deg wedge')

    # -------------------------------------------------------------------- tf
    def transform_for(self, source):
        """Cached source->base transform. These are static, so one lookup holds."""
        if source in self._tf_cache:
            return self._tf_cache[source]
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base, source, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.get_logger().warn(
                f'no TF {self.base} <- {source} yet; is the URDF being published?',
                throttle_duration_sec=10.0)
            return None
        R = quat_to_matrix(tf.transform.rotation)
        t = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z])
        self._tf_cache[source] = (R, t)
        return R, t

    def nearest_in_wedge(self, pts):
        """Min distance from base_link origin to any point inside the wedge."""
        if pts.size == 0:
            return float('nan')
        pts = pts[(pts[:, 2] >= self.zmin) & (pts[:, 2] <= self.zmax)]
        if pts.size == 0:
            return float('nan')
        bearing = np.arctan2(pts[:, 1], pts[:, 0])
        pts = pts[np.abs(bearing) <= self.half_fov]
        if pts.size == 0:
            return float('nan')
        return float(np.min(np.linalg.norm(pts, axis=1)))

    # ------------------------------------------------------------------ scan
    def on_scan(self, msg: LaserScan):
        tf = self.transform_for(msg.header.frame_id)
        if tf is None:
            return
        R, t = tf

        r = np.asarray(msg.ranges, dtype=np.float32)
        ang = msg.angle_min + np.arange(r.size, dtype=np.float32) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        r, ang = r[ok], ang[ok]
        if r.size == 0:
            self.lidar_r = float('nan')
            return

        local = np.stack([r * np.cos(ang), r * np.sin(ang), np.zeros_like(r)], axis=1)
        self.lidar_r = self.nearest_in_wedge(local @ R.T + t)

    # ----------------------------------------------------------------- depth
    def on_info(self, msg: CameraInfo):
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        if msg.k[0] > 0:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_depth(self, msg: Image):
        if self.K is None:
            self.get_logger().warn('waiting for depth camera_info (intrinsics)',
                                   throttle_duration_sec=10.0)
            return
        if msg.encoding not in ('16UC1', 'mono16'):
            self.get_logger().warn(
                f'unexpected depth encoding {msg.encoding}; expected 16UC1',
                throttle_duration_sec=10.0)
            return
        tf = self.transform_for(msg.header.frame_id)
        if tf is None:
            return
        R, t = tf

        try:
            buf = np.frombuffer(msg.data, dtype=np.uint16)
            img = buf.reshape(msg.height, msg.step // 2)[:, :msg.width]
        except ValueError:
            return

        h, w = img.shape
        rh, rw = int(h * self.hfrac), int(w * self.wfrac)
        y0, x0 = (h - rh) // 2, (w - rw) // 2
        roi = img[y0:y0 + rh:self.stride, x0:x0 + rw:self.stride]

        Z = roi.astype(np.float32) / 1000.0            # mm -> m
        valid = (Z >= self.dmin) & (Z <= self.dmax)
        if np.count_nonzero(valid) < 50:
            self.cam_r = float('nan')
            return

        fx, fy, cx, cy = self.K
        vs, us = np.nonzero(valid)
        # Map ROI indices back to full-image pixel coordinates before applying
        # intrinsics, otherwise every point is skewed toward the image centre.
        u = x0 + us * self.stride
        v = y0 + vs * self.stride
        z = Z[valid]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        pts_optical = np.stack([x, y, z], axis=1)
        pts_base = pts_optical @ R.T + t

        d = np.linalg.norm(pts_base, axis=1)
        bearing = np.arctan2(pts_base[:, 1], pts_base[:, 0])
        keep = (np.abs(bearing) <= self.half_fov) & \
               (pts_base[:, 2] >= self.zmin) & (pts_base[:, 2] <= self.zmax)
        d = d[keep]
        # A percentile, not the strict minimum: one hot pixel or a speck of
        # depth noise would otherwise dominate every frame.
        self.cam_r = float(np.percentile(d, self.pct)) if d.size >= 50 else float('nan')

    # ---------------------------------------------------------------- report
    def report(self):
        l, c = self.lidar_r, self.cam_r
        self.pub_l.publish(Float32(data=float(l) if math.isfinite(l) else float('nan')))
        self.pub_c.publish(Float32(data=float(c) if math.isfinite(c) else float('nan')))

        if math.isfinite(l) and math.isfinite(c):
            delta = c - l
            agree = abs(delta) <= self.tol
            if agree:
                note = 'agree'
            elif c < l:
                # The asymmetry that matters: the camera seeing something
                # nearer means the lidar is missing a real obstacle, so the
                # guard believes it has more clearance than it does.
                note = 'CAMERA SEES CLOSER - lidar may be missing this obstacle'
            else:
                note = 'lidar sees closer (likely outside the camera FOV or below it)'
            txt = f'lidar={l:.2f} m  camera={c:.2f} m  delta={delta:+.2f} m  {note}'
        else:
            agree = False
            txt = (f'lidar={"n/a" if not math.isfinite(l) else f"{l:.2f} m"}  '
                   f'camera={"n/a" if not math.isfinite(c) else f"{c:.2f} m"}  '
                   f'(waiting for both sensors)')

        self.pub_a.publish(Bool(data=bool(agree)))
        self.pub_s.publish(String(data=txt))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
