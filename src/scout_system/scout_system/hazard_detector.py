#!/usr/bin/env python3
"""Per-frame hazard detector with camera + LiDAR fusion (SKELETON).

Pipeline:
    camera image  --HSV-->  colored blob detections (color + bbox)
                                     |
                                     v   bearing
                                     |
    /scan         --cluster->  cone-candidate clusters (bearing + range)
                                     |
                                     v   bearing match
                                     |
                            fused (color + 3D position)
                                     |
                                     v
                            /hazards/raw + RViz markers



You implement four algorithm methods (see TODOs):
    * ``_detection_bearing`` -- bbox -> bearing range in robot/lidar frame
    * ``_cluster_scan``      -- segment scan in camera FOV by depth jumps
    * ``_match_cluster``     -- associate one detection to one cluster
    * ``_fuse_to_map``       -- orchestrator that calls the above

Everything else (subscriptions, image conversion, scan caching, TF
lookup, marker building, publishing) is plumbed.
"""

import math
from dataclasses import dataclass
from posix import minor
from typing import Dict, List, Optional, Tuple
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix
from visualization_msgs.msg import Marker, MarkerArray

from scout_msgs.msg import Hazard
from scout_system.detectors import Detection, build_detector


SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


_COLOR_TO_RGBA = {
    'red':    ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9),
    'yellow': ColorRGBA(r=1.0, g=0.95, b=0.1, a=0.9),
    'blue':   ColorRGBA(r=0.1, g=0.3, b=1.0, a=0.9),
    'green':  ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.9),
}


@dataclass
class _ScanCluster:
    """One contiguous chunk of LaserScan rays seeing the same object.
    Bearings are in the lidar's frame (positive = CCW / robot's left,
    zero = robot forward). Algorithm produces these, _match_cluster
    consumes them.
    """
    bearing: float        # weighted mean angle [rad]
    range: float          # mean range [m]
    bearing_min: float    # leftmost ray angle [rad]
    bearing_max: float    # rightmost ray angle [rad]
    n_rays: int
    border_left_range: float = float('nan')
    border_right_range: float = float('nan')


class HazardDetector(Node):
    def __init__(self):
        super().__init__('hazard_detector')

        # --------------- params --------------------------------------
        self.declare_parameter('params_file', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('lidar_frame', 'base_scan')
        self.declare_parameter('max_scan_age_s', 0.3)
        self.declare_parameter('cluster_depth_jump_m', 0.20)
        self.declare_parameter('cluster_max_width_m', 0.30)
        self.declare_parameter('cluster_min_rays', 1)
        self.declare_parameter('cone_clearance_m', 0.5)
        self.declare_parameter('max_range_margin_m', 0.4)
        self.declare_parameter('bearing_match_tol_deg', 5.0)
        self.declare_parameter('max_range_m', 4.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.info_topic = self.get_parameter('camera_info_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.max_scan_age = float(self.get_parameter('max_scan_age_s').value)
        self.depth_jump = float(self.get_parameter('cluster_depth_jump_m').value)
        self.max_cluster_width = float(self.get_parameter('cluster_max_width_m').value)
        self.min_rays = int(self.get_parameter('cluster_min_rays').value)
        self.cone_clearance = float(self.get_parameter('cone_clearance_m').value)
        self.max_range_margin = float(self.get_parameter('max_range_margin_m').value)
        self.match_tol = math.radians(float(self.get_parameter('bearing_match_tol_deg').value))
        self.max_range = float(self.get_parameter('max_range_m').value)

        self.cfg = self._load_config()
        det_cfg = self.cfg.get('detector', {})
        backend = det_cfg.get('backend', 'hsv')
        self.detector = build_detector(backend, det_cfg)
        self.color_to_category: Dict[str, str] = self.cfg.get('color_to_category', {})


        self.bridge = CvBridge()
        self.intrinsics: Optional[np.ndarray] = None
        self.image_size: Optional[Tuple[int, int]] = None  # (W, H)
        self.latest_scan: Optional[LaserScan] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.info_topic, self.intrinsics_cb, SENSOR_QOS)
        self.create_subscription(Image, self.image_topic, self._image_cb, SENSOR_QOS)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, SENSOR_QOS)

        self.pub_raw = self.create_publisher(Hazard, '/hazards/raw', 20)
        self.pub_markers = self.create_publisher(MarkerArray, '/hazards/raw/markers', 10)

        self.get_logger().info(
            f"HazardDetector ready (backend={backend}, image={self.image_topic}, "
            f"scan={self.scan_topic}, lidar_frame={self.lidar_frame})"
        )

    # ------------------------------------------------------------------ callbacks brebv
    def _load_config(self) -> dict:
        path = self.get_parameter('params_file').value
        if not path:
            return {}
        try:
            with open(path) as file:
                return yaml.safe_load(file) or {}
        except Exception:
            print("failed darn tootin")
            return {}

    def intrinsics_cb(self, msg: CameraInfo):
        if self.intrinsics is None:
            self.intrinsics = np.array(msg.k).reshape(3, 3)
            self.image_size = (msg.width, msg.height)
            k = self.intrinsics
            print(f"Camera intrinsics {k}")

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _image_cb(self, msg: Image):
    #   image hsv processing brev

        if self.intrinsics is None or self.latest_scan is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            return


        detections = self.detector.detect(frame)
        if not detections:
            return

        scan = self.latest_scan
        img_ns = Time.from_msg(msg.header.stamp).nanoseconds
        scan_ns = Time.from_msg(scan.header.stamp).nanoseconds
        differ_s = abs(img_ns - scan_ns) / 1e

        if self.max_scan_age < differ_s:
            print("there is too much diff")
            return

        markers = MarkerArray()
        for idx, det in enumerate(detections):
            map_point = self._fuse_to_map(det, scan, msg.header.stamp)
            if map_point is None:
                continue
            hazard = Hazard()
            hazard.header.stamp = msg.header.stamp
            hazard.header.frame_id = self.map_frame
            hazard.id = -1
            hazard.color = det.label
            hazard.category = self.color_to_category.get(det.label, 'unknown')
            hazard.position = map_point
            hazard.confidence = det.confidence
            self.pub_raw.publish(hazard)
            markers.markers.append(self._make_marker(idx, hazard, msg.header.stamp))

        if markers.markers:
            self.pub_markers.publish(markers)

    # ===================================================================
    # ALGORITHM (TODO)
    # ===================================================================

    def _fuse_to_map(self, det: Detection, scan: LaserScan, stamp) -> Optional[Point]:

        """Top-level: HSV detection + LiDAR scan -> 3D point in map frame.
        TODO(you): orchestrate the four steps:
        """

        det_bearings = self._detection_bearing(det)
        if det_bearings is None:
            return None

        thresh = math.pi / 2.0
        clusters = self._cluster_scan(scan, -thresh, +thresh)
        if not clusters:
            return None

        cluster = self._match_cluster(det_bearings, clusters)
      
        if cluster is None:
            return None

        r = cluster.range
        alpha = cluster.bearing

        x_l = r * math.cos(alpha)
        y_l = r * math.sin(alpha)
        z_l = 0.0
        
        map_point = self._lidar_to_map(x_l, y_l, z_l, scan.header.stamp)

        return map_point

    def _detection_bearing(self, det: Detection) -> Optional[Tuple[float, float, float]]:
        
        """TODO: Map detection bbox (image pixels) -> bearing range (lidar frame).
        Hints:
            1. Convert each pixel column X to a *camera-frame*
               bearing using the pinhole model:
            2. Convert each camera bearing to a *lidar-frame* bearing.
               On the TB3 burger, the camera principal axis points
               forward (+x in the robot frame) and so does the lidar's
               zero ray. Lidar yaw is positive CCW (left), opposite of
               camera u. So the conversion is just sign flip:
                   bearing_lidar = -bearing_cam
            3. return left, center, right bearings in lidar frame.
        """

        left_pixel = det.cx - det.w / 2
        center_pixel = det.cx
        right_pixel = det.cx + det.w / 2
        cx = self.intrinsics[0, 2]
        fx = self.intrinsics[0, 0]

        if left_pixel < 0 or right_pixel > self.image_size[0]:
            return None
            
        bearing_cam_left = math.atan2(left_pixel - cx, fx)
        bearing_cam_right = math.atan2(right_pixel - cx, fx)
        bearing_cam_center = math.atan2(center_pixel - cx, fx)

        bearing_lidar_left = -bearing_cam_left
        bearing_lidar_right = -bearing_cam_right
        bearing_lidar_center = -bearing_cam_center

        return (bearing_lidar_left, bearing_lidar_center, bearing_lidar_right)

    def _cluster_scan(self, scan: LaserScan, bearing_lo: float, bearing_hi: float,) -> List[_ScanCluster]:

        
        """Segment the LaserScan inside [bearing_lo, bearing_hi] by depth jumps.

        TODO(you):
            1. Walk the scan rays. Bearing of ray i is
                   angle_i = scan.angle_min + i * scan.angle_increment
               (wrap into (-pi, pi] if your lidar's angle_max > pi).
               Skip rays whose bearing is outside [bearing_lo, bearing_hi]
               and rays whose range is inf, NaN, < scan.range_min,
               > scan.range_max, or > self.max_range.
            2. Group consecutive valid rays into clusters: start a new
               cluster whenever the range of the current ray differs
               from the previous valid ray by more than self.depth_jump.
            3. For each cluster, compute:
                 - mean bearing (simple mean is fine, or weight by 1/range)
                 - mean range
                 - bearing_min, bearing_max
                 - n_rays
            4. Filter:
                 - drop if n_rays < self.min_rays
                 - compute physical width at the cluster's range:
                     width = 2 * range * sin((bearing_max - bearing_min) / 2)
                   drop if width > self.max_cluster_width (walls /
                   big stuff). A 14 cm cone shows up as <= ~0.20 m
                   wide cluster, so 0.30 m is a comfy ceiling.
            5. Return the list of surviving _ScanCluster objects.

        Tip: scan.angle_min/max define full lidar coverage. For a
        360-deg lidar this is typically [0, 2pi] or [-pi, pi]. If your
        bbox bearing range straddles the wrap (e.g. -0.1 .. +0.1 on a
        [0, 2pi] lidar), handle that by normalizing all angles to
        (-pi, pi] before comparing. ``math.atan2(sin(a), cos(a))`` is
        a one-line wrap helper.
        """
        lo, hi = min(bearing_lo, bearing_hi), max(bearing_lo, bearing_hi)
        n_rays = len(scan.ranges)
        if n_rays == 0:
            return []

        # The loop walks the scan CIRCULARLY (i = (start + j) % n_rays)
        # so a cone straddling the lidar's wraparound (e.g. forward on a
        # 0..2pi lidar, where ray 0 and ray n-1 are physically adjacent
        # but linearly far apart) clusters as a single object instead of
        # two halves. We pick `start` to be an out-of-window index so
        # the loop's circular seam falls in the don't-care region.
        start = 0
        for k in range(n_rays):
            a = scan.angle_min + k * scan.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            if a < lo or a > hi:
                start = k
                break

        clusters: List[_ScanCluster] = []
        current: Optional[_ScanCluster] = None
        prev_r = None                            # range of last valid in-window ray
        border_left_for_next = float('nan')      # left-border range of next cluster

        def _close_unbordered():
            """Close any open cluster with NaN right border."""
            nonlocal current
            if current is not None:
                current.border_right_range = float('nan')
                clusters.append(current)
                current = None

        for j in range(n_rays):
            i = (start + j) % n_rays
            r = scan.ranges[i]
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))  # wrap to (-pi, pi]
            if angle < lo or angle > hi:
                # Out-of-window. Treat like a hard cluster terminator
                # (we can't keep prev_r across the back of the robot:
                # a ray at +pi/2 and one at -pi/2 are not adjacent in
                # space even if they are adjacent in array order after
                # masking out everything in between).
                _close_unbordered()
                prev_r = None
                border_left_for_next = float('nan')
                continue
            if not math.isfinite(r) or r < scan.range_min or r > self.max_range:
                # No-return / out-of-range. We don't know what's there,
                # so close with NaN right border (= "unverified").
                _close_unbordered()
                prev_r = None
                border_left_for_next = float('nan')
                continue
            depth_jumped = (prev_r is not None and abs(r - prev_r) > self.depth_jump)
            if current is not None and depth_jumped:
                # Cluster ends, *this* valid ray is its right border.
                current.border_right_range = r
                clusters.append(current)
                current = None
                # Next cluster's left border = previous ray's range.
                border_left_for_next = prev_r
            if current is None:
                current = _ScanCluster(
                    bearing=angle, range=r,
                    bearing_min=angle, bearing_max=angle, n_rays=1,
                    border_left_range=border_left_for_next,
                    border_right_range=float('nan'),
                )
                border_left_for_next = float('nan')
            else:
                n = current.n_rays
                current.bearing = (current.bearing * n + angle) / (n + 1)
                current.range = (current.range * n + r) / (n + 1)
                current.bearing_min = min(current.bearing_min, angle)
                current.bearing_max = max(current.bearing_max, angle)
                current.n_rays = n + 1
            prev_r = r
        # Loop done. Any still-open cluster has unverified right
        # border -> NaN -> dropped below. (With circular start the
        # only way to reach here open is if the entire window is one
        # uninterrupted blob, which should never look like a cone.)
        _close_unbordered()

        out: List[_ScanCluster] = []
        max_range_floor = self.max_range - self.max_range_margin
        for c in clusters:
            if c.n_rays < self.min_rays:
                continue
            width = 2.0 * c.range * math.sin((c.bearing_max - c.bearing_min) / 2.0)
            if width > self.max_cluster_width:
                continue
            # Wall-slice-at-max-range filter: a wall whose visible
            # portion is clipped by the lidar's range limit (3.5 m on
            # the burger) sits *exactly* at max_range and looks like
            # a thin object. Reject anything within max_range_margin
            # of the lidar's hard limit. This is the surest way to
            # kill those wall ghosts since they are *defined* by
            # being right at that limit.
            if c.range > max_range_floor:
                continue
            # Pillar-sub-cluster filter: if either side is bordered
            # by a VALID FINITE range that is NOT meaningfully farther
            # than this cluster (the "near-equal range neighbour"
            # signature of part-of-a-larger-object), reject. NaN on
            # one side is fine -- it just means open space / out of
            # range beyond, which is consistent with a free-standing
            # cone in the middle of an arena.
            def _too_close(border: float) -> bool:
                return (not math.isnan(border)
                        and border < c.range + self.cone_clearance)
            if _too_close(c.border_left_range):
                continue
            if _too_close(c.border_right_range):
                continue
            out.append(c)
        return out



    def _match_cluster(self,det_bearings: Tuple[float, float, float],clusters: List[_ScanCluster], ) -> Optional[_ScanCluster]:
        # TODO: 

        middle = det_bearings[1]
        retval = None
        min_dist = self.match_tol

        for c in clusters:
            dist = abs(c.bearing - middle)
            if dist < min_dist:
                retval = c
                min_dist = dist  

        return retval

    # ===================================================================
    # PLUMBING (DONE)
    # ===================================================================

    def _lidar_to_map(
        self, x_l: float, y_l: float, z_l: float, stamp
    ) -> Optional[Point]:
        """Transform a point from self.lidar_frame to self.map_frame.

        Uses the passed stamp (typically scan.header.stamp) so that the
        lidar coordinates and the lidar->map transform are pinned to the
        same instant in time. Falls back to the latest TF if the buffer
        doesn't have history that far back (typical at startup).
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.lidar_frame, Time.from_msg(stamp),
                timeout=Duration(seconds=0.1),
            )
        except Exception:  # noqa: BLE001
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.lidar_frame, Time(),
                    timeout=Duration(seconds=0.2),
                )
            except Exception as e:
                self.get_logger().warn(
                    f'TF lookup failed: {e}', throttle_duration_sec=5.0
                )
                return None
        q = tf.transform.rotation
        R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        t = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z])
        p = R @ np.array([x_l, y_l, z_l]) + t
        return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))

    def _camera_hfov(self) -> float:
        """Camera horizontal FOV in radians. Computed from intrinsics."""
        if self.intrinsics is None or self.image_size is None:
            return math.radians(60.0)
        fx = self.intrinsics[0, 0]
        return 2.0 * math.atan2(self.image_size[0] / 2.0, fx)

    def _make_marker(self, idx: int, hz: Hazard, stamp) -> Marker:
        """RViz SPHERE marker at hz.position, coloured by hz.color."""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = 'hazards_raw'
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = hz.position
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.lifetime.sec = 1
        marker.color = _COLOR_TO_RGBA.get(
            hz.color, ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        )
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = HazardDetector()
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
