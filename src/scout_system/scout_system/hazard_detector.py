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

The HSV side is unchanged. The depth side switched from monocular
projection (which fought us at every step on cubes) to LiDAR fusion:
camera tells us color + bearing, lidar tells us range. Each does what
it's best at, and the failure modes barely overlap.

You implement four algorithm methods (see TODOs):
    * ``_detection_bearing`` -- bbox -> bearing range in robot/lidar frame
    * ``_cluster_scan``      -- segment scan in camera FOV by depth jumps
    * ``_match_cluster``     -- associate one detection to one cluster
    * ``_fuse_to_map``       -- orchestrator that calls the above

Everything else (subscriptions, image conversion, scan caching, TF
lookup, marker building, publishing) is plumbed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
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

    border_*_range carries the *range of the first valid ray just past
    this cluster's edge* on that side, or NaN if the next ray was
    invalid (no return / over max_range). The cone-shape filter
    insists both border ranges are valid finite values significantly
    farther than cluster.range -- a real cone has the wall behind it
    at clearance, while wall slices at max_range have NaN borders and
    pillar sub-clusters have borders at near-equal range.
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
        # TB3 burger lidar frame. Override for the real robot if needed.
        self.declare_parameter('lidar_frame', 'base_scan')
        # Cached scan must be fresher than this when an image arrives.
        self.declare_parameter('max_scan_age_s', 0.3)
        # Range jump (m) between consecutive rays that splits a cluster.
        self.declare_parameter('cluster_depth_jump_m', 0.20)
        # Reject clusters whose physical width at their range exceeds
        # this -- filters walls + big objects, keeps cone-sized things.
        self.declare_parameter('cluster_max_width_m', 0.30)
        # Min rays per cluster. 1 lets faraway cones through; 2-3 is
        # less noisy.
        self.declare_parameter('cluster_min_rays', 1)
        # Pillar-rejection clearance. A border on either side must be
        # EITHER (a) "no return / unverified" (NaN) -- consistent with
        # a free-standing cone with nothing close behind/beside it --
        # OR (b) at least this much farther than the cluster itself.
        # If a border is a valid finite range BUT close to the
        # cluster's range, we treat the cluster as part of a larger
        # multi-bit object (the pillar) and reject it.
        self.declare_parameter('cone_clearance_m', 0.5)
        # Wall-slice-at-max-range filter: reject any cluster within
        # this margin of the lidar's max_range. The lidar's range
        # limit clips far walls into thin "objects" sitting exactly
        # at max_range, which is the clearest signature we have to
        # kill. 0.4 m at max_range=3.5 -> reject anything beyond 3.1 m.
        self.declare_parameter('max_range_margin_m', 0.4)
        # Max angular gap (deg) between detection bearing and cluster
        # bearing for them to be associated.
        self.declare_parameter('bearing_match_tol_deg', 5.0)
        # Sanity cap on how far we'll trust a fused detection.
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
        self.match_tol = math.radians(
            float(self.get_parameter('bearing_match_tol_deg').value)
        )
        self.max_range = float(self.get_parameter('max_range_m').value)

        self.cfg = self._load_config()
        det_cfg = self.cfg.get('detector', {})
        backend = det_cfg.get('backend', 'hsv')
        self.detector = build_detector(backend, det_cfg)
        self.color_to_category: Dict[str, str] = self.cfg.get('color_to_category', {})

        # --------------- state ---------------------------------------
        self.bridge = CvBridge()
        self.intrinsics: Optional[np.ndarray] = None
        self.image_size: Optional[Tuple[int, int]] = None  # (W, H)
        self.latest_scan: Optional[LaserScan] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.info_topic, self._info_cb, SENSOR_QOS)
        self.create_subscription(Image, self.image_topic, self._image_cb, SENSOR_QOS)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, SENSOR_QOS)

        self.pub_raw = self.create_publisher(Hazard, '/hazards/raw', 20)
        self.pub_markers = self.create_publisher(MarkerArray, '/hazards/raw/markers', 10)

        self.get_logger().info(
            f"HazardDetector ready (backend={backend}, image={self.image_topic}, "
            f"scan={self.scan_topic}, lidar_frame={self.lidar_frame})"
        )

    # ------------------------------------------------------------------ config
    def _load_config(self) -> dict:
        """Load ``hazard_params.yaml`` off disk. Plumbing only."""
        path = self.get_parameter('params_file').value
        if not path:
            self.get_logger().warn('No params_file provided; using empty config.')
            return {}
        try:
            with open(path) as fh:
                return yaml.safe_load(fh) or {}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to load {path}: {exc}')
            return {}

    # ------------------------------------------------------------------ sub cbs
    def _info_cb(self, msg: CameraInfo):
        """Cache the 3x3 intrinsic matrix + image size on first message."""
        if self.intrinsics is None:
            self.intrinsics = np.array(msg.k).reshape(3, 3)
            self.image_size = (msg.width, msg.height)
            k = self.intrinsics
            self.get_logger().info(
                f'Camera intrinsics received (fx={k[0, 0]:.1f}, fy={k[1, 1]:.1f}, '
                f'cx={k[0, 2]:.1f}, cy={k[1, 2]:.1f}, hfov={math.degrees(self._camera_hfov()):.1f} deg)'
            )

    def _scan_cb(self, msg: LaserScan):
        """Cache the latest LaserScan for fusion in _image_cb. Plumbing only."""
        self.latest_scan = msg

    def _image_cb(self, msg: Image):
        """Process one frame: HSV -> for each detection, fuse with scan, publish.

        Plumbing only -- you write _fuse_to_map (and its helpers).
        """
        if self.intrinsics is None or self.latest_scan is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(
                f'Failed to convert image: {e}', throttle_duration_sec=5.0
            )
            return
        detections = self.detector.detect(frame)
        if not detections:
            return

        # Snapshot the scan up front so all detections in this frame fuse
        # against the same range data (otherwise _scan_cb might update
        # self.latest_scan mid-loop).
        scan = self.latest_scan
        # Scan-vs-image desync: this is what actually matters for fusion
        # correctness. If the scan was taken too far before/after the
        # image, the robot may have rotated/moved between them and the
        # cluster will land in the wrong map cell.
        img_ns = Time.from_msg(msg.header.stamp).nanoseconds
        scan_ns = Time.from_msg(scan.header.stamp).nanoseconds
        desync_s = abs(img_ns - scan_ns) / 1e9
        if desync_s > self.max_scan_age:
            self.get_logger().warn(
                f'Scan/image desync ({desync_s:.2f}s > {self.max_scan_age:.2f}s); '
                f'skipping frame.',
                throttle_duration_sec=5.0,
            )
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
    # ALGORITHM (TODO YOU)
    # ===================================================================

    def _fuse_to_map(
        self, det: Detection, scan: LaserScan, stamp
    ) -> Optional[Point]:
        """Top-level: HSV detection + LiDAR scan -> 3D point in map frame.

        TODO(you): orchestrate the four steps:
            1. Compute the detection's bearing range in the robot/lidar
               frame (call ``self._detection_bearing(det)``). If it
               returns None (clipped bbox), bail with None.
            2. Cluster the lidar scan inside the bbox angular extent
               (call ``self._cluster_scan(scan, left_bearing, right_bearing)``).
               If the result is empty, bail with None.
            3. Pick the cluster that best matches this detection's
               bearing (call ``self._match_cluster(...)``). If nothing
               matches within tolerance, bail with None.
            4. Convert the cluster's polar (bearing, range) into a
               cartesian point in the lidar frame:
                    x_l = range * cos(bearing)
                    y_l = range * sin(bearing)
                    z_l = 0.0
               Then TF that point into self.map_frame using
               ``self._lidar_to_map(x_l, y_l, z_l, stamp)`` and return it.

        Return None on any failure so _image_cb can just skip.
        """
        # raise NotImplementedError("TODO(you): implement detection + scan fusion.")

        det_bearings = self._detection_bearing(det)
        if det_bearings is None:
            return None
        # Cluster across the FRONT HALF of the lidar (+-pi/2). Camera
        # FOV is +-31 deg, so cones are always well inside this window
        # with +59 deg of lidar context on either side -- way more than
        # enough for the bilateral cone-shape filter to find the wall
        # behind. Going wider hits a wraparound problem at the lidar's
        # angle-zero seam (cones straight ahead get bisected).
        clusters = self._cluster_scan(scan, -math.pi / 2.0, +math.pi / 2.0)
        if not clusters:
            return None
        cluster = self._match_cluster(det_bearings, clusters)
        if cluster is None:
            return None
        x_l = cluster.range * math.cos(cluster.bearing)
        y_l = cluster.range * math.sin(cluster.bearing)
        z_l = 0.0
        # Use SCAN time (not image time, not "now") for the TF lookup.
        # The cluster's (x_l, y_l) is the cone's position in the lidar
        # frame *as that frame was at scan.header.stamp*. Looking up the
        # lidar->map transform at any other time misaligns the rotation
        # and smears phantom positions across the map during rotation.
        map_point = self._lidar_to_map(x_l, y_l, z_l, scan.header.stamp)
        return map_point

    def _detection_bearing(
        self, det: Detection
    ) -> Optional[Tuple[float, float, float]]:
        """Map detection bbox (image pixels) -> bearing range (lidar frame).

        TODO(you):
            1. From the bbox, compute three pixel columns:
                 left   = det.cx - det.w / 2
                 center = det.cx
                 right  = det.cx + det.w / 2
               If left or right is within 1-2 px of the image border
               (use self.image_size), the bbox is clipped and the
               bearing won't be reliable -- return None.
            2. Convert each pixel column ``u`` to a *camera-frame*
               bearing using the pinhole model:
                   bearing_cam = atan2(u - cx_intr, fx)
               where cx_intr = self.intrinsics[0, 2] and fx =
               self.intrinsics[0, 0]. Positive bearing_cam = right of
               principal axis.
            3. Convert each camera bearing to a *lidar-frame* bearing.
               On the TB3 burger, the camera principal axis points
               forward (+x in the robot frame) and so does the lidar's
               zero ray. Lidar yaw is positive CCW (left), opposite of
               camera u. So the conversion is just sign flip:
                   bearing_lidar = -bearing_cam
               (If you ever swap to a robot where the camera and lidar
               aren't co-aligned, replace this with a TF-based rotation
               of a unit vector instead.)
            4. Return (left_bearing, center_bearing, right_bearing) in
               the lidar frame. NOTE: a left pixel becomes a positive
               (left) lidar bearing, so left_bearing > right_bearing
               after the sign flip. That's fine -- just keep track of
               which is which when you pass them to _cluster_scan.
        """
        # raise NotImplementedError("TODO(you): implement bbox -> bearing range.")

        left = det.cx - det.w / 2
        center = det.cx
        right = det.cx + det.w / 2
        if left < 0 or right > self.image_size[0]:
            return None
            
        bearing_cam_left = math.atan2(left - self.intrinsics[0, 2], self.intrinsics[0, 0])
        bearing_cam_right = math.atan2(right - self.intrinsics[0, 2], self.intrinsics[0, 0])
        bearing_cam_center = math.atan2(center - self.intrinsics[0, 2], self.intrinsics[0, 0])

        bearing_lidar_left = -bearing_cam_left
        bearing_lidar_right = -bearing_cam_right
        bearing_lidar_center = -bearing_cam_center

        return (bearing_lidar_left, bearing_lidar_center, bearing_lidar_right)

    def _cluster_scan(
        self,
        scan: LaserScan,
        bearing_lo: float,
        bearing_hi: float,
    ) -> List[_ScanCluster]:
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



    def _match_cluster(
        self,
        det_bearings: Tuple[float, float, float],
        clusters: List[_ScanCluster],
    ) -> Optional[_ScanCluster]:
        """Pick the cluster best matching this detection.

        TODO(you):
            1. Simplest: pick the cluster whose mean bearing is closest
               to det_bearings[1] (the detection's center bearing).
               Use ``abs(cluster.bearing - center_bearing)``, with the
               same wrap-fix from _cluster_scan if needed.
            2. Reject if that closest cluster's bearing is more than
               self.match_tol away from the detection center -- means
               the lidar didn't actually see anything where the camera
               points.
            3. Optional bonus: prefer clusters whose mean bearing falls
               INSIDE the bbox angular extent
               (min(det_bearings[0], det_bearings[2]) ..
                max(det_bearings[0], det_bearings[2])),
               and only fall back to plain nearest if none do. Helps
               when two cones are close in bearing.
            4. Return the picked cluster, or None if nothing matches.
        """
        # raise NotImplementedError("TODO(you): implement detection -> cluster match.")

        best_cluster = None
        best_dist = self.match_tol
        for cluster in clusters:
            dist = abs(cluster.bearing - det_bearings[1])
            if dist < best_dist:
                best_cluster = cluster
                best_dist = dist
        return best_cluster

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
