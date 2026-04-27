#!/usr/bin/env python3
"""Per-frame hazard detector.

Subscribes to the camera image + intrinsics, runs a configurable backend
(see ``scout_system.detectors``), projects each detection's pixel center
onto the world *ground plane* (z = cube_half_height) using TF between
``map`` and the camera optical frame, and publishes one ``scout_msgs/Hazard``
per detection on ``/hazards/raw`` plus a visualization MarkerArray on
``/hazards/raw/markers``.

Pose estimation is deliberately simple for v1:
    * intrinsics from /camera/camera_info,
    * back-project pixel to a ray in the optical frame,
    * intersect that ray with the plane z = cube_half_height in ``map``.

This is good enough while the cubes sit on a flat floor. The clear
TODO hook is laser fusion (use /scan range at the detection's bearing
for real depth) -- swap it in here once the HSV path is solid.
"""
from __future__ import annotations

from typing import Dict, Optional

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
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


class HazardDetector(Node):
    def __init__(self):
        super().__init__('hazard_detector')

        self.declare_parameter('params_file', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('cube_size_m', 0.15)
        self.declare_parameter('max_range_m', 3.5)

        self.image_topic = self.get_parameter('image_topic').value
        self.info_topic = self.get_parameter('camera_info_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.cube_size = float(self.get_parameter('cube_size_m').value)
        self.max_range = float(self.get_parameter('max_range_m').value)

        self.cfg = self._load_config()
        det_cfg = self.cfg.get('detector', {})
        backend = det_cfg.get('backend', 'hsv')
        self.detector = build_detector(backend, det_cfg)
        self.color_to_category: Dict[str, str] = self.cfg.get('color_to_category', {})

        self.bridge = CvBridge()
        self.intrinsics: Optional[np.ndarray] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.info_topic, self._info_cb, SENSOR_QOS)
        self.create_subscription(Image, self.image_topic, self._image_cb, SENSOR_QOS)

        self.pub_raw = self.create_publisher(Hazard, '/hazards/raw', 20)
        self.pub_markers = self.create_publisher(MarkerArray, '/hazards/raw/markers', 10)

        self.get_logger().info(
            f"HazardDetector ready (backend={backend}, image={self.image_topic})"
        )

    # ------------------------------------------------------------------ config
    def _load_config(self) -> dict:
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
        if self.intrinsics is None:
            k = np.array(msg.k).reshape(3, 3)
            self.intrinsics = k
            self.get_logger().info(
                f'Camera intrinsics received (fx={k[0, 0]:.1f}, fy={k[1, 1]:.1f}, '
                f'cx={k[0, 2]:.1f}, cy={k[1, 2]:.1f})'
            )

    def _image_cb(self, msg: Image):
        if self.intrinsics is None:
            return  # wait for camera_info
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge failure: {exc}', throttle_duration_sec=5.0)
            return

        detections = self.detector.detect(frame)
        if not detections:
            return

        stamp = msg.header.stamp
        camera_frame = msg.header.frame_id or 'camera_link'

        markers = MarkerArray()
        for idx, det in enumerate(detections):
            point_map = self._pixel_to_map(det, camera_frame, stamp)
            if point_map is None:
                continue

            hz = Hazard()
            hz.header.stamp = stamp
            hz.header.frame_id = self.map_frame
            hz.id = -1
            hz.color = det.label
            hz.category = self.color_to_category.get(det.label, det.label)
            hz.position = point_map
            hz.confidence = det.confidence
            hz.observation_count = 1
            self.pub_raw.publish(hz)

            markers.markers.append(self._make_marker(idx, hz, stamp))

        if markers.markers:
            self.pub_markers.publish(markers)

    # ------------------------------------------------------------------ geom
    def _pixel_to_map(
        self, det: Detection, camera_frame: str, stamp
    ) -> Optional[Point]:
        """Back-project pixel (cx, cy) onto plane z=cube_half in map frame."""
        if self.intrinsics is None:
            return None
        k = self.intrinsics
        fx, fy = k[0, 0], k[1, 1]
        cx_i, cy_i = k[0, 2], k[1, 2]

        # Ray in the camera *optical* frame (x-right, y-down, z-forward).
        x_n = (det.cx - cx_i) / fx
        y_n = (det.cy - cy_i) / fy
        ray_cam = np.array([x_n, y_n, 1.0])
        ray_cam /= np.linalg.norm(ray_cam)

        # Lookup camera->map transform at the image stamp.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, camera_frame, stamp,
                rclpy.duration.Duration(seconds=0.3),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'TF {self.map_frame}<-{camera_frame} unavailable: {exc}',
                throttle_duration_sec=5.0,
            )
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        R = _quat_to_mat(q.x, q.y, q.z, q.w)
        origin = np.array([t.x, t.y, t.z])
        ray_map = R @ ray_cam

        # Intersect with plane z = plane_z.
        plane_z = self.cube_size / 2.0
        if abs(ray_map[2]) < 1e-6:
            return None
        s = (plane_z - origin[2]) / ray_map[2]
        if s <= 0 or s > self.max_range:
            return None

        p = origin + s * ray_map
        return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))

    def _make_marker(self, idx: int, hz: Hazard, stamp) -> Marker:
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = stamp
        m.ns = 'hazards_raw'
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = hz.position
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color = _COLOR_TO_RGBA.get(hz.color, ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8))
        m.lifetime.sec = 1
        return m


def _quat_to_mat(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Minimal quaternion -> 3x3 rotation matrix (no external deps)."""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy)],
        [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx)],
        [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)],
    ])


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
