#!/usr/bin/env python3
"""Per-frame hazard detector (SKELETON).

This node owns the **ROS-side plumbing only**; you write the perception +
geometry. The responsibilities expected of your completed version are:

    * subscribe to a camera image + CameraInfo stream,
    * hand each frame to ``self.detector`` (a ``scout_system.detectors``
      backend) and iterate over the returned ``Detection`` objects,
    * back-project each detection's center pixel onto the world ground
      plane (z = cube_half_height) in the ``map`` frame using the camera
      intrinsics and the TF tree,
    * publish one ``scout_msgs/Hazard`` per detection on ``/hazards/raw``,
    * publish a MarkerArray to ``/hazards/raw/markers`` for RViz.

The initial geometry is deliberately simple (ray-plane intersection with
a flat-floor assumption). Swapping in laser fusion later is a clean
upgrade path -- the node contract doesn't have to change.
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
        """Cache the 3x3 intrinsic matrix on the first message we see."""
        if self.intrinsics is None:
            self.intrinsics = np.array(msg.k).reshape(3, 3)
            k = self.intrinsics
            self.get_logger().info(
                f'Camera intrinsics received (fx={k[0, 0]:.1f}, fy={k[1, 1]:.1f}, '
                f'cx={k[0, 2]:.1f}, cy={k[1, 2]:.1f})'
            )

    def _image_cb(self, msg: Image):
        """Process one frame.

        TODO(you):
            1. Return early if ``self.intrinsics`` is None (CameraInfo not
               yet seen) -- no point running detection without a projection.
            2. Convert ``msg`` to a BGR numpy image via ``self.bridge``.
               Wrap in try/except so a single bad frame doesn't kill the
               node; use ``self.get_logger().warn(..., throttle_duration_sec=5.0)``.
            3. Run ``detections = self.detector.detect(frame)``. If empty,
               return.
            4. For each detection, call ``self._pixel_to_map(det, ...)``.
               If it returns None (bad TF, behind camera, too far), skip.
            5. Build a ``scout_msgs/Hazard``, stamp it in ``self.map_frame``,
               set ``id = -1`` (the tracker assigns real IDs), fill
               color/category/position/confidence, and publish on
               ``self.pub_raw``.
            6. Append a marker (see ``_make_marker``) to a local
               ``MarkerArray`` and publish once at the end if non-empty.
        """
        return

    # ------------------------------------------------------------------ geom
    def _pixel_to_map(
        self, det: Detection, camera_frame: str, stamp
    ) -> Optional[Point]:
        """Back-project (det.cx, det.cy) onto the plane z = cube_size / 2 in ``self.map_frame``.

        TODO(you):
            1. Build a unit ray in the camera optical frame from the
               pinhole model: x_n = (u - cx) / fx, y_n = (v - cy) / fy,
               z_n = 1.0, then normalize. Remember the optical frame
               convention is x-right, y-down, z-forward.
            2. Look up the transform from ``camera_frame`` to
               ``self.map_frame`` at ``stamp`` using
               ``self.tf_buffer.lookup_transform(...)``. Give it a
               timeout (~0.3 s). On failure, warn (throttled) and
               return None.
            3. Rotate the ray by the transform's orientation and add
               the translation as the ray origin. You can either
               hand-roll a quaternion->rotmat (see numpy), or use
               ``tf_transformations.quaternion_matrix`` from the
               ``tf_transformations`` package.
            4. Intersect that ray with the plane z = cube_size / 2:
               if |ray.z| < eps return None; else
               s = (plane_z - origin.z) / ray.z. Reject s <= 0
               (behind camera) and s > self.max_range (sanity cap).
            5. Return ``Point(x=..., y=..., z=plane_z)``.

        Returns None for any failure so the caller can just skip.
        """
        raise NotImplementedError("TODO(you): implement pixel -> map-frame projection.")

    def _make_marker(self, idx: int, hz: Hazard, stamp) -> Marker:
        """Build an RViz SPHERE marker at ``hz.position`` coloured by ``hz.color``.

        TODO(you):
            - ns='hazards_raw', id=idx, type=SPHERE, action=ADD.
            - scale ~0.2 m, short lifetime (~1 s) so stale detections
              expire on their own.
            - Use ``_COLOR_TO_RGBA`` (module-level above) for the fill,
              falling back to white if the label is unknown.
        """
        raise NotImplementedError("TODO(you): implement marker construction.")


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
