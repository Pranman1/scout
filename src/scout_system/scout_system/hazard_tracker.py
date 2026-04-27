#!/usr/bin/env python3
"""Hazard tracker / deduplicator.

Merges noisy per-frame detections from ``/hazards/raw`` into a stable
list of confirmed hazards published with TRANSIENT_LOCAL durability on
``/hazards/confirmed`` (so the mission_manager can subscribe late and
still get them).

Clustering strategy (simple and good enough for 3 well-separated cubes):
    * Same color + within ``merge_radius_m`` -> same track, running
      average of position weighted by observation_count.
    * Track becomes "confirmed" once ``min_observations`` samples land
      on it.

When ``/scout/mapping_complete`` flips true, the current confirmed list
is serialized to ``hazards_file`` so you can replay it later via
``task:=mission``.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from scout_msgs.msg import Hazard


LATCHED_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


_COLOR_TO_RGBA = {
    'red':    ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0),
    'yellow': ColorRGBA(r=1.0, g=0.95, b=0.1, a=1.0),
    'blue':   ColorRGBA(r=0.1, g=0.3, b=1.0, a=1.0),
    'green':  ColorRGBA(r=0.1, g=1.0, b=0.2, a=1.0),
}


@dataclass
class _Track:
    id: int
    color: str
    category: str
    x: float
    y: float
    z: float
    confidence: float = 0.0
    observations: int = 0
    confirmed: bool = False
    history: List[float] = field(default_factory=list)   # running confidences


class HazardTracker(Node):
    def __init__(self):
        super().__init__('hazard_tracker')

        self.declare_parameter('merge_radius_m', 0.5)
        self.declare_parameter('min_observations', 3)
        self.declare_parameter('hazards_file', '')
        self.declare_parameter('republish_period_s', 2.0)

        self.merge_radius = float(self.get_parameter('merge_radius_m').value)
        self.min_obs = int(self.get_parameter('min_observations').value)
        self.hazards_file = self.get_parameter('hazards_file').value
        period = float(self.get_parameter('republish_period_s').value)

        self._tracks: List[_Track] = []
        self._next_id = 0
        self._snapshot_written = False

        self.create_subscription(Hazard, '/hazards/raw', self._on_raw, 20)
        self.create_subscription(
            Bool, '/scout/mapping_complete', self._on_mapping_done, LATCHED_QOS
        )

        self.pub_conf = self.create_publisher(Hazard, '/hazards/confirmed', LATCHED_QOS)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/hazards/confirmed/markers', LATCHED_QOS
        )

        self.create_timer(period, self._republish)

        self.get_logger().info(
            f'HazardTracker ready (merge_radius={self.merge_radius:.2f}m, '
            f'min_observations={self.min_obs})'
        )

    # ------------------------------------------------------------------ core
    def _on_raw(self, hz: Hazard):
        track = self._find_track(hz.color, hz.position.x, hz.position.y)
        if track is None:
            track = _Track(
                id=self._next_id,
                color=hz.color,
                category=hz.category,
                x=hz.position.x,
                y=hz.position.y,
                z=hz.position.z,
            )
            self._next_id += 1
            self._tracks.append(track)

        # Running mean weighted by observation count.
        n = track.observations
        track.x = (track.x * n + hz.position.x) / (n + 1)
        track.y = (track.y * n + hz.position.y) / (n + 1)
        track.z = (track.z * n + hz.position.z) / (n + 1)
        track.observations = n + 1

        track.history.append(hz.confidence)
        if len(track.history) > 20:
            track.history = track.history[-20:]
        track.confidence = sum(track.history) / len(track.history)

        if not track.confirmed and track.observations >= self.min_obs:
            track.confirmed = True
            self.get_logger().info(
                f'[confirmed] id={track.id} {track.color}/{track.category} '
                f'at ({track.x:.2f}, {track.y:.2f}) n={track.observations}'
            )
            self._publish_confirmed(track)

    def _find_track(self, color: str, x: float, y: float) -> Optional[_Track]:
        best = None
        best_d = self.merge_radius
        for t in self._tracks:
            if t.color != color:
                continue
            d = math.hypot(t.x - x, t.y - y)
            if d <= best_d:
                best = t
                best_d = d
        return best

    # ------------------------------------------------------------------ output
    def _track_to_hazard(self, t: _Track) -> Hazard:
        h = Hazard()
        h.header.stamp = self.get_clock().now().to_msg()
        h.header.frame_id = 'map'
        h.id = t.id
        h.color = t.color
        h.category = t.category
        h.position.x = t.x
        h.position.y = t.y
        h.position.z = t.z
        h.confidence = float(t.confidence)
        h.observation_count = int(t.observations)
        return h

    def _publish_confirmed(self, t: _Track):
        self.pub_conf.publish(self._track_to_hazard(t))

    def _republish(self):
        # Refresh the latched topic + rebuild the RViz markers.
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        for t in self._tracks:
            if not t.confirmed:
                continue
            self.pub_conf.publish(self._track_to_hazard(t))
            markers.markers.append(self._make_marker(t, now))
        if markers.markers:
            self.pub_markers.publish(markers)

    def _make_marker(self, t: _Track, stamp) -> Marker:
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.ns = 'hazards_confirmed'
        m.id = t.id
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = t.x
        m.pose.position.y = t.y
        m.pose.position.z = t.z + 0.25
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.3
        m.scale.z = 0.5
        m.color = _COLOR_TO_RGBA.get(t.color, ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
        return m

    # ------------------------------------------------------------------ snapshot
    def _on_mapping_done(self, msg: Bool):
        if not msg.data or self._snapshot_written:
            return
        self._write_snapshot()

    def _write_snapshot(self):
        if not self.hazards_file:
            return
        confirmed = [t for t in self._tracks if t.confirmed]
        payload = {
            'hazards': [
                {
                    'id': t.id,
                    'color': t.color,
                    'category': t.category,
                    'x': t.x,
                    'y': t.y,
                    'z': t.z,
                    'observations': t.observations,
                    'confidence': t.confidence,
                }
                for t in confirmed
            ],
        }
        os.makedirs(os.path.dirname(self.hazards_file), exist_ok=True)
        with open(self.hazards_file, 'w') as fh:
            json.dump(payload, fh, indent=2)
        self._snapshot_written = True
        self.get_logger().info(
            f'Snapshot written ({len(confirmed)} hazards) -> {self.hazards_file}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = HazardTracker()
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
