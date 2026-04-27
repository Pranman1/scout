#!/usr/bin/env python3
"""Hazard tracker / deduplicator (SKELETON).

Merges noisy per-frame detections coming in on ``/hazards/raw`` into a
stable list of confirmed hazards, published with TRANSIENT_LOCAL
durability on ``/hazards/confirmed`` so the mission_manager can
subscribe late and still receive the full list.

You decide the clustering strategy. A reasonable first pass:
    * same color + within ``merge_radius_m`` -> same track
      (running average of position weighted by observation count),
    * track becomes "confirmed" once it has ``min_observations``
      samples on it,
    * when ``/scout/mapping_complete`` latches true, dump the confirmed
      set to ``hazards_file`` (JSON) for replay via ``task:=mission``.
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
    """One candidate hazard accumulating observations.

    This is just a data container -- feel free to add / rename fields
    to match whatever tracking rule you pick.
    """
    id: int
    color: str
    category: str
    x: float
    y: float
    z: float
    confidence: float = 0.0
    observations: int = 0
    confirmed: bool = False
    history: List[float] = field(default_factory=list)


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
        """Associate ``hz`` with an existing track or spawn a new one.

        TODO(you):
            1. Find the nearest existing track that matches ``hz.color``
               and is within ``self.merge_radius`` (see ``_find_track``).
            2. If none, append a new ``_Track`` using ``self._next_id``
               and bump the counter.
            3. Update the track: running-mean position weighted by
               ``observations``, bump observation count, append
               ``hz.confidence`` to ``history`` (cap length ~20),
               recompute mean confidence.
            4. If the track isn't yet confirmed but now has
               ``observations >= self.min_obs``, mark it confirmed,
               log it, and call ``self._publish_confirmed(track)``.
        """
        raise NotImplementedError("TODO(you): implement detection -> track association.")

    def _find_track(self, color: str, x: float, y: float) -> Optional[_Track]:
        """Return the nearest same-color track within merge_radius, else None.

        TODO(you): linear scan over ``self._tracks``; keep the closest
        whose distance <= ``self.merge_radius``. For 3-5 tracks this is
        trivially fast.
        """
        raise NotImplementedError("TODO(you): implement nearest-track lookup.")

    # ------------------------------------------------------------------ output
    def _track_to_hazard(self, t: _Track) -> Hazard:
        """Pack a ``_Track`` into a ``scout_msgs/Hazard``. Pure plumbing."""
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
        """Publish one confirmed hazard on the latched topic.

        TODO(you): this is a one-liner -- build the Hazard via
        ``self._track_to_hazard`` and publish on ``self.pub_conf``.
        """
        raise NotImplementedError("TODO(you): publish a single confirmed hazard.")

    def _republish(self):
        """Timer: rebroadcast every confirmed track + refresh the RViz markers.

        TODO(you):
            - For each confirmed track: publish via ``self.pub_conf``
              (keeps the TRANSIENT_LOCAL topic warm for late subscribers)
              and append a marker from ``_make_marker`` to a
              ``MarkerArray``.
            - Publish the MarkerArray only if non-empty.
        """
        return

    def _make_marker(self, t: _Track, stamp) -> Marker:
        """Build an RViz CYLINDER marker for a confirmed track.

        TODO(you): ns='hazards_confirmed', id=t.id, type=CYLINDER,
        scale ~0.3 x 0.3 x 0.5, positioned at (t.x, t.y, t.z + 0.25),
        colour from ``_COLOR_TO_RGBA``.
        """
        raise NotImplementedError("TODO(you): implement confirmed-hazard marker.")

    # ------------------------------------------------------------------ snapshot
    def _on_mapping_done(self, msg: Bool):
        """Write the JSON snapshot exactly once when mapping completes.

        TODO(you): if ``msg.data`` is True and ``self._snapshot_written``
        is still False, call ``self._write_snapshot()``.
        """
        raise NotImplementedError("TODO(you): trigger the snapshot write.")

    def _write_snapshot(self):
        """Serialize the confirmed track list to ``self.hazards_file`` as JSON.

        TODO(you):
            - Bail out if ``self.hazards_file`` is empty.
            - Build a dict like ``{'hazards': [<one entry per confirmed
              track>]}``, including whatever fields you want to replay
              later (id/color/category/x/y/z/observations/confidence).
            - ``os.makedirs(os.path.dirname(path), exist_ok=True)`` then
              ``json.dump(payload, fh, indent=2)``.
            - Set ``self._snapshot_written = True`` and log.
        """
        raise NotImplementedError("TODO(you): implement the JSON snapshot writer.")


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
