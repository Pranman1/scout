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
        # Hard cap on simultaneous tracks per color. Default 1 because
        # the scout arena has exactly one cone of each colour. Once
        # this cap is reached, any further same-colour observations
        # always update the existing track instead of spawning a new
        # one (treating "outlier" detections as noise rather than a
        # second cone). Bump to >1 only if your environment can
        # legitimately have multiple objects of the same colour.
        self.declare_parameter('max_tracks_per_color', 1)

        self.merge_radius = float(self.get_parameter('merge_radius_m').value)
        self.min_obs = int(self.get_parameter('min_observations').value)
        self.hazards_file = self.get_parameter('hazards_file').value
        self.max_per_color = int(self.get_parameter('max_tracks_per_color').value)
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

        Policy (in order):
          1. If there is at least one same-colour track within
             ``self.merge_radius``, update the closest one.
          2. Else if the same-colour track count is already at
             ``self.max_per_color``, treat ``hz`` as part of the
             closest same-colour track anyway -- domain knowledge says
             there's only one cone of each colour, so an observation
             far from the running mean is more likely to be a noisy
             fusion than a genuine second object.
          3. Else create a new track.
        After updating, opportunistically merge any *other* same-colour
        tracks into the one we just touched (cleans up legacy phantoms
        from earlier in the run that ended up beyond merge_radius from
        the now-converged track).
        """
        same_color = [t for t in self._tracks if t.color == hz.color]
        x, y = hz.position.x, hz.position.y

        if same_color:
            closest = min(same_color, key=lambda t: math.hypot(t.x - x, t.y - y))
            dist = math.hypot(closest.x - x, closest.y - y)
            if dist <= self.merge_radius or len(same_color) >= self.max_per_color:
                self._update_track(closest, hz)
                # Opportunistic dedup: anything else of the same colour
                # is a phantom we should fold in.
                for t in list(same_color):
                    if t is not closest:
                        self._merge_tracks(into=closest, victim=t)
                return

        # New track.
        track = _Track(
            id=self._next_id,
            color=hz.color,
            category=hz.category,
            x=x, y=y, z=hz.position.z,
            confidence=hz.confidence,
            observations=1,
            history=[hz.confidence],
        )
        self._tracks.append(track)
        self.get_logger().info(
            f'New track {track.id} ({track.color}) at ({x:.2f}, {y:.2f})'
        )
        self._next_id += 1
        if track.observations >= self.min_obs:
            track.confirmed = True
            self._publish_confirmed(track)

    def _update_track(self, track: '_Track', hz: Hazard) -> None:
        """Fold a fresh observation into a track using a correct running mean.

        The previous code incremented ``observations`` before the mean
        update, which mathematically over-weighted the prior position
        and made tracks slowly *drift* away from the true cone. After
        N samples the position became (N*obs_1 + obs_N+1)/(N+1) instead
        of the actual mean. Once a track drifted past merge_radius, a
        fresh observation at the true position would spawn a phantom
        track -- exactly the symptom we saw in rviz.
        """
        n = track.observations
        track.x = (track.x * n + hz.position.x) / (n + 1)
        track.y = (track.y * n + hz.position.y) / (n + 1)
        track.z = (track.z * n + hz.position.z) / (n + 1)
        track.confidence = (track.confidence * n + hz.confidence) / (n + 1)
        track.observations = n + 1
        track.history.append(hz.confidence)
        if len(track.history) > 20:
            track.history.pop(0)
        if not track.confirmed and track.observations >= self.min_obs:
            track.confirmed = True
            self.get_logger().info(
                f'Track {track.id} ({track.color}) confirmed at '
                f'({track.x:.2f}, {track.y:.2f}) after {track.observations} obs'
            )
            self._publish_confirmed(track)

    def _merge_tracks(self, into: '_Track', victim: '_Track') -> None:
        """Absorb ``victim`` into ``into`` (weighted by observation counts)."""
        a, b = into.observations, victim.observations
        total = a + b
        if total <= 0:
            return
        into.x = (into.x * a + victim.x * b) / total
        into.y = (into.y * a + victim.y * b) / total
        into.z = (into.z * a + victim.z * b) / total
        into.confidence = (into.confidence * a + victim.confidence * b) / total
        into.observations = total
        into.confirmed = into.confirmed or victim.confirmed
        if victim in self._tracks:
            self._tracks.remove(victim)
        self.get_logger().info(
            f'Merged phantom track {victim.id} -> {into.id} ({into.color})'
        )



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
        self.pub_conf.publish(self._track_to_hazard(t))

    def _republish(self):
        """Timer: rebroadcast every confirmed track + refresh the RViz markers.

        TODO(you):
            - For each confirmed track: publish via ``self.pub_conf``
              (keeps the TRANSIENT_LOCAL topic warm for late subscribers)
              and append a marker from ``_make_marker`` to a
              ``MarkerArray``.
            - Publish the MarkerArray only if non-empty.
        """
        markers = MarkerArray()
        for track in self._tracks:
            if track.confirmed:
                markers.markers.append(self._make_marker(track, self.get_clock().now().to_msg()))
                self._publish_confirmed(track)
        if markers.markers:
            self.pub_markers.publish(markers)
        return

    def _make_marker(self, t: _Track, stamp) -> Marker:
        """Build an RViz CYLINDER marker for a confirmed track.

        TODO(you): ns='hazards_confirmed', id=t.id, type=CYLINDER,
        scale ~0.3 x 0.3 x 0.5, positioned at (t.x, t.y, t.z + 0.25),
        colour from ``_COLOR_TO_RGBA``.
        """
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = 'hazards_confirmed'
        marker.id = t.id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = t.x
        marker.pose.position.y = t.y
        marker.pose.position.z = t.z + 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.5
        marker.color = _COLOR_TO_RGBA.get(t.color, ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0))
        return marker

    # ------------------------------------------------------------------ snapshot
    def _on_mapping_done(self, msg: Bool):
        """Write the JSON snapshot exactly once when mapping completes.

        TODO(you): if ``msg.data`` is True and ``self._snapshot_written``
        is still False, call ``self._write_snapshot()``.
        """
        if msg.data and not self._snapshot_written:
            self._write_snapshot()
            self._snapshot_written = True
            self.get_logger().info('Snapshot written')
        return

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
        if not self.hazards_file:
            self.get_logger().error('No hazards_file; refusing to write snapshot.')
            return
        payload = {'hazards': []}
        for track in self._tracks:
            if track.confirmed:
                payload['hazards'].append(track.__dict__)
        os.makedirs(os.path.dirname(self.hazards_file), exist_ok=True)
        with open(self.hazards_file, 'w') as f:
            json.dump(payload, f, indent=2)
        self._snapshot_written = True
        self.get_logger().info(f'Snapshot written to {self.hazards_file}')
        return


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
