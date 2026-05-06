#!/usr/bin/env python3
"""Hazard tracker / deduplicator (SKELETON).
"""
import json
import os
from dataclasses import dataclass, field
from typing import List, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy,QoSProfile,ReliabilityPolicy,)
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
    observations: int = 0
    confirmed: bool = False


class HazardTracker(Node):
    def __init__(self):
        super().__init__('hazard_tracker')

        self.declare_parameter('merge_radius_m', 0.5)
        self.declare_parameter('min_observations', 3)
        self.declare_parameter('hazards_file', '')
        self.declare_parameter('republish_period_s', 2.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_namespace', '')

        self.merge_radius = float(self.get_parameter('merge_radius_m').value)
        self.min_obs = int(self.get_parameter('min_observations').value)
        self.hazards_file = self.get_parameter('hazards_file').value
        period = float(self.get_parameter('republish_period_s').value)
        self.robot_ns = self.get_parameter('robot_namespace').value
        
        self.map_frame = self.get_parameter('map_frame').value

        self._tracks: List[_Track] = []
        self._next_id = 0
        self._snapshot_written = False

        self.create_subscription(Hazard, '/hazards/raw', self._on_raw, 20)
        self.create_subscription(Bool, '/scout/mapping_complete', self._on_mapping_done, LATCHED_QOS)

        self.pub_conf = self.create_publisher(Hazard, '/hazards/confirmed', LATCHED_QOS)
        self.pub_markers = self.create_publisher(MarkerArray, '/hazards/confirmed/markers', LATCHED_QOS)

        self.create_timer(period, self._republish)
    
    def _add_namespace(self, frame_id: str) -> str:
        if not self.robot_ns or frame_id.startswith('/'):
            return frame_id
        return f"{self.robot_ns}/{frame_id}"

 
    def _on_raw(self, hz: Hazard):
        """Associate ``hz`` with an existing track or spawn a new one."""

        same_color = []
        
        for t in self._tracks:
            if t.color == hz.color:
                same_color.append(t)
        
        x, y, z = hz.position.x, hz.position.y, hz.position.z

        if same_color:
            f = lambda z: ((z.x - x)**2 + (z.y - y)**2)**0.5
            closest = min(same_color, key=f)
            dist = f(closest)
            
            if dist <= self.merge_radius:
                self._update_track(closest, hz)
                return
            return 

        track = _Track(
            id=self._next_id,
            color=hz.color,
            category=hz.category,
            x=x, 
            y=y, 
            z=z,
            observations=1,
        )
        self._next_id += 1
        self._tracks.append(track)

    def _update_track(self, track: '_Track', hz: Hazard) -> None:
        """Fold a fresh observation into a track using a correct running mean."""

        n = track.observations + 1
        track.x = (track.x * track.observations + hz.position.x) / n
        track.y = (track.y * track.observations + hz.position.y) / n
        track.z = (track.z * track.observations + hz.position.z) / n 
        track.observations = n

        if not track.confirmed and track.observations >= self.min_obs:
            track.confirmed = True

    def _track_to_hazard(self, t: _Track) -> Hazard:
        """Pack a ``_Track`` into a ``scout_msgs/Hazard``. Pure plumbing."""

        h = Hazard()
        h.header.stamp = self.get_clock().now().to_msg()
        h.header.frame_id = self.map_frame
        h.id = t.id
        h.color = t.color
        h.category = t.category
        h.position.x = t.x
        h.position.y = t.y
        h.position.z = t.z
        h.observation_count = int(t.observations)
        return h

    def _publish_confirmed(self, t: _Track):
        """TODO: Publish one confirmed hazard on the latched topic.
        """
        self.pub_conf.publish(self._track_to_hazard(t))

    def _republish(self):
        """Timer: rebroadcast every confirmed track + refresh the RViz markers.
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
        """
        marker = Marker()
        marker.header.frame_id = self.map_frame
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

    # dumpsoning the confirmed positons to a given file 
    
    def _on_mapping_done(self, msg: Bool):
        """TODO Write the JSON snapshot exactly once when mapping completes.
        """
        if msg.data and not self._snapshot_written:
            self._write_snapshot()
            self._snapshot_written = True
        return

    def _write_snapshot(self):
        """TODO Serialize the confirmed track list to ``self.hazards_file`` as JSON.
            - Bail out if ``self.hazards_file`` is empty.
            - Build a dict
        """
       
       
        if not self.hazards_file:
            return
        dumpson = {'hazards': []}
        for track in self._tracks:
            if track.confirmed:
                dumpson['hazards'].append(track.__dict__)
        
        # the following line is from a sperate project i saw online
        os.makedirs(os.path.dirname(self.hazards_file), exist_ok=True)
        with open(self.hazards_file, 'w') as f:
            json.dump(dumpson, f, indent=2)
    
        self._snapshot_written = True
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
