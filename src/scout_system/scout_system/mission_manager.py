#!/usr/bin/env python3
"""Mission manager for the scout (SKELETON).

High-level state machine that runs **after** Nav2 is up and a map is
loaded. Expected behaviour in your completed version:

1. Wait for Nav2 to activate and for a valid map -> base_link transform
   (i.e. AMCL has localized the robot).
2. Source the hazard list:
     * primary: latched ``/hazards/confirmed`` from hazard_tracker,
     * fallback: ``waypoints_file`` parameter (absolute map-frame X/Y,
       '#' comments, one per line).
3. For each hazard, drive to a stand-off pose in front of it (don't
   collide with the cube), publish ``/mission_status`` transitions,
   call the ``/request_package`` service (the UR7 seam) with the
   hazard's category, then move on.
4. Publish a final ``PHASE_COMPLETE`` (or ``PHASE_ABORTED`` if nothing
   to visit).

``MissionStatus`` phase constants are defined in
``scout_msgs/msg/MissionStatus.msg`` -- prefer
``MissionStatus.PHASE_EN_ROUTE`` etc. over raw ints.
"""
from __future__ import annotations

import math
import os
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from scout_msgs.msg import Hazard, MissionStatus
from scout_msgs.srv import RequestPackage


LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('standoff_distance', 0.6)
        self.declare_parameter('hazards_wait_timeout', 3.0)
        self.declare_parameter('request_package_timeout', 15.0)

        self.waypoints_file: str = self.get_parameter('waypoints_file').value
        self.standoff: float = float(self.get_parameter('standoff_distance').value)
        self.hazards_wait_timeout: float = float(self.get_parameter('hazards_wait_timeout').value)
        self.req_timeout: float = float(self.get_parameter('request_package_timeout').value)

        self.navigator = BasicNavigator()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.hazards: List[Hazard] = []
        self.create_subscription(
            Hazard, '/hazards/confirmed', self._hazard_cb, LATCHED_QOS
        )

        self.status_pub = self.create_publisher(MissionStatus, '/mission_status', 10)
        self.request_client = self.create_client(RequestPackage, '/request_package')

    # ------------------------------------------------------------------ helpers
    def _hazard_cb(self, msg: Hazard):
        """Append ``msg`` to ``self.hazards`` if its id isn't already present.

        TODO(you): de-dup by id; the latched topic re-sends periodically
        so this callback will be hit multiple times per hazard.
        """
        raise NotImplementedError("TODO(you): dedupe-append incoming hazards.")

    def _publish_status(
        self,
        phase: int,
        message: str = '',
        current_id: int = -1,
        current_cat: str = '',
        visited: int = 0,
    ):
        """Fill a ``MissionStatus`` msg and publish it. Pure plumbing."""
        s = MissionStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.phase = phase
        s.current_hazard_id = current_id
        s.current_hazard_category = current_cat
        s.hazards_total = len(self.hazards)
        s.hazards_visited = visited
        s.message = message
        self.status_pub.publish(s)

    def _is_localized(self) -> bool:
        """Return True if map -> base_link is currently available.

        TODO(you): use ``self.tf_buffer.can_transform('map', 'base_link',
        Time(), timeout=Duration(seconds=1.0))`` inside a try/except.
        """
        raise NotImplementedError("TODO(you): implement the localization check.")

    def _current_xy(self) -> Tuple[float, float]:
        """Return (x, y) of base_link in map; (0.0, 0.0) on TF failure.

        TODO(you): ``self.tf_buffer.lookup_transform('map', 'base_link',
        Time())`` and return ``(tf.transform.translation.x, .y)``.
        """
        raise NotImplementedError("TODO(you): implement current-pose lookup.")

    def _standoff_pose(self, target: Point, robot_x: float, robot_y: float) -> PoseStamped:
        """Build a PoseStamped sitting ``self.standoff`` m in front of ``target``,
        facing it from the robot's current position.

        TODO(you):
            1. Compute the unit vector from (robot_x, robot_y) -> target
               (guard against zero length).
            2. Goal XY = target - standoff * unit_vector (so we stop
               *in front of* the cube, not on it).
            3. Goal yaw = atan2(dy, dx) so the robot faces the target.
            4. Pack into a ``PoseStamped`` with frame_id='map' and
               current stamp. Remember yaw -> quaternion:
               qz = sin(yaw/2), qw = cos(yaw/2), qx=qy=0.
        """
        raise NotImplementedError("TODO(you): implement standoff pose geometry.")

    def _collect_waypoint_hazards(self) -> List[Hazard]:
        """Parse ``waypoints_file`` into fake ``Hazard`` messages (fallback).

        TODO(you):
            - Return [] if no file set or file doesn't exist.
            - Each line: 'x y  # optional comment'. Skip blanks/'#'.
            - Build one Hazard per line with color='unknown',
              category='waypoint', id=line_index, confidence=1.0.
        """
        raise NotImplementedError("TODO(you): implement waypoints fallback parser.")

    def _request_package(self, hazard: Hazard) -> bool:
        """Call the ``/request_package`` service and log the response.

        TODO(you):
            1. ``wait_for_service(timeout_sec=2.0)``. Return False with
               a warn if the service never shows up.
            2. Build a ``RequestPackage.Request`` filled from ``hazard``.
            3. ``future = self.request_client.call_async(req)`` and spin
               manually until ``future.done()`` or ``self.req_timeout``
               elapses (``spin_once(self, timeout_sec=0.1)`` in a loop).
            4. Return ``resp.accepted`` on success, False on timeout/
               failure.
        """
        raise NotImplementedError("TODO(you): implement the package-request RPC.")

    # ------------------------------------------------------------------ main flow
    def run(self):
        """Drive the mission to completion.

        TODO(you) -- flow outline:
            1. Publish PHASE_IDLE status.
            2. ``self.navigator.waitUntilNav2Active()``.
            3. Spin until ``self._is_localized()``. Log a throttled hint
               about the RViz 2D Pose Estimate tool every few seconds.
            4. Wait up to ``self.hazards_wait_timeout`` for the latched
               ``/hazards/confirmed`` topic to deliver.
            5. If still empty, fall back to ``_collect_waypoint_hazards``.
               If still empty, publish PHASE_ABORTED and return.
            6. Publish PHASE_PLANNING, then loop over hazards:
                 a. PHASE_EN_ROUTE + ``self.navigator.goToPose(goal)``.
                    Spin until ``isTaskComplete()``. Check
                    ``getResult() == TaskResult.SUCCEEDED``; warn-and-
                    continue on failure.
                 b. On success: PHASE_AT_HAZARD -> ``_request_package``
                    -> bump visited counter.
            7. Final PHASE_COMPLETE with the visit count.
        """
        raise NotImplementedError("TODO(you): implement the mission state machine.")


def main():
    rclpy.init()
    node = MissionManager()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
