#!/usr/bin/env python3
"""Mission manager for the scout.

Responsibilities (all after Nav2 is up and a map is loaded):

1. Wait for Nav2 to activate and for the map->base_link transform to
   exist (i.e. AMCL has localized us).
2. Collect the hazard list:
     * primary source: latched ``/hazards/confirmed`` topic from
       hazard_tracker,
     * fallback source: ``waypoints_file`` parameter (parsed like the
       grazen waypoints.txt -- absolute map-frame X/Y, '#' comments).
3. For each hazard: drive to a stand-off pose in front of it, publish
   ``/mission_status`` transitions, call the ``/request_package`` service
   (UR7 seam) with the hazard's category, then move on.
4. Publish a final ``PHASE_COMPLETE`` status.
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
        self.declare_parameter('standoff_distance', 0.6)   # meters in front of hazard
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
        for existing in self.hazards:
            if existing.id == msg.id:
                return
        self.hazards.append(msg)

    def _publish_status(
        self,
        phase: int,
        message: str = '',
        current_id: int = -1,
        current_cat: str = '',
        visited: int = 0,
    ):
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
        try:
            return self.tf_buffer.can_transform(
                'map', 'base_link', Time(), timeout=Duration(seconds=1.0)
            )
        except Exception:  # noqa: BLE001
            return False

    def _current_xy(self) -> Tuple[float, float]:
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:  # noqa: BLE001
            return 0.0, 0.0

    def _standoff_pose(self, target: Point, robot_x: float, robot_y: float) -> PoseStamped:
        """Build a PoseStamped in front of ``target``, facing it, at
        ``self.standoff`` meters along the robot->target ray."""
        dx = target.x - robot_x
        dy = target.y - robot_y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            ux, uy = 1.0, 0.0
        else:
            ux, uy = dx / dist, dy / dist

        goal_x = target.x - ux * self.standoff
        goal_y = target.y - uy * self.standoff
        yaw = math.atan2(uy, ux)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _collect_waypoint_hazards(self) -> List[Hazard]:
        """Parse the fallback waypoints.txt into dummy Hazard messages."""
        if not self.waypoints_file or not os.path.exists(self.waypoints_file):
            return []
        fake: List[Hazard] = []
        with open(self.waypoints_file) as fh:
            for idx, raw in enumerate(fh):
                line = raw.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                try:
                    x, y = float(parts[0]), float(parts[1])
                except (ValueError, IndexError):
                    continue
                h = Hazard()
                h.id = idx
                h.color = 'unknown'
                h.category = 'waypoint'
                h.position.x = x
                h.position.y = y
                h.confidence = 1.0
                h.observation_count = 1
                fake.append(h)
        return fake

    def _request_package(self, hazard: Hazard) -> bool:
        if not self.request_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/request_package service unavailable; skipping.')
            return False
        req = RequestPackage.Request()
        req.hazard_id = hazard.id
        req.category = hazard.category
        req.hazard_position = hazard.position
        future = self.request_client.call_async(req)
        end_time = self.get_clock().now() + Duration(seconds=self.req_timeout)
        while rclpy.ok() and not future.done():
            if self.get_clock().now() > end_time:
                self.get_logger().warn('/request_package timed out.')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        resp = future.result()
        if resp is None:
            return False
        self.get_logger().info(
            f'[arm] accepted={resp.accepted} package={resp.package_label} '
            f'prep_s={resp.estimated_prep_seconds:.1f} msg="{resp.message}"'
        )
        return resp.accepted

    # ------------------------------------------------------------------ main flow
    def run(self):
        self._publish_status(MissionStatus.PHASE_IDLE, 'mission_manager starting')

        self.get_logger().info('Waiting for Nav2...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 active.')

        self.get_logger().info('Waiting for localization (map->base_link)...')
        while rclpy.ok() and not self._is_localized():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info(
                '  need a 2D Pose Estimate in RViz if this hangs',
                throttle_duration_sec=5.0,
            )
        self.get_logger().info('Localized.')

        # Give the latched /hazards/confirmed topic a moment to arrive.
        deadline = self.get_clock().now() + Duration(seconds=self.hazards_wait_timeout)
        while rclpy.ok() and self.get_clock().now() < deadline and not self.hazards:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.hazards:
            self.get_logger().warn('No hazards received; falling back to waypoints file.')
            self.hazards = self._collect_waypoint_hazards()

        if not self.hazards:
            self.get_logger().error('Nothing to visit. Mission aborted.')
            self._publish_status(MissionStatus.PHASE_ABORTED, 'no hazards, no waypoints')
            return

        self._publish_status(
            MissionStatus.PHASE_PLANNING,
            f'Visiting {len(self.hazards)} hazard(s)',
        )

        visited = 0
        for idx, hazard in enumerate(self.hazards):
            robot_x, robot_y = self._current_xy()
            goal = self._standoff_pose(hazard.position, robot_x, robot_y)

            self.get_logger().info(
                f'[{idx + 1}/{len(self.hazards)}] -> hazard id={hazard.id} '
                f'cat={hazard.category} at ({hazard.position.x:.2f}, {hazard.position.y:.2f})'
            )
            self._publish_status(
                MissionStatus.PHASE_EN_ROUTE,
                f'en route to {hazard.category}',
                current_id=hazard.id,
                current_cat=hazard.category,
                visited=visited,
            )

            self.navigator.goToPose(goal)
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)

            if self.navigator.getResult() != TaskResult.SUCCEEDED:
                self.get_logger().warn(f'Nav2 failed for hazard {hazard.id}; continuing.')
                continue

            self._publish_status(
                MissionStatus.PHASE_AT_HAZARD,
                f'at {hazard.category}; requesting package',
                current_id=hazard.id,
                current_cat=hazard.category,
                visited=visited,
            )
            self._request_package(hazard)
            visited += 1

        self._publish_status(
            MissionStatus.PHASE_COMPLETE,
            f'visited {visited}/{len(self.hazards)} hazards',
            visited=visited,
        )
        self.get_logger().info(f'Mission complete: {visited}/{len(self.hazards)} visited.')


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
