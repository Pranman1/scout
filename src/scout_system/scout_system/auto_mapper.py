#!/usr/bin/env python3
"""Auto-map monitor.

Watches explore_lite's ``/explore/frontiers`` marker stream. When no
frontiers have been published for ``completion_timeout`` seconds the
occupancy grid is saved to ``map_path`` via nav2_map_server, and a
``/scout/mapping_complete`` std_msgs/Bool is latched so downstream nodes
(mission_manager, hazard_tracker snapshot) can react.

The node intentionally does **not** send SIGINT to the process group any
more (grazen used to): that behavior made sense for a standalone mapping
launch but is wrong for the "scout" flow where we want to *continue*
into the mission phase without relaunching anything.
"""
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray


class AutoMapper(Node):
    def __init__(self):
        super().__init__('auto_mapper')

        self.declare_parameter('map_path', '')
        self.declare_parameter('completion_timeout', 10.0)
        self.declare_parameter('save_interval', 0.0)       # 0 = no periodic backups
        self.declare_parameter('shutdown_on_complete', False)

        self.map_path = self.get_parameter('map_path').value
        self.completion_timeout = float(self.get_parameter('completion_timeout').value)
        self.save_interval = float(self.get_parameter('save_interval').value)
        self.shutdown_on_complete = bool(self.get_parameter('shutdown_on_complete').value)

        self.last_frontier_time = time.time()
        self.last_save_time = time.time()
        self.exploration_complete = False
        self.frontier_count = 0

        marker_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            MarkerArray, '/explore/frontiers', self._frontier_cb, marker_qos
        )

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.complete_pub = self.create_publisher(Bool, '/scout/mapping_complete', latched)
        self.complete_pub.publish(Bool(data=False))

        self.create_timer(2.0, self._tick)

        self.get_logger().info('Auto Mapper ready')
        self.get_logger().info(f'  map_path           = {self.map_path}')
        self.get_logger().info(f'  completion_timeout = {self.completion_timeout}s')
        if self.save_interval > 0:
            self.get_logger().info(f'  save_interval      = {self.save_interval}s (backup)')

    def _frontier_cb(self, msg: MarkerArray):
        active = [m for m in msg.markers if m.action != 2]  # 2 == DELETE
        self.frontier_count = len(active)
        if self.frontier_count > 0:
            self.last_frontier_time = time.time()

    def _tick(self):
        if self.exploration_complete:
            return
        now = time.time()
        if self.save_interval > 0 and now - self.last_save_time >= self.save_interval:
            self._save_map(suffix='_backup')
            self.last_save_time = now

        gap = now - self.last_frontier_time
        if gap >= self.completion_timeout:
            self.get_logger().info('=' * 50)
            self.get_logger().info(
                f'EXPLORATION COMPLETE (no frontiers for {self.completion_timeout:.1f}s)'
            )
            self.get_logger().info('=' * 50)
            self.exploration_complete = True
            self._save_map()
            self.complete_pub.publish(Bool(data=True))
            if self.shutdown_on_complete:
                self.get_logger().info('shutdown_on_complete=True -> stopping ROS.')
                rclpy.shutdown()
        else:
            self.get_logger().info(
                f'exploring: frontiers={self.frontier_count}, '
                f'idle={gap:.1f}/{self.completion_timeout:.1f}s',
                throttle_duration_sec=10.0,
            )

    def _save_map(self, suffix: str = '') -> bool:
        if not self.map_path:
            self.get_logger().error('No map_path; cannot save.')
            return False
        path = f'{self.map_path}{suffix}'
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.get_logger().info(f'Saving map -> {path}')
        try:
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path],
                capture_output=True,
                text=True,
                timeout=30,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error('map_saver_cli timed out')
            return False
        if result.returncode == 0:
            self.get_logger().info(f'Map saved: {path}.yaml')
            return True
        self.get_logger().error(f'map_saver_cli failed: {result.stderr.strip()}')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapper()
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
