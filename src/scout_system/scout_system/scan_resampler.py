#!/usr/bin/env python3
"""LDS-02 scan resampler (real-robot only).

The real LDS-02 laser publishes a variable number of readings per scan
(typically 250-256). SLAM Toolbox latches onto the first scan's count and
drops any that differ, which manifests as mapping that starts, freezes,
starts, freezes.

This node subscribes to ``/scan``, linearly interpolates every scan up
to a fixed count (default 360 -> 1 deg resolution), and republishes on
``/scan_filtered`` with sensor-compatible QoS. SLAM is configured to
listen on ``/scan_filtered`` in real mode. Sim is unaffected -- the
stock Gazebo laser already publishes a constant count, so the resampler
isn't launched there.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan


class ScanResampler(Node):
    def __init__(self):
        super().__init__('scan_resampler')

        self.declare_parameter('target_count', 360)
        self.target_count = int(self.get_parameter('target_count').value)

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(LaserScan, 'scan', self._cb, sensor_qos)
        self.pub = self.create_publisher(LaserScan, 'scan_filtered', sensor_qos)
        self.get_logger().info(f'Scan resampler running (target count = {self.target_count})')

    def _cb(self, msg: LaserScan):
        n_in = len(msg.ranges)
        if n_in == 0:
            return

        input_angles = np.linspace(msg.angle_min, msg.angle_max, n_in)
        input_ranges = np.array(msg.ranges)
        target_angles = np.linspace(msg.angle_min, msg.angle_max, self.target_count)
        target_ranges = np.interp(target_angles, input_angles, input_ranges)

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = (msg.angle_max - msg.angle_min) / (self.target_count - 1)
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = target_ranges.tolist()
        out.intensities = []
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanResampler()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
