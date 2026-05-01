#!/usr/bin/env python3
"""LDS-02 scan resampler, real-robot only (SKELETON).

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
       
        if len(msg.ranges) == 0:
            return

        in_axis = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        out_axis = np.linspace(msg.angle_min, msg.angle_max, self.target_count)
        resampled = np.interp(out_axis, in_axis, msg.ranges)

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = (msg.angle_max - msg.angle_min) / (self.target_count - 1)
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = resampled.tolist()
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
