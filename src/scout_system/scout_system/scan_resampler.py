#!/usr/bin/env python3
"""LDS-02 scan resampler, real-robot only (SKELETON).

Background: the LDS-02 publishes a variable number of readings per scan
(typically 250-256). SLAM Toolbox latches onto the first scan's count
and drops any that don't match, which looks like mapping that freezes /
un-freezes at random. The workaround is to resample every incoming
``/scan`` to a fixed count and republish on ``/scan_filtered``, which
SLAM listens to in real mode. Sim is unaffected (the Gazebo laser
already publishes a constant count) so this node isn't launched there.

You write ``_cb``. The rest is plumbing.
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
        """Resample ``msg.ranges`` to ``self.target_count`` samples and republish.

        TODO(you):
            1. Bail early if ``len(msg.ranges) == 0``.
            2. Build the *input* angle axis: ``np.linspace(angle_min,
               angle_max, len(msg.ranges))``.
            3. Build the *target* angle axis: ``np.linspace(angle_min,
               angle_max, self.target_count)``.
            4. ``np.interp`` target onto input. Linear interpolation is
               fine -- the LDS-02 is already noisy enough that fancier
               schemes don't buy you much.
            5. Populate a fresh ``LaserScan``:
                - copy header / angle_min / angle_max / time_increment /
                  scan_time / range_min / range_max verbatim,
                - set ``angle_increment = (angle_max - angle_min) /
                  (self.target_count - 1)``,
                - ``ranges = resampled.tolist()``, ``intensities = []``.
            6. Publish.
        """
        raise NotImplementedError("TODO(you): implement scan resampling.")


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
