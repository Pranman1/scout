#!/usr/bin/env python3
"""Stub UR7: simulates the arm placing a colored block in the scout's box.

Listens on /scout/package_request for a color (std_msgs/String),
waits prep_seconds to mimic arm motion, then publishes True on
/scout/package_ready so the scout knows it can drive off.
"""
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class Ur7Stub(Node):
    def __init__(self):
        super().__init__('ur7_stub')
        self.declare_parameter('prep_seconds', 2.0)
        self.prep = float(self.get_parameter('prep_seconds').value)

        self.ready_pub = self.create_publisher(Bool, '/scout/package_ready', 10)
        self.create_subscription(String, '/scout/package_request', self._on_request, 10)
        self.get_logger().info(f'UR7 stub ready (prep={self.prep:.1f}s)')

    def _on_request(self, msg: String):
        self.get_logger().info(f'[ur7_stub] placing {msg.data} block')
        threading.Timer(self.prep, self._signal_ready).start()

    def _signal_ready(self):
        self.ready_pub.publish(Bool(data=True))


def main():
    rclpy.init()
    node = Ur7Stub()
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
