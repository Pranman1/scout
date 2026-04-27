#!/usr/bin/env python3
"""Manual WASD teleop + on-demand map save.

Opens in its own xterm (see launch file). Press 'p' to save the current
SLAM map to the path configured via the ``map_path`` parameter (no .yaml
suffix -- nav2_map_server adds .yaml/.pgm for you).
"""
import os
import select
import subprocess
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


MSG = """
---------------------------
   SCOUT MANUAL MAPPING
---------------------------
     w
   a s d    (drive)
     x

p     : SAVE MAP NOW
space : force stop
CTRL-C: quit
---------------------------
"""


class ManualMapper(Node):
    def __init__(self):
        super().__init__('manual_mapper')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.declare_parameter('map_path', '')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 1.0)

        self.map_path = self.get_parameter('map_path').value
        self.speed = float(self.get_parameter('linear_speed').value)
        self.turn = float(self.get_parameter('angular_speed').value)

    def save_map(self):
        if not self.map_path:
            self.get_logger().error('No map_path parameter set; refusing to save.')
            return

        os.makedirs(os.path.dirname(self.map_path), exist_ok=True)
        self.get_logger().info(f'Saving map to: {self.map_path} ...')
        try:
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self.map_path],
                check=True,
            )
            self.get_logger().info('Map saved.')
        except subprocess.CalledProcessError as exc:
            self.get_logger().error(f'map_saver_cli failed: {exc}')

    def _get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        print(MSG)
        try:
            while rclpy.ok():
                key = self._get_key()
                x, th = 0.0, 0.0
                if key == 'w':
                    x = self.speed
                elif key == 'x':
                    x = -self.speed
                elif key == 'a':
                    th = self.turn
                elif key == 'd':
                    th = -self.turn
                elif key in (' ', 's'):
                    pass
                elif key == 'p':
                    self.save_map()
                elif key == '\x03':
                    break

                twist = Twist()
                twist.linear.x = float(x)
                twist.angular.z = float(th)
                self.publisher_.publish(twist)
        finally:
            self.publisher_.publish(Twist())


settings = None


def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ManualMapper()
    try:
        node.run()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
