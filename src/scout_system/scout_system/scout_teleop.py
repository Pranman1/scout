#!/usr/bin/env python3
"""Standalone keyboard teleop for the scout (SKELETON).
"""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


MSG = """
---------------------------
      SCOUT TELEOP
---------------------------
     w
   a   d    (drive)
     s

space : force stop
CTRL-C: quit
---------------------------
"""


class ScoutTeleop(Node):
    def __init__(self):
        super().__init__('scout_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 1.5)

        self.speed = float(self.get_parameter('linear_speed').value)
        self.turn = float(self.get_parameter('angular_speed').value)

    def _get_key(self):
        """Non-blocking single-keypress read from stdin in raw mode. Plumbing.

        Returns '' if no key was pressed within the 0.1s window, else a
        single character. The terminal is restored to its prior settings
        by ``main()`` on shutdown; this function only flips raw mode on
        for the duration of the read.
        """
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        """TODO: Main teleop loop: read keys, publish Twists."""

        print(MSG)
        while rclpy.ok():
            key = self._get_key()
            twist = Twist()
            if key == 'w':
                twist.linear.x = self.speed
            elif key == 's':
                twist.linear.x = -self.speed
            elif key == 'a':
                twist.angular.z = self.turn
            elif key == 'd':
                twist.angular.z = -self.turn
            elif key == ' ':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key == '\x03':
                break
            self.publisher_.publish(twist)


settings = None


def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ScoutTeleop()
    try:
        node.run()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
