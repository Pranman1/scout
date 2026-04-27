#!/usr/bin/env python3
"""Standalone keyboard teleop for the scout (SKELETON).

Instant-velocity style: pressing a key sets the target velocity directly
(not an accumulator like turtlebot3_teleop_key). Releasing / pressing any
other key stops the robot.

Publishes ``geometry_msgs/Twist`` on ``/cmd_vel`` at whatever rate the
loop in ``run()`` ticks. The plumbing (publisher, parameters, raw-mode
keyboard read, main()) is provided; you write the key -> Twist mapping
loop in ``run()``.

Usage::

    source ~/turtle_test/install/setup.bash
    source ~/scout_ws/install/setup.bash
    export TURTLEBOT3_MODEL=burger_cam
    ros2 run scout_system scout_teleop

Reuse note: once this works, the same loop body (plus a 'p' -> save_map
branch) is what ``manual_mapper.run()`` needs. Copy-paste or factor into
a shared helper -- your call.
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

        self.declare_parameter('linear_speed', 0.3)
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
        """Main teleop loop: read keys, publish Twists.

        TODO(you):
            - Print ``MSG`` once as the banner (so the user knows the key
              bindings when the node starts).
            - Loop while ``rclpy.ok()``:
                * ``key = self._get_key()``
                * Build a fresh ``Twist()`` each iteration (do NOT carry
                  state across iterations -- that would re-create the
                  accumulator behavior you're trying to avoid).
                * Map keys -> velocities:
                    'w' -> linear.x = +self.speed
                    'x' -> linear.x = -self.speed
                    'a' -> angular.z = +self.turn
                    'd' -> angular.z = -self.turn
                    ' ' / 's' / '' (no key) -> leave Twist at zero
                    '\\x03' (Ctrl-C) -> break out of the loop
                * Publish the Twist every iteration so the robot stops
                  automatically when no key is held.
            - In a ``finally`` block, publish one final zero Twist so the
              robot doesn't coast after the node exits.

        Edge cases to think about:
            - What happens if the user presses an unrecognized key?
              (Answer: same as no key -- zero Twist, robot stops. The
              "else" branch handles this implicitly if you build a fresh
              Twist every iteration.)
            - Should diagonal motion be allowed (w+a simultaneously)?
              Terminal keyboard reads are single-char, so you get one at
              a time anyway -- diagonals aren't really possible here
              without a more sophisticated input method. Accept this
              limitation for now.
        """
        # raise NotImplementedError("TODO(you): implement the key -> Twist loop.")
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
