#!/usr/bin/env python3
"""Mission manager for the scout (SKELETON).
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
from rclpy.qos import (DurabilityPolicy,QoSProfile,ReliabilityPolicy,)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from scout_msgs.msg import Hazard, MissionStatus
from scout_msgs.srv import RequestPackage
from std_msgs.msg import Bool, String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from enum import Enum

LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


"""
My state plan


1)standby, 2)mapping, 3)at_arm, 4)ready_to_navigate, 5)navigating_to_hazard, 6)at_hazard, 7)returning_home, 8)complete, 9)aborted
"""

class State(Enum):
    STANDBY = "standby"
    MAPPING = "mapping"
    AT_ARM = "at_arm"
    READY_TO_NAVIGATE = "ready_to_navigate"
    NAVIGATING_TO_HAZARD = "navigating_to_hazard"
    AT_HAZARD = "at_hazard"
    RETURNING_HOME = "returning_home"
    COMPLETE = "complete"
    ABORTED = "aborted"


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('tick_period', 1.0)
        self.tick_period = float(self.get_parameter('tick_period').value)

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('standoff_distance', 0.6)
        self.declare_parameter('hazards_wait_timeout', 3.0)
        self.declare_parameter('request_package_timeout', 15.0)
        # 'map' in sim and real -- the global frame is shared even when the
        # robot's TFs are namespaced as scout/... in real mode.
        self.declare_parameter('map_frame', 'map')
        self.create_timer(self.tick_period, self.tick_callback)

        self.waypoints_file: str = self.get_parameter('waypoints_file').value
        self.standoff: float = float(self.get_parameter('standoff_distance').value)
        self.hazards_wait_timeout: float = float(self.get_parameter('hazards_wait_timeout').value)
        self.req_timeout: float = float(self.get_parameter('request_package_timeout').value)
        self.map_frame: str = self.get_parameter('map_frame').value

        self.navigator = BasicNavigator()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state = State.STANDBY
        self.current_hazard: Hazard = None
        self.hazards_total = 0
        self.mapping_complete = False
        self.hazard_dump: List[Hazard] = []
        self.hazard_dump_ids: List[int] = []
        self.hazards: List[Hazard] = []
        self.goal_sent = False
        self.carrying_package = False
        self.tocker_int = int(10/self.tick_period)
        self.tocker = 0
        self.nav_buffers = 10
        self.create_subscription(Hazard, '/hazards/confirmed', self.hazard_cb, LATCHED_QOS)
        self.create_subscription(Bool, '/scout/mapping_complete', self.mapping_complete_cb, LATCHED_QOS)
        self.create_subscription(Bool, '/scout/package_ready', self.package_ready_cb, 10)


        self.status_pub = self.create_publisher(MissionStatus, '/mission_status', 10)
        self.package_request_pub = self.create_publisher(String, '/scout/package_request', 10)

    # ------------------------------------------------------------------ helpers

    def hazard_cb(self, msg: Hazard):
        if msg.id not in self.hazard_dump_ids:
            self.hazard_dump.append(msg)
            self.hazards_total += 1
            self.hazard_dump_ids.append(msg.id)

    def mapping_complete_cb(self, msg: Bool):
        if msg.data:
            self.mapping_complete = True
    
    def package_ready_cb(self, msg: Bool):
        if msg.data:
            self.carrying_package = True

    def adjust_nav_params(self,xytol: float,yawtol:float):
        client = self.create_client(SetParameters, '/controller_server/set_parameters')
        if client.wait_for_service(timeout_sec=2.0):
            req = SetParameters.Request()
            req.parameters = [
                Parameter(name='goal_checker.xy_goal_tolerance',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=xytol)),
                Parameter(name='goal_checker.yaw_goal_tolerance',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=yawtol)),
                Parameter(name='FollowPath.xy_goal_tolerance',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=xytol)),    
            ]
            client.call_async(req)

            
            
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

   
    
    
    
    
    def tick_callback(self):

        if self.state == State.STANDBY:
            self.state = State.MAPPING

        elif self.state == State.MAPPING:
            if self.mapping_complete:
                self.hazards = self.hazard_dump
                self.state = State.AT_ARM
            else:
                return

        elif self.state == State.AT_ARM:
            if len(self.hazards) == 0:
                self.state = State.COMPLETE
                return
            next_hazard = self.hazards[0]
            next_color = next_hazard.color
            self.package_request_pub.publish(String(data=f"{next_color}"))
            self.state = State.READY_TO_NAVIGATE

        elif self.state == State.READY_TO_NAVIGATE:
            if self.carrying_package:
                self.state = State.NAVIGATING_TO_HAZARD
                self.current_hazard = self.hazards.pop(0)

        elif self.state == State.NAVIGATING_TO_HAZARD:
            if self.goal_sent == False:
                self.adjust_nav_params(0.2,3.14)
                self.goal_sent = True
                pose = PoseStamped()
                pose.header.frame_id = self.map_frame
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position = self.current_hazard.position
                pose.pose.orientation.w = 1.0
                self.navigator.goToPose(pose)

            if self.navigator.isTaskComplete():
                res = self.navigator.getResult()
                if res == TaskResult.SUCCEEDED:
                    self.state = State.AT_HAZARD
                else:
                    self.nav_buffers -= 1
                    if self.nav_buffers == 0:
                        self.state = State.ABORTED
                        return
                    self.state = State.NAVIGATING_TO_HAZARD

                self.goal_sent = False
            return
            
        elif self.state == State.AT_HAZARD:
            if not self.carrying_package:
                self.adjust_nav_params(0.1,0.1)
                self.state = State.RETURNING_HOME
                return
            if self.tocker == self.tocker_int:
                self.tocker = 0
                self.carrying_package = False
            self.tocker += 1

        elif self.state == State.RETURNING_HOME:
            if self.goal_sent == False:
                self.goal_sent = True
                pose = PoseStamped()
                pose.header.frame_id = self.map_frame
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = 0.0
                pose.pose.position.y = 0.0
                pose.pose.orientation.w = 1.0
                self.navigator.goToPose(pose)

            if self.navigator.isTaskComplete():
                res = self.navigator.getResult()
                if res == TaskResult.SUCCEEDED:
                    self.state = State.AT_ARM
                else:
                    self.nav_buffers -= 1
                    if self.nav_buffers == 0:
                        self.state = State.ABORTED
                        return
                    self.state = State.RETURNING_HOME

                self.goal_sent = False

            return
        elif self.state == State.COMPLETE:
            self.get_logger().info("Mission completed")
            return
        elif self.state == State.ABORTED:
            self.get_logger().error("Mission aborted")
            return
        else:
            self.get_logger().error(f"Unknown state: {self.state}")
            return



        
def main():
    rclpy.init()
    node = MissionManager()
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
