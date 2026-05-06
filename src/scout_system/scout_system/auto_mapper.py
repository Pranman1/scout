#!/usr/bin/env python3
"""Bounded frontier explorer + map saver (SKELETON).
based on explorlite but made to suit our needs, za pipeline ia as follows:

map topic -> detect frontiers -> filter by polygon bound -> pick best frontier -> send goal to nav -> repeat till no trgets -> save map  + bub lish mapping complete -> publish final map 
"""
import os
import subprocess
import time
from dataclasses import dataclass
from enum import Enum, auto
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy,QoSProfile,ReliabilityPolicy,)
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point as PointMsg, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from shapely.geometry import Point, Polygon
import shapely.vectorized
from std_msgs.msg import Bool, ColorRGBA, Header
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray
from scipy.ndimage import binary_dilation, label
import numpy as np
import scipy.ndimage
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class State(Enum):
    INIT = auto()         # waiting for first /map
    PICK_TARGET = auto()  # detect frontiers, pick one, send nav goal
    NAVIGATING = auto()   # waiting for Nav2 to finish current goal
    COMPLETE = auto()     # map saved, /scout/mapping_complete latched True
    FAILED = auto()     
    RETURNING_HOME = auto()  # unrecoverable; you decide when to enter this


@dataclass
class Frontier:
    """One candidate target. Algorithm produces these, FSM consumes them."""
    x: float        # world meters (map frame)
    y: float        # world meters (map frame)
    size: int
    mass: int       # number of cells in the cluster (proxy for value)


class AutoMapper(Node):
    def __init__(self):
        super().__init__('auto_mapper')

        # params brev -
        self.declare_parameter('map_path', '')
        self.declare_parameter('shutdown_on_complete', False)
        self.declare_parameter('tick_period', 1.0)
        self.declare_parameter('robot_frame', 'scout/base_link')
        self.declare_parameter('map_frame', 'scout/map')
        self.declare_parameter('max_consecutive_empty', 3)
        self.declare_parameter(
            'bounds_polygon',
            [0.0, 0.0, 4.0, 0.0, 4.0, 1.5, 0.0, 1.5],
            # [0.0, 0.0, 0.5, 0.0, 0.5, 0.5, 0.0, 0.5],
            # [-4.0, -4.0, 4.0, -4.0, 4.0, 4.0, -4.0, 4.0],
            #  [-0.5, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5, 0.5],
            )

        self.map_path = self.get_parameter('map_path').value
        self.shutdown_on_complete = bool(self.get_parameter('shutdown_on_complete').value)
        self.tick_period = float(self.get_parameter('tick_period').value)
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.max_consecutive_empty = int(self.get_parameter('max_consecutive_empty').value)

        flat = list(self.get_parameter('bounds_polygon').value)
        if len(flat) < 6 or len(flat) % 2 != 0:
            raise ValueError("not correct polygon pts formarat")

        verts = list(zip(flat[0::2], flat[1::2]))
        self.polygon = Polygon(verts)

        # state you knwo what im sayin brev -

        self.state = State.INIT
        self.latest_map = None          
        self.current_target = None      
        self.nav_in_progress = False
        self.nav_succeeded = False
        self.nav_started_t = 0.0
        self.blacklist = set()
        self.consecutive_empty = 0

        # ---------- ROS plumbing 
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.complete_pub = self.create_publisher(
            Bool, '/scout/mapping_complete', latched_qos
        )
        self.complete_pub.publish(Bool(data=False))

        self.markers_pub = self.create_publisher(
            MarkerArray, '/scout/frontier_markers', 10
        )

        self.final_map_pub = self.create_publisher(
            OccupancyGrid, '/scout/final_map', latched_qos
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self.create_timer(self.tick_period, self._tick)
        self.create_timer(2.0, self._publish_polygon_marker)
        self.get_logger().info('Auto Mapper ready')
        self.get_logger().info(f'  map_path        = {self.map_path}')
        self.get_logger().info(f'  bounds polygon  = {len(verts)} vertices')
        self.get_logger().info(f'  tick_period     = {self.tick_period}s')

    # algorithm's brev 


    def _detect_frontiers(self, grid):
        """Find frontier clusters in the occupancy grid.
        """
        # raise NotImplementedError("TODO(you): implement frontier detection.")

        data = np.array(grid.data).reshape(grid.info.height, grid.info.width)

        # I dotnf uly get thsi it jsut kind eorked revist - we just made some masks, 
        # from what i gather first mask shoidl be to find all the non wall reifons, then teh labels gives us the areas of non walls, 

        robot_xy = self._get_robot_xy()
        structure = np.array([[True, True, True],[True, True, True],[True, True, True]])

        if robot_xy is not None:
            non_wall = (data != 100)
            nw_labels, num = scipy.ndimage.label(non_wall, structure=structure)

            gridx, gridy = self._world_to_grid(robot_xy[0], robot_xy[1], grid.info)

            room_id = nw_labels[gridy, gridx]
            if room_id != 0:
                data[(nw_labels != 0) & (nw_labels != room_id)] = 100

        frontier_clusters = []
        free_mask = data == 0
        unknown_mask = data == -1
        frontier_mask = free_mask & binary_dilation(unknown_mask)
        unknown_labels, num_unknown = scipy.ndimage.label(unknown_mask, structure=structure)
        unknown_sizes = np.bincount(unknown_labels.ravel())
        
        # the above bit is liek b lob 1 has 4 then blob2 has 3 et index 2 is blob 2



        labels, num_labels = scipy.ndimage.label(frontier_mask)
        for label_id in range(1, num_labels + 1):
            rows, cols = np.where(labels == label_id)
            if len(rows) > 0 and len(cols) > 0:
                avgr = sum(rows)/len(rows)
                avgc = sum(cols)/len(cols)
                x, y = self._grid_to_world(avgc, avgr, grid.info)
                if self._in_polygon(x, y):
                    size = len(rows)
                    if self._is_near_robot(x, y, self._get_robot_xy()):
                      continue
                  
                    if size > 2 and not self._is_in_blacklist(x, y):
                      cluster_mask = labels == label_id
                      neighbors = binary_dilation(cluster_mask)
                      touched_ids = np.unique(unknown_labels[neighbors&unknown_mask])
                      touched_ids = touched_ids[touched_ids != 0]
                      mass = int(unknown_sizes[touched_ids].sum())
                      if mass < 5:
                        continue

                      frontier_clusters.append(Frontier(x, y, size, mass))

        self._publish_frontier_markers(frontier_clusters)
        return frontier_clusters

    def _is_near_robot(self, x, y, robot_xy, tolerance=0.7):
        dx = x - robot_xy[0]
        dy = y - robot_xy[1]
        
        return (dx * dx + dy * dy) ** 0.5 < tolerance

    def _is_in_blacklist(self, x, y,tolerance=0.1):
        for blacklisted_x, blacklisted_y in self.blacklist:
          if abs(x - blacklisted_x) < tolerance and abs(y - blacklisted_y) < tolerance:
            return True
        return False


    def _postprocess_grid(self, grid, threshold=5, max_iters=2):
        """Majority-vote cleanup of unknown cells.
        """
        data = np.array(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)
        kernel = np.array([[1, 1, 1],[1, 0, 1],[1, 1, 1]], dtype=int)

        for i in range(max_iters):
            unknown = (data == -1)
            if not unknown.any():
                break
            # free_mask = (data == 0).astype(int)
            wall_mask = (data == 100).astype(int)
            # the above bit is like a convoolution mask that counts the number of free or wall neighbors
    
            wall_neighbors = scipy.ndimage.convolve(wall_mask, kernel, mode='constant', cval=0)
            make_wall = unknown & (wall_neighbors >= threshold)
            if not make_wall.any():
                break
            # data[make_free] = 0
            data[make_wall] = 100  



        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        H, W = data.shape

        xs = ox + (np.arange(W) + 0.5) * res
        ys = oy + (np.arange(H) + 0.5) * res
        X, Y = np.meshgrid(xs, ys)

        inside = shapely.vectorized.contains(self.polygon, X, Y)
        data[~inside] = 100
        data[(data == -1) & inside] = 100  

        out = OccupancyGrid()
        out.header = grid.header
        out.info = grid.info
        out.data = data.flatten().tolist()
        return out

    def _select_target(self, frontiers, robot_xy):
        """Pick the best frontier to navigate to
        """
        # raise NotImplementedError("TODO(you): implement target selection.")

        if not frontiers:
            return None

        max_score = -float(1000000000)
        best_frontier = None

        for front in frontiers:
            dx = front.x - robot_xy[0]
            dy = front.y - robot_xy[1]
            distance = (dx * dx + dy * dy) ** 0.5

            score = front.size - 15 * distance + 1 * front.mass
            if score > max_score:
                max_score = score
                best_frontier = front

        return best_frontier



    def _tick(self):
        """One step of the explorer FSM. Called every ``tick_period``.
        """
        # raise NotImplementedError("TODO(you): implement the FSM tick.")

        if self.state == State.INIT:
            if self.latest_map is not None:
                self.state = State.PICK_TARGET
                self.final_map_pub.publish(self.latest_map)

        elif self.state == State.PICK_TARGET:
            robot_xy = self._get_robot_xy()
            if robot_xy is None:
                return
            frontiers = self._detect_frontiers(self.latest_map)
            target = self._select_target(frontiers, robot_xy)
            if target is None:
                self.consecutive_empty += 1
                self.get_logger().warn(f"{self.consecutive_empty} empty found")

                if self.consecutive_empty >= self.max_consecutive_empty:
                    self._save_map()

                    # below is boilerplae i pulled form online

                    client = self.create_client(SetParameters, '/controller_server/set_parameters')
                    if client.wait_for_service(timeout_sec=2.0):
                        req = SetParameters.Request()
                        req.parameters = [
                            Parameter(name='goal_checker.xy_goal_tolerance',
                                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.05)),
                            Parameter(name='goal_checker.yaw_goal_tolerance',
                                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.05)),
                            Parameter(name='FollowPath.xy_goal_tolerance',
                                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.05)), 
                        ]
                        client.call_async(req)

                    # client = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
                    # if client.wait_for_service(timeout_sec=2.0):
                    #     req = SetParameters.Request()
                    #     req.parameters = [
                    #         Parameter(name='static_layer.map_topic',
                    #                 value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                    #            string_value='/scout/final_map'))   
                    #     ]
                    #     client.call_async(req)


                    self._send_nav_goal(0.0, 0.0)
                    self.state = State.RETURNING_HOME
            else:
                self.consecutive_empty = 0
                if self._send_nav_goal(target.x, target.y):
                    self.current_target = target
                    self.state = State.NAVIGATING

        elif self.state == State.NAVIGATING:
            if not self.nav_in_progress and not self.nav_succeeded:
                self.blacklist.add((self.current_target.x, self.current_target.y))
                self.state = State.PICK_TARGET
            if not self.nav_in_progress:
                self.state = State.PICK_TARGET
        elif self.state == State.RETURNING_HOME:
            if not self.nav_in_progress:
                self.state = State.COMPLETE
                self._publish_complete()
        elif self.state == State.COMPLETE:
            if self.shutdown_on_complete:
                rclpy.shutdown()
        else:
            self.get_logger().error(f'Unknown state: {self.state}')
# plumbing brev - helper funcs

    def _map_cb(self, msg):
        self.latest_map = msg

    def _get_robot_xy(self):
        """Look up robot's (x, y) in map frame via TF. Returns None on failure."""
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time(),timeout=Duration(seconds=0.5), )
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException as e:
            return None

    def _world_to_grid(self, wx, wy, info):
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)
        return mx, my

    def _grid_to_world(self, mx, my, info):
        wx = info.origin.position.x + (mx + 0.5) * info.resolution
        wy = info.origin.position.y + (my + 0.5) * info.resolution
        return wx, wy

    def _in_polygon(self, x, y):
        return self.polygon.contains(Point(x, y))

    def _send_nav_goal(self, x, y, yaw=0.0):
    
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not ready.')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        self.nav_in_progress = True
        self.nav_succeeded = False
        self.nav_started_t = time.time()

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal.')
            self.nav_in_progress = False
            self.nav_succeeded = False
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        result = future.result()
        status = result.status
        self.nav_in_progress = False
        self.nav_succeeded = (status == GoalStatus.STATUS_SUCCEEDED)
        if not self.nav_succeeded:
            self.get_logger().warn(f'Nav goal ended with status={status}')
            


    def _save_map(self):
        if not self.map_path:
            return False
        if self.latest_map is None:
            return False

        cleaned = self._postprocess_grid(self.latest_map)
        self.final_map_pub.publish(cleaned)
        time.sleep(0.5)

        os.makedirs(os.path.dirname(self.map_path), exist_ok=True)
        self.get_logger().info(f'Saving map to: {self.map_path} ...')
        try:
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', self.map_path,
                 '-t', '/scout/final_map'],
                check=True, capture_output=True, text=True, timeout=30,
            )
            self.get_logger().info('Map sav')
            return True
        except subprocess.TimeoutExpired:
            self.get_logger().error('map_saver_cli timed out.')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'map_saver_cli failed: {e.stderr}')
        return False

    def _publish_complete(self):
        self.complete_pub.publish(Bool(data=True))
        self.get_logger().info('/scout/mapping_complete -> True')

    def _publish_polygon_marker(self):
        m = Marker()
        m.header = Header(frame_id=self.map_frame,stamp=self.get_clock().now().to_msg())
        m.ns = 'bounds'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.8)
        coords = list(self.polygon.exterior.coords)
        for x, y in coords:
            m.points.append(PointMsg(x=float(x), y=float(y), z=0.0))
        self.markers_pub.publish(MarkerArray(markers=[m]))

    def _publish_frontier_markers(self, frontiers):
        """Optional helper: visualize candidate frontiers in RViz."""
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.ns = 'frontiers'
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, f in enumerate(frontiers):
            m = Marker()
            m.header = Header(frame_id=self.map_frame, stamp=self.get_clock().now().to_msg())
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = f.x
            m.pose.position.y = f.y
            m.pose.position.z = 0.05
            diameter = max(0.1, min(0.6, (f.mass ** 0.5) * 0.02))
            m.scale.x = m.scale.y = m.scale.z = diameter
            m.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)
            arr.markers.append(m)
        self.markers_pub.publish(arr)


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
