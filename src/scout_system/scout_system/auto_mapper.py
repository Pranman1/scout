#!/usr/bin/env python3
"""Bounded frontier explorer + map saver (SKELETON).

Replaces explore_lite. Pipeline:

    /map (OccupancyGrid)            ROS topic
        |
        v
    detect frontiers                YOU
    filter by polygon bound         (plumbing helper provided)
    select best target              YOU
        |
        v
    Nav2 NavigateToPose action      (plumbing helper provided)
        |
        v
    repeat until no targets in bound
        |
        v
    save map + publish              (plumbing helpers provided)
    /scout/mapping_complete = True

You implement three methods (see TODOs):
  * ``_detect_frontiers(grid)`` -- list of Frontier from /map
  * ``_select_target(frontiers, robot_xy)`` -- pick one Frontier
  * ``_tick()`` -- FSM step (states defined below)

Everything else (TF lookup, polygon check, action client send/result,
map save, RViz markers) is already wired up.

Dependencies::

    sudo apt install python3-shapely
"""
import os
import subprocess
import time
from dataclasses import dataclass
from enum import Enum, auto

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

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

class State(Enum):
    INIT = auto()         # waiting for first /map
    PICK_TARGET = auto()  # detect frontiers, pick one, send nav goal
    NAVIGATING = auto()   # waiting for Nav2 to finish current goal
    COMPLETE = auto()     # map saved, /scout/mapping_complete latched True
    FAILED = auto()       # unrecoverable; you decide when to enter this


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

        # ---------- params -----------------------------------------------
        self.declare_parameter('map_path', '')
        self.declare_parameter('shutdown_on_complete', False)
        self.declare_parameter('tick_period', 1.0)
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        # How many back-to-back empty frontier detections before we declare
        # the map done. Bigger = more tolerant to transient empty passes
        # after a nav failure, smaller = exits faster on a truly done map.
        self.declare_parameter('max_consecutive_empty', 3)
        # Default polygon: matches the sim arena allowed region we sized
        # earlier (south-west biased to keep the dock-side spawn clear).
        # For the real lab, override this in the launch file.
        self.declare_parameter(
            'bounds_polygon',
         # [0.0, 0.0, 4.0, 0.0, 4.0, 2.0, 0.0, 2.0],
            [0.0, 0.0, 0.5, 0.0, 0.5, 0.5, 0.0, 0.5],
        )

        self.map_path = self.get_parameter('map_path').value
        self.shutdown_on_complete = bool(
            self.get_parameter('shutdown_on_complete').value
        )
        self.tick_period = float(self.get_parameter('tick_period').value)
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.max_consecutive_empty = int(
            self.get_parameter('max_consecutive_empty').value
        )

        flat = list(self.get_parameter('bounds_polygon').value)
        if len(flat) < 6 or len(flat) % 2 != 0:
            raise ValueError(
                f'bounds_polygon needs >=3 vertices as flat xy list; '
                f'got {len(flat)} elements.'
            )
        verts = list(zip(flat[0::2], flat[1::2]))
        self.polygon = Polygon(verts)

        # ---------- state ------------------------------------------------
        self.state = State.INIT
        self.latest_map = None             # type: OccupancyGrid | None
        self.current_target = None         # type: Frontier | None
        self.nav_in_progress = False
        self.nav_succeeded = False
        self.nav_started_t = 0.0
        self.blacklist = set()
        # Counter of back-to-back empty frontier detections; reset whenever
        # we actually pick a target. Only declares COMPLETE once this hits
        # self.max_consecutive_empty, so one flaky pass doesn't end the run.
        self.consecutive_empty = 0
        self.returning_home = False
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # ---------- ROS plumbing -----------------------------------------
        # SLAM publishes /map as latched (TRANSIENT_LOCAL); match it.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos
        )

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

    # ===================================================================
    # ALGORITHM (TODO YOU)
    # ===================================================================

    def _detect_frontiers(self, grid):
        """Find frontier clusters in the occupancy grid.

        TODO(you):
            - ``data = np.array(grid.data).reshape(grid.info.height,
              grid.info.width)`` -- values are -1 (unknown), 0 (free),
              100 (occupied).
            - Find FREE cells that have at least one UNKNOWN 4-neighbor.
              Those are frontier cells.
            - Cluster contiguous frontier cells (e.g.
              ``scipy.ndimage.label`` with a 4- or 8-connectivity mask).
            - For each cluster:
                * compute centroid in grid coords (mean of indices)
                * convert to world coords via ``self._grid_to_world``
                * filter out clusters whose centroid is OUTSIDE
                  ``self._in_polygon``
                * skip very small clusters (e.g. < 5 cells -- noise)
            - Return a list of ``Frontier(x, y, size)`` in world coords.
            - Optional: call ``self._publish_frontier_markers(result)``
              at the end so RViz shows them.
        """
        # raise NotImplementedError("TODO(you): implement frontier detection.")

        data = np.array(grid.data).reshape(grid.info.height, grid.info.width)

        # I dotnf uly get thsi it jsut kind eorked revist 

        robot_xy = self._get_robot_xy()
        if robot_xy is not None:
            non_wall = (data != 100)
            nw_labels, _ = scipy.ndimage.label(
                non_wall, structure=np.ones((3, 3), bool)
            )
            rc, rr = self._world_to_grid(robot_xy[0], robot_xy[1], grid.info)
            room_id = nw_labels[rr, rc]
            if room_id != 0:
                data[(nw_labels != 0) & (nw_labels != room_id)] = 100

        frontier_clusters = []
        free_mask = data == 0
        unknown_mask = data == -1
        frontier_mask = free_mask & binary_dilation(unknown_mask)
        structure = np.ones((3, 3),dtype=bool)
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
                      touched_ids = np.unique(unknown_labels[neighbors&unknown_mask])#
                      touched_ids = touched_ids[touched_ids != 0]
                      mass = int(unknown_sizes[touched_ids].sum())
                      if mass < 30:
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


    def _postprocess_grid(self, grid, threshold=7, max_iters=5):
        """Majority-vote cleanup of unknown cells.

        For each unknown pixel, look at its 8 neighbors:
          - If >= ``threshold`` of them are FREE, flip the pixel to free.
          - If >= ``threshold`` of them are WALL, flip the pixel to wall.
        Iterate up to ``max_iters`` times so fixes can propagate (an
        unknown surrounded by 5 free + 3 unknown might pass on iter 2).
        """
        data = np.array(grid.data, dtype=np.int8).reshape(
            grid.info.height, grid.info.width
        )
        # 3x3 kernel that counts neighbors only (centre = 0).
        kernel = np.array([[1, 1, 1],
                           [1, 0, 1],
                           [1, 1, 1]], dtype=int)

        for _ in range(max_iters):
            unknown = (data == -1)
            if not unknown.any():
                break
            free_mask = (data == 0).astype(int)
            wall_mask = (data == 100).astype(int)
            # the above bit is like a convolution mask that counts the number of free or wall neighbors
            free_neighbors = scipy.ndimage.convolve(
                free_mask, kernel, mode='constant', cval=0
            )
            wall_neighbors = scipy.ndimage.convolve(
                wall_mask, kernel, mode='constant', cval=0
            )
            make_free = unknown & (free_neighbors >= threshold) & (wall_neighbors == 0)
            make_wall = unknown & (wall_neighbors >= threshold)
            if not make_free.any() and not make_wall.any():
                break
            data[make_free] = 0
            data[make_wall] = 100

        # --- Seal off pockets disconnected from the robot's room. ---
        robot_xy = self._get_robot_xy()
        if robot_xy is not None:
            non_wall = (data != 100)
            nw_labels, _ = scipy.ndimage.label(
                non_wall, structure=np.ones((3, 3), bool)
            )
            rc, rr = self._world_to_grid(robot_xy[0], robot_xy[1], grid.info)
            room_id = nw_labels[rr, rc]
            if room_id != 0:
                data[(nw_labels != 0) & (nw_labels != room_id)] = 100

        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        H, W = data.shape

        xs = ox + (np.arange(W) + 0.5) * res
        ys = oy + (np.arange(H) + 0.5) * res
        X, Y = np.meshgrid(xs, ys)

        inside = shapely.vectorized.contains(self.polygon, X, Y)
        data[~inside] = 100
        data[(data == -1) & inside] = 100  # or 0, whichever is safer for you

        out = OccupancyGrid()
        out.header = grid.header
        out.info = grid.info
        out.data = data.flatten().tolist()
        return out

    def _select_target(self, frontiers, robot_xy):
        """Pick the best frontier to navigate to.

        TODO(you):
            - Simplest: nearest by Euclidean distance to ``robot_xy``.
            - Better: score = a*size - b*distance, pick max.
            - Return the chosen Frontier, or None if list is empty.
        """
        # raise NotImplementedError("TODO(you): implement target selection.")

        if not frontiers:
            return None

        max_score = -float('inf')
        best_frontier = None

        for frontier in frontiers:
            dx = frontier.x - robot_xy[0]
            dy = frontier.y - robot_xy[1]
            distance = (dx * dx + dy * dy) ** 0.5

            score = frontier.size - 15 * distance + 1 * frontier.mass
            if score > max_score:
                max_score = score
                best_frontier = frontier

        return best_frontier



    def _tick(self):
        """One step of the explorer FSM. Called every ``tick_period``.

        TODO(you): wire the state machine. Suggested skeleton:


        Edge cases worth thinking about:
            - Nav2 fails on a target. Retry once? Blacklist and try
              another? Just go back to PICK_TARGET? (Easiest: latter.)
            - latest_map updates while NAVIGATING. Do you re-pick? Or
              wait for current goal to finish? (Easiest: wait.)
            - No frontiers found on first PICK_TARGET (robot starts in
              fully-mapped area). COMPLETE immediately is fine.
        """
        # raise NotImplementedError("TODO(you): implement the FSM tick.")


        if self.state == State.INIT:
            if self.latest_map is not None:
                self.state = State.PICK_TARGET

        elif self.state == State.PICK_TARGET:
            robot_xy = self._get_robot_xy()
            if robot_xy is None:
                return
            frontiers = self._detect_frontiers(self.latest_map)
            target = self._select_target(frontiers, robot_xy)
            if target is None:
                self.consecutive_empty += 1
                self.get_logger().warn(
                    f'No frontiers found '
                    f'({self.consecutive_empty}/{self.max_consecutive_empty}).'
                )
                if self.consecutive_empty >= self.max_consecutive_empty:
                    self._save_map()
                    self._publish_complete()
                    self.get_logger().info(
                        f'Stable empty for {self.consecutive_empty} ticks. '
                        'Saving and completing map.'
                    )
                    self.returning_home = True
                    self._send_nav_goal(0.0, 0.0)  # return to start pose
                    self.state = State.COMPLETE
                # else stay in PICK_TARGET; map may update before next tick.
            else:
                self.consecutive_empty = 0
                if self._send_nav_goal(target.x, target.y):
                    self.current_target = target
                    self.state = State.NAVIGATING

        elif self.state == State.NAVIGATING:

            if not self.nav_in_progress and not self.nav_succeeded:
                self.get_logger().warn('Nav goal failed. Retrying...')
                self.blacklist.add((self.current_target.x, self.current_target.y))
                self.state = State.PICK_TARGET
            if not self.nav_in_progress:
                self.state = State.PICK_TARGET

        elif self.state == State.COMPLETE:
            if self.shutdown_on_complete:
                rclpy.shutdown()
            # else: stay in COMPLETE (terminal, no-op forever)
        else:
            self.get_logger().error(f'Unknown state: {self.state}')
        

    # ===================================================================
    # PLUMBING (DONE)
    # ===================================================================

    def _map_cb(self, msg):
        self.latest_map = msg

    def _get_robot_xy(self):
        """Look up robot's (x, y) in map frame via TF. Returns None on failure."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
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
        """Fire-and-forget Nav2 goal. Returns False if server not ready.

        On goal completion, ``_on_goal_result`` clears
        ``nav_in_progress`` and sets ``nav_succeeded``. The FSM polls
        these flags.
        """
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not ready.')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        # yaw=0 for now; FSM can pick a smarter heading if you want.
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
        elif self.returning_home:
            self.returning_home = False
            self._align_to_zero()

    def _align_to_zero(self):
        """Spin in place until yaw in map frame is near 0."""
        import math
        deadline = time.time() + 8.0
        while time.time() < deadline and rclpy.ok():
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.robot_frame, rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
            except TransformException:
                break
            q = tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            if abs(yaw) < 0.05:  # ~3 degrees
                break
            twist = Twist()
            twist.angular.z = -0.5 if yaw > 0 else 0.5
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('Aligned to start orientation.')

    def _save_map(self):
        if not self.map_path:
            self.get_logger().error('No map_path; refusing to save.')
            return False
        if self.latest_map is None:
            self.get_logger().error('No /map received yet; cannot save.')
            return False

        cleaned = self._postprocess_grid(self.latest_map)
        self.final_map_pub.publish(cleaned)
        # Give map_saver_cli a moment to subscribe + receive the latched msg.
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
            self.get_logger().info('Map saved.')
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
        m.header = Header(frame_id=self.map_frame,
                          stamp=self.get_clock().now().to_msg())
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
        # Clear old frontier markers first.
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.ns = 'frontiers'
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, f in enumerate(frontiers):
            m = Marker()
            m.header = Header(frame_id=self.map_frame,
                              stamp=self.get_clock().now().to_msg())
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = f.x
            m.pose.position.y = f.y
            m.pose.position.z = 0.05
            # Sqrt-scale mass into a diameter; clamp to [0.1m, 0.6m] so
            # tiny frontiers are still visible and giant ones don't dwarf
            # the map. Tweak the 0.02 multiplier for taste.
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
