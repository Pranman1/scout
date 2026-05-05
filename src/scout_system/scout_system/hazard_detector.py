#!/usr/bin/env python3
"""Per-frame hazard detector with camera + LiDAR fusion (SKELETON).

Input -> camera image -> HSV -> colored blob detections (color + bbox) -> bearing range in robot/lidar frame -> cluster -> match -> fused (color + 3D position) -> /hazards/raw + RViz markers
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import ( DurabilityPolicy, QoSProfile,ReliabilityPolicy,)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix
from visualization_msgs.msg import Marker, MarkerArray
from scout_msgs.msg import Hazard
from scout_system.detectors import Detection, build_detector
from shapely.geometry import Point as ShapelyPoint, Polygon


SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


_COLOR_TO_RGBA = {
    'red':    ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9),
    'yellow': ColorRGBA(r=1.0, g=0.95, b=0.1, a=0.9),
    'blue':   ColorRGBA(r=0.1, g=0.3, b=1.0, a=0.9),
    'green':  ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.9),
}




@dataclass
class _ScanCluster:
    """One contiguous chunk of LaserScan rays seeing the same object.
    Bearings are in the lidar's frame (positive = CCW / robot's left,
    zero = robot forward). Algorithm produces these, _match_cluster
    consumes them.
    """
    bearing: float        # weighted mean angle [rad]
    range: float          # mean range [m]
    bearing_min: float    # leftmost ray angle [rad]
    bearing_max: float    # rightmost ray angle [rad]
    n_rays: int


class HazardDetector(Node):
    def __init__(self):
        super().__init__('hazard_detector')

        # --------------- params --------------------------------------
        self.declare_parameter('params_file', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('lidar_frame', 'base_scan')
        self.declare_parameter('max_scan_age_s', 0.3)
        self.declare_parameter('cluster_depth_jump_m', 0.20)
        self.declare_parameter('cluster_max_width_m', 0.30)
        self.declare_parameter('cluster_min_rays', 1)
        self.declare_parameter('bearing_match_tol_deg', 20.0)
        self.declare_parameter('max_range_m', 4.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.info_topic = self.get_parameter('camera_info_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.max_scan_age = float(self.get_parameter('max_scan_age_s').value)
        self.depth_jump = float(self.get_parameter('cluster_depth_jump_m').value)
        self.max_cluster_width = float(self.get_parameter('cluster_max_width_m').value)
        self.min_rays = int(self.get_parameter('cluster_min_rays').value)
        self.match_tol = math.radians(float(self.get_parameter('bearing_match_tol_deg').value))
        self.max_range = float(self.get_parameter('max_range_m').value)
        self.merger_tol = 0.1

        self.cfg = self._load_config()
        det_cfg = self.cfg.get('detector', {})
        backend = det_cfg.get('backend', 'hsv')
        self.detector = build_detector(backend, det_cfg)
        self.color_to_category: Dict[str, str] = self.cfg.get('color_to_category', {})


        self.bridge = CvBridge()
        self.intrinsics: Optional[np.ndarray] = None
        self.image_size: Optional[Tuple[int, int]] = None  # (W, H)
        self.latest_scan: Optional[LaserScan] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.info_topic, self.intrinsics_cb, SENSOR_QOS)
        self.create_subscription(Image, self.image_topic, self._image_cb, SENSOR_QOS)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, SENSOR_QOS)

        self.pub_raw = self.create_publisher(Hazard, '/hazards/raw', 20)
        self.pub_markers = self.create_publisher(MarkerArray, '/hazards/raw/markers', 10)

        self.get_logger().info(
            f"HazardDetector ready (backend={backend}, image={self.image_topic}, "
            f"scan={self.scan_topic}, lidar_frame={self.lidar_frame})"
        )
        self.declare_parameter(
            'bounds_polygon',
            # [0.0, 0.0, 4.0, 0.0, 4.0, 1.5, 0.0, 1.5],
            # [0.0, 0.0, 0.5, 0.0, 0.5, 0.5, 0.0, 0.5],
            [-4.0, -4.0, 4.0, -4.0, 4.0, 4.0, -4.0, 4.0],
            #  [-0.5, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5, 0.5],
        )

        flat = list(self.get_parameter('bounds_polygon').value)
        if len(flat) < 6 or len(flat) % 2 != 0:
            raise ValueError("not correct polygon pts formarat")

        verts = list(zip(flat[0::2], flat[1::2]))
        self.polygon = Polygon(verts)

    # ------------------------------------------------------------------ callbacks brebv
   
    def _load_config(self) -> dict:
        path = self.get_parameter('params_file').value
        if not path:
            return {}
        try:
            with open(path) as file:
                return yaml.safe_load(file) or {}
        except Exception:
            print("failed darn tootin")
            return {}

    def intrinsics_cb(self, msg: CameraInfo):
        if self.intrinsics is None:
            self.intrinsics = np.array(msg.k).reshape(3, 3)
            self.image_size = (msg.width, msg.height)
            k = self.intrinsics
            print(f"Camera intrinsics {k}")

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    
    def _image_cb(self, msg: Image):
    #   image hsv processing brev
        if self.intrinsics is None or self.latest_scan is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            return


        detections = self.detector.detect(frame)
        if not detections:
            return

        scan = self.latest_scan
        img_ns = Time.from_msg(msg.header.stamp).nanoseconds
        scan_ns = Time.from_msg(scan.header.stamp).nanoseconds
        differ_s = abs(img_ns - scan_ns) / 1e9

        if self.max_scan_age < differ_s:
            print("there is too much diff")
            return

        markers = MarkerArray()
        map_points = []

        dets = [d for d in detections if d is not None]

        detmaps = self._fuse_to_map2(dets, scan, msg.header.stamp)
        if detmaps is None:
            return

        for idx, (det,map_point) in enumerate(detmaps):
            if map_point is None:
                continue

            hazard = Hazard()
            hazard.header.stamp = msg.header.stamp
            hazard.header.frame_id = self.map_frame
            hazard.id = -1
            hazard.color = det.label
            hazard.category = self.color_to_category.get(det.label, 'unknown')
            hazard.position = map_point
            hazard.confidence = det.confidence
            self.pub_raw.publish(hazard)

            markers.markers.append(self._make_marker(idx, hazard, msg.header.stamp))


        # for idx, det in enumerate(detections):
        #     map_point = self._fuse_to_map(det, scan, msg.header.stamp)
        #     if map_point is None:
        #         continue

        #     hazard = Hazard()
        #     hazard.header.stamp = msg.header.stamp
        #     hazard.header.frame_id = self.map_frame
        #     hazard.id = -1
        #     hazard.color = det.label
        #     hazard.category = self.color_to_category.get(det.label, 'unknown')
        #     hazard.position = map_point
        #     hazard.confidence = det.confidence
        #     self.pub_raw.publish(hazard)

        #     markers.markers.append(self._make_marker(idx, hazard, msg.header.stamp))

        if markers.markers:
            self.pub_markers.publish(markers)

    # ALGORITHM stuff brev (TODO)
  

    def _fuse_to_map(self, det: Detection, scan: LaserScan, stamp) -> Optional[Point]:

        """Top-level: HSV detection + LiDAR scan -> 3D point in map frame.
        """

        det_bearings = self._detection_bearing(det)
        if det_bearings is None:
            return None

        thresh = math.pi / 2.0
        clusters = self._cluster_scan(scan, -thresh, +thresh)
        if not clusters:
            return None

        cluster = self._match_cluster(det_bearings, clusters)      
        if cluster is None:
            return None

        r = cluster.range
        alpha = cluster.bearing

        x = r * math.cos(alpha)
        y= r * math.sin(alpha)
        z = 0.0
        
        map_point = self._lidar_to_map(x, y, z, scan.header.stamp)

        return map_point





    def _fuse_to_map2(self, dets: List[Detection], scan: LaserScan, stamp) -> Optional[List[Tuple[Detection, Point]]]:

        """Top-level: HSV detection + LiDAR scan -> 3D point in map frame.
        """
        det_bearings = []
        det_list = []

        for det in dets:
            bearings = self._detection_bearing(det)
            if bearings is None:
                continue
            
            det_bearings.append(bearings)
            det_list.append(det)


        thresh = math.pi / 2.0
        clusters = self._cluster_scan(scan, -thresh, +thresh)
        if not clusters:
            return None

        clusters = self._match_cluster2(det_bearings, clusters)      
        if clusters is None:
            return None

        map_points = []

        for cluster in clusters:
            if cluster is None:
                map_points.append(None)
                continue
            r = cluster.range
            alpha = cluster.bearing
            x = r * math.cos(alpha)
            y= r * math.sin(alpha)
            z = 0.0
            map_point = self._lidar_to_map(x, y, z, scan.header.stamp)
            map_points.append(map_point)

        return zip(det_list, map_points)


    def _detection_bearing(self, det: Detection) -> Optional[Tuple[float, float, float]]:
        
        """TODO: Map detection bbox (image pixels) -> bearing range (lidar frame).
        Hints:
            1. Convert each pixel column X to a *camera-frame*
               bearing using the pinhole model:
            2. Convert each camera bearing to a *lidar-frame* bearing.
               On the TB3 burger, the camera principal axis points
               forward (+x in the robot frame) and so does the lidar's
               zero ray. Lidar yaw is positive CCW (left), opposite of
               camera u. So the conversion is just sign flip:
                   bearing_lidar = -bearing_cam
            3. return left, center, right bearings in lidar frame.
        """

        left_pixel = det.cx - det.w / 2
        center_pixel = det.cx
        right_pixel = det.cx + det.w / 2
        cx = self.intrinsics[0, 2]
        fx = self.intrinsics[0, 0]

        if left_pixel < 0 or right_pixel > self.image_size[0]:
            return None
            
        bearing_cam_left = math.atan2(left_pixel - cx, fx)
        bearing_cam_right = math.atan2(right_pixel - cx, fx)
        bearing_cam_center = math.atan2(center_pixel - cx, fx)

        bearing_lidar_left = -bearing_cam_left
        bearing_lidar_right = -bearing_cam_right
        bearing_lidar_center = -bearing_cam_center

        return (bearing_lidar_left, bearing_lidar_center, bearing_lidar_right)



    def convert_360_to_pi(self,angle:float) -> float:
        if angle > math.pi:
            angle -= 2 * math.pi
        return angle



    def _cluster_scan(self, scan: LaserScan, bearing_lo: float, bearing_hi: float,) -> List[_ScanCluster]:

        """TODO:Segment the LaserScan inside [bearing_lo, bearing_hi] by depth jumps.
        """

        lo = bearing_lo
        hi = bearing_hi

        
        num_rays = len(scan.ranges)
        if num_rays == 0:
            return []

        lo_index = int(lo/scan.angle_increment + num_rays)
        hi_index = int(hi/scan.angle_increment)

        adapted_ranges =[]
        adapted_bearings =[]


        for i in range(lo_index, num_rays):
            adapted_ranges.append(scan.ranges[i])
            adapted_bearings.append(self.convert_360_to_pi(scan.angle_min + i * scan.angle_increment))
        for i in range(0, hi_index):
            adapted_ranges.append(scan.ranges[i])
            adapted_bearings.append(self.convert_360_to_pi(scan.angle_min + i * scan.angle_increment))


        # scan_1 = adapted_ranges[0]
        # bearing_1 = adapted_bearings[0]
        clusters: List[_ScanCluster] = []
        current = None
        
        prev_range = None

        for r, bearing in zip(adapted_ranges, adapted_bearings):
            if not math.isfinite(r) or r == 0.0 or r == None:
                # self.get_logger().info("invalid range")
                continue
            if r > self.max_range:
                # self.get_logger().info("range is too large")
                clusters.append(current)
                current = _ScanCluster(bearing=bearing, range=r, bearing_min=bearing, bearing_max=bearing, n_rays=1)
                prev_range = r
                continue
            if current is None or prev_range is None:
                # self.get_logger().info("current is None or prev_range is None,new cluster`")
                prev_range = r
                current = _ScanCluster(bearing=bearing, range=r, bearing_min=bearing, bearing_max=bearing, n_rays=1)
                continue
            if abs(r - prev_range) > self.depth_jump:
                # self.get_logger().info("depth jump is too large,new cluster")
                clusters.append(current)
                prev_range = r
                current = _ScanCluster(bearing=bearing, range=r, bearing_min=bearing, bearing_max=bearing, n_rays=1)
                continue
            current = self._update_cluster(current, r, bearing)
            prev_range = r
        
        if current is not None:
            clusters.append(current)


        retval = []

        cluster_cluster = []
        prev_cluster = None

        for c in clusters:
            if c is None:
                continue
            if prev_cluster is None:
                prev_cluster = c
                continue

            r_cur = c.range
            alpha_cur = c.bearing

            x_cur = r_cur * math.cos(alpha_cur)
            y_cur = r_cur * math.sin(alpha_cur)
            z_cur = 0.0

            r_prev = prev_cluster.range
            alpha_prev = prev_cluster.bearing

            x_prev = r_prev * math.cos(alpha_prev)
            y_prev = r_prev * math.sin(alpha_prev)
            z_prev = 0.0

            diff = ((x_cur - x_prev)**2 + (y_cur - y_prev)**2)**0.5
            if diff > self.merger_tol:
                cluster_cluster.append(prev_cluster)
                prev_cluster = c
                continue

            prev_cluster = self._merge_clusters(prev_cluster, c)
        
        if prev_cluster is not None:
            cluster_cluster.append(prev_cluster)

            


        for c in cluster_cluster:
            if c is None:
                continue
            
            r = c.range
            alpha = c.bearing

            x = r * math.cos(alpha)
            y= r * math.sin(alpha)
            z = 0.0

            map_point = self._lidar_to_map(x, y, z, scan.header.stamp)

            if map_point is None:
                # self.get_logger().info("map point is None")
                continue
            x ,y ,z= map_point.x, map_point.y, map_point.z

            if not self.polygon.contains(ShapelyPoint(x, y)):
                # self.get_logger().info("point is not in polygon")
                continue

            if c.range>self.max_range:
                # self.get_logger().info("range is too large")
                continue
            if self.min_rays > c.n_rays:
                # self.get_logger().info("number of rays is too small")
                continue
            width = 2*c.range*math.sin(abs(c.bearing_max - c.bearing_min)/2)
            if width > self.max_cluster_width:
                # self.get_logger().info("width is too large")
                continue

            retval.append(c)
        return retval


        
    
    def _update_cluster(self, current: _ScanCluster, r: float, bearing: float) -> _ScanCluster:

        current.range = (current.range * current.n_rays + r) / (current.n_rays + 1)
        current.bearing = (current.bearing * current.n_rays + bearing) / (current.n_rays + 1)
        current.bearing_min = min(current.bearing_min, bearing)
        current.bearing_max = max(current.bearing_max, bearing)
        current.n_rays += 1
        return current
    
    def _merge_clusters(self, a: _ScanCluster, b: _ScanCluster) -> _ScanCluster:

        a_min = a.bearing_min
        a_max = a.bearing_max
        a_range = a.range
        a_n_rays = a.n_rays
        a_bearing = a.bearing
        b_min = b.bearing_min
        b_max = b.bearing_max
        b_range = b.range
        b_n_rays = b.n_rays
        b_bearing = b.bearing


        new_min = min(a_min, b_min)
        new_max = max(a_max, b_max)
        new_range = (a_range * a_n_rays + b_range * b_n_rays) / (a_n_rays + b_n_rays)
        new_n_rays = a_n_rays + b_n_rays
        new_bearing = (a_bearing * a_n_rays + b_bearing * b_n_rays) / (a_n_rays + b_n_rays)

        return _ScanCluster(bearing=new_bearing, range=new_range, bearing_min=new_min, bearing_max=new_max, n_rays=new_n_rays)
        



    def _match_cluster(self,det_bearings: Tuple[float, float, float],clusters: List[_ScanCluster], ) -> Optional[_ScanCluster]:
        # TODO: 

        middle = det_bearings[1]
        retval = None
        min_dist = self.match_tol

        for c in clusters:
            dist = abs(c.bearing - middle)
            if dist < min_dist:
                retval = c
                min_dist = dist  

        if retval is None:
            self.get_logger().info("no cluster found")
            
        return retval






    def _match_cluster2(self,det_bearings: List[Tuple[float, float, float]], clusters: List[_ScanCluster], ) -> Optional[List[_ScanCluster]]:
        # at this stage the det_breaing is compelte and not None, so need for each detection we start iwht our scan_clsuter is none and asing the corect asisgemtns fo scna clusters to the detectiosn 
        # note that not all sacna clsuters need be asigned 

       

        retval = [None] * len(det_bearings)

        score_array = [[None for i in range(len(clusters) + 1)] for j in range(len(det_bearings))]

        for i in range(len(det_bearings)):
            for j in range(len(clusters)):
                score_array[i][j] = self._score_cluster(det_bearings[i], clusters[j])
        
        for i in range(len(det_bearings)):
            score_array[i][len(clusters)] = 5/360 * math.pi

        for i in range(len(det_bearings)):
            index = score_array[i].index(min(score_array[i]))
            if index < len(clusters):
                retval[i] = clusters[index]
                print(retval[i].bearing, det_bearings[i][1])
            else:
                retval[i] = None

        return retval


    # PLUMBING stuff brev (DONZO washington)



    def _score_cluster(self, det_bearing: Tuple[float, float, float], cluster: _ScanCluster) -> float:
        return abs(det_bearing[1] - cluster.bearing)


    def _lidar_to_map(self, x: float, y: float, z: float, stamp) -> Optional[Point]:
        """Transform a point from self.lidar_frame to self.map_frame.
        """
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.lidar_frame, Time(),timeout=Duration(seconds=0.3), )
        except Exception:
            return None

        q = tf.transform.rotation
        R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

        t = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z])

        p = R @ np.array([x, y, z]) + t

        retx = float(p[0])
        rety = float(p[1])
        retz = float(p[2])

        return Point(x=retx, y=rety, z=retz)


    def _make_marker(self, idx: int, hz: Hazard, stamp) -> Marker:
        """RViz SPHERE marker at hz.position, coloured by hz.color."""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = 'hazards_raw'
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = hz.position
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.lifetime.sec = 1
        marker.color = _COLOR_TO_RGBA.get(
            hz.color, ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        )
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = HazardDetector()
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
