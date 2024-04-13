import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.time import Duration
import numpy as np
import math
from typing import List

import warnings
warnings.simplefilter('ignore', np.RankWarning)

# Msg
import rclpy.time
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
# Srv
from example_interfaces.srv import Trigger
# Tf
from tf2_ros import TransformBroadcaster, TransformListener
from tf2_ros.buffer import Buffer

# Util
import util
from util import LineSegment
from config import get_config
cfg = get_config()

CONST_90_DEG_RADIANS = math.radians(90)

class GRFNode(Node):
    def __init__(self):
        super().__init__('GRF_node')

        self.points = None
        self.dist_ahead = None

        self.timer = self.create_timer(0.2, self.find_walls)
        self.x_add = 0.0

        # Publishers
        self.wall_pub = self.create_publisher(Marker, "/arena/walls", 20)
        self.wall_segment_pub = self.create_publisher(Marker, "/arena/wall_segments", 20)
        self.corner_pub = self.create_publisher(Marker, "/arena/corners", 20)

        # Subscribers
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        # self.clock_sub = self.create_subscription()

        # Services
        self.grf_init_srv = self.create_service(Trigger, 'init_grf', self.init_grf)

        # TF
        self.create_timer(0.03, self.publish_transform)
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=1.0))
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.odom_data = None

    def init_grf(self, req: Trigger.Request, resp: Trigger.Response):
        resp.message = "Hello from grf node!"
        resp.success = True
        return resp
    
    def publish_transform(self):
        t = TransformStamped()

        try:
            odom: TransformStamped = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
            odom_data = odom.transform
        except:
            self.get_logger().info("Could not get transform from odom to base_link")
            return
        
        if odom_data is None: return
        t_odom = odom_data

        # if self.odom_data is None: return
        # t_odom = self.odom_data
        # t.transform.rotation.x = 0.0
        # t.transform.rotation.y = 0.0
        # t.transform.rotation.z = t_odom.orientation.z
        # t.transform.rotation.w = t_odom.orientation.w * -1
        # t.transform.translation.x = -t_odom.position.x
        # t.transform.translation.y = t_odom.position.y
        # t.transform.translation.z = -t_odom.position.z

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = t_odom.rotation.z * -1
        t.transform.rotation.w = t_odom.rotation.w * -1

        t.transform.translation.x = t_odom.translation.x
        t.transform.translation.y = t_odom.translation.y
        t.transform.translation.z = t_odom.translation.z

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'arena'

        self.tf_broadcaster.sendTransform(t)
    
    def odom_callback(self, data: Odometry):
        self.odom_data = data.pose.pose

    def scan_callback(self, data: LaserScan):
        if data is not None:
            self.dist_ahead = data.ranges[len(data.ranges)//2]
            self.points = util.scan_to_cartesian_points(data.ranges, cfg.LIDAR_MIN_ANGLE, cfg.LIDAR_MAX_ANGLE)

    def fit_line(self, points, remove_outliers=True):
        x = np.array([point[0] for point in points])
        y = np.array([point[1] for point in points])

        # Least squares regression cannot fit a line that is vertical well
        # Therefor check the standard deviation of the points to check if the line to fit would be near vertical
        # If it is swap x and y axis so least squares can fit the line
        # Equations from Justas https://stats.stackexchange.com/questions/57685/line-of-best-fit-linear-regression-over-vertical-line
        swapped = False
        x_std = np.std(x)
        y_std = np.std(y)

        # Check if verticality is > 1.5, specific value not hugely important
        if (y_std/x_std > 1.5):
            tmp = x
            x = y
            y = tmp
            swapped = True

        # Fit 1st degree polynomial (straight line) using least squares method
        slope, intercept = np.polyfit(x, y, 1)

        if remove_outliers:
            # Calculate distances for each index
            distances = []
            seg = LineSegment(slope, intercept, 0,0)
            for i in range(len(x)):
                distances.append(seg.distance_from_line(x[i], y[i]))
            
            q75 = np.percentile(distances, [75])
            
            valid_idxs = []
            # Check if data point is an outlier, if not add to valid_idxs
            for idx, dist in enumerate(distances):
                if dist - cfg.SCAN_POINT_OUTLIER_THRESH < q75:
                    valid_idxs.append(idx)
            
            # Check if there are any outliers
            if len(valid_idxs) != len(x) and len(valid_idxs) != 0:
                # Remove outliers from data
                x_1 = x[valid_idxs]
                y_1 = y[valid_idxs]
                
                # After removing outliers, check verticality again
                x_std = np.std(x_1)
                y_std = np.std(y_1)

                # Check if verticality is > 1.5, specific value not hugely important
                if (y_std/x_std > 1.5):
                    tmp = x_1
                    x_1 = y_1
                    y_1 = tmp
                    swapped = not swapped
                
                # Recalculate line of best fit
                # self.get_logger().info(f"Len x_1: {len(x_1)}")
                slope, intercept = np.polyfit(x_1, y_1, 1)

        if swapped:
            intercept = -intercept / slope
            slope = 1/slope

        return slope, intercept
    
    def find_walls(self):
        if self.points is None: return
        points = self.points

        # Remove points if they are too close together, helps remove noise from lidar scan
        tmp_points = []
        prev_point = None
        for point in points:
            if prev_point is None:
                tmp_points.append(point)
                prev_point = point
                continue

            dx = prev_point[0] - point[0]
            dy = prev_point[1] - point[1]
            d = math.sqrt(dx**2 + dy**2)

            if d < cfg.SCAN_POINT_MAX_DENSITY: continue
            prev_point = point
            tmp_points.append(point)
        
        points = tmp_points

        # Calculate line segments from lidar scan
        segments = []
        segment_start_idx = 0
        while segment_start_idx < len(points):
            # If end reached, next segment contains all points to the end
            if len(points) - segment_start_idx < cfg.SCAN_LINE_SEGMENT_LENGTH:
                segment = points[segment_start_idx:]
            else: # Segment contains specified number of points
                segment = points[segment_start_idx:segment_start_idx + cfg.SCAN_LINE_SEGMENT_LENGTH + 1]

            slope, intercept = self.fit_line(segment, remove_outliers=False)
            start_idx = segment_start_idx

            segments.append(LineSegment(slope, intercept, start_idx, len(segment) + start_idx - 1))
            segment_start_idx += cfg.SCAN_LINE_SEGMENT_LENGTH

        joined_segments: List[LineSegment] = []
        prev_seg = None
        prev_segment_joined_flag = False
        # Loop over segments, compare previous and current segment and decide whether they should be joined
        for idx, seg in enumerate(segments):
                if prev_seg == None: 
                    prev_seg = seg
                    continue
                
                # If segments have similar slope and end/start within threshold, join segments
                if (abs((seg.slope - prev_seg.slope) / (1 + seg.slope * prev_seg.slope)) <= cfg.LINE_SLOPE_JOIN_THRESH and 
                    abs(points[prev_seg.end_idx][0] - points[seg.start_idx][0]) + abs(points[prev_seg.end_idx][1] - points[seg.start_idx][1]) <= cfg.LINE_GAP_JOIN_THRESH):
                    prev_segment_joined_flag = True
                    # Create new line segment from points encapsulated by previous and current segments
                    new_segment = points[prev_seg.start_idx:seg.end_idx]
                    slope, intercept = self.fit_line(new_segment)

                    new_line = LineSegment(slope, intercept, prev_seg.start_idx, seg.end_idx)
                    prev_seg = new_line
                    # If segment is last in the loop add it to the list, won't be added by else statement below
                    if idx == len(segments)-1: joined_segments.append(prev_seg)
                else:
                    # If the previous segment was joined with another segment, add it to the list
                    if prev_segment_joined_flag:
                        joined_segments.append(prev_seg)
                        prev_segment_joined_flag = False
                    # else discard previous line
                    prev_seg = seg
        
        corner_points: List[Point] = []
        for seg in joined_segments:
            for seg2 in joined_segments:
                if seg == seg2: continue
                angle = abs(math.atan((seg.slope - seg2.slope) / (1 + seg.slope * seg2.slope)))
                if (abs(angle - CONST_90_DEG_RADIANS) < cfg.LINE_CORNER_ANGLE_THRESH):
                    x, y = seg.get_line_intersection(seg2)
                    exist_flag = False
                    for point in corner_points:
                        if point.x == x and point.y == y:
                            exist_flag = True
                            break
                    if not exist_flag:
                        corner_points.append(util.makePoint(x, y, 0.5))
    
        ## Visualize walls in rviz
        marker_points = []
        for seg in joined_segments:
            # Map last point covered by segment to closest point on segment line
            start_x, start_y = seg.closest_point_on_line(np.array([points[seg.start_idx][0], points[seg.start_idx][1]]))
            end_x, end_y = seg.closest_point_on_line(np.array([points[seg.end_idx - 1][0], points[seg.end_idx - 1][1]]))

            marker_points.append(util.makePoint(start_x, start_y, 0.1)) # Start point
            marker_points.append(util.makePoint(end_x, end_y, 0.1)) # End point

        util.Rviz.visualize_points(marker_points, Marker.LINE_LIST, self.wall_pub, 
                                            util.Rviz.LASER_LINK, self.get_clock().now(), 0.05, util.Colors.CYAN_TSP, 'walls')

        marker_points = []    
        for seg in segments:
            # Map last point covered by segment to closest point on segment line
            start_x, start_y = seg.closest_point_on_line(np.array([points[seg.start_idx][0], points[seg.start_idx][1]]))
            end_x, end_y = seg.closest_point_on_line(np.array([points[seg.end_idx - 1][0], points[seg.end_idx - 1][1]]))

            marker_points.append(util.makePoint(start_x, start_y, 0.2)) # Start point
            marker_points.append(util.makePoint(end_x, end_y, 0.2)) # End point
        
        util.Rviz.visualize_points(marker_points, Marker.LINE_LIST, self.wall_segment_pub, 
                                            util.Rviz.LASER_LINK, self.get_clock().now(), 0.05, util.Colors.RED_TSP, 'wall_segments')

        util.Rviz.visualize_points(corner_points, Marker.SPHERE_LIST, self.corner_pub, 
                                            util.Rviz.LASER_LINK, self.get_clock().now(), 0.05, util.Colors.ORANGE, 'corners')
        ##

def main(args=None):
    rclpy.init(args=args)

    node = GRFNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()