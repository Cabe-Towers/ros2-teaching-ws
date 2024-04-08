import rclpy
from rclpy.node import Node
from rclpy.time import Duration
import numpy as np
import math
import util
from util import LineSegment
import warnings
from config import get_config
cfg = get_config()

warnings.simplefilter('ignore', np.RankWarning)

# Msg
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class GRFNode(Node):
    def __init__(self):
        super().__init__('GRF_node')

        self.points = None
        self.dist_ahead = None

        self.timer = self.create_timer(1.0, self.find_walls)

        # Publishers
        self.wall_vis_pub = self.create_publisher(Marker, "/wall_vis", 20)
        self.wall_vis_pub_test = self.create_publisher(Marker, "/wall_vis_test", 20)

        # Subscribers
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, data: LaserScan):
        if data is not None:
            self.dist_ahead = data.ranges[len(data.ranges)//2]
            self.points = util.scan_to_cartesian_points(data.ranges, cfg.LIDAR_MIN_ANGLE, cfg.LIDAR_MAX_ANGLE)

    def fit_line(self, points):
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
        segment_num = 0
        while segment_num < len(points):
            # If end reached, next segment contains all points to the end
            if len(points) - segment_num < cfg.SCAN_LINE_SEGMENT_LENGTH:
                segment = points[segment_num:]
            else: # Segment contains specified number of points
                segment = points[segment_num:segment_num + cfg.SCAN_LINE_SEGMENT_LENGTH + 1]

            slope, intercept = self.fit_line([segment[0], segment[-1]])
            start_idx = segment_num

            # If at end of the list, set segment end to
            # if segment_num + cfg.SCAN_LINE_SEGMENT_LENGTH >= len(points):
            #     segment_num = len(points)
            # else:
            #     segment_num += cfg.SCAN_LINE_SEGMENT_LENGTH

            segments.append(LineSegment(slope, intercept, start_idx, len(segment) + start_idx))
            segment_num += cfg.SCAN_LINE_SEGMENT_LENGTH

        joined_segments = []
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

        ## Visualize walls in rviz
        marker_points = []    
        for seg in joined_segments:
            # Map last point covered by segment to closest point on segment line
            start_x, start_y = seg.closest_point_on_line(np.array([points[seg.start_idx][0], points[seg.start_idx][1]]))
            end_x, end_y = seg.closest_point_on_line(np.array([points[seg.end_idx - 1][0], points[seg.end_idx - 1][1]]))

            marker_points.append([
            util.makePoint(start_x, start_y), # Start point
            util.makePoint(end_x, end_y), # End point
            ])

        self.visualize_segments(marker_points)

        marker_points = []    
        for seg in segments:
            # Map last point covered by segment to closest point on segment line
            start_x, start_y = seg.closest_point_on_line(np.array([points[seg.start_idx][0], points[seg.start_idx][1]]))
            end_x, end_y = seg.closest_point_on_line(np.array([points[seg.end_idx - 1][0], points[seg.end_idx - 1][1]]))

            marker_points.append([
            util.makePoint(start_x, start_y, 1.0), # Start point
            util.makePoint(end_x, end_y, 1.0), # End point
            ])
        
        self.visualize_segments(marker_points, rgba=(1.0, 0.0, 0.0, 0.5), scale=0.02, publisher=self.wall_vis_pub_test)

        ##
            
    def visualize_segments(self, segment_points, rgba = (0.0, 1.0, 0.0, 0.5), scale = 0.05, publisher = None):
        mk = Marker()
        mk.header.frame_id = '/laser_link'
        mk.id = 0
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = "walls"
        mk.action = Marker.ADD
        mk.lifetime = Duration(seconds=1).to_msg()

        mk.type = Marker.LINE_LIST
        mk.scale.x = scale

        mk.color.r = rgba[0]
        mk.color.g = rgba[1]
        mk.color.b = rgba[2]
        mk.color.a = rgba[3]

        for line in segment_points:
            mk.points.append(line[0])
            mk.points.append(line[1])
        
        if publisher is None: self.wall_vis_pub.publish(mk)
        else: publisher.publish(mk)


def main(args=None):
    rclpy.init(args=args)

    node = GRFNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()