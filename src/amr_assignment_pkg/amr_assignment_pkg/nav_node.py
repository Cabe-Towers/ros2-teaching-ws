import rclpy
from rclpy.node import Node
from rclpy.time import Duration
import numpy as np

import util
import config
cfg = config.Config()

# Message types
from geometry_msgs.msg import Twist, Point, Pose
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from nav_msgs.msg import MapMetaData

# Tf
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import PointStamped

class OccupancyCell():
    occupied = False

class OccupancyGrid():
    def __init__(self, grid_size, cell_size):
        self.grid_size = grid_size
        self.cell_size = cell_size

        self.cell_x_count = int(self.grid_size // self.cell_size + 1) # Add one for some extra padding
        self.cell_y_count = int(self.grid_size // self.cell_size + 1) # Add one for some extra padding
        self.cells = []

        self.init_cells()
    
    def init_cells(self):
        for x_idx in range(0, self.cell_x_count):
            row = []
            for y_idx in range(0, self.cell_y_count):
                row.append(OccupancyCell())
            self.cells.append(row)
    
    def clear_cells(self):
        for row in self.cells:
            for cell in row:
                cell.occupied = False

    def get_cell_idx_from_xy(self, x, y):
        row_idx = int(x / self.cell_size + self.cell_x_count / 2)
        col_idx = int(y / self.cell_size + self.cell_y_count / 2)

        if row_idx < self.cell_x_count - 1 and col_idx < self.cell_y_count - 1:
            return (row_idx, col_idx)
        return False
    
    def get_cell_occupied(self, x, y):
        idx = self.get_cell_idx_from_xy(x, y)
        if not idx:
            return False
        return self.cells[idx[0]][idx[1]].occupied
    
    def add_points_xylist(self, xy_list):
        for point in xy_list:
            idx = self.get_cell_idx_from_xy(point[0], point[1])
            if not idx: continue
            self.cells[idx[0]][idx[1]].occupied = True
    
    def add_points(self, points):
        for point in points:
            idx = self.get_cell_idx_from_xy(point.point.y, point.point.x) # X and Y are flipped for scan data
            if not idx: continue
            self.cells[idx[0]][idx[1]].occupied = True
                
    
    def publish_occupancy_grid(self, publisher, frame_id, stamp, node):
        
        # self.add_points([[0,0]])
        # self.add_points([[0.3,1]])
        # self.add_points([[1,1]])

        # node.get_logger().info(f"xy is {self.get_cell_idx_from_xy(0.3, 1)}")

        msg = OccupancyGridMsg()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp

        meta = MapMetaData()
        meta.height = int(self.cell_x_count)
        meta.width = int(self.cell_y_count)
        meta.resolution = self.grid_size / self.cell_x_count
        meta.map_load_time = stamp

        pose = Pose()
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        pose.position.x = self.grid_size / -2
        pose.position.y = self.grid_size / -2
        pose.position.z = 0.0

        meta.origin = pose
        msg.info = meta
        
        data = []
        idx = 0
        for row in self.cells:
            for cell in row:
                if cell.occupied:
                    data.append(100)
                else: data.append(0)
            idx += 1

        msg.data = data

        publisher.publish(msg)
        

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')

        self.occupancy_grid = OccupancyGrid(3.2, 0.1)
        
        # Publishers
        self.occupancy_map_pub = self.create_publisher(OccupancyGridMsg, '/occupancy_map', 20)
        self.create_timer(1.0, self.occupancy_grid_publish)

        # Subscribers
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def occupancy_grid_publish(self):
        self.occupancy_grid.publish_occupancy_grid(self.occupancy_map_pub, 'arena', self.get_clock().now().to_msg(), self)

    def scan_callback(self, data: LaserScan):
        xy_points = util.scan_to_cartesian_points(data.ranges, cfg.LIDAR_MIN_ANGLE, cfg.LIDAR_MAX_ANGLE)
        points = []
        try:
            for point in xy_points:
                point_stamped = PointStamped()
                point_stamped.point = util.makePoint(point[0], point[1])
                point_stamped.header.stamp = data.header.stamp
                point_stamped.header.frame_id = data.header.frame_id

                tf_point = self.tf_buffer.transform_full(point_stamped, 'arena', rclpy.time.Time(), data.header.frame_id)
                points.append(tf_point)
                
        except Exception as e:
            self.get_logger().info(f"Could not transform scan data to arena frame {e}")
            return
        
        self.occupancy_grid.add_points(points)


def main(args=None):
    rclpy.init(args=args)

    node = NavNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()