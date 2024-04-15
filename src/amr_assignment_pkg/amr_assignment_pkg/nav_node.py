import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.time import Duration
import numpy as np
import math
from scipy.spatial.transform import Rotation
from simple_pid import PID

import util
from pathfinding import a_star_search
import config
cfg = config.Config()

# Message types
from geometry_msgs.msg import Twist, Point, Pose, Transform
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from nav_msgs.msg import MapMetaData

from amr_interfaces.srv import GetBoxLocations, Int16

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

    # A* search path from src to dest cells (row, col)
    def path_find(self, src, dest, node):
        # Construct grid to use for path finding
        grid = []
        for row in self.cells:
            grid_row = []
            for cell in row:
                # TODO: add padding on search grid
                grid_row.append(cell.occupied)
            grid.append(grid_row)

        waypoints = a_star_search(grid, src, dest, node)
        node.get_logger().info(f"Len waypoints is {len(waypoints)}")
        if waypoints == []: return False
        waypoint_points = []
        for row_col in waypoints:
            x, y = self.get_xy_from_cell_idx(row_col[0], row_col[1])
            waypoint_points.append(util.makePoint(x, y))
        return waypoint_points
    
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
    
    def get_xy_from_cell_idx(self, row, col):
        x = self.cell_size * (row - self.cell_x_count / 2)
        y = self.cell_size * (col - self.cell_y_count / 2)
        return x, y
    
    def get_cell_occupied(self, x, y):
        idx = self.get_cell_idx_from_xy(x, y)
        if not idx:
            return False
        return self.cells[idx[0]][idx[1]].occupied
    
    def add_points(self, points):
        for point in points:
            idx = self.get_cell_idx_from_xy(point.x, point.y) # Points are reversed for some reason
            if not idx: continue
            self.cells[idx[0]][idx[1]].occupied = True

    def add_points_with_transform(self, points, tf_buffer: Buffer, frame_id, target_frame_id):
        tf_points = []
        try:
            for point in points:
                point_stamped = PointStamped()
                point_stamped.point = point
                point_stamped.header.frame_id = frame_id
                tf_point = tf_buffer.transform_full(point_stamped, target_frame_id, rclpy.time.Time(), frame_id)
                tf_points.append(tf_point.point)
        except Exception as e:
            self.get_logger().info(f"Occupancy grid could not transform point data: {e}")
        
        self.add_points(tf_points)
    
    def publish_occupancy_grid(self, publisher, frame_id, stamp, node):
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
        for col_id in range(0, self.cell_y_count):
            for row_id in range(0, self.cell_x_count):
                if self.cells[row_id][col_id].occupied:
                    data.append(100)
                else: data.append(0)

        msg.data = data

        publisher.publish(msg)
        

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')
        self.occupancy_grid = OccupancyGrid(3.2, 0.05)

        self.heading_pid = PID(2.8, 0, 0, setpoint=0, output_limits=(-0.6, 0.6))
        self.heading_set = math.radians(45)

        # Timers
        self.occupancy_grid_cb_group = ReentrantCallbackGroup()
        self.create_timer(1.0, self.occupancy_grid_publish, callback_group=self.occupancy_grid_cb_group)
        self.create_timer(0.1, self.run_turn_to_heading)
        self.create_timer(2.0, self.test_waypoint)
        
        # Publishers
        self.occupancy_map_pub = self.create_publisher(OccupancyGridMsg, '/occupancy_map', 20)
        self.control_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.waypoint_pub = self.create_publisher(Marker, '/nav/waypoints', 20)

        # Subscribers
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)


        # Services
        self.set_heading_srv = self.create_service(Int16, '/nav/set_heading', self.set_heading_srv_cb)

        # Clients
        self.depth_callback_group = ReentrantCallbackGroup()
        self.get_box_locations_client = self.create_client(GetBoxLocations, '/depth/get_box_locations', callback_group=self.depth_callback_group)

        # Tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        while not self.get_box_locations_client.wait_for_service(timeout_sec=1): self.get_logger().info("Waiting for depth box service")

    def test_waypoint(self):
        
        points = self.occupancy_grid.path_find(self.occupancy_grid.get_cell_idx_from_xy(-1, 0), self.occupancy_grid.get_cell_idx_from_xy(1, 0), self)
        if not points:
            self.get_logger().info("No path")
            print("No path!")
            return
        util.Rviz.visualize_points(points, Marker.CUBE_LIST, self.waypoint_pub, util.Rviz.ARENA, self.get_clock().now(), 0.1, util.Colors.MAGENTA, 'waypoints')

    def run_turn_to_heading(self):
        msg = Twist()
        current_heading = self.get_robot_heading()
        error = self.heading_set - current_heading
        # if self.get_turn_direction(current_heading, self.heading_set): error = -error
        if error > math.pi:
            error -= 2 * math.pi
        elif error < -math.pi:
            error += 2 * math.pi
        
        vel = self.heading_pid(error)
        # self.get_logger().info(f"Error is : {math.degrees(error)} - Vel is : {vel} - Current Heading : {math.degrees(current_heading)}")
        # msg.angular.z = (vel if self.get_turn_direction(current_heading, self.heading_set) else -vel)
        msg.angular.z = vel
        self.control_pub.publish(msg)

    def set_heading_srv_cb(self, req: Int16.Request, resp):
        self.heading_set = req.req_int
        return
    
    # Returns turn direction for a given heading and current heading true = right (cw), false = left (anti-cw)
    def get_turn_direction(self, heading, target_heading):
        # Mod accounts for wrapping around the circle
        cw_dist = (target_heading - heading) % (2 * math.pi)
        acw_dist = (heading - target_heading) % (2 * math.pi)
        return cw_dist < acw_dist

    # Returns the current robot heading in radians
    def get_robot_heading(self):
        t: Transform|bool = self.get_base_link_transform()
        if t == False: return self.heading_set
        q = t.rotation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        heading = -rot.as_euler('xyz', False)[2]
        if heading < 0:
            heading += math.radians(360) # Convert from -180-0-180 to 0-360 format

        return heading

    # Gets the transform from the base link frame to the arena frame
    def get_base_link_transform(self):
        try:
            t = self.tf_buffer.lookup_transform('arena', 'base_link', rclpy.time.Time())
        except:
            self.get_logger().info("Could not get transform from 'base_link' to 'arena'")
            return False
        return t.transform
    
    def occupancy_grid_publish(self):
        req = GetBoxLocations.Request()
        self.depth_future = self.get_box_locations_client.call_async(req)
        while not self.depth_future.done(): # Multi-threading goodness
            self.get_clock().sleep_for(Duration(seconds=0.01))
        resp: GetBoxLocations.Response = self.depth_future.result()

        self.occupancy_grid.add_points_with_transform(resp.red_boxes, self.tf_buffer, resp.frame_id, 'arena')
        self.occupancy_grid.add_points_with_transform(resp.green_boxes, self.tf_buffer, resp.frame_id, 'arena')
        self.occupancy_grid.publish_occupancy_grid(self.occupancy_map_pub, 'arena', self.get_clock().now().to_msg(), self)
        return

    def scan_callback(self, data: LaserScan):
        xy_points = util.scan_to_cartesian_points(data.ranges, cfg.LIDAR_MIN_ANGLE, cfg.LIDAR_MAX_ANGLE)
        points = []
        try:
            for point in xy_points: # This is dumb
                point_stamped = PointStamped()
                point_stamped.point = util.makePoint(point[0], point[1]) # X and Y are reversed, probably bad transform
                point_stamped.header.frame_id = data.header.frame_id
                tf_point = self.tf_buffer.transform_full(point_stamped, 'arena', rclpy.time.Time(), data.header.frame_id)

                points.append(tf_point.point)
                
        except Exception as e:
            self.get_logger().info(f"Could not transform scan data to arena frame {e}")
            return
        
        self.occupancy_grid.add_points(points)


def main(args=None):
    rclpy.init(args=args)

    node = NavNode()
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()