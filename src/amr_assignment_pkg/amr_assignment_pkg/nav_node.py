import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.action import ActionServer
import numpy as np
import math
from scipy.spatial.transform import Rotation
from simple_pid import PID

import rclpy.time
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
from amr_interfaces.action import NavHeading, NavLocation

# Tf
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import PointStamped

class OccupancyCell():
    occupied = False
    data_source = None


class OccupancyGrid():
    def __init__(self, grid_size, cell_size, inflate_size=2):
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.inflate_size = inflate_size
        self.discard_once = False

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
        inflate_offsets = self.generate_inflate_offsets(self.inflate_size)
        grid = []
        for row_idx, row in enumerate(self.cells):
            grid_row = []
            for col_idx, cell in enumerate(row):
                if cell.occupied:
                    grid_row.append(True)
                else:
                    inflate_occupied = False
                    for offset in inflate_offsets:
                        if not self.check_row_col_valid(row_idx + offset[0], col_idx + offset[1]): continue
                        # node.get_logger().info(f"RowCol: {row_idx + offset[0]}, {col_idx + offset[1]} XY count {self.cell_x_count}, {self.cell_y_count}")
                        if self.cells[row_idx + offset[0]][col_idx + offset[1]].occupied:
                            inflate_occupied = True
                            break
                    grid_row.append(inflate_occupied)
            grid.append(grid_row)

        waypoints = a_star_search(grid, src, dest, node)
        # node.get_logger().info(f"Len waypoints is {len(waypoints)}")
        if waypoints == []: return False
        waypoint_points = []
        for row_col in waypoints:
            x, y = self.get_xy_from_cell_idx(row_col[0], row_col[1])
            waypoint_points.append(util.makePoint(x, y))
        return waypoint_points
    
    def generate_inflate_offsets(self, inflate_amount):
        offsets = []
        for row in range(-inflate_amount, inflate_amount + 1):
            for col in range(-inflate_amount, inflate_amount + 1):
                if row == 0 and col == 0: continue
                offsets.append([row, col])
        return offsets

    
    def clear_cells(self):
        for row in self.cells:
            for cell in row:
                cell.occupied = False
    
    def discard_old_data(self):
        self.discard_once = True

    def check_row_col_valid(self, row, col):
        return (row >= 0) and (row < self.cell_x_count) and (col >= 0) and (col < self.cell_y_count)

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
        if self.discard_once: 
            self.discard_once = False
            self.clear_cells()
        for point in points:
            idx = self.get_cell_idx_from_xy(point.x, point.y)
            if not idx: continue
            self.cells[idx[0]][idx[1]].occupied = True

    def add_points_with_transform(self, points, tf_buffer: Buffer, frame_id, target_frame_id, stamp):
        tf_points = []
        for point in points:
            point_stamped = PointStamped()
            point_stamped.point = point
            point_stamped.header.frame_id = frame_id
            tf_point = tf_buffer.transform_full(point_stamped, target_frame_id, stamp, frame_id)
            tf_points.append(tf_point.point)
        
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
        self.occupancy_grid = OccupancyGrid(3.2, 0.05, 3)

        self.heading_pid = PID(2.8, 0, 0, setpoint=0, output_limits=(-cfg.MAX_ANGULAR_VELOCITY, cfg.MAX_ANGULAR_VELOCITY))
        self.linear_pid = PID(2.8, 0, 0, setpoint=0, output_limits=(-cfg.MAX_LINEAR_VELOCITY, cfg.MAX_LINEAR_VELOCITY))

        self.heading_set = False
        self.next_waypoint: Point = False
        self.goal_point: Point = False

        # State
        self.no_path_flag = False
        self.heading_accept = False
        self.waypoint_hit = False
        self.follow_waypoint = False
        self.allow_rgbd_data = False

        # Data
        self.last_scan_points = None
        self.last_scan_stamp = None
        self.scan_future = None

        # Timers
        self.occupancy_grid_cb_group = ReentrantCallbackGroup()
        self.create_timer(0.5, self.occupancy_grid_publish, callback_group=self.occupancy_grid_cb_group)
        self.create_timer(0.1, self.run_navigate)
        self.create_timer(0.5, self.generate_waypoints)
        
        # Publishers
        self.occupancy_map_pub = self.create_publisher(OccupancyGridMsg, '/occupancy_map', 20)
        self.control_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.waypoint_pub = self.create_publisher(Marker, '/nav/waypoints', 20)

        # Subscribers
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Actions
        self.set_heading_action = ActionServer(self, NavHeading, '/nav/set_heading', self.set_heading_action_handler)
        self.set_goal_action = ActionServer(self, NavLocation, '/nav/set_goal_location', self.set_goal_location_action_handler)

        # Services
        # self.set_heading_srv = self.create_service(Int16, '/nav/set_heading', self.set_heading_srv_cb)

        # Clients
        self.depth_cb_group = ReentrantCallbackGroup()
        self.get_box_locations_client = self.create_client(GetBoxLocations, '/depth/get_box_locations', callback_group=self.depth_cb_group)

        # Tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        while not self.get_box_locations_client.wait_for_service(timeout_sec=1): self.get_logger().info("Waiting for depth box service")

    def generate_waypoints(self):
        # self.get_logger().info("Gen wps")
        if self.goal_point == False:
            return
        
        x,y = self.get_robot_position()
        points = self.occupancy_grid.path_find(self.occupancy_grid.get_cell_idx_from_xy(x, y), 
                                               self.occupancy_grid.get_cell_idx_from_xy(self.goal_point.x, self.goal_point.y), self)
        if not points:
            self.occupancy_grid.discard_old_data()
            self.get_logger().info("No path")
            self.no_path_flag = True
            self.goal_point = False
            return
        if self.waypoint_hit and len(points) > 1:
            self.waypoint_hit = False
            points.pop(0)
        self.next_waypoint = points[0]

        util.Rviz.visualize_points(points, Marker.LINE_STRIP, self.waypoint_pub, util.Rviz.ARENA, self.get_clock().now(), 0.05, util.Colors.MAGENTA, 'waypoints')

    def run_navigate(self):
        # self.get_logger().info("Nav")
        msg = Twist()
        if self.follow_waypoint and self.next_waypoint != False: # Follow waypoint
            distance_err, heading_err = self.get_heading_and_dist_toward_point(self.next_waypoint)
        elif self.heading_set != False: # Turn to set heading
            distance_err = 0
            heading_err = self.heading_set - self.get_robot_heading()
        else: # Do nothing
            distance_err = 0
            heading_err = 0

        angular_vel = self.heading_pid(heading_err)
        linear_vel = self.linear_pid(distance_err)

        if abs(heading_err) > cfg.HEADING_MAX_ERROR: # Stop linear movement and wait for heading to align
            self.heading_accept = False
        elif self.heading_accept == False and abs(heading_err) < cfg.HEADING_PROCEED_ANGLE_ACCEPT:
            self.heading_accept = True

        if distance_err < cfg.WAYPOINT_ACCEPT_RADIUS and self.follow_waypoint:
            self.waypoint_hit = True
            linear_vel = 0
            angular_vel = 0
            # if self.goal_point.x == 0: self.goal_point = util.makePoint(1.0, 0.0)
            # else: self.goal_point = util.makePoint(0.0, 0.0)
        elif self.heading_accept == False: # Don't move while heading is not accepted
            linear_vel = 0

        # self.get_logger().info(f"Heading err: {math.degrees(heading_err)} Distance err: {distance_err} Lv: {linear_vel} Av {angular_vel} Acc: {self.heading_accept}")
        if abs(angular_vel) < 0.05 and linear_vel < 0.1:
            self.allow_rgbd_data = True
        else: 
            self.allow_rgbd_data = False

        msg.angular.z = float(angular_vel)
        msg.linear.x = float(-linear_vel)
        self.control_pub.publish(msg)

    def set_heading_action_handler(self, goal_handle):
        self.get_logger().info(f"Setting heading to {math.degrees(self.heading_set)}")
        self.set_heading(goal_handle.request.heading)
        while not self.heading_accept:
            self.get_clock().sleep_for(Duration(seconds=0.1))

        goal_handle.succeed()
        if not goal_handle.request.hold_after_complete:
            self.heading_set = False
        return NavHeading.Result()
    
    def set_heading(self, heading):
        self.waypoint_hit = False
        self.heading_accept = False
        self.follow_waypoint = False
        self.no_path_flag = False
        self.goal_point = False
        self.heading_set = heading
    
    def set_goal_location_action_handler(self, goal_handle):
        self.get_logger().info("Setting goal")
        self.set_goal_point(goal_handle.request.destination) ## Add checking for self.next_waypoint being false here
        distance_err = cfg.WAYPOINT_ACCEPT_RADIUS + 1
        while not distance_err < cfg.WAYPOINT_ACCEPT_RADIUS and not self.no_path_flag:
            self.get_clock().sleep_for(Duration(seconds=0.5))
            if self.next_waypoint != False:
                distance_err, _ = self.get_heading_and_dist_toward_point(self.next_waypoint)
        
        if self.no_path_flag:
            goal_handle.cancel()
        else:
            goal_handle.succeed()
        
        return NavLocation.Result()

    
    def set_goal_point(self, point: Point):
        self.goal_point = point
        self.follow_waypoint = True
        self.heading_set = False
        self.heading_accept = False
        self.waypoint_hit = False
    
    def get_heading_and_dist_toward_point(self, p: Point):
        # Heading
        x, y = self.get_robot_position()
        point_heading = -math.atan2(p.y - y, p.x - x)

        current_heading = self.get_robot_heading()
        heading_dist = point_heading - current_heading
        if heading_dist > math.pi:
            heading_dist -= 2 * math.pi
        elif heading_dist < -math.pi:
            heading_dist += 2 * math.pi
        
        # Distance
        distance = math.sqrt((p.x - x) ** 2 + (p.y - y) ** 2)
        return distance, heading_dist
        
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
    
    def get_robot_position(self):
        t: Transform|bool = self.get_base_link_transform()
        if t == False: return 0.0, 0.0
        x = t.translation.x
        y = t.translation.y
        return x, y


    # Gets the transform from the base link frame to the arena frame
    def get_base_link_transform(self):
        try:
            t = self.tf_buffer.lookup_transform('arena', 'base_link', rclpy.time.Time())
        except:
            self.get_logger().info("Could not get transform from 'base_link' to 'arena'")
            return False
        return t.transform
    
    def occupancy_grid_publish(self):
        if self.allow_rgbd_data:
            req = GetBoxLocations.Request()
            self.depth_future = self.get_box_locations_client.call_async(req)
            while not self.depth_future.done(): # Multi-threading goodness
                self.get_clock().sleep_for(Duration(seconds=0.01))
            resp: GetBoxLocations.Response = self.depth_future.result()
            try:
                self.occupancy_grid.add_points_with_transform(resp.red_boxes, self.tf_buffer, resp.frame_id, 'arena', resp.stamp)
                self.occupancy_grid.add_points_with_transform(resp.green_boxes, self.tf_buffer, resp.frame_id, 'arena', resp.stamp)
            except Exception as e:
                self.get_logger().info(f"Could not transform box data to arena frame: {e}")

        self.occupancy_grid.publish_occupancy_grid(self.occupancy_map_pub, 'arena', self.get_clock().now().to_msg(), self)
        return

    def scan_callback(self, data: LaserScan):
        self.last_scan_points = data.ranges
        self.last_scan_stamp = data.header.stamp
        self.scan_future = self.tf_buffer.wait_for_transform_async('arena', data.header.frame_id, data.header.stamp)
        self.scan_future.add_done_callback(self.add_scan_points_cb)
        # xy_points = util.scan_to_cartesian_points(data.ranges, cfg.LIDAR_MIN_ANGLE, cfg.LIDAR_MAX_ANGLE)
        # points = []
        # try:
        #     for point in xy_points: # This is dumb
        #         point_stamped = PointStamped()
        #         point_stamped.point = util.makePoint(point[0], point[1]) # X and Y are reversed, probably bad transform
        #         point_stamped.header.frame_id = data.header.frame_id
        #         future = self.tf_buffer.wait_for_transform_async('arena', data.header.frame_id, data.header.stamp)
        #         while not future.ad:
        #             continue
        #         tf_point = self.tf_buffer.transform_full(point_stamped, 'arena', data.header.stamp, data.header.frame_id)

        #         points.append(tf_point.point)
                
        # except Exception as e:
        #     self.get_logger().info(f"Could not transform scan data to arena frame type: {e}")
        #     return
        
    def add_scan_points_cb(self, future):
        stamp = self.last_scan_stamp
        xy_points = util.scan_to_cartesian_points(self.last_scan_points, cfg.LIDAR_MIN_ANGLE, cfg.LIDAR_MAX_ANGLE)
        points = []
        for point in xy_points: # This is dumb
                point_stamped = PointStamped()
                point_stamped.point = util.makePoint(point[0], point[1])
                point_stamped.header.frame_id = 'laser_link'
                tf_point = self.tf_buffer.transform_full(point_stamped, 'arena', stamp, 'laser_link')
                points.append(tf_point.point)
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