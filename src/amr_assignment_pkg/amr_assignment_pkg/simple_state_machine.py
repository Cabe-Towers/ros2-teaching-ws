##
## Basic solution for level 1
## This file is an early version of the project and is not used in the final solution
##
from enum import Enum
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import math

# Msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry

class RobotState(Enum):
    SEARCH = 0
    PUSH = 1
    REVERSE = 2
    HALT = 3

class SimpleStateMachine(Node):
    def __init__(self):
        super().__init__('simple_state_machine')

        self.state = RobotState.SEARCH
        self.timer = self.create_timer(0.05, self.state_machine)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)
        self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', self.depth_callback, 1)
        self.control_pub = self.create_publisher(Twist, '/cmd_vel', 30)

        self.points_vis_pub = self.create_publisher(Marker, '/marker_test', 20)
        self.box_pub = self.create_publisher(Marker, '/box_locations', 20)

        self.depth_data = None
        self.dist_ahead = None

        self.dist_to_block = None
        self.block_dist_from_center = None
        self.cam_data_ready = False

        self.push_yaw_angle = None
        self.push_start_position = None

        self.last_push_yaw_angle = 0
        self.last_yaw_behind = False
        self.current_yaw_angle = None
        self.current_position = None

        self.reverse_position_start = None
        self.reverse_distance = None

        self.br = CvBridge()

    def state_machine(self):
        match self.state:
            case RobotState.SEARCH:
                if self.cam_data_ready: 
                    self.cam_data_ready = False
                    self.run_search()
            case RobotState.PUSH:
                self.run_push()
            case RobotState.REVERSE:
                self.run_reverse()
            case RobotState.HALT:
                self.get_logger().info("Done!")
                self.destroy_node()
    
    def run_search(self):
        self.get_logger().info(f"Current Yaw: {self.current_yaw_angle}")
        msg = Twist()
        msg.angular.z = 0.6

        if self.current_yaw_angle > self.last_push_yaw_angle and self.last_yaw_behind:
            self.state = RobotState.HALT
            return
        
        self.last_yaw_behind = self.current_yaw_angle < self.last_push_yaw_angle

        # if self.dist_to_block is not None and self.dist_ahead is not None:
        #     self.get_logger().info(f"Dist block from wall {self.dist_ahead - self.dist_to_block}")

        if self.block_dist_from_center is None:
            pass
        elif self.block_dist_from_center < 10 and self.dist_ahead - self.dist_to_block > 0.25:
            # self.get_logger().info(f"GOT BLOCK")
            self.state = RobotState.PUSH
            return
        elif self.block_dist_from_center < 20:
            msg.angular.z = 0.05
        elif self.block_dist_from_center < 50:
            msg.angular.z = 0.15
        elif self.block_dist_from_center < 100:
            msg.angular.z = 0.3
        elif self.block_dist_from_center < 150:
            msg.angular.z = 0.6
        # elif self.block_dist_from_center < 250:
        #     msg.angular.z = 0.3

        self.control_pub.publish(msg)

    def run_push(self):
        # self.get_logger().info(f"Dist ahead {self.dist_ahead}")
        if self.dist_ahead is not None and self.dist_ahead < 0.3:
            self.state = RobotState.REVERSE
            return
        
        if self.push_yaw_angle is None:
            self.push_yaw_angle = self.current_yaw_angle
            self.last_push_yaw_angle = self.current_yaw_angle
        if self.push_start_position is None:
            self.push_start_position = self.current_position
        
        
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = self.proportional_steering_control(self.current_yaw_angle, self.push_yaw_angle, 3)
        self.control_pub.publish(msg)

    def run_reverse(self):
        if self.reverse_distance is None:
            self.reverse_distance = self.dist_between_points(self.push_start_position, self.current_position)
        
        if self.reverse_position_start is None:
            self.reverse_position_start = self.current_position

        dist = self.reverse_distance - self.dist_between_points(self.current_position, self.reverse_position_start)
        vel = 0.3
        if dist < 0.05:
            self.state = RobotState.SEARCH
            self.push_start_position = None
            self.push_yaw_angle = None
            self.reverse_position_start = None
            self.reverse_distance = None
            return
        if dist < 0.3:
            vel = 0.1

        msg = Twist()
        msg.linear.x = -vel
        msg.angular.z = self.proportional_steering_control(self.current_yaw_angle, self.push_yaw_angle, 0.8)
        self.control_pub.publish(msg)


    def odom_callback(self, data: Odometry):
        q = data.pose.pose.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        _, _, self.current_yaw_angle = rot.as_euler('xyz', degrees=False)
        self.current_position = data.pose.pose.position
        # self.get_logger().info(f"Current Yaw: {math.degrees(self.current_yaw_angle)}")

    def proportional_steering_control(self, current_heading, desired_heading, kp, max_turn_rate = 0.6):
        heading_diff = desired_heading - current_heading

        turn_rate = heading_diff * kp
        turn_rate = max(min(turn_rate, max_turn_rate), -max_turn_rate)
        return turn_rate

    def scan_callback(self, data: LaserScan):
        front_idx = len(data.ranges) // 2
        self.dist_ahead = data.ranges[front_idx]

    def camera_callback(self, data: Image):
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_thresh = cv2.inRange(hsv_img,
                                 np.array((0, 20, 20)),
                                 np.array((255, 255, 255)))

        hsv_contours, hierachy = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
        
        block_center_dists = []
        block_dists = []
        block_detected = False
        for c in hsv_contours:
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if self.depth_data is not None and cy > data.height // 2:
                    if self.depth_data[cy + 30][cx] - self.depth_data[cy][cx] < 0.25:
                        continue
                    dist_from_center = cx - (data.width / 2)
                    block_detected = True
                    block_dists.append(self.depth_data[cy][cx])
                    block_center_dists.append(abs(dist_from_center))
        
        
        self.cam_data_ready = True
        if not block_detected:
            self.dist_to_block = None
            self.block_dist_from_center = None
        else:
            # self.get_logger().info(f"Len blocks detected = {len(block_center_dists)}")
            self.block_dist_from_center = min(block_center_dists)
            self.dist_to_block = block_dists[block_center_dists.index(min(block_center_dists))]



    def depth_callback(self, data: Image):
        self.depth_data = self.br.imgmsg_to_cv2(data, "32FC1")

    def dist_between_points(self, p1, p2):
        x1 = p1.x
        y1 = p1.y
        y2 = p2.y
        x2 = p2.x
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)



def main(args=None):
    print('Starting node...')

    rclpy.init(args=args)
    node = SimpleStateMachine()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()