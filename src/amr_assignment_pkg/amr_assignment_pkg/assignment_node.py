import rclpy
from rclpy.node import Node
import numpy as np
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images

# Message types
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from amr_interfaces.msg import Num

class AssignmentNode(Node):
    def __init__(self):
        super().__init__("assignment_node")

        self.state = "halt"
        self.dist_ahead = 100
        self.heading = 0


        self.desired_heading = 0.0
        self.heading_accept_error = 0.1

        self.depth_data = None

        self.br = CvBridge()

        # Publishers
        self.drive_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 20)

        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)
        # self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', self.depth_callback, 1)

    def camera_callback(self, data: Image):
        cv2.namedWindow("Image window", 1)
        # cv2.namedWindow("Raw image", 1)
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
            cv_image_copy = cv_image.copy()
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
        
        for c in hsv_contours:
            # This allows to compute the area (in pixels) of a contour
            a = cv2.contourArea(c)
            # and if the area is big enough, we draw the outline
            # of the contour (in blue)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0), 10)
            
            M = cv2.moments(c)
            if ['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(cv_image, (cx, cy), 7, (0, 0, 255), -1)
                if self.depth_data is not None:
                    cv2.putText(cv_image, f"Depth: {round(self.depth_data[cy][cx], 2)}", (cx - 20, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                # cv2.putText(cv_image, f"X {cx}, Y: {cy}", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        cv2.imshow("Image window", cv_image)
        # cv2.imshow("Raw image", cv_image_copy)
        cv2.waitKey(1)

    def scan_callback(self, msg: LaserScan):
        length = len(msg.ranges)
        # self.get_logger().info('Got dist: %s' % msg.ranges[length//2])
        self.dist_ahead = msg.ranges[length//2]

        if self.dist_ahead < 0.25 and self.state == 'fwd':
            self._logger.info(f"Hit wall, turning...")
            self.desired_heading = (self.heading + math.pi) % (2*math.pi)
            self._logger.info(f"Heading is {self.heading}, target heading is {self.desired_heading}")
            self.state = 'turn'

        msg = Twist()
        if self.state == 'fwd':
            msg.linear.x = 0.2
        if self.state == 'turn':
            msg.angular.z = 0.6
        if self.state == 'rev':
            msg.linear.x = 0.05
        if self.state == 'halt':
            return
        self.drive_pub.publish(msg)
    
    def odom_callback(self, msg: Odometry):
        _, _, yaw_rad = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.heading = yaw_rad + math.pi
        if abs(self.dist_direct_toward_heading(yaw_rad, self.desired_heading)) < self.heading_accept_error and self.state == 'turn':
            self._logger.info(f"Turn finished. State: {self.state}")
            self.state = "fwd"
    
    def dist_direct_toward_heading(self, heading, target_heading):
        heading = heading + math.pi

        dist_around = heading - target_heading
        dist_across = (2*math.pi) - (heading + target_heading)

        if abs(dist_around) < abs (dist_across):
            return dist_around
        else:
            return dist_across

    def is_heading_within_threshold(self, heading, desired_heading, threshold):
        """
        Checks if a heading is within a threshold of a desired heading.

        Args:
            heading: The current heading in radians.
            desired_heading: The desired heading in radians.
            threshold: The threshold in radians.

        Returns:
            True if the heading is within the threshold of the desired heading, False otherwise.
        """

        # Handle the case where the headings are on opposite sides of the circle
        if abs(heading - desired_heading) > math.pi:
            distance = 2 * math.pi - abs(heading - desired_heading)
        else:
            distance = abs(heading - desired_heading)

        return distance <= threshold
        

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)

    node = AssignmentNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
