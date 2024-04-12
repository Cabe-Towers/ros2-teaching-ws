import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import numpy as np

from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
import util
import config
cfg = config.Config()

class DepthCameraNode(Node):
    def __init__(self):
        super().__init__('depth_camera')
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)
        # self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', self.depth_callback, 1)
        self.create_subscription(PointCloud2, '/limo/depth_camera_link/points', self.depth_callback, 1)
        self.control_sub = self.create_publisher(Twist, '/cmd_vel', 30)

        # self.points_vis_pub = self.create_publisher(Marker, '/marker_test', 20)
        self.red_box_pub = self.create_publisher(Marker, '/box_locations/red', 20)
        self.green_box_pub = self.create_publisher(Marker, '/box_locations/green', 20)
        self.red_marker_pub = self.create_publisher(Marker, '/arena_marker/red', 20)
        self.green_marker_pub = self.create_publisher(Marker, '/arena_marker/green', 20)

        self.depth_data = None

        self.br = CvBridge()

    def check_camera_exclusion(self, x, y):
        return (x < cfg.DEPTH_CAMERA_WIDTH - cfg.CAMERA_EXCLUDE_EDGES_X
                and x > cfg.CAMERA_EXCLUDE_EDGES_X
                and y < cfg.DEPTH_CAMERA_HEIGHT - cfg.CAMERA_EXCLUDE_EDGES_Y
                and y > cfg.CAMERA_EXCLUDE_EDGES_Y)
    

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
        
        red_box_points = []
        green_box_points = []
        red_marker_point: Point = None
        green_marker_point: Point = None
        for c in hsv_contours:
            
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if self.depth_data is not None and self.check_camera_exclusion(cx, cy):
                    x,y,z = self.point_from_pointcloud(cx,cy)
                    color = self.is_pixel_red_green(cx, cy, hsv_img)
                    if cy > cfg.DEPTH_CAMERA_HEIGHT / 2: # Box detected
                        if color == 0: # Green
                            green_box_points.append(util.makePoint(float(x),float(y),float(z) + cfg.DEPTH_BOX_Z_ADJUST))
                        elif color == 1: # Red
                            red_box_points.append(util.makePoint(float(x),float(y),float(z) + cfg.DEPTH_BOX_Z_ADJUST))
                    else: # Marker detected
                        if color == 0: # Green
                            green_marker_point = util.makePoint(float(x),float(y),float(z))
                        elif color == 1: # Red
                            red_marker_point = util.makePoint(float(x),float(y),float(z))

                    
        self.visualise_boxes(red_box_points, rgba=(1.0, 0.0, 0.0, 1.0), publisher=self.red_box_pub)
        self.visualise_boxes(green_box_points, rgba=(0.0, 1.0, 0.0, 1.0), publisher=self.green_box_pub)
        if red_marker_point is not None:
            self.visualise_points([red_marker_point], (1.0, 0.0, 0.0, 1.0), 0.1, self.red_marker_pub)
        if green_marker_point is not None:
            self.visualise_points([green_marker_point], (0.0, 1.0, 0.0, 1.0), 0.1, self.green_marker_pub)

    # Image in hsv format image[y][x][0-2]
    # Returns 0 if green, 1 if red, -1 otherwise
    def is_pixel_red_green(self, x, y, image):
        if image[y][x][0] < 10:
            return 1
        elif image[y][x][0] > 55 and image[y][x][0] < 105:
            return 0
        return -1


    def depth_callback(self, data: PointCloud2):
        # points = []
        np_points = point_cloud2.read_points_numpy(data)
        self.depth_data = np_points

        # for i in range(0, cfg.DEPTH_CAMERA_WIDTH - 2):
        #     x,y,z = self.point_from_pointcloud(i, 180)
        #     points.append(util.makePoint(float(x),float(y),float(z)))
        # self.visualise_points(points)

    def point_from_pointcloud(self, x, y):
        idx = y * cfg.DEPTH_CAMERA_WIDTH + x
        x_cord = self.depth_data[idx][0]
        y_cord = self.depth_data[idx][1]
        z_cord = self.depth_data[idx][2]
        return x_cord, y_cord, z_cord



    def visualise_points(self, points, rgba = (0.0, 1.0, 0.0, 0.5), scale = 0.05, publisher = None):
        mk = Marker()
        mk.header.frame_id = '/depth_link'
        mk.id = 0
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = "point"
        mk.action = Marker.ADD
        mk.lifetime = Duration(seconds=1).to_msg()

        mk.type = Marker.SPHERE_LIST
        mk.scale.x = scale
        mk.scale.y = scale

        mk.color.r = rgba[0]
        mk.color.g = rgba[1]
        mk.color.b = rgba[2]
        mk.color.a = rgba[3]

        for point in points:
            mk.points.append(point)
        
        if publisher is None: self.points_vis_pub.publish(mk)
        else: publisher.publish(mk)

    def visualise_boxes(self, points, rgba = (0.0, 1.0, 0.0, 0.5), scale = 0.05, publisher = None):
        mk = Marker()
        mk.header.frame_id = '/depth_link'
        mk.id = 0
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = "point"
        mk.action = Marker.ADD
        mk.lifetime = Duration(seconds=1).to_msg()

        mk.type = Marker.CUBE_LIST
        mk.scale.x = scale
        mk.scale.y = scale
        mk.scale.z = scale

        mk.color.r = rgba[0]
        mk.color.g = rgba[1]
        mk.color.b = rgba[2]
        mk.color.a = rgba[3]

        for point in points:
            mk.points.append(point)
        
        if publisher is not None:
            publisher.publish(mk)

def main(args=None):
    print('Starting colour_contours.py.')

    rclpy.init(args=args)
    colour_contours = DepthCameraNode()
    rclpy.spin(colour_contours)

    colour_contours.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()