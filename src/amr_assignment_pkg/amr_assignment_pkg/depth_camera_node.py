import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import numpy as np

from amr_interfaces.srv import GetArenaMarker, GetBoxLocations

from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2
import util
import config
cfg = config.Config()

class DepthCameraNode(Node):
    def __init__(self):
        super().__init__('depth_camera')

        self.green_marker_point = None
        self.red_marker_point = None
        self.green_box_points = []
        self.red_box_points = []
        self.depth_data = None
        self.image_observation_timestamp = TimeMsg()
        self.depth_observation_timestamp = TimeMsg()
        self.last_depth_stamp = TimeMsg()

        # Subscriptions
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)
        # self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', self.depth_callback, 1)
        self.create_subscription(PointCloud2, '/limo/depth_camera_link/points', self.depth_callback, 1)
        self.control_sub = self.create_publisher(Twist, '/cmd_vel', 30)

        # Publishers
        # self.points_vis_pub = self.create_publisher(Marker, '/marker_test', 20)
        self.red_box_pub = self.create_publisher(Marker, '/box_locations/red', 20)
        self.green_box_pub = self.create_publisher(Marker, '/box_locations/green', 20)
        self.red_marker_pub = self.create_publisher(Marker, '/arena/red_marker', 20)
        self.green_marker_pub = self.create_publisher(Marker, '/arena/green_marker', 20)
        
        # Services
        self.get_marker_srv = self.create_service(GetArenaMarker, '/depth/get_markers', self.get_marker_srv_cb)
        self.get_boxes_srv = self.create_service(GetBoxLocations, '/depth/get_box_locations', self.get_box_locations_cb)

        self.br = CvBridge()

    # Provides the location of visible markers in the depth_link frame
    def get_marker_srv_cb(self, req, resp: GetArenaMarker.Response):
        if self.green_marker_point is not None:
            resp.green_marker_visible = True
            resp.green_marker_point = self.green_marker_point
        else: resp.green_marker_visible = False

        if self.red_marker_point is not None:
            resp.red_marker_visible = True
            resp.red_marker_point = self.red_marker_point
        else: resp.red_marker_visible = False

        resp.frame_id = 'depth_link'
        return resp
    
    def get_box_locations_cb(self, req, resp: GetBoxLocations.Response):
        resp.green_boxes = self.green_box_points
        resp.red_boxes = self.red_box_points
        resp.frame_id = 'depth_link'
        resp.stamp = self.image_observation_timestamp
        resp.depth_stamp = self.depth_observation_timestamp
        return resp

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

        hsv_contours, _ = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
        
        timestamp = self.get_clock().now().to_msg()
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

        self.green_marker_point = green_marker_point
        self.red_marker_point = red_marker_point

        self.green_box_points = green_box_points
        self.red_box_points = red_box_points
        self.image_observation_timestamp = timestamp
        self.depth_observation_timestamp = self.last_depth_stamp

        # Rviz
        util.Rviz.visualize_points(red_box_points, Marker.CUBE_LIST, self.red_box_pub, 
                                            util.Rviz.DEPTH_LINK, self.get_clock().now(), 0.05, util.Colors.RED, 'red_box')
        util.Rviz.visualize_points(green_box_points, Marker.CUBE_LIST, self.green_box_pub, 
                                            util.Rviz.DEPTH_LINK, self.get_clock().now(), 0.05, util.Colors.GREEN, 'green_box')
        if red_marker_point is not None:
            util.Rviz.visualize_points([red_marker_point], Marker.SPHERE_LIST, self.red_marker_pub, 
                                                util.Rviz.DEPTH_LINK, self.get_clock().now(), 0.1, util.Colors.RED, 'red_marker')
        if green_marker_point is not None:
            util.Rviz.visualize_points([green_marker_point], Marker.SPHERE_LIST, self.green_marker_pub, 
                                 util.Rviz.DEPTH_LINK, self.get_clock().now(), 0.1, util.Colors.GREEN, 'green_marker')

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
        self.last_depth_stamp = data.header.stamp

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
    

def main(args=None):
    rclpy.init(args=args)
    colour_contours = DepthCameraNode()
    rclpy.spin(colour_contours)

    colour_contours.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()