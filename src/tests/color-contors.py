#!/usr/bin/env python

# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

def truncate(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    s = '{}'.format(f)
    if 'e' in s or 'E' in s:
        return '{0:.{1}f}'.format(f, n)
    i, p, d = s.partition('.')
    return '.'.join([i, (d+'0'*n)[:n]])

class ColourContours(Node):
    def __init__(self):
        super().__init__('colour_contours')
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)
        self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', self.depth_callback, 1)
        self.control_sub = self.create_publisher(Twist, '/cmd_vel', 30)

        self.depth_data = None

        self.br = CvBridge()

    def camera_callback(self, data):
        #self.get_logger().info("camera_callback")

        cv2.namedWindow("Image window", 1)
        # cv2.namedWindow("Raw image", 1)
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
            cv_image_copy = cv_image.copy()
        except CvBridgeError as e:
            print(e)

        # using the BGR colour space, create a mask for everything
        # that is in a certain range
        # bgr_thresh = cv2.inRange(cv_image,
        #                          np.array((150, 150, 50)),
        #                          np.array((255, 255, 255)))

        # It often is better to use another colour space, that is
        # less sensitive to illumination (brightness) changes.
        # The HSV colour space is often a good choice. 
        # So, we first change the colour space here...
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # ... and now let's create a binary (mask) image, looking for 
        # any hue (range: 0-255), but for something brightly
        # colours (high saturation: > 150)
        hsv_thresh = cv2.inRange(hsv_img,
                                 np.array((0, 20, 20)),
                                 np.array((255, 255, 255)))

        # just for the fun of it, print the mean value 
        # of each HSV channel within the mask 
        # print(cv2.mean(hsv_img[:, :, 0], mask = hsv_thresh)[0])
        # print(cv2.mean(hsv_img[:, :, 1], mask = hsv_thresh)[0])
        # print(cv2.mean(hsv_img[:, :, 2], mask = hsv_thresh)[0])

        # This is how we could find actual contours in
        # the BGR image, but we won't do this now.
        # _, bgr_contours, hierachy = cv2.findContours(
        #     bgr_thresh.copy(),
        #     cv2.RETR_TREE,
        #     cv2.CHAIN_APPROX_SIMPLE)

        # Instead find the contours in the mask generated from the
        # HSV image.
        hsv_contours, hierachy = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
        
        # in hsv_contours we now have an array of individual
        # closed contours (basically a polgon around the 
        # blobs in the mask). Let's iterate over all those found 
        # contours.
        for c in hsv_contours:
            # This allows to compute the area (in pixels) of a contour
            a = cv2.contourArea(c)
            # and if the area is big enough, we draw the outline
            # of the contour (in blue)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0), 10)
            
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(cv_image, (cx, cy), 7, (0, 0, 255), -1)
                if self.depth_data is not None:
                    cv2.putText(cv_image, f"Depth: {truncate(self.depth_data[cy][cx], 2)} X, Y: {cx}, {cy}", (cx - 20, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

                
        # print('====')

        #cv_image_small = cv2.resize(cv_image, (0,0), fx=0.4, fy=0.4) # reduce image size
        cv2.imshow("Image window", cv_image)
        # cv2.imshow("Raw image", cv_image_copy)
        cv2.waitKey(1)

    def depth_callback(self, data: Image):
        # cv2.namedWindow("Raw image", 1)
        try:
            cv_image = self.br.imgmsg_to_cv2(data, "32FC1")

            self.depth_data = cv_image

            # cv2.imshow("Raw image", cv_image)
            # cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)


def main(args=None):
    print('Starting colour_contours.py.')

    rclpy.init(args=args)

    colour_contours = ColourContours()

    rclpy.spin(colour_contours)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_contours.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()