import rclpy
from rclpy.node import Node
from rclpy.time import Duration
import numpy as np

import util
import config
cfg = config.Config()

# Message types
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan