from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from rclpy.time import Duration
from enum import Enum

def makePoint(x, y, z=0.0) -> Point:
    p = Point()
    p.x = x
    p.y = y
    p.z = z
    return p

def scan_to_cartesian_points(scan_data, angle_min, angle_max) -> list:
    total_angle = abs(angle_min) + angle_max

    points = []
    for idx, dist in enumerate(scan_data):
        if dist == float('inf'): continue # Sometimes scan data is infinite, looks like a glitch
        # Calculate angle of scan at idx
        angle = (idx/len(scan_data)) * total_angle + angle_min
        # Convert from polar coordinates to cartesian 
        y = math.sin(angle) * dist
        x = math.cos(angle) * dist

        points.append([x, y])
    return points

def polar_to_cartesian_point(distance, angle):
    # Convert from polar coordinates to cartesian 
    y = math.sin(angle) * distance
    x = math.cos(angle) * distance

    return makePoint(x, y)

class LineSegment:
    def __init__(self, slope: float, intercept: float, start_idx: int, end_idx: int):
        self.slope = slope
        self.intercept = intercept
        self.start_idx = start_idx
        self.end_idx = end_idx

    def distance_from_line(self, x, y) -> float:
        # Calculates perpendicular distance of a point from a line
        # Perpendicular distance formula: d = (ax + by + c) / sqrt(a^2 + b^2) or d = (mx - y + c) / sqrt(m^2 + 1)
        return abs(self.slope * x - y + self.intercept) / math.sqrt(self.slope**2 + 1)
    
    def get_line_intersection(self, line) -> tuple:
        x = (self.intercept - line.intercept) / (line.slope - self.slope)
        y = self.slope * x + self.intercept
        return x, y

    def closest_point_on_line(self, p) -> np.array:
        m = self.slope
        b = self.intercept
        # Get the equation of the perpendicular line that passes through point p
        if m == 0:
            perp_m = np.inf  # Line is horizontal, perpendicular line will be vertical
            perp_b = p[0]  # Perpendicular line will pass through x-coordinate of p
        else:
            perp_m = -1/m  # Slope of the perpendicular line is negative reciprocal of m
            perp_b = p[1] - perp_m * p[0]  # Perpendicular line equation using point-slope form

        # Find the intersection point of the two lines
        if perp_m == np.inf:
            x_intersect = p[0]  # If the line is horizontal, x-coordinate of intersection is same as p's x
            y_intersect = m * x_intersect + b  # Solve for y using the equation of the original line
        else:
            x_intersect = (b - perp_b) / (perp_m - m)  # Solve for x-coordinate of intersection
            y_intersect = perp_m * x_intersect + perp_b  # Solve for y-coordinate using the equation of the perpendicular line

        return np.array([x_intersect, y_intersect])


class Rviz:

    LASER_LINK = '/laser_link'
    DEPTH_LINK = '/depth_link'
    ARENA = '/arena'

    def visualize_points(points_array, marker_type, publisher, frame_id, time_stamp, scale, rgba, namespace):
        mk = Marker()
        mk.header.frame_id = frame_id
        mk.id = 0
        mk.header.stamp = time_stamp.to_msg()
        mk.ns = namespace
        mk.action = Marker.ADD
        mk.lifetime = Duration(seconds=1).to_msg()

        mk.type = marker_type
        if type(scale) is list:
            mk.scale.x = scale[0]
            mk.scale.y = scale[1]
            mk.scale.z = scale[2]
        else:
            mk.scale.x = scale
            mk.scale.y = scale
            mk.scale.z = scale

        if type(rgba) is Colors: rgba = rgba.value
        mk.color.r = rgba[0]
        mk.color.g = rgba[1]
        mk.color.b = rgba[2]
        mk.color.a = rgba[3]

        for point in points_array:
            mk.points.append(point)
        
        publisher.publish(mk)

class Colors(Enum):
    RED = (1.0, 0.0, 0.0, 1.0)
    RED_TSP = (1.0, 0.0, 0.0, 0.5)

    ORANGE = (1.0, 0.5, 0.0, 1.0)
    ORANGE_TSP = (1.0, 0.5, 0.0, 0.5)

    YELLOW = (1.0, 1.0, 0.0, 1.0)
    YELLOW_TSP = (1.0, 1.0, 0.0, 0.5)

    GREEN = (0.0, 1.0, 0.0, 1.0)
    GREEN_TSP = (0.0, 1.0, 0.0, 0.5)

    BLUE = (0.0, 0.0, 1.0, 1.0)
    BLUE_TSP = (0.0, 0.0, 1.0, 0.5)

    INDIGO = (0.6, 0.0, 1.0, 1.0)
    INDIGO_TSP = (0.6, 0.0, 1.0, 0.5)

    VIOLET = (0.75, 0.0, 1.0, 1.0)
    VIOLET_TSP = (0.75, 0.0, 1.0, 0.5)

    WHITE = (1.0, 1.0, 1.0, 1.0)
    WHITE_TSP = (1.0, 1.0, 1.0, 0.5)

    MAGENTA = (1.0, 0.0, 1.0, 1.0)
    MAGENTA_TSP = (1.0, 0.0, 1.0, 0.5)

    CYAN = (0.0, 1.0, 1.0, 1.0)
    CYAN_TSP = (0.0, 1.0, 1.0, 0.5)

    PINK = (1.0, 0.75, 0.75, 1.0)
    PINK_TSP = (1.0, 0.75, 0.75, 0.5)