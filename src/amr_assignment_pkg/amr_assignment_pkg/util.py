import math
import numpy as np
from geometry_msgs.msg import Point

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