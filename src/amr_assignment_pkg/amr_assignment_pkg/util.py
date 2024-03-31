import math

def scan_to_cartesian_points(scan_data, angle_min, angle_max):
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