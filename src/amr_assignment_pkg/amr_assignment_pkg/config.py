import math

class Config:
    LIDAR_MIN_ANGLE = -2.03
    LIDAR_MAX_ANGLE = 2.03

    SCAN_LINE_SEGMENT_LENGTH = 8
    LINE_SLOPE_JOIN_THRESH = math.radians(18)
    LINE_GAP_JOIN_THRESH = 0.4
    SCAN_POINT_MAX_DENSITY = 0.03

def get_config() -> Config:
    cfg = Config()
    return cfg