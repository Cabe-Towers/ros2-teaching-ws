import math

# Config all in one place for maintainability
class Config:
    ## Global reference frame
    LIDAR_MIN_ANGLE = -2.03
    LIDAR_MAX_ANGLE = 2.03

    SCAN_LINE_SEGMENT_LENGTH = 8
    LINE_SLOPE_JOIN_THRESH = math.radians(18)
    LINE_GAP_JOIN_THRESH = 0.4
    SCAN_POINT_MAX_DENSITY = 0.03
    SCAN_POINT_OUTLIER_THRESH = 0.1

    LINE_CORNER_ANGLE_THRESH = math.radians(10)
    ##

    ## RGBD camera
    DEPTH_CAMERA_WIDTH = 640
    DEPTH_CAMERA_HEIGHT = 480
    DEPTH_BOX_Z_ADJUST = 0.02
    CAMERA_EXCLUDE_EDGES_X = 20
    CAMERA_EXCLUDE_EDGES_Y = 20
    ##

    ## Navigation
    MAX_ANGULAR_VELOCITY = 0.6
    MAX_LINEAR_VELOCITY = 0.3

    WAYPOINT_ACCEPT_RADIUS = 0.05
    HEADING_MAX_ERROR = math.radians(25)
    HEADING_PROCEED_ANGLE_ACCEPT = math.radians(2)

    BOX_NAVIGATION_OFFSET = 0.3

    PUSH_TO_X_VALUE = 1.22
    BOX_PUSHED_X_VALUE = 1.1

def get_config() -> Config:
    cfg = Config()
    return cfg