# rclpy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus

# Message and interfaces
from example_interfaces.srv import Trigger
from geometry_msgs.msg import Point
from amr_interfaces.srv import GetArenaMarker, SetArenaMarker, LineOfSight
from amr_interfaces.action import NavHeading, NavLocation, SearchBoxes
from amr_interfaces.action._nav_location import NavLocation_GetResult_Response
from amr_interfaces.action._search_boxes import SearchBoxes_GetResult_Response

# Util and config
from enum import Enum
import util
import config
cfg = config.Config()

# All possible robot states
class RobotState(Enum):
    HALT = 0
    INIT = 1
    SEARCH = 2
    NAVIGATE = 3
    PUSH = 4

class StateMachine(Node):
    def __init__(self):
        super().__init__("state_node")
        self.state: RobotState = RobotState.INIT

        # Box locations
        self.red_boxes = []
        self.green_boxes = []
        self.push_box_y = 0.0
        self.push_box_x = 0.0
        self.push_box_color = ''

        # Clients
        self.grf_init_client = self.create_client(Trigger, '/arena/init_frame')
        self.depth_arena_marker_get_client = self.create_client(GetArenaMarker, '/depth/get_markers')
        self.set_arena_markers_client = self.create_client(SetArenaMarker, '/arena/set_markers')
        self.check_line_of_sight_client = self.create_client(LineOfSight, '/nav/check_line_of_sight')
        self.clear_occupancy_grid_client = self.create_client(Trigger, '/nav/clear_occupancy_grid')

        # Action clients
        self.set_heading_action_client = ActionClient(self, NavHeading, '/nav/set_heading')
        self.set_goal_action_client = ActionClient(self, NavLocation, '/nav/set_goal_location')
        self.set_manual_waypoint_action_client = ActionClient(self, NavLocation, '/nav/set_waypoint')
        self.reverse_turn_action_client = ActionClient(self, NavLocation, '/nav/reverse_turn')
        self.search_for_boxes_action_client = ActionClient(self, SearchBoxes, '/nav/scan_boxes')

        # Wait for services to init
        while not self.grf_init_client.wait_for_service(timeout_sec=1.0): self.get_logger().info("Waiting for GRF init service")
        while not self.depth_arena_marker_get_client.wait_for_service(timeout_sec=1.0): self.get_logger().info("Waiting for arena marker get service")
        while not self.set_arena_markers_client.wait_for_service(timeout_sec=1.0): self.get_logger().info("Waiting arena marker set service")

        # Start
        self.run_robot()

    def run_robot(self):
        while self.state != RobotState.HALT:
            match self.state:
                case RobotState.INIT:
                    self.init_robot()
                case RobotState.SEARCH:
                    self.run_search()
                case RobotState.NAVIGATE:
                    self.run_navigate()
                case RobotState.PUSH:
                    self.run_push()

        self.get_logger().info("Robot halt")
        exit()

    def init_robot(self):
        # Init global reference frame
        req = Trigger.Request()
        self.future = self.grf_init_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f"Got future: {self.future.result().message}")

        # # Get location of green marker
        # while True:
        #     self.get_logger().info("Trying to get green marker...")
        #     req = GetArenaMarker.Request()
        #     self.future = self.depth_arena_marker_get_client.call_async(req)
        #     rclpy.spin_until_future_complete(self, self.future)
        #     resp: GetArenaMarker.Response = self.future.result()
        #     if not resp.green_marker_visible:
        #         self.get_logger().error("Could not find marker for GRF init, trying again")
        #         sleep(1)
        #         continue
        #     break

        # # Turn 180
        # goal = NavHeading.Goal()
        # goal.heading = math.radians(180)
        # goal.hold_after_complete = False
        # self.future = self.set_heading_action_client.send_goal_async(goal)
        # rclpy.spin_until_future_complete(self, self.future)

        # # Get location of green marker
        # while True:
        #     self.get_logger().info("Trying to get red marker...")
        #     req = GetArenaMarker.Request()
        #     self.future = self.depth_arena_marker_get_client.call_async(req)
        #     rclpy.spin_until_future_complete(self, self.future)
        #     resp: GetArenaMarker.Response = self.future.result()
        #     if not resp.red_marker_visible:
        #         self.get_logger().error("Could not find marker for GRF init, trying again")
        #         sleep(1)
        #         continue
        #     break

        self.get_logger().error("Init done")
        self.state = RobotState.SEARCH

    # Asks nav node to search for boxes and gets them from action result
    def run_search(self):
        self.clear_occupancy_grid() # Clear occupancy grid before every search
        self.get_logger().info("Start search")
        goal = SearchBoxes.Goal()
        self.future = self.search_for_boxes_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, self.future)
        goal_handle = self.future.result()
        self.future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.future)
        result : SearchBoxes_GetResult_Response = self.future.result()
        
        self.green_boxes = result.result.green_boxes
        self.red_boxes = result.result.red_boxes

        self.get_logger().info(f"Done searching for boxes. Found {len(self.green_boxes)} green boxes and {len(self.red_boxes)} red boxes")

        self.state = RobotState.NAVIGATE
    
    # Loops over detected boxes and navigates to them if they can be pushed
    def run_navigate(self):
        self.get_logger().info(f"Finding box to push...")
        for box in self.green_boxes: # Green boxes
            if box.x > cfg.BOX_PUSHED_X_VALUE: continue # Ignore box if it's already near the wall
            if self.line_of_sight_req(box, 'green_boxes') == False: continue # Check to see if box can be pushed in a straight line to the wall
            if self.send_navigation_goal(util.makePoint(box.x - cfg.BOX_NAVIGATION_OFFSET, box.y)): # Try to go to box
                self.get_logger().info(f"Successfully navigated to box")
                if self.line_of_sight_req(box, 'green_boxes') == False: continue # Once at box check line of sight again
                # Set state variables and push
                self.push_box_color = 'green'
                self.push_box_y = box.y
                self.push_box_x = box.x
                self.state = RobotState.PUSH
                return
            else:
                self.get_logger().info(f"Navigation to box failed, trying another box")
        
        for box in self.red_boxes: # Red boxes
            if box.x < -cfg.BOX_PUSHED_X_VALUE: continue # Ignore box if it's already near the wall
            if self.line_of_sight_req(box, 'red_boxes') == False: continue # Check to see if box can be pushed in a straight line to the wall
            if self.send_navigation_goal(util.makePoint(box.x + cfg.BOX_NAVIGATION_OFFSET, box.y)): # Try to go to box
                self.get_logger().info(f"Successfully navigated to box")
                if self.line_of_sight_req(box, 'red_boxes') == False: continue # Once at box check line of sight again
                # Set state variables and push
                self.push_box_color = 'red'
                self.push_box_y = box.y
                self.push_box_x = box.x
                self.state = RobotState.PUSH
                return
            else:
                self.get_logger().info(f"Navigation to box failed, trying another box")
        
        self.get_logger().info(f"Unable to push any more boxes, remaining boxes are blocked")
        self.state = RobotState.HALT
    
    # Push box toward wall then return to center and go to search state
    def run_push(self):
        self.get_logger().info(f"Pushing box...")
        color_mult = 1 if self.push_box_color == 'green' else -1 # Used to modify offsets for pushing in different directions
        self.send_waypoint(util.makePoint(cfg.PUSH_TO_X_VALUE * color_mult, self.push_box_y)) # Push box to wall
        self.get_logger().info(f"Reversing")
        self.send_reverse_turn(util.makePoint(self.push_box_x + cfg.BOX_NAVIGATION_OFFSET * -color_mult, self.push_box_y)) # Reverse slightly and turn around
        self.get_logger().info(f"Done pushing box, returning to center")
        self.send_navigation_goal(util.makePoint(0.0, 0.0)) # Return to center
        self.get_logger().info(f"Done pushing box, state changed to search")
        self.state = RobotState.SEARCH
        return

    def clear_occupancy_grid(self):
        req = Trigger.Request()
        self.future = self.clear_occupancy_grid_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)

    # Calls the line of sight service and returns true if LOS is clear of obstacles
    def line_of_sight_req(self, box, ignore_data_source):
        req = LineOfSight.Request()
        # Used to modify offsets for pushing in different directions
        if (ignore_data_source) == 'green_boxes': color_mult = 1
        else: color_mult = -1
        req.start = util.makePoint(box.x + cfg.BOX_NAVIGATION_OFFSET * -color_mult, box.y)
        req.end = util.makePoint(cfg.PUSH_TO_X_VALUE * color_mult, box.y)
        self.get_logger().info(f"Start, end {req.start} {req.end}")
        req.ignore_data_source = ignore_data_source # Ignore specific data source and drive through box or boxes
        self.future = self.check_line_of_sight_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().clear

    # Sends a manual waypoint without pathing to the nav node using an action, returns when waypoint reached
    def send_waypoint(self, point: Point):
        goal = NavLocation.Goal()
        goal.destination = point
        self.future = self.set_manual_waypoint_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, self.future)
        goal_handle = self.future.result()
        self.future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.future)
        return
    
    # Sends the reverse and turn around command, returns when complete
    def send_reverse_turn(self, point: Point):
        goal = NavLocation.Goal()
        goal.destination = point
        self.future = self.reverse_turn_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, self.future)
        goal_handle = self.future.result()
        self.future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.future)
        return

    # Sends a destination to the nav node to path to, returns when complete for navigation fails
    def send_navigation_goal(self, point: Point):
        goal = NavLocation.Goal()
        goal.destination = point
        self.future = self.set_goal_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, self.future)
        goal_handle = self.future.result()
        self.future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.future)
        result: NavLocation_GetResult_Response = self.future.result()
        if result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f"Failed to navigate to point X: {point.x} Y: {point.y}")
            return False
        return True


def main(args=None):
    rclpy.init(args=args)

    node = StateMachine()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
