import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from time import sleep
from rclpy.duration import Duration
import numpy as np
import math
from enum import Enum

from example_interfaces.srv import Trigger
from geometry_msgs.msg import Point
from amr_interfaces.srv import GetArenaMarker, SetArenaMarker
from amr_interfaces.action import NavHeading, NavLocation
from amr_interfaces.action._nav_location import NavLocation_GetResult_Response
import util

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
        self.move_done = False

        # Clients
        self.grf_init_client = self.create_client(Trigger, '/arena/init_frame')
        self.depth_arena_marker_get_client = self.create_client(GetArenaMarker, '/depth/get_markers')
        self.set_arena_markers_client = self.create_client(SetArenaMarker, '/arena/set_markers')

        # Action clients
        self.set_heading_action_client = ActionClient(self, NavHeading, '/nav/set_heading')
        self.set_goal_action_client = ActionClient(self, NavLocation, '/nav/set_goal_location')

        # Wait for services
        while not self.grf_init_client.wait_for_service(timeout_sec=1.0): self.get_logger().info("Waiting for GRF init service")
        while not self.depth_arena_marker_get_client.wait_for_service(timeout_sec=1.0): self.get_logger().info("Waiting for arena marker get service")
        while not self.set_arena_markers_client.wait_for_service(timeout_sec=1.0): self.get_logger().info("Waiting arena marker set service")

        # self.test()
        self.run_robot()

    def set_move_done(self):
        self.move_done = True

    def test(self):
        req = Trigger.Request()
        self.future = self.grf_init_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f"Got future: {self.future.result().message}")

        while True:
            self.get_logger().error("Trying to get markers...")
            req = GetArenaMarker.Request()
            self.future = self.depth_arena_marker_get_client.call_async(req)
            rclpy.spin_until_future_complete(self, self.future)
            resp: GetArenaMarker.Response = self.future.result()
            if not resp.green_marker_visible and not resp.red_marker_visible:
                self.get_logger().error("Could not find markers for GRF init, trying again")
                sleep(1)
                continue
            break
                
        
        req = SetArenaMarker.Request()
        req.frame_id = resp.frame_id
        req.green_marker_visible = resp.green_marker_visible
        req.green_marker_point = resp.green_marker_point
        req.red_marker_visible = resp.red_marker_visible
        req.red_marker_point = resp.red_marker_point
        self.future = self.set_arena_markers_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        resp: SetArenaMarker.Response = self.future.result()

        self.get_logger().info(f"Set marker done: resp - {resp.success}")

        goal = NavHeading.Goal()
        goal.heading = math.radians(180)
        goal.hold_after_complete = False
        self.future = self.set_heading_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, self.future)
        # _ = self.future.result()
        self.get_logger().info(f"Spin DONE")

    def run_robot(self):
        while self.state != RobotState.HALT:
            if self.state == RobotState.INIT:
                self.init_robot()
        exit()

    def init_robot(self):
        # Init global reference frame
        req = Trigger.Request()
        self.future = self.grf_init_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f"Got future: {self.future.result().message}")

        # Get location of green marker
        while True:
            self.get_logger().error("Trying to get green marker...")
            req = GetArenaMarker.Request()
            self.future = self.depth_arena_marker_get_client.call_async(req)
            rclpy.spin_until_future_complete(self, self.future)
            resp: GetArenaMarker.Response = self.future.result()
            if not resp.green_marker_visible:
                self.get_logger().error("Could not find marker for GRF init, trying again")
                sleep(1)
                continue
            break

        # Turn 180
        goal = NavHeading.Goal()
        goal.heading = math.radians(180)
        goal.hold_after_complete = False
        self.future = self.set_heading_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, self.future)

        # Get location of green marker
        while True:
            self.get_logger().error("Trying to get red marker...")
            req = GetArenaMarker.Request()
            self.future = self.depth_arena_marker_get_client.call_async(req)
            rclpy.spin_until_future_complete(self, self.future)
            resp: GetArenaMarker.Response = self.future.result()
            if not resp.red_marker_visible:
                self.get_logger().error("Could not find marker for GRF init, trying again")
                sleep(1)
                continue
            break

        self.send_navigation_goal(util.makePoint(1.0, -1.0))
        self.send_navigation_goal(util.makePoint(-1.0, 1.0))
        

        self.state = RobotState.HALT

    def send_navigation_goal(self, point: Point):
        goal = NavLocation.Goal()
        goal.destination = point
        self.future = self.set_goal_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, self.future)
        goal_handle = self.future.result()
        self.future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.future)
        result: NavLocation_GetResult_Response = self.future.result()
        self.get_logger().info(f"Action done ({type(result)}) Status {result.status}")
        if result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Failed to navigate to top left")
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
