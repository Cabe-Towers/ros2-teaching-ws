import rclpy
from rclpy.node import Node
import numpy as np
import math
from enum import Enum

from example_interfaces.srv import Trigger
from amr_interfaces.srv import GetArenaMarker, SetArenaMarker

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

        # Clients
        self.grf_init_client = self.create_client(Trigger, '/arena/init_frame')
        self.depth_arena_marker_get_client = self.create_client(GetArenaMarker, '/depth/get_markers')
        self.set_arena_markers_client = self.create_client(SetArenaMarker, '/arena/set_markers')

        # Wait for services
        while not self.grf_init_client.wait_for_service(timeout_sec=1.0): self.get_logger().info("Waiting for GRF init service")
        while not self.depth_arena_marker_get_client.wait_for_service(timeout_sec=1.0): self.get_logger().info("Waiting for arena marker get service")
        while not self.set_arena_markers_client.wait_for_service(timeout_sec=1.0): self.get_logger().info("Waiting arena marker set service")

        self.test()

    def test(self):
        req = Trigger.Request()
        self.future = self.grf_init_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f"Got future: {self.future.result().message}")

        req = GetArenaMarker.Request()
        self.future = self.depth_arena_marker_get_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        resp: GetArenaMarker.Response = self.future.result()

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


        

def main(args=None):
    rclpy.init(args=args)

    node = StateMachine()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
