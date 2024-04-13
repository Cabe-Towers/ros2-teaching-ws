import rclpy
from rclpy.node import Node
import numpy as np
import math
from enum import Enum

from example_interfaces.srv import Trigger

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
        self.grf_init_srv = self.create_client(Trigger, 'init_grf')
        while not self.grf_init_srv.wait_for_service(timeout_sec=1.0): self.get_logger().info("Waiting for GRF init service")

        self.test()

    def test(self):
        req = Trigger.Request()
        self.future = self.grf_init_srv.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f"Got future: {self.future.result().message}")
        

def main(args=None):
    rclpy.init(args=args)

    node = StateMachine()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
