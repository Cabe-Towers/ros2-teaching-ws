import rclpy

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("test_node")
    logger = node.get_logger()

    logger.info("Hello from logger!!!")
    print('Hi from amr_assignment_pkg!!!')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
