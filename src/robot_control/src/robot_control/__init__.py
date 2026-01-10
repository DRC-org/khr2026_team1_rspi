import logging

import rclpy

from .ros2_node import RobotController

logging.basicConfig(level=logging.INFO)


def main(args=None):
    rclpy.init(args=args)

    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
