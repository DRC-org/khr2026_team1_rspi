import logging

import rclpy

from .odometry_node import OdometryNode

logging.basicConfig(level=logging.INFO)


def main(args=None):
    rclpy.init(args=args)

    node = OdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
