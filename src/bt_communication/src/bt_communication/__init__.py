import logging

import rclpy

from .ros2_node import BluetoothROS2Node

logging.basicConfig(level=logging.INFO)


def main(args=None):
    rclpy.init(args=args)

    node = BluetoothROS2Node()
    node.start_bluetooth_server()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
