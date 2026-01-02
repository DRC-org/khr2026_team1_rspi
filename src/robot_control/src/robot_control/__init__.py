import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_control_node")

        self.publisher = self.create_publisher(String, "robot_control", 10)
        self.subscriber = self.create_subscription(
            String, "robot_feedback", self.on_robot_feedback, 10
        )

        self.get_logger().info("Robot Controller Node initialized")

    def on_robot_feedback(self, msg: String):
        self.get_logger().info(f"Received feedback: {msg.data}")


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
