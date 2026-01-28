import json
import math

from rclpy.node import Node
from std_msgs.msg import String

from .constants import MAX_SPEED_MPS
from .m3508 import M3508Controller
from .vec2 import Vec2


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_control_node")

        self.publisher = self.create_publisher(String, "robot_control", 10)
        self.subscriber_feedback = self.create_subscription(
            String, "robot_feedback", self.on_robot_feedback, 10
        )
        self.subscriber_controller = self.create_subscription(
            String, "bluetooth_rx", self.on_controller_command, 10
        )

        self.m3508_cntl = M3508Controller()

        # コマンドを 50 ms ごとに送信する
        # → デバッグで一時的に 1 秒に変更
        self.create_timer(1, self.send_control_command)

        self.get_logger().info("Robot Controller Node initialized")

    def on_robot_feedback(self, msg: String) -> None:
        self.get_logger().info(f"Received feedback: {msg.data}")

    def on_controller_command(self, msg: String) -> None:
        self.get_logger().info(f"Received controller command: {msg.data}")

        try:
            commands: dict = json.loads(msg.data)
            self.get_logger().info(f"Parsed commands: {commands}")

            for command in commands:
                if "type" in command and command["type"] == "joystick":
                    l_x = command.get("l_x", 0)
                    l_y = command.get("l_y", 0)
                    r = -int(command.get("r", 0))

                    self.m3508_cntl.set_target_velocity(
                        Vec2(x=l_y / 10 * MAX_SPEED_MPS, y=l_x / 10 * MAX_SPEED_MPS),
                        r
                        / 10
                        * math.pi,  # 1 秒で半回転を最大にする（最高速度で旋回すると速すぎるため）
                    )

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")

    def send_control_command(self) -> None:
        """
        ESP32 へコマンドを送信する。50 ms ごとに呼び出される。
        """

        command = {
            "m3508_rpms": {
                "fl": self.m3508_cntl.target_rpm_fl,
                "fr": self.m3508_cntl.target_rpm_fr,
                "rl": self.m3508_cntl.target_rpm_rl,
                "rr": self.m3508_cntl.target_rpm_rr,
            }
        }

        msg = String()
        msg.data = json.dumps(command)
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent control command: {msg.data}")
