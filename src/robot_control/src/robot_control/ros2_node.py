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

        self.publisher_control = self.create_publisher(String, "robot_control", 10)
        self.publisher_feedback = self.create_publisher(String, "bluetooth_tx", 10)
        self.subscriber_feedback = self.create_subscription(
            String, "robot_feedback", self.on_robot_feedback, 10
        )
        self.subscriber_controller = self.create_subscription(
            String, "bluetooth_rx", self.on_controller_command, 10
        )

        self.m3508_cntl = M3508Controller()

        # フィードバック用のバッファ（最新のメッセージを保持）
        self.feedback_buffer = None

        # コマンドを 50 ms ごとに送信する
        self.create_timer(0.05, self.send_control_command)
        # フィードバックを 100 ms ごとに送信する
        self.create_timer(0.1, self.send_feedback)

        self.get_logger().info("Robot Controller Node initialized")

    def on_robot_feedback(self, msg: String) -> None:
        self.get_logger().info(f"Received feedback: {msg.data}")

        # 最新のフィードバックをバッファに保存
        self.feedback_buffer = msg

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

    def send_feedback(self) -> None:
        """
        フィードバックを送信する。100 ms ごとに呼び出される。
        """
        if self.feedback_buffer is not None:
            self.publisher_feedback.publish(self.feedback_buffer)
            self.feedback_buffer = None

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
        self.publisher_control.publish(msg)
        self.get_logger().info(f"Sent control command: {msg.data}")
