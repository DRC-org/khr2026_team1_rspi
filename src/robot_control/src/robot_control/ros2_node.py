import json
import math

from rclpy.node import Node
from robot_msgs.msg import HandMessage, RingMechanism, WheelMessage, YaguraMechanism
from std_msgs.msg import String

from .constants import MAX_SPEED_MPS
from .hands import HandsController
from .m3508 import M3508Controller
from .vec2 import Vec2

_YAGURA_POS = {
    YaguraMechanism.POS_UP: "up",
    YaguraMechanism.POS_DOWN: "down",
    YaguraMechanism.POS_STOPPED: "stopped",
    YaguraMechanism.POS_UP_DONE: "up_done",
    YaguraMechanism.POS_DOWN_DONE: "down_done",
}

_YAGURA_STATE = {
    YaguraMechanism.STATE_OPEN: "open",
    YaguraMechanism.STATE_CLOSE: "close",
    YaguraMechanism.STATE_STOPPED: "stopped",
    YaguraMechanism.STATE_OPEN_DONE: "open_done",
    YaguraMechanism.STATE_CLOSE_DONE: "close_done",
}

_RING_POS = {
    RingMechanism.POS_PICKUP: "pickup",
    RingMechanism.POS_YAGURA: "yagura",
    RingMechanism.POS_HONMARU: "honmaru",
    RingMechanism.POS_STOPPED: "stopped",
    RingMechanism.POS_PICKUP_DONE: "pickup_done",
    RingMechanism.POS_YAGURA_DONE: "yagura_done",
    RingMechanism.POS_HONMARU_DONE: "honmaru_done",
}

_RING_STATE = {
    RingMechanism.STATE_OPEN: "open",
    RingMechanism.STATE_CLOSE: "close",
    RingMechanism.STATE_STOPPED: "stopped",
    RingMechanism.STATE_OPEN_DONE: "open_done",
    RingMechanism.STATE_CLOSE_DONE: "close_done",
}


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_control_node")

        self.pub_wheel_control = self.create_publisher(
            WheelMessage, "wheel_control", 10
        )
        self.pub_hand_control = self.create_publisher(HandMessage, "hand_control", 10)
        self.pub_bt_feedback = self.create_publisher(String, "bluetooth_tx", 10)
        self.sub_wheel_feedback = self.create_subscription(
            WheelMessage, "wheel_feedback", self.on_wheel_feedback, 10
        )
        self.sub_hand_feedback = self.create_subscription(
            HandMessage, "hand_feedback", self.on_hand_feedback, 10
        )
        self.sub_bt_controller = self.create_subscription(
            String, "bluetooth_rx", self.on_controller_command, 10
        )

        self.m3508_cntl = M3508Controller()
        self.hands_cntl = HandsController()

        # フィードバック用に最新のメッセージを保持
        self.wheel_fb_buffer: WheelMessage | None = None
        self.hand_fb_buffer: HandMessage | None = None

        # ESP32 にコマンドを 50 ms ごとに送信する
        self.create_timer(0.05, self.send_control_command)
        # コントローラにフィードバックを 100 ms ごとに送信する
        self.create_timer(0.1, self.send_controller_feedback)

        self.get_logger().info("Robot Controller Node initialized")

    def on_wheel_feedback(self, msg: WheelMessage) -> None:
        self.wheel_fb_buffer = msg

    def on_hand_feedback(self, msg: HandMessage) -> None:
        self.hand_fb_buffer = msg

    def on_controller_command(self, msg: String) -> None:
        try:
            command: dict = json.loads(msg.data)
            self.get_logger().info(f"From controller: {command}")

            if "type" not in command:
                return

            if command["type"] == "joystick":
                l_x = command.get("l_x", 0)
                l_y = command.get("l_y", 0)
                r = -int(command.get("r", 0))

                self.m3508_cntl.set_target_velocity(
                    Vec2(x=l_y / 10 * MAX_SPEED_MPS, y=l_x / 10 * MAX_SPEED_MPS),
                    r
                    / 10
                    * math.pi,  # 1 秒で半回転を最大にする（最高速度で旋回すると速すぎるため）
                )

            elif command["type"] == "hand":
                target = command.get("target", "")
                type = command.get("type", "")
                action = command.get("action", "")

                self.hands_cntl.set_target(target, type, action)

            elif command["type"] == "pid_gains":
                # PIDゲインをESP32に転送
                kp = max(0.0, min(10.0, command.get("kp", 0.5)))
                ki = max(0.0, min(1.0, command.get("ki", 0.05)))
                kd = max(0.0, min(1.0, command.get("kd", 0.0)))

                self.m3508_cntl.set_target_pid_gains(kp, ki, kd)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")

    def send_controller_feedback(self) -> None:
        """
        コントローラにフィードバックを送信する。100 ms ごとに呼び出される。
        """
        if self.wheel_fb_buffer is None:
            self.get_logger().warn("Wheel feedback buffer is None")
            return
        if self.hand_fb_buffer is None:
            self.get_logger().warn("Hand feedback buffer is None")
            return

        wheel_fb = self.wheel_fb_buffer
        hand_fb = self.hand_fb_buffer

        data = {
            "m3508_rpms": {
                "fl": wheel_fb.m3508_rpms.fl,
                "fr": wheel_fb.m3508_rpms.fr,
                "rl": wheel_fb.m3508_rpms.rl,
                "rr": wheel_fb.m3508_rpms.rr,
            },
            "m3508_gains": {
                "kp": wheel_fb.m3508_gains[0].kp,  # type: ignore[index]
                "ki": wheel_fb.m3508_gains[0].ki,  # type: ignore[index]
                "kd": wheel_fb.m3508_gains[0].kd,  # type: ignore[index]
            },
            "m3508_terms": {
                "fl": {
                    "p": wheel_fb.m3508_terms[0].fl.p,  # type: ignore[index]
                    "i": wheel_fb.m3508_terms[0].fl.i,  # type: ignore[index]
                    "d": wheel_fb.m3508_terms[0].fl.d,  # type: ignore[index]
                },
                "fr": {
                    "p": wheel_fb.m3508_terms[0].fr.p,  # type: ignore[index]
                    "i": wheel_fb.m3508_terms[0].fr.i,  # type: ignore[index]
                    "d": wheel_fb.m3508_terms[0].fr.d,  # type: ignore[index]
                },
                "rl": {
                    "p": wheel_fb.m3508_terms[0].rl.p,  # type: ignore[index]
                    "i": wheel_fb.m3508_terms[0].rl.i,  # type: ignore[index]
                    "d": wheel_fb.m3508_terms[0].rl.d,  # type: ignore[index]
                },
                "rr": {
                    "p": wheel_fb.m3508_terms[0].rr.p,  # type: ignore[index]
                    "i": wheel_fb.m3508_terms[0].rr.i,  # type: ignore[index]
                    "d": wheel_fb.m3508_terms[0].rr.d,  # type: ignore[index]
                },
            },
            "yagura": {
                "1_pos": _YAGURA_POS.get(hand_fb.yagura_1.pos, "stopped"),
                "1_state": _YAGURA_STATE.get(hand_fb.yagura_1.state, "stopped"),
                "2_pos": _YAGURA_POS.get(hand_fb.yagura_2.pos, "stopped"),
                "2_state": _YAGURA_STATE.get(hand_fb.yagura_2.state, "stopped"),
            },
            "ring": {
                "1_pos": _RING_POS.get(hand_fb.ring_1.pos, "stopped"),
                "1_state": _RING_STATE.get(hand_fb.ring_1.state, "stopped"),
                "2_pos": _RING_POS.get(hand_fb.ring_2.pos, "stopped"),
                "2_state": _RING_STATE.get(hand_fb.ring_2.state, "stopped"),
            },
        }

        msg = String()
        msg.data = json.dumps(data)
        self.pub_bt_feedback.publish(msg)

        self.wheel_fb_buffer = None
        self.hand_fb_buffer = None

    def send_control_command(self) -> None:
        """
        ESP32 へコマンドを送信する。50 ms ごとに呼び出される。
        """

        wheel_msg = WheelMessage()
        wheel_msg.m3508_rpms.fl = self.m3508_cntl.target_rpm_fl
        wheel_msg.m3508_rpms.fr = self.m3508_cntl.target_rpm_fr
        wheel_msg.m3508_rpms.rl = self.m3508_cntl.target_rpm_rl
        wheel_msg.m3508_rpms.rr = self.m3508_cntl.target_rpm_rr

        if (
            self.m3508_cntl.target_kp is not None
            and self.m3508_cntl.target_ki is not None
            and self.m3508_cntl.target_kd is not None
        ):
            wheel_msg.m3508_gains.size = 1  # type: ignore[assignment]
            wheel_msg.m3508_gains[0].kp = self.m3508_cntl.target_kp  # type: ignore[index]
            wheel_msg.m3508_gains[0].ki = self.m3508_cntl.target_ki  # type: ignore[index]
            wheel_msg.m3508_gains[0].kd = self.m3508_cntl.target_kd  # type: ignore[index]

        self.pub_wheel_control.publish(wheel_msg)

        hand_msg = HandMessage()
        hand_msg.yagura_1.pos = self.hands_cntl.yagura_1_pos
        hand_msg.yagura_1.state = self.hands_cntl.yagura_1_state
        hand_msg.yagura_2.pos = self.hands_cntl.yagura_2_pos
        hand_msg.yagura_2.state = self.hands_cntl.yagura_2_state
        hand_msg.ring_1.pos = self.hands_cntl.ring_1_pos
        hand_msg.ring_1.state = self.hands_cntl.ring_1_state
        hand_msg.ring_2.pos = self.hands_cntl.ring_2_pos
        hand_msg.ring_2.state = self.hands_cntl.ring_2_state

        self.pub_hand_control.publish(hand_msg)
