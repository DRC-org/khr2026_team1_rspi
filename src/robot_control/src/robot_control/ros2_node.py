import json
import math
import time

from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_msgs.msg import (
    HandMessage,
    PIDGains,
    RingMechanism,
    WheelMessage,
    YaguraMechanism,
)
from std_msgs.msg import String

from .constants import MAX_SPEED_MPS
from .hands import HandsController
from .heading_pid import HeadingPID
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
        self.sub_nav_mode = self.create_subscription(
            String, "/nav_mode", self.on_nav_mode, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10
        )

        self._nav_mode = "manual"
        self.m3508_cntl = M3508Controller()
        self.hands_cntl = HandsController()

        # フィードバック用に最新のメッセージを保持
        self.wheel_fb_buffer: WheelMessage | None = None
        self.hand_fb_buffer: HandMessage | None = None
        self._hand_fb_warned = False  # ハンド未接続の WARN は初回のみ出す

        self._current_yaw: float | None = None
        self._heading_pid = HeadingPID(kp=1.0, ki=0.0, kd=0.05)
        self._target_yaw: float | None = None
        self._last_cmd_time = time.monotonic()

        # ESP32 にコマンドを 50 ms ごとに送信する
        self.create_timer(0.05, self.send_control_command)
        # コントローラにフィードバックを 100 ms ごとに送信する
        self.create_timer(0.1, self.send_controller_feedback)

        self.get_logger().info("Robot Controller Node initialized")

    def on_nav_mode(self, msg: String) -> None:
        prev = self._nav_mode
        self._nav_mode = msg.data
        # auto モードに切り替わったらジョイスティック入力をリセット
        if prev == "manual" and self._nav_mode == "auto":
            self.m3508_cntl.set_target_velocity(Vec2(0.0, 0.0), 0.0)
            self._target_yaw = None
            self._heading_pid.reset()

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def on_wheel_feedback(self, msg: WheelMessage) -> None:
        self.wheel_fb_buffer = msg

    def on_hand_feedback(self, msg: HandMessage) -> None:
        self.hand_fb_buffer = msg

    def on_controller_command(self, msg: String) -> None:
        try:
            command: dict = json.loads(msg.data)
            self.get_logger().debug(f"From controller: {command}")

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

            elif command["type"] == "hand_control":
                target: str = command.get("target", "")
                control_type: str = command.get("control_type", "")
                action: str = command.get("action", "")

                enum_dict = {}
                if target.startswith("yagura"):
                    if control_type == "pos":
                        enum_dict = _YAGURA_POS
                    elif control_type == "state":
                        enum_dict = _YAGURA_STATE
                elif target.startswith("ring"):
                    if control_type == "pos":
                        enum_dict = _RING_POS
                    elif control_type == "state":
                        enum_dict = _RING_STATE

                keys = [k for k, v in enum_dict.items() if v == action]

                if keys and keys[0] is not None:
                    self.hands_cntl.set_target(target, control_type, keys[0])
                else:
                    self.get_logger().warn(f"Unknown action: {action}")

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
        ハンド ESP32 が未接続でも、ホイールフィードバックは送信する。
        """
        if self.wheel_fb_buffer is None:
            self.get_logger().warn("Wheel feedback buffer is None")
            return

        wheel_fb = self.wheel_fb_buffer
        hand_fb = self.hand_fb_buffer

        if hand_fb is None and not self._hand_fb_warned:
            self.get_logger().warn(
                "Hand feedback not available (hand ESP32 not connected?)"
            )
            self._hand_fb_warned = True

        data: dict = {
            "m3508_rpms": {
                "fl": wheel_fb.m3508_rpms.fl,
                "fr": wheel_fb.m3508_rpms.fr,
                "rl": wheel_fb.m3508_rpms.rl,
                "rr": wheel_fb.m3508_rpms.rr,
            },
        }

        if wheel_fb.m3508_gains:
            data["m3508_gains"] = {
                "kp": wheel_fb.m3508_gains[0].kp,  # type: ignore[index]
                "ki": wheel_fb.m3508_gains[0].ki,  # type: ignore[index]
                "kd": wheel_fb.m3508_gains[0].kd,  # type: ignore[index]
            }
        else:
            self.get_logger().debug("m3508_gains is empty in wheel_feedback")

        if wheel_fb.m3508_terms:
            data["m3508_terms"] = {
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
            }
        else:
            self.get_logger().debug("m3508_terms is empty in wheel_feedback")

        if hand_fb is not None:
            self._hand_fb_warned = False  # 再接続時にフラグをリセット
            data["yagura"] = {
                "1_pos": _YAGURA_POS.get(hand_fb.yagura_1.pos, "stopped"),
                "1_state": _YAGURA_STATE.get(hand_fb.yagura_1.state, "stopped"),
                "2_pos": _YAGURA_POS.get(hand_fb.yagura_2.pos, "stopped"),
                "2_state": _YAGURA_STATE.get(hand_fb.yagura_2.state, "stopped"),
            }
            data["ring"] = {
                "1_pos": _RING_POS.get(hand_fb.ring_1.pos, "stopped"),
                "1_state": _RING_STATE.get(hand_fb.ring_1.state, "stopped"),
                "2_pos": _RING_POS.get(hand_fb.ring_2.pos, "stopped"),
                "2_state": _RING_STATE.get(hand_fb.ring_2.state, "stopped"),
            }

        msg = String()
        msg.data = json.dumps(data)
        self.pub_bt_feedback.publish(msg)

        self.wheel_fb_buffer = None
        self.hand_fb_buffer = None

    def send_control_command(self) -> None:
        """
        ESP32 へコマンドを送信する。50 ms ごとに呼び出される。
        wheel_control は manual モードのみ送信（auto モードは cmd_vel_bridge が担当）。
        hand_control は auto/manual 両モードで送信（on_arrive シーケンス対応）。
        """
        now = time.monotonic()
        dt = now - self._last_cmd_time
        self._last_cmd_time = now

        if self._nav_mode != "auto":
            OMEGA_THRESHOLD = 0.05  # rad/s: これ以下を「直進」とみなす
            VELOCITY_THRESHOLD = 0.01  # m/s: これ以下を「停止」とみなす
            cmd_omega = self.m3508_cntl.target_omega
            is_moving = self.m3508_cntl.target_velocity.length() > VELOCITY_THRESHOLD
            omega_correction = 0.0

            if self._current_yaw is not None:
                if not is_moving or abs(cmd_omega) > OMEGA_THRESHOLD:
                    # 停止中または意図的な旋回中は target_yaw を追従・PID をリセット
                    # （停止時にその場旋回で補正する挙動を防ぐ）
                    self._target_yaw = self._current_yaw
                    self._heading_pid.reset()
                else:
                    if self._target_yaw is None:
                        self._target_yaw = self._current_yaw
                    omega_correction = self._heading_pid.compute(
                        self._target_yaw, self._current_yaw, dt
                    )

            # ベース RPM（並進 + 意図的旋回）をスケーリング込みで計算し、
            # ヨー補正はスケーリング後に個別クランプで加算する。
            # omega_correction を calc_motor_rpms に混ぜると比例スケーリングで消えるため分離する。
            base_rpms = self.m3508_cntl.calc_motor_rpms(
                self.m3508_cntl.target_velocity,
                cmd_omega,
            )
            corrected_rpms = self.m3508_cntl.apply_omega_correction(base_rpms, omega_correction)

            wheel_msg = WheelMessage()
            wheel_msg.m3508_rpms.fl = corrected_rpms[0]
            wheel_msg.m3508_rpms.fr = corrected_rpms[1]
            wheel_msg.m3508_rpms.rl = corrected_rpms[2]
            wheel_msg.m3508_rpms.rr = corrected_rpms[3]

            if (
                self.m3508_cntl.target_kp is not None
                and self.m3508_cntl.target_ki is not None
                and self.m3508_cntl.target_kd is not None
            ):
                gains = PIDGains()
                gains.kp = self.m3508_cntl.target_kp
                gains.ki = self.m3508_cntl.target_ki
                gains.kd = self.m3508_cntl.target_kd
                wheel_msg.m3508_gains = [gains]  # type: ignore[assignment]

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
