import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_control.heading_pid import HeadingPID
from robot_msgs.msg import WheelMessage
from std_msgs.msg import String

# robot_control/constants.py と同じ値
WHEEL_RADIUS = 0.04925
GEAR_RATIO = 19.20320855614973
L_X = 0.1725
L_Y = 0.2425
G = L_X + L_Y
MAX_RPM = 12000.0
# M3508 が電流不足で不動になるのを防ぐ最小 RPM（静止摩擦を超えるしきい値）
# 300 RPM ≈ 0.08 m/s — 静止摩擦を超えるのに十分な最小値
MIN_RPM = 300.0
# cmd_vel がこの秒数来なければゼロ RPM を送って停止（ESP32 watchdog より確実な安全停止）
CMD_VEL_TIMEOUT = 0.3

MS_TO_RPM = (60.0 * GEAR_RATIO) / (2.0 * math.pi * WHEEL_RADIUS)


class CmdVelBridgeNode(Node):
    """
    /cmd_vel (geometry_msgs/Twist) を wheel_control (WheelMessage) に変換するノード。

    /nav_mode が "auto" のときだけ変換・送信する。
    "manual" のときは何も送信しない（robot_control ノードが担当）。

    逆運動学（m3508.py と同じ符号系）:
        v_fl = +Vx + Vy - G*ω
        v_fr = -Vx + Vy - G*ω
        v_rl = +Vx - Vy - G*ω
        v_rr = -Vx - Vy - G*ω
    """

    def __init__(self):
        super().__init__("cmd_vel_bridge_node")

        self._mode = "manual"
        self._heading_pid = HeadingPID(kp=0.02, ki=0.0, kd=0.001)
        self._current_yaw: float | None = None
        self._target_yaw: float | None = None
        self._last_twist: Twist | None = None
        self._last_cmd_vel_time: float | None = None
        self._last_timer_time: float = time.monotonic()

        self._sub_mode = self.create_subscription(
            String, "/nav_mode", self._on_mode, 10
        )
        self._sub_cmd_vel = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10
        )
        self._sub_odom = self.create_subscription(
            Odometry, "/odom", self._on_odom, 10
        )
        self._pub = self.create_publisher(WheelMessage, "wheel_control", 10)
        self.create_timer(0.05, self._on_timer)

        self.get_logger().info("CmdVel Bridge Node initialized (mode: manual)")

    def _on_mode(self, msg: String) -> None:
        prev = self._mode
        self._mode = msg.data
        self.get_logger().info(f"nav_mode: {prev} → {self._mode}")

        # モード切り替え時に PID 状態をリセット（前モードのヨー角を引き継がない）
        self._heading_pid.reset()
        self._target_yaw = None

        # auto → manual 切り替え時にゼロ指令を送って急停止を防ぐ
        if prev == "auto" and self._mode == "manual":
            self._publish_rpms(0.0, 0.0, 0.0, 0.0)

        # manual → auto 切り替え時にタイマー・キャッシュ状態をリセット
        if prev == "manual" and self._mode == "auto":
            self._last_twist = None
            self._last_cmd_vel_time = None
            self._last_timer_time = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self._mode != "auto":
            return
        self._last_twist = msg
        self._last_cmd_vel_time = time.monotonic()

    def _on_timer(self) -> None:
        if self._mode != "auto":
            return

        now = time.monotonic()
        dt = now - self._last_timer_time
        self._last_timer_time = now

        # cmd_vel が来ていない、またはタイムアウト → ゼロ指令で停止
        if self._last_twist is None or self._last_cmd_vel_time is None:
            return
        if now - self._last_cmd_vel_time > CMD_VEL_TIMEOUT:
            self._publish_rpms(0.0, 0.0, 0.0, 0.0)
            self._last_twist = None
            self._heading_pid.reset()
            self._target_yaw = None
            return

        msg = self._last_twist
        vx = msg.linear.x   # 前後 [m/s]
        vy = msg.linear.y   # 左右 [m/s]（左が正）
        omega = msg.angular.z  # 角速度 [rad/s]（反時計が正）

        # 逆運動学: ロボット速度 → 各車輪の線速度 [m/s]
        v_fl = +vx + vy - G * omega
        v_fr = -vx + vy - G * omega
        v_rl = +vx - vy - G * omega
        v_rr = -vx - vy - G * omega

        # 線速度 → RPM
        rpms = [v * MS_TO_RPM for v in (v_fl, v_fr, v_rl, v_rr)]

        # 最大 RPM を超える場合はスケーリング
        max_abs = max(abs(r) for r in rpms)
        if max_abs > MAX_RPM:
            scale = MAX_RPM / max_abs
            rpms = [r * scale for r in rpms]
        elif 0 < max_abs < MIN_RPM:
            # 車輪比率を保ったまま MIN_RPM まで底上げ（静止摩擦対策）
            scale = MIN_RPM / max_abs
            rpms = [r * scale for r in rpms]

        # ヨー角維持 PID（直進中のドリフト抑制）
        OMEGA_THRESHOLD = 0.02   # rad/s: Nav2 のゆっくりしたカーブ中も PID を無効化
        VELOCITY_THRESHOLD = 0.01  # m/s: これ以下を「停止」とみなす

        if self._current_yaw is not None:
            is_moving = math.hypot(vx, vy) > VELOCITY_THRESHOLD
            if not is_moving or abs(omega) > OMEGA_THRESHOLD:
                # 停止中・意図的旋回中: target_yaw を追従し PID をリセット
                self._target_yaw = self._current_yaw
                self._heading_pid.reset()
            else:
                if self._target_yaw is None:
                    self._target_yaw = self._current_yaw
                omega_correction = self._heading_pid.compute(
                    self._target_yaw, self._current_yaw, dt
                )
                # 逆運動学より純回転は全 4 輪に -G*omega が均等に加わる
                delta = -G * omega_correction * MS_TO_RPM
                rpms = [max(-MAX_RPM, min(MAX_RPM, r + delta)) for r in rpms]

        self._publish_rpms(*rpms)

    def _publish_rpms(
        self, fl: float, fr: float, rl: float, rr: float
    ) -> None:
        msg = WheelMessage()
        msg.m3508_rpms.fl = fl
        msg.m3508_rpms.fr = fr
        msg.m3508_rpms.rl = rl
        msg.m3508_rpms.rr = rr
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
