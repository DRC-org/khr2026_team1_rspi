import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from robot_msgs.msg import WheelMessage
from std_msgs.msg import String

# robot_control/constants.py と同じ値
WHEEL_RADIUS = 0.04925
GEAR_RATIO = 19.20320855614973
L_X = 0.1725
L_Y = 0.2425
G = L_X + L_Y
MAX_RPM = 12000.0
MIN_RPM_FWD = 100.0
MIN_RPM_LAT = 100.0
# cmd_vel がこの秒数来なければゼロ RPM を送って停止（ESP32 watchdog より確実な安全停止）
CMD_VEL_TIMEOUT = 0.3

MS_TO_RPM = (60.0 * GEAR_RATIO) / (2.0 * math.pi * WHEEL_RADIUS)


class CmdVelBridgeNode(Node):
    """
    /cmd_vel (geometry_msgs/Twist) を wheel_control (WheelMessage) に変換するノード。

    /nav_mode が "auto" のときだけ変換・送信する。
    "manual" のときは何も送信しない（robot_control ノードが担当）。

    逆運動学（ROS標準座標系: Vy>0=左）:
        v_fl = +Vx - Vy - G*ω
        v_fr = -Vx - Vy - G*ω
        v_rl = +Vx + Vy - G*ω
        v_rr = -Vx + Vy - G*ω
    """

    def __init__(self):
        super().__init__("cmd_vel_bridge_node")

        self._mode = "manual"
        self._last_twist: Twist | None = None
        self._last_cmd_vel_time: float | None = None

        self._sub_mode = self.create_subscription(
            String, "/nav_mode", self._on_mode, 10
        )
        self._sub_cmd_vel = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10
        )
        self._pub = self.create_publisher(WheelMessage, "wheel_control", 10)
        self.create_timer(0.05, self._on_timer)

        self.get_logger().info("CmdVel Bridge Node initialized (mode: manual)")

    def _on_mode(self, msg: String) -> None:
        prev = self._mode
        self._mode = msg.data
        self.get_logger().info(f"nav_mode: {prev} → {self._mode}")

        # auto → manual 切り替え時にゼロ指令を送って急停止を防ぐ
        if prev == "auto" and self._mode == "manual":
            self._publish_rpms(0.0, 0.0, 0.0, 0.0)

        # manual → auto 切り替え時にキャッシュ状態をリセット
        if prev == "manual" and self._mode == "auto":
            self._last_twist = None
            self._last_cmd_vel_time = None

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self._mode != "auto":
            return
        self._last_twist = msg
        self._last_cmd_vel_time = time.monotonic()

    def _on_timer(self) -> None:
        if self._mode != "auto":
            return

        if self._last_twist is None or self._last_cmd_vel_time is None:
            return
        if time.monotonic() - self._last_cmd_vel_time > CMD_VEL_TIMEOUT:
            self._publish_rpms(0.0, 0.0, 0.0, 0.0)
            self._last_twist = None
            return

        msg = self._last_twist
        vx = msg.linear.x   # 前後 [m/s]
        # メカナム横移動はローラー空転でスリップするため、指令値を増幅して実移動量を補償する
        vy = msg.linear.y * 1.35   # 左右 [m/s]（左が正）
        omega = msg.angular.z  # 角速度 [rad/s]（反時計が正）

        # 逆運動学: ロボット速度 → 各車輪の線速度 [m/s]
        # ROS標準座標系(Vy>0=左)のまま使用。m3508.py は内部で vy を反転しているため符号が異なる
        v_fl = +vx - vy - G * omega
        v_fr = -vx - vy - G * omega
        v_rl = +vx + vy - G * omega
        v_rr = -vx + vy - G * omega

        # 線速度 → RPM
        rpms = [v * MS_TO_RPM for v in (v_fl, v_fr, v_rl, v_rr)]

        max_abs = max(abs(r) for r in rpms)

        if max_abs > MAX_RPM:
            scale = MAX_RPM / max_abs
            rpms = [r * scale for r in rpms]
        elif 0.0 < max_abs:
            lateral_ratio = abs(vy) / (abs(vx) + abs(vy) + 1e-9)
            effective_min = MIN_RPM_FWD + (MIN_RPM_LAT - MIN_RPM_FWD) * lateral_ratio
            if max_abs < effective_min:
                scale = effective_min / max_abs
                rpms = [r * scale for r in rpms]

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
