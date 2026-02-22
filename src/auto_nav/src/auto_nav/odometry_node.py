import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_msgs.msg import WheelMessage
from tf2_ros import TransformBroadcaster

# robot_control/constants.py と同じ値を使う
WHEEL_RADIUS = 0.04925  # タイヤ半径 [m]
GEAR_RATIO = 19.20320855614973  # ギア比
L_X = 0.1725  # ロボット中心 → タイヤ中心（前後方向）[m]
L_Y = 0.2425  # ロボット中心 → タイヤ中心（左右方向）[m]
G = L_X + L_Y  # 前進運動学の幾何係数

# RPM → タイヤ接地面の線速度 [m/s]
RPM_TO_MS = 2.0 * math.pi * WHEEL_RADIUS / (60.0 * GEAR_RATIO)


class OdometryNode(Node):
    """
    wheel_feedback (WheelMessage) を受け取り、
    4輪オムニの前進運動学でロボット速度を計算し、
    /odom と TF (odom → base_link) を配信するノード。

    前進運動学（逆運動学の疑似逆行列から導出）:
        v_fl = Vx + Vy - G*ω
        v_fr = -Vx + Vy - G*ω   (FR/RR は取付向きにより符号反転済み)
        v_rl = Vx - Vy - G*ω
        v_rr = -Vx - Vy - G*ω

        → Vx    = ( v_fl - v_fr + v_rl - v_rr) / 4
           Vy    = ( v_fl + v_fr - v_rl - v_rr) / 4
           omega = -(v_fl + v_fr + v_rl + v_rr) / (4 * G)
    """

    def __init__(self):
        super().__init__("odometry_node")

        self._sub = self.create_subscription(
            WheelMessage, "wheel_feedback", self._on_feedback, 10
        )
        self._pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self._tf_br = TransformBroadcaster(self)

        # デッドレコニング状態
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_time = None

        self.get_logger().info("Odometry Node initialized")

    def _on_feedback(self, msg: WheelMessage) -> None:
        now = self.get_clock().now()

        if self._last_time is None:
            self._last_time = now
            return

        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        # dt が異常値のときはスキップ（起動直後・通信途絶など）
        if dt <= 0.0 or dt > 0.5:
            return

        # RPM → 車輪の線速度 [m/s]
        # FR・RR は取付向きが逆なので、コマンド時に符号反転されている。
        # フィードバック RPM はその符号をそのまま返すため、変換式はそのまま適用できる。
        v_fl = msg.m3508_rpms.fl * RPM_TO_MS
        v_fr = msg.m3508_rpms.fr * RPM_TO_MS
        v_rl = msg.m3508_rpms.rl * RPM_TO_MS
        v_rr = msg.m3508_rpms.rr * RPM_TO_MS

        # 前進運動学: 車輪速度 → ロボット機体速度
        vx = (v_fl - v_fr + v_rl - v_rr) / 4.0  # 前後速度 [m/s]
        vy = (v_fl + v_fr - v_rl - v_rr) / 4.0  # 左右速度 [m/s]（左が正）
        omega = -(v_fl + v_fr + v_rl + v_rr) / (4.0 * G)  # 角速度 [rad/s]（反時計が正）

        # デッドレコニング積分（オイラー法）
        self._theta += omega * dt
        self._x += (vx * math.cos(self._theta) - vy * math.sin(self._theta)) * dt
        self._y += (vx * math.sin(self._theta) + vy * math.cos(self._theta)) * dt

        # θ から四元数（z軸回転のみ）
        qz = math.sin(self._theta / 2.0)
        qw = math.cos(self._theta / 2.0)
        stamp = now.to_msg()

        self._publish_odom(stamp, vx, vy, omega, qz, qw)
        self._broadcast_tf(stamp, qz, qw)

    def _publish_odom(
        self,
        stamp,
        vx: float,
        vy: float,
        omega: float,
        qz: float,
        qw: float,
    ) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        self._pub_odom.publish(odom)

    def _broadcast_tf(self, stamp, qz: float, qw: float) -> None:
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_br.sendTransform(tf)
