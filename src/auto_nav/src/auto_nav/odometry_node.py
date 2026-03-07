import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_msgs.msg import WheelMessage


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
    /odom_raw を配信するノード。TF(odom→base_link) は ekf_filter_node が担当する。

    前進運動学（逆運動学の疑似逆行列から導出）:
        v_fl = Vx + Vy - G*ω
        v_fr = -Vx + Vy - G*ω   (FR/RR は取付向きにより符号反転済み)
        v_rl = Vx - Vy - G*ω
        v_rr = -Vx - Vy - G*ω

        → Vx    = ( v_fl - v_fr + v_rl - v_rr) / 4
           Vy    = -( v_fl + v_fr - v_rl - v_rr) / 4
           omega = -(v_fl + v_fr + v_rl + v_rr) / (4 * G)
    """

    def __init__(self):
        super().__init__("odometry_node")

        self._sub = self.create_subscription(
            WheelMessage, "wheel_feedback", self._on_feedback, 10
        )
        self._pub_odom = self.create_publisher(Odometry, "/odom_raw", 10)

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
        vy = -(v_fl + v_fr - v_rl - v_rr) / 4.0  # 左右速度 [m/s]（左が正）
        omega = -(v_fl + v_fr + v_rl + v_rr) / (4.0 * G)  # 角速度 [rad/s]

        # デッドレコニング積分（オイラー法）
        # 位置積分には更新前の theta を使う（theta_old）
        # 先に theta を使うと積分タイミングがずれ、回転中に誤差が蓄積する
        theta_old = self._theta
        self._theta += omega * dt
        self._x += (vx * math.cos(theta_old) - vy * math.sin(theta_old)) * dt
        self._y += (vx * math.sin(theta_old) + vy * math.cos(theta_old)) * dt

        # θ から四元数（z軸回転のみ）
        qz = math.sin(self._theta / 2.0)
        qw = math.cos(self._theta / 2.0)
        stamp = now.to_msg()

        self._publish_odom(stamp, vx, vy, omega, qz, qw)

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

        # 共分散行列（6x6 = 36要素, row-major）
        # 2D 走行のため z/roll/pitch 成分は大きな値（実質未知）にする
        # インデックス: 0=x,x  7=y,y  14=z,z  21=rr  28=pp  35=yaw,yaw
        odom.pose.covariance[0] = 0.01    # x [m^2]
        odom.pose.covariance[7] = 0.01    # y [m^2]
        odom.pose.covariance[14] = 1e6    # z (unused in 2D)
        odom.pose.covariance[21] = 1e6    # roll (unused in 2D)
        odom.pose.covariance[28] = 1e6    # pitch (unused in 2D)
        odom.pose.covariance[35] = 0.1    # yaw [rad^2]（IMU融合を EKF に委譲するため不確実性を増やす）

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        odom.twist.covariance[0] = 0.01   # vx [m^2/s^2]
        odom.twist.covariance[7] = 0.01   # vy [m^2/s^2]
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.1   # omega [rad^2/s^2]（同上）

        self._pub_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
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
