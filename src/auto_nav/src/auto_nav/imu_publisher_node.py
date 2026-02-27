import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from robot_msgs.msg import WheelMessage

# odometry_node.py と同じ定数（omega_enc 計算に使用）
WHEEL_RADIUS = 0.04925
GEAR_RATIO = 19.20320855614973
L_X = 0.1725
L_Y = 0.2425
G = L_X + L_Y
RPM_TO_MS = 2.0 * math.pi * WHEEL_RADIUS / (60.0 * GEAR_RATIO)

# 静止判定の閾値 [rad/s]
# エンコーダ由来の omega がこれ未満のとき「静止」と判断し、
# gz バイアスを EKF に積分させないよう共分散を大きくする
_OMEGA_MOVING_THRESHOLD = 0.01

# gz 共分散: 走行中（バイアス影響小）と静止中（バイアスを EKF に無視させる）
# 旧値 0.01 は IMU を強く信頼しすぎてスパイク時に直撃した。
# エンコーダ omega 共分散(0.1)と近い値にすることでスパイク耐性を向上。
_GZ_COV_MOVING = 0.05
_GZ_COV_STATIONARY = 10.0  # 実質無視


class ImuPublisherNode(Node):
    """
    wheel_feedback (WheelMessage) から LSM9DS1 の gyro/accel データを取り出し、
    sensor_msgs/Imu として /imu に配信するノード。

    - 角速度: gz (yaw) のみ EKF が使用する。gx/gy は 2D 走行では不使用。
    - 静止検出: エンコーダ omega が閾値未満のとき gz 共分散を大きくし、
      ジャイロバイアスが EKF の yaw 推定に積分されるのを防ぐ。
      （旧 odometry_node の `abs(omega_enc) > 0.01` 条件と同等の効果）
    - 線形加速度: 第一フェーズでは EKF に渡さない（imu0_config で false）。
      ax/ay の単位（mg か m/s^2 か）を実測確認後、第二フェーズで有効化を検討する。
    - 姿勢: 屋内では磁気干渉で yaw が不正確なため使用しない（orientation_covariance[0] = -1）。
    """

    def __init__(self):
        super().__init__("imu_publisher_node")
        self._sub = self.create_subscription(
            WheelMessage, "wheel_feedback", self._on_feedback, 10
        )
        self._pub = self.create_publisher(Imu, "/imu", 10)

    def _on_feedback(self, msg: WheelMessage) -> None:
        if not msg.lsm9ds1_values:
            # IMU データなし → EKF は /odom_raw のみで継続動作する
            return

        # エンコーダ由来の omega で静止判定
        v_fl = msg.m3508_rpms.fl * RPM_TO_MS
        v_fr = msg.m3508_rpms.fr * RPM_TO_MS
        v_rl = msg.m3508_rpms.rl * RPM_TO_MS
        v_rr = msg.m3508_rpms.rr * RPM_TO_MS
        omega_enc = -(v_fl + v_fr + v_rl + v_rr) / (4.0 * G)
        is_moving = abs(omega_enc) > _OMEGA_MOVING_THRESHOLD

        imu_val = msg.lsm9ds1_values[0]
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"

        # 姿勢は使用しない（-1 で robot_localization に無効を通知）
        imu_msg.orientation_covariance[0] = -1.0

        # 角速度 [rad/s]（LSM9DS1 は dps 出力）
        imu_msg.angular_velocity.x = imu_val.gx * math.pi / 180.0
        imu_msg.angular_velocity.y = imu_val.gy * math.pi / 180.0
        imu_msg.angular_velocity.z = imu_val.gz * math.pi / 180.0
        imu_msg.angular_velocity_covariance[0] = 0.1   # gx（2D未使用）
        imu_msg.angular_velocity_covariance[4] = 0.1   # gy（2D未使用）
        # 静止中はジャイロバイアスを EKF に積分させない
        imu_msg.angular_velocity_covariance[8] = _GZ_COV_MOVING if is_moving else _GZ_COV_STATIONARY

        # 線形加速度: 単位未確認のため ekf_params.yaml 側で無効化済み
        imu_msg.linear_acceleration.x = imu_val.ax
        imu_msg.linear_acceleration.y = imu_val.ay
        imu_msg.linear_acceleration.z = imu_val.az
        imu_msg.linear_acceleration_covariance[0] = 0.05
        imu_msg.linear_acceleration_covariance[4] = 0.05
        imu_msg.linear_acceleration_covariance[8] = 0.05

        self._pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
