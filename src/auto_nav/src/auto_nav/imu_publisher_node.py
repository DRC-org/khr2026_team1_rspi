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

_G_TO_MS2 = 9.80665  # LSM9DS1 calcAccel は g 単位で返すため m/s² に変換

# キャリブレーション用静止判定の閾値 [m/s]
# omega_enc は直進時も 0 になるため、全輪の平均速度で判定する
_SPEED_MOVING_THRESHOLD = 0.02

# gz 共分散: バイアスキャリブレーション済みのため常に固定値を使用
# （キャリブ前は静止中に高くしてバイアス積分を防いでいたが、キャリブ後は不要）
_GZ_COV = 0.05

# 起動時バイアスキャリブレーション: 静止状態のサンプルを N 個収集して gz オフセットを推定
_CALIB_SAMPLES = 100  # ~5 秒分（wheel_feedback は ~20 Hz）


class ImuPublisherNode(Node):
    """
    wheel_feedback (WheelMessage) から LSM9DS1 の gyro/accel データを取り出し、
    sensor_msgs/Imu として /imu に配信するノード。

    - 角速度: gz (yaw) のみ EKF が使用する。起動時に静止キャリブレーションでバイアスを除去。
    - gz 共分散: バイアスキャリブレーション済みのため常に固定値。
      直進時は omega_enc ≈ 0 になるため、静止判定ベースの共分散切り替えは使わない。
    - 線形加速度: LSM9DS1 calcAccel は g 単位で返す。m/s² に変換して配信するが、
      EKF への入力は imu0_config で false のまま（有効化は別途判断）。
    - 姿勢: 屋内では磁気干渉で yaw が不正確なため使用しない（orientation_covariance[0] = -1）。
    """

    def __init__(self):
        super().__init__("imu_publisher_node")
        self._sub = self.create_subscription(
            WheelMessage, "wheel_feedback", self._on_feedback, 10
        )
        self._pub = self.create_publisher(Imu, "/imu", 10)

        self._gz_bias: float = 0.0
        self._calib_buf: list[float] = []
        self._calibrated: bool = False

    def _on_feedback(self, msg: WheelMessage) -> None:
        if not msg.lsm9ds1_values:
            # IMU データなし → EKF は /odom_raw のみで継続動作する
            return

        # キャリブレーション用の静止判定
        # omega_enc は直進時も 0 になるため、全輪の平均速度で判定する
        v_fl = msg.m3508_rpms.fl * RPM_TO_MS
        v_fr = msg.m3508_rpms.fr * RPM_TO_MS
        v_rl = msg.m3508_rpms.rl * RPM_TO_MS
        v_rr = msg.m3508_rpms.rr * RPM_TO_MS
        total_speed = (abs(v_fl) + abs(v_fr) + abs(v_rl) + abs(v_rr)) / 4.0
        is_stationary = total_speed <= _SPEED_MOVING_THRESHOLD

        imu_val = msg.lsm9ds1_values[0]
        gz_raw = imu_val.gz * math.pi / 180.0

        # 起動時キャリブレーション: 静止中のサンプルを蓄積してバイアスを推定
        if not self._calibrated:
            if is_stationary:
                self._calib_buf.append(gz_raw)
                if len(self._calib_buf) >= _CALIB_SAMPLES:
                    self._gz_bias = sum(self._calib_buf) / len(self._calib_buf)
                    self._calibrated = True
                    self.get_logger().info(
                        f"IMU gz bias calibrated: "
                        f"{math.degrees(self._gz_bias):.4f} deg/s "
                        f"({self._gz_bias:.6f} rad/s)"
                    )
            # キャリブ完了まで IMU を配信しない（EKF は /odom_raw のみで動作）
            return

        # gz 符号診断: ratio≈+1.0 なら正常、≈-1.0 なら gz を反転する必要あり
        omega_enc = -(v_fl + v_fr + v_rl + v_rr) / (4.0 * G)
        gz_corrected = gz_raw - self._gz_bias
        if abs(omega_enc) > 0.05:
            self.get_logger().debug(
                f"gz={gz_corrected:.4f} rad/s  omega_enc={omega_enc:.4f} rad/s  "
                f"ratio={gz_corrected/omega_enc:.2f}"
            )

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"

        # 姿勢は使用しない（-1 で robot_localization に無効を通知）
        imu_msg.orientation_covariance[0] = -1.0

        # 角速度 [rad/s]（バイアス補正済み）
        imu_msg.angular_velocity.x = imu_val.gx * math.pi / 180.0
        imu_msg.angular_velocity.y = imu_val.gy * math.pi / 180.0
        imu_msg.angular_velocity.z = gz_raw - self._gz_bias
        imu_msg.angular_velocity_covariance[0] = 0.1   # gx（2D未使用）
        imu_msg.angular_velocity_covariance[4] = 0.1   # gy（2D未使用）
        imu_msg.angular_velocity_covariance[8] = _GZ_COV

        # 線形加速度 [m/s²]（LSM9DS1 calcAccel は g 単位 → m/s² 変換済み）
        imu_msg.linear_acceleration.x = imu_val.ax * _G_TO_MS2
        imu_msg.linear_acceleration.y = imu_val.ay * _G_TO_MS2
        imu_msg.linear_acceleration.z = imu_val.az * _G_TO_MS2
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
