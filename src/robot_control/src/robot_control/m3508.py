import math

from .constants import M3508_MAX_RPM, WHEEL_RADIUS
from .vec2 import Vec2


class M3508Controller:
    """
    M3508 モータの制御クラス
    """

    def __init__(self):
        self.target_rpm_fl = 0.0  # Front Left (ID: 1)
        self.target_rpm_fr = 0.0  # Front Right (ID: 2)
        self.target_rpm_rl = 0.0  # Rear Left (ID: 3)
        self.target_rpm_rr = 0.0  # Rear Right (ID: 4)

        self.target_velocity = Vec2(0.0, 0.0)
        self.target_omega = 0.0  # 反時計回りの角速度 [rad/s]

    def set_target_velocity(self, velocity: Vec2, omega: float) -> None:
        """
        目標速度を設定する
        Args:
            velocity (Vec2): 目標速度ベクトル [m/s]
            omega (float): 反時計回りの角速度 [rad/s]
        """
        self.target_velocity = velocity
        self.target_omega = omega

        target_rpms = self.calc_motor_rpms(self.target_velocity, self.target_omega)
        self.target_rpm_fl = target_rpms[0]
        self.target_rpm_fr = target_rpms[1]
        self.target_rpm_rl = target_rpms[2]
        self.target_rpm_rr = target_rpms[3]

    def calc_motor_rpms(self, velocity: Vec2, omega: float) -> list[float]:
        """
        Vx, Vy, Omega から各モータの RPM を計算する

        Args:
            velocity (Vec2): 速度ベクトル [m/s]
            omega (float): 反時計回りの角速度 [rad/s]

        Returns:
            list[float]: 各モータの目標回転数 [rev/min]
        """

        l_x = 0.1725  # ロボットの中心からタイヤの中心まで（前後方向） [m]
        l_y = 0.2425  # ロボットの中心からタイヤの中心まで（左右方向） [m]
        gear_ratio = 19.20320855614973  # モータのギア比

        geometry_factor = l_x + l_y
        rpm_calc_const = (60 * gear_ratio) / (2 * math.pi * WHEEL_RADIUS)

        # 機体の傾きをベクトルに反映
        # TODO: IMU から傾きを取得して反映させる
        rotated_velocity = velocity.rotate(0.0)
        vx = rotated_velocity.x  # 前後方向の速度 [m/s]
        vy = rotated_velocity.y  # 左右方向の速度 [m/s]

        # 逆運動学方程式
        v_fl = vx - vy - geometry_factor * omega
        v_fr = vx + vy + geometry_factor * omega
        v_rl = vx + vy - geometry_factor * omega
        v_rr = vx - vy + geometry_factor * omega

        raw_velocities = [v_fl, v_fr, v_rl, v_rr]

        # 各モータの回転数を計算
        rpms = [v * rpm_calc_const for v in raw_velocities]

        # 最大回転数を超える場合はスケーリング
        max_rpm_magnitude = max([abs(rpm) for rpm in rpms])

        if max_rpm_magnitude > M3508_MAX_RPM:
            scale_factor = M3508_MAX_RPM / max_rpm_magnitude
            rpms = [rpm * scale_factor for rpm in rpms]

        return rpms
