import math


class HeadingPID:
    def __init__(self, kp: float = 0.05, ki: float = 0.0, kd: float = 0.005):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral = 0.0
        self._prev_error = 0.0

    @staticmethod
    def _normalize(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0

    def compute(self, target_deg: float, current_deg: float, dt: float) -> float:
        error = self._normalize(target_deg - current_deg)
        self._integral += error * dt
        self._integral = max(-30.0, min(30.0, self._integral))  # アンチワインドアップ
        derivative = (error - self._prev_error) / dt if dt > 1e-6 else 0.0
        self._prev_error = error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-math.pi / 4, min(math.pi / 4, output))  # 最大 ±45°/s に制限

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
