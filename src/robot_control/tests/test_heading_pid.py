import math

import pytest

from robot_control.heading_pid import HeadingPID


class TestNormalize:
    def test_181_wraps_negative(self):
        assert abs(HeadingPID._normalize(181.0) - (-179.0)) < 1e-9

    def test_minus_181_wraps_positive(self):
        assert abs(HeadingPID._normalize(-181.0) - 179.0) < 1e-9

    def test_360_wraps_to_zero(self):
        assert abs(HeadingPID._normalize(360.0)) < 1e-9

    def test_zero_unchanged(self):
        assert abs(HeadingPID._normalize(0.0)) < 1e-9

    def test_minus_180(self):
        assert abs(HeadingPID._normalize(-180.0) - (-180.0)) < 1e-9

    def test_180(self):
        assert abs(HeadingPID._normalize(180.0) - (-180.0)) < 1e-9


class TestPControl:
    def test_proportional_output(self):
        pid = HeadingPID(kp=0.01, ki=0.0, kd=0.0)
        output = pid.compute(target_deg=10.0, current_deg=0.0, dt=0.05)
        assert abs(output - 0.01 * 10.0) < 1e-9

    def test_negative_error(self):
        pid = HeadingPID(kp=0.1, ki=0.0, kd=0.0)
        output = pid.compute(target_deg=0.0, current_deg=10.0, dt=0.05)
        assert output < 0

    def test_zero_error(self):
        pid = HeadingPID(kp=0.1, ki=0.0, kd=0.0)
        output = pid.compute(target_deg=5.0, current_deg=5.0, dt=0.05)
        assert abs(output) < 1e-9


class TestAntiWindup:
    def test_integral_clamp(self):
        pid = HeadingPID(kp=0.0, ki=1.0, kd=0.0)
        for _ in range(1000):
            pid.compute(target_deg=100.0, current_deg=0.0, dt=0.1)
        assert pid._integral <= 30.0
        assert pid._integral >= -30.0

    def test_negative_integral_clamp(self):
        pid = HeadingPID(kp=0.0, ki=1.0, kd=0.0)
        for _ in range(1000):
            pid.compute(target_deg=-100.0, current_deg=0.0, dt=0.1)
        assert pid._integral >= -30.0


class TestOutputClamp:
    def test_max_output(self):
        pid = HeadingPID(kp=10.0, ki=0.0, kd=0.0)
        output = pid.compute(target_deg=90.0, current_deg=0.0, dt=0.05)
        assert output <= math.pi / 4 + 1e-9

    def test_min_output(self):
        pid = HeadingPID(kp=10.0, ki=0.0, kd=0.0)
        output = pid.compute(target_deg=-90.0, current_deg=0.0, dt=0.05)
        assert output >= -math.pi / 4 - 1e-9


class TestReset:
    def test_reset_clears_state(self):
        pid = HeadingPID(kp=0.1, ki=0.1, kd=0.1)
        pid.compute(target_deg=10.0, current_deg=0.0, dt=0.05)
        pid.reset()
        assert pid._integral == 0.0
        assert pid._prev_error == 0.0
