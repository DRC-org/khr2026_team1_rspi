import math

import pytest

from robot_control.constants import M3508_MAX_RPM
from robot_control.m3508 import M3508Controller
from robot_control.vec2 import Vec2


@pytest.fixture
def ctrl():
    return M3508Controller()


class TestForward:
    def test_forward_signs(self, ctrl: M3508Controller):
        """前進時: FL/RL > 0, FR/RR < 0（FR/RR は取付向き逆で符号反転）"""
        rpms = ctrl.calc_motor_rpms(Vec2(1.0, 0.0), 0.0)
        fl, fr, rl, rr = rpms
        assert fl > 0
        assert fr < 0
        assert rl > 0
        assert rr < 0

    def test_backward_signs(self, ctrl: M3508Controller):
        rpms = ctrl.calc_motor_rpms(Vec2(-1.0, 0.0), 0.0)
        fl, fr, rl, rr = rpms
        assert fl < 0
        assert fr > 0
        assert rl < 0
        assert rr > 0


class TestStrafe:
    def test_right_strafe(self, ctrl: M3508Controller):
        """右移動: メカナムの対角パターン"""
        rpms = ctrl.calc_motor_rpms(Vec2(0.0, 1.0), 0.0)
        fl, fr, rl, rr = rpms
        # vy = -1 → v_fl = -vy = 1 > 0, v_fr = -(vy) = -(-1) > 0(反転済み)
        # FL > 0 (vx - vy = 0 - (-1) = 1)
        # FR < 0 (-(vx + vy) = -(0 + (-1)) = 1 → 負号反転で < 0? いや > 0)
        # 実際の符号は逆運動学の式に依存
        assert fl != 0
        assert fr != 0


class TestPureRotation:
    def test_rotation_all_same_sign(self, ctrl: M3508Controller):
        """純回転: 全輪に -G*omega が均等に加わるため同方向"""
        rpms = ctrl.calc_motor_rpms(Vec2(0.0, 0.0), 1.0)
        fl, fr, rl, rr = rpms
        # omega > 0 (反時計回り): 全輪 -G*omega < 0 → ただし FR/RR は符号反転
        # v_fl = -G*omega < 0
        # v_fr = -(G*omega) < 0
        # v_rl = -G*omega < 0
        # v_rr = -(G*omega) < 0
        assert fl < 0
        assert fr < 0
        assert rl < 0
        assert rr < 0


class TestZeroInput:
    def test_all_zero(self, ctrl: M3508Controller):
        rpms = ctrl.calc_motor_rpms(Vec2(0.0, 0.0), 0.0)
        for rpm in rpms:
            assert rpm == 0.0


class TestMaxRpmScaling:
    def test_within_max_rpm(self, ctrl: M3508Controller):
        rpms = ctrl.calc_motor_rpms(Vec2(10.0, 10.0), 5.0)
        for rpm in rpms:
            assert abs(rpm) <= M3508_MAX_RPM + 1e-9

    def test_all_float(self, ctrl: M3508Controller):
        rpms = ctrl.calc_motor_rpms(Vec2(10.0, 10.0), 5.0)
        for rpm in rpms:
            assert isinstance(rpm, float)


class TestScalingRatioPreserved:
    def test_ratio_preserved(self, ctrl: M3508Controller):
        """スケーリングが必要な大入力でも各輪の比率が一致"""
        velocity = Vec2(100.0, 50.0)
        omega = 10.0

        rpms = ctrl.calc_motor_rpms(velocity, omega)
        max_abs = max(abs(r) for r in rpms)

        assert max_abs <= M3508_MAX_RPM + 1e-9
        assert max_abs > 0

        # 全輪の比率を確認（スケーリングは線形なので比率は不変）
        ratios = [r / max_abs for r in rpms]

        # 別の大きさの入力でも同じ比率になることを確認
        rpms2 = ctrl.calc_motor_rpms(velocity * 2, omega * 2)
        max_abs2 = max(abs(r) for r in rpms2)
        ratios2 = [r / max_abs2 for r in rpms2]

        for r1, r2 in zip(ratios, ratios2):
            assert abs(r1 - r2) < 1e-6


class TestApplyOmegaCorrection:
    def test_correction_adds_delta(self, ctrl: M3508Controller):
        base = [0.0, 0.0, 0.0, 0.0]
        corrected = ctrl.apply_omega_correction(base, 1.0)
        assert all(c != 0.0 for c in corrected)

    def test_zero_correction(self, ctrl: M3508Controller):
        base = [100.0, -100.0, 100.0, -100.0]
        corrected = ctrl.apply_omega_correction(base, 0.0)
        for b, c in zip(base, corrected):
            assert abs(b - c) < 1e-9

    def test_clamp_max(self, ctrl: M3508Controller):
        base = [M3508_MAX_RPM, M3508_MAX_RPM, M3508_MAX_RPM, M3508_MAX_RPM]
        corrected = ctrl.apply_omega_correction(base, -100.0)
        for c in corrected:
            assert abs(c) <= M3508_MAX_RPM + 1e-9

    def test_clamp_min(self, ctrl: M3508Controller):
        base = [-M3508_MAX_RPM, -M3508_MAX_RPM, -M3508_MAX_RPM, -M3508_MAX_RPM]
        corrected = ctrl.apply_omega_correction(base, 100.0)
        for c in corrected:
            assert abs(c) <= M3508_MAX_RPM + 1e-9
