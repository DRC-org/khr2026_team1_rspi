import pytest
import math
from robot_control.vec2 import Vec2
from robot_control.m3508 import M3508Controller
from robot_control.constants import M3508_MAX_RPM, L_X, L_Y, GEAR_RATIO, WHEEL_RADIUS

def test_m3508_init():
    ctrl = M3508Controller()
    assert ctrl.target_rpm_fl == 0.0
    assert ctrl.target_rpm_fr == 0.0
    assert ctrl.target_rpm_rl == 0.0
    assert ctrl.target_rpm_rr == 0.0
    assert ctrl.target_velocity.x == 0.0
    assert ctrl.target_velocity.y == 0.0
    assert ctrl.target_omega == 0.0

def test_m3508_calc_motor_rpms_forward():
    ctrl = M3508Controller()
    vel = Vec2(1.0, 0.0) # Forward 1 m/s
    omega = 0.0
    
    rpms = ctrl.calc_motor_rpms(vel, omega)
    
    # In a typical mecanum drive moving purely forward:
    # v_fl = vx, v_fr = vx, v_rl = vx, v_rr = vx
    expected_vel = 1.0
    rpm_calc_const = (60 * GEAR_RATIO) / (2 * math.pi * WHEEL_RADIUS)
    expected_rpm = expected_vel * rpm_calc_const
    
    assert len(rpms) == 4
    for rpm in rpms:
        assert math.isclose(rpm, expected_rpm)

def test_m3508_calc_motor_rpms_sideways():
    ctrl = M3508Controller()
    vel = Vec2(0.0, 1.0) # Sideways (left)
    omega = 0.0
    
    rpms = ctrl.calc_motor_rpms(vel, omega)
    rpm_calc_const = (60 * GEAR_RATIO) / (2 * math.pi * WHEEL_RADIUS)
    
    # Left: v_fl = -vy, v_fr = +vy, v_rl = +vy, v_rr = -vy
    assert math.isclose(rpms[0], -rpm_calc_const) # FL
    assert math.isclose(rpms[1], rpm_calc_const)  # FR
    assert math.isclose(rpms[2], rpm_calc_const)  # RL
    assert math.isclose(rpms[3], -rpm_calc_const) # RR

def test_m3508_calc_motor_rpms_rotation():
    ctrl = M3508Controller()
    vel = Vec2(0.0, 0.0)
    omega = 1.0 # 1 rad/s CCW
    
    rpms = ctrl.calc_motor_rpms(vel, omega)
    rpm_calc_const = (60 * GEAR_RATIO) / (2 * math.pi * WHEEL_RADIUS)
    geom = L_X + L_Y
    
    # Rotate CCW: v_fl = -geom*omega, v_fr = geom*omega, v_rl = -geom*omega, v_rr = geom*omega
    assert math.isclose(rpms[0], -geom * omega * rpm_calc_const) # FL
    assert math.isclose(rpms[1], geom * omega * rpm_calc_const)  # FR
    assert math.isclose(rpms[2], -geom * omega * rpm_calc_const) # RL
    assert math.isclose(rpms[3], geom * omega * rpm_calc_const)  # RR

def test_m3508_scaling():
    ctrl = M3508Controller()
    # Request extreme velocity that would exceed M3508_MAX_RPM
    vel = Vec2(100.0, 0.0)
    omega = 0.0
    
    rpms = ctrl.calc_motor_rpms(vel, omega)
    
    # Max RPM should hit M3508_MAX_RPM and all other RPMs scale down
    for rpm in rpms:
        assert math.isclose(abs(rpm), M3508_MAX_RPM)

def test_m3508_set_target_velocity():
    ctrl = M3508Controller()
    vel = Vec2(1.0, 0.0)
    omega = 0.0
    
    ctrl.set_target_velocity(vel, omega)
    
    assert ctrl.target_velocity.x == 1.0
    assert ctrl.target_velocity.y == 0.0
    assert ctrl.target_omega == 0.0
    
    rpm_calc_const = (60 * GEAR_RATIO) / (2 * math.pi * WHEEL_RADIUS)
    assert math.isclose(ctrl.target_rpm_fl, rpm_calc_const)
    assert math.isclose(ctrl.target_rpm_fr, rpm_calc_const)
    assert math.isclose(ctrl.target_rpm_rl, rpm_calc_const)
    assert math.isclose(ctrl.target_rpm_rr, rpm_calc_const)
