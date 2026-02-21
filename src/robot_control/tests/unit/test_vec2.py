import math
from robot_control.vec2 import Vec2

def test_vec2_init():
    v = Vec2(1.5, 2.5)
    assert v.x == 1.5
    assert v.y == 2.5

def test_vec2_default_init():
    v = Vec2()
    assert v.x == 0.0
    assert v.y == 0.0

def test_vec2_add():
    v1 = Vec2(1.0, 2.0)
    v2 = Vec2(3.0, 4.0)
    v3 = v1 + v2
    assert v3.x == 4.0
    assert v3.y == 6.0

def test_vec2_sub():
    v1 = Vec2(5.0, 5.0)
    v2 = Vec2(2.0, 3.0)
    v3 = v1 - v2
    assert v3.x == 3.0
    assert v3.y == 2.0

def test_vec2_mul():
    v1 = Vec2(2.0, 3.0)
    v2 = v1 * 2.0
    assert v2.x == 4.0
    assert v2.y == 6.0

def test_vec2_truediv():
    v1 = Vec2(4.0, 6.0)
    v2 = v1 / 2.0
    assert v2.x == 2.0
    assert v2.y == 3.0

def test_vec2_length():
    v = Vec2(3.0, 4.0)
    assert v.length() == 5.0

def test_vec2_normalize():
    v = Vec2(3.0, 4.0)
    v_norm = v.normalize()
    assert math.isclose(v_norm.length(), 1.0)
    assert math.isclose(v_norm.x, 0.6)
    assert math.isclose(v_norm.y, 0.8)

def test_vec2_normalize_zero():
    v = Vec2(0.0, 0.0)
    v_norm = v.normalize()
    assert v_norm.x == 0.0
    assert v_norm.y == 0.0

def test_vec2_rotate():
    v = Vec2(1.0, 0.0)
    v_rot = v.rotate(90.0)
    assert math.isclose(v_rot.x, 0.0, abs_tol=1e-9)
    assert math.isclose(v_rot.y, 1.0)

def test_vec2_to_tuple():
    v = Vec2(1.2, 3.4)
    t = v.to_tuple()
    assert t == (1.2, 3.4)
