import math

import pytest

from robot_control.vec2 import Vec2


class TestVec2Arithmetic:
    def test_add(self):
        result = Vec2(1, 2) + Vec2(3, 4)
        assert result.x == 4
        assert result.y == 6

    def test_sub(self):
        result = Vec2(5, 3) - Vec2(2, 1)
        assert result.x == 3
        assert result.y == 2

    def test_mul(self):
        result = Vec2(2, 3) * 4
        assert result.x == 8
        assert result.y == 12

    def test_div(self):
        result = Vec2(6, 9) / 3
        assert result.x == 2.0
        assert result.y == 3.0


class TestVec2Length:
    def test_unit_x(self):
        assert Vec2(1, 0).length() == 1.0

    def test_unit_y(self):
        assert Vec2(0, 1).length() == 1.0

    def test_345_triangle(self):
        assert Vec2(3, 4).length() == 5.0

    def test_zero(self):
        assert Vec2(0, 0).length() == 0.0


class TestVec2Normalize:
    def test_unit_vector(self):
        n = Vec2(3, 4).normalize()
        assert abs(n.length() - 1.0) < 1e-9

    def test_zero_vector(self):
        n = Vec2(0, 0).normalize()
        assert n.x == 0
        assert n.y == 0

    def test_direction_preserved(self):
        v = Vec2(5, 0)
        n = v.normalize()
        assert abs(n.x - 1.0) < 1e-9
        assert abs(n.y) < 1e-9


class TestVec2Rotate:
    def test_90_degrees(self):
        v = Vec2(1, 0).rotate(90)
        assert abs(v.x) < 1e-9
        assert abs(v.y - 1.0) < 1e-9

    def test_180_degrees(self):
        v = Vec2(1, 0).rotate(180)
        assert abs(v.x - (-1.0)) < 1e-9
        assert abs(v.y) < 1e-9

    def test_360_degrees(self):
        v = Vec2(3, 4).rotate(360)
        assert abs(v.x - 3.0) < 1e-9
        assert abs(v.y - 4.0) < 1e-9


class TestVec2ToTuple:
    def test_to_tuple(self):
        assert Vec2(1.5, 2.5).to_tuple() == (1.5, 2.5)
