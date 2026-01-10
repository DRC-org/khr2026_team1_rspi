from math import cos, radians, sin


class Vec2:
    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = x
        self.y = y

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> "Vec2":
        return Vec2(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar: float) -> "Vec2":
        return Vec2(self.x / scalar, self.y / scalar)

    def length(self) -> float:
        return (self.x**2 + self.y**2) ** 0.5

    def normalize(self) -> "Vec2":
        length_ = self.length()
        if length_ == 0:
            return Vec2(0, 0)
        return self / length_

    def rotate(self, angle_deg: float) -> "Vec2":
        angle_rad = radians(angle_deg)
        cos_a = cos(angle_rad)
        sin_a = sin(angle_rad)
        return Vec2(
            self.x * cos_a - self.y * sin_a,
            self.x * sin_a + self.y * cos_a,
        )

    def to_tuple(self) -> tuple:
        return (self.x, self.y)
