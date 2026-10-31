"""Shared pose and steering helpers used by the straight range-guide runtime."""
from dataclasses import dataclass, asdict
import math


@dataclass
class Pose:
    x: float
    y: float
    yaw: float = 0.0
    speed: float = 0.0

    def json(self):
        return asdict(self)


def clamp(value, low, high):
    return max(low, min(high, value))


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def steer(pose, target, speed):
    dx, dy = target[0]-pose.x, target[1]-pose.y
    d = math.hypot(dx, dy)
    error = angle(math.atan2(dy, dx)-pose.yaw)
    w = clamp(2.6*error, -0.85, 0.85)
    # 大きな向きの差では、その場旋回する。曲がり角の内側を横切らない。
    v = min(speed, 1.2*d) * max(0, math.cos(error)) if abs(error) < 0.4 else 0.0
    return v, w
