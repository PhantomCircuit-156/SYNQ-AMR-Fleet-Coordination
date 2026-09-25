"""MiR100 wheel control and simulated localization, without traffic policy."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, inf, sin

from config.webots import (
    HALF_WHEEL_TRACK_METERS,
    HEADING_TOLERANCE_RADIANS,
    MAX_ANGULAR_SPEED_RAD_S,
    MAX_LINEAR_SPEED_M_S,
    MAX_WHEEL_SPEED_RAD_S,
    WHEEL_RADIUS_METERS,
)


@dataclass(frozen=True, slots=True)
class WorldPose:
    x: float
    y: float
    heading: float


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class Mir100ExecutionAdapter:
    """Translate a selected waypoint into MiR100 motor speeds."""

    def __init__(self, robot: object, timestep_ms: int) -> None:
        self.robot = robot
        self.left_motor = robot.getDevice("middle_left_wheel_joint")
        self.right_motor = robot.getDevice("middle_right_wheel_joint")
        self.left_encoder = robot.getDevice("middle_left_wheel_joint_sensor")
        self.right_encoder = robot.getDevice("middle_right_wheel_joint_sensor")
        self.gps = robot.getDevice("synq_gps")
        self.imu = robot.getDevice("synq_imu")
        self.gps.enable(timestep_ms)
        self.imu.enable(timestep_ms)
        self.left_encoder.enable(timestep_ms)
        self.right_encoder.enable(timestep_ms)
        for motor in (self.left_motor, self.right_motor):
            motor.setPosition(inf)
            motor.setVelocity(0.0)
        self.enabled = True
        self._previous_wheel_sample: tuple[float, float, float] | None = None

    def get_pose(self) -> WorldPose:
        x, y, _ = self.gps.getValues()
        heading = self.imu.getRollPitchYaw()[2]
        return WorldPose(x, y, heading)

    def get_velocity(self) -> tuple[float, float]:
        sample = (self.robot.getTime(), self.left_encoder.getValue(), self.right_encoder.getValue())
        previous = self._previous_wheel_sample
        self._previous_wheel_sample = sample
        if previous is None or sample[0] <= previous[0]:
            return (0.0, 0.0)
        elapsed = sample[0] - previous[0]
        left = (sample[1] - previous[1]) * WHEEL_RADIUS_METERS / elapsed
        right = (sample[2] - previous[2]) * WHEEL_RADIUS_METERS / elapsed
        return ((left + right) / 2, (right - left) / (2 * HALF_WHEEL_TRACK_METERS))

    def stop(self) -> None:
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled
        if not enabled:
            self.stop()

    def is_at(self, target: tuple[float, float], tolerance_m: float) -> bool:
        pose = self.get_pose()
        return hypot(target[0] - pose.x, target[1] - pose.y) <= tolerance_m

    def move_toward(self, target: tuple[float, float]) -> None:
        if not self.enabled:
            self.stop()
            return
        pose = self.get_pose()
        dx = target[0] - pose.x
        dy = target[1] - pose.y
        distance = hypot(dx, dy)
        desired_heading = atan2(dy, dx)
        heading_error = atan2(sin(desired_heading - pose.heading), cos(desired_heading - pose.heading))

        angular = _clamp(1.5 * heading_error, MAX_ANGULAR_SPEED_RAD_S)
        if abs(heading_error) > HEADING_TOLERANCE_RADIANS:
            linear = 0.0
        else:
            linear = min(MAX_LINEAR_SPEED_M_S, 0.8 * distance)
        left = (linear - angular * HALF_WHEEL_TRACK_METERS) / WHEEL_RADIUS_METERS
        right = (linear + angular * HALF_WHEEL_TRACK_METERS) / WHEEL_RADIUS_METERS
        self.left_motor.setVelocity(_clamp(left, MAX_WHEEL_SPEED_RAD_S))
        self.right_motor.setVelocity(_clamp(right, MAX_WHEEL_SPEED_RAD_S))
