"""Focused checks for the simulated MiR100 execution boundary."""

from math import isclose, pi

from config.webots import GOAL_CELL, START_CELL, cell_to_world, world_to_cell
from execution.webots_adapter import Mir100ExecutionAdapter


class FakeMotor:
    def __init__(self) -> None:
        self.velocity = 0.0
        self.position = None

    def setPosition(self, position: float) -> None:
        self.position = position

    def setVelocity(self, velocity: float) -> None:
        self.velocity = velocity


class FakeSensor:
    def __init__(self, value=0.0) -> None:
        self.value = value
        self.enabled = False

    def enable(self, timestep: int) -> None:
        self.enabled = True

    def getValue(self) -> float:
        return self.value

    def getValues(self) -> tuple[float, float, float]:
        return self.value

    def getRollPitchYaw(self) -> tuple[float, float, float]:
        return self.value


class FakeRobot:
    def __init__(self) -> None:
        self.time = 0.0
        self.devices = {
            "middle_left_wheel_joint": FakeMotor(),
            "middle_right_wheel_joint": FakeMotor(),
            "middle_left_wheel_joint_sensor": FakeSensor(),
            "middle_right_wheel_joint_sensor": FakeSensor(),
            "synq_gps": FakeSensor((0.0, 0.0, 0.0)),
            "synq_imu": FakeSensor((0.0, 0.0, 0.0)),
        }

    def getDevice(self, name: str):
        return self.devices[name]

    def getTime(self) -> float:
        return self.time


def test_grid_world_mapping_round_trip() -> None:
    for cell in (START_CELL, GOAL_CELL):
        assert world_to_cell(*cell_to_world(cell)) == cell


def test_mir100_forward_turn_and_stop() -> None:
    robot = FakeRobot()
    adapter = Mir100ExecutionAdapter(robot, 32)
    left = robot.getDevice("middle_left_wheel_joint")
    right = robot.getDevice("middle_right_wheel_joint")
    assert robot.getDevice("middle_left_wheel_joint_sensor").enabled

    adapter.move_toward((1.0, 0.0))
    assert left.velocity > 0
    assert isclose(left.velocity, right.velocity)

    adapter.move_toward((0.0, 1.0))
    assert left.velocity < 0 < right.velocity

    adapter.stop()
    assert left.velocity == right.velocity == 0.0


def test_mir100_measured_velocity_uses_wheel_encoders() -> None:
    robot = FakeRobot()
    adapter = Mir100ExecutionAdapter(robot, 32)
    assert adapter.get_velocity() == (0.0, 0.0)
    robot.time = 1.0
    robot.getDevice("middle_left_wheel_joint_sensor").value = 2.0
    robot.getDevice("middle_right_wheel_joint_sensor").value = 2.0
    linear, angular = adapter.get_velocity()
    assert isclose(linear, 0.125)
    assert isclose(angular, 0.0)

    robot.time = 2.0
    robot.getDevice("middle_left_wheel_joint_sensor").value -= pi
    robot.getDevice("middle_right_wheel_joint_sensor").value += pi
    linear, angular = adapter.get_velocity()
    assert isclose(linear, 0.0, abs_tol=1e-9)
    assert angular > 0
