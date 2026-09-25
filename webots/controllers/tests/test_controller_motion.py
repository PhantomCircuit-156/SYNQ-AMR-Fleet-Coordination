"""Ideal encoder check of the original one-robot route; not Webots physics."""
import contextlib
import importlib.util
import io
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "synq_controller"))

class Motor:
    def __init__(self):
        self.velocity = 0.0
        self.position = 0.0
    def setPosition(self, value):
        pass
    def setVelocity(self, value):
        self.velocity = value

class Sensor:
    def __init__(self, motor):
        self.motor = motor
    def enable(self, timestep):
        pass
    def getValue(self):
        return self.motor.position

class FakeRobot:
    def __init__(self):
        self.time = 0.0
        self.steps = 0
        self.left = Motor()
        self.right = Motor()
        self.devices = {
            "middle_left_wheel_joint": self.left,
            "middle_right_wheel_joint": self.right,
            "middle_left_wheel_joint_sensor": Sensor(self.left),
            "middle_right_wheel_joint_sensor": Sensor(self.right),
        }
    def getName(self):
        return "Mir100"
    def getBasicTimeStep(self):
        return 32
    def getDevice(self, name):
        return self.devices[name]
    def getTime(self):
        return self.time
    def step(self, timestep):
        if self.steps >= 7000:
            return -1
        dt = timestep / 1000
        self.left.position += self.left.velocity * dt
        self.right.position += self.right.velocity * dt
        self.time += dt
        self.steps += 1
        return 0

class MotionCheck(unittest.TestCase):
    def test_single_robot_route_finishes(self):
        stub = types.ModuleType("controller")
        stub.Robot = FakeRobot
        old = sys.modules.get("controller")
        sys.modules["controller"] = stub
        try:
            path = ROOT / "synq_controller" / "synq_controller.py"
            spec = importlib.util.spec_from_file_location("synq_controller", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                module.main()
            self.assertIn("SYNQ R1 TASK_COMPLETE", output.getvalue())
        finally:
            if old is None:
                del sys.modules["controller"]
            else:
                sys.modules["controller"] = old

if __name__ == "__main__":
    unittest.main()
