"""R1 single-robot SYNQ plan-and-drive probe for Webots R2025a."""

from __future__ import annotations

import sys
from math import hypot
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from controller import Robot

from config.webots import (
    GOAL_CELL,
    GRID,
    START_CELL,
    WAYPOINT_TOLERANCE_METERS,
    cell_to_world,
)
from execution.webots_adapter import Mir100ExecutionAdapter
from models.robot_state import RobotState
from simulation.robot import RobotAgent


def main() -> None:
    robot = Robot()
    timestep_ms = int(robot.getBasicTimeStep())
    adapter = Mir100ExecutionAdapter(robot, timestep_ms)
    agent = RobotAgent(robot_id="R1", position=START_CELL)
    agent.attach_static_map(GRID)
    agent.assign_goal(GOAL_CELL)
    pending = None
    braking_steps = 0
    announced_complete = False

    print(f"SYNQ R1: start={START_CELL}, goal={GOAL_CELL}", flush=True)
    while robot.step(timestep_ms) != -1:
        pose = adapter.get_pose()
        current = cell_to_world(agent.position)
        if pending is None and hypot(pose.x - current[0], pose.y - current[1]) > 0.35:
            adapter.stop()
            raise RuntimeError("R1 pose differs from its confirmed grid cell; reset the world")

        if pending is not None:
            target = cell_to_world(pending)
            if adapter.is_at(target, WAYPOINT_TOLERANCE_METERS):
                adapter.stop()
                linear_speed, angular_speed = adapter.get_velocity()
                braking_steps = braking_steps + 1 if abs(linear_speed) < 0.04 and abs(angular_speed) < 0.08 else 0
                if braking_steps >= 3:
                    agent.confirm_move(pending)
                    print(f"SYNQ R1: arrived {pending}, state={agent.state.name}", flush=True)
                    pending = None
                    braking_steps = 0
            else:
                braking_steps = 0
                adapter.get_velocity()
                adapter.move_toward(target)
            continue

        if agent.state is RobotState.TASK_COMPLETE:
            adapter.stop()
            if not announced_complete:
                print(f"SYNQ R1: task complete at {agent.position}; GPS=({pose.x:.3f}, {pose.y:.3f})", flush=True)
                announced_complete = True
            continue

        destination = agent.decide_next_move()
        if destination is None:
            adapter.stop()
            if agent.state is RobotState.UNREACHABLE:
                raise RuntimeError("SYNQ R1 has no route to its goal")
            continue
        pending = destination
        print(f"SYNQ R1: selected {pending}", flush=True)
        adapter.move_toward(cell_to_world(pending))

    adapter.stop()


if __name__ == "__main__":
    main()
