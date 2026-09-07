"""Deterministic scenarios for the initial three-robot warehouse demonstration."""

from __future__ import annotations

from dataclasses import dataclass
from random import Random

from models.grid import Cell, WarehouseGrid
from models.mission import Payload, PayloadType, TaskType
from models.priority import TaskPriority
from simulation.fleet import FleetSimulation
from simulation.robot import RobotAgent
from tasks.lifecycle import DeterministicTaskDispatcher, FleetTask


@dataclass(frozen=True, slots=True)
class Zone:
    name: str
    left: int
    top: int
    width: int
    height: int
    color: tuple[int, int, int]


@dataclass(frozen=True, slots=True)
class WarehouseScenario:
    grid: WarehouseGrid
    zones: tuple[Zone, ...]
    robot_colors: dict[str, tuple[int, int, int]]


def build_basic_three_robot_scenario(seed: int = 0) -> tuple[WarehouseScenario, FleetSimulation]:
    """Create three independent non-conflicting routes for visual demonstration."""
    Random(seed)  # Reserved for future controlled scenario variation.
    shelf_cells = frozenset(
        Cell(x, y)
        for x0, x1, y0, y1 in (
            (5, 13, 5, 8),
            (18, 27, 5, 8),
            (5, 13, 16, 19),
            (18, 27, 16, 19),
        )
        for x in range(x0, x1 + 1)
        for y in range(y0, y1 + 1)
    )
    grid = WarehouseGrid(width=40, height=25, obstacles=shelf_cells)
    robots = (
        RobotAgent("R1", Cell(2, 2)),
        RobotAgent("R2", Cell(2, 12)),
        RobotAgent("R3", Cell(2, 22)),
    )
    goals = (Cell(36, 2), Cell(36, 12), Cell(36, 22))
    payloads = (
        Payload("TOTE-1042", PayloadType.TOTE, 12.0),
        Payload("CARTON-208", PayloadType.CARTON, 18.5, fragile=True),
        Payload("BIN-731", PayloadType.PARTS_BIN, 9.0),
    )
    dispatcher = DeterministicTaskDispatcher()
    for index, (robot, goal, payload) in enumerate(zip(robots, goals, payloads, strict=True), start=1):
        task = FleetTask(
            f"M-{index:03d}", goal, priority=TaskPriority.NORMAL,
            task_type=TaskType.PICKUP_DELIVERY, source=Cell(robot.position.x + 1, robot.position.y), payload=payload,
        )
        dispatcher.add_task(task)
        dispatcher.assign_initial(task.task_id, robot)
    scenario = WarehouseScenario(
        grid=grid,
        zones=(
            Zone("LOADING", 1, 1, 8, 3, (65, 133, 199)),
            Zone("PACKING", 29, 10, 10, 4, (102, 187, 106)),
            Zone("DISPATCH", 29, 20, 10, 4, (255, 167, 38)),
        ),
        robot_colors={"R1": (229, 57, 53), "R2": (30, 136, 229), "R3": (67, 160, 71)},
    )
    return scenario, FleetSimulation(grid=grid, robots=robots, task_dispatcher=dispatcher)