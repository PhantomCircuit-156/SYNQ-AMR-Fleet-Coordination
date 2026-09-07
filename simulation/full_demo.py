"""Deterministic full-product demonstration setup.

The scenario schedules environment and mission inputs elsewhere; robot traffic
choices remain entirely local to each RobotAgent.
"""
from __future__ import annotations

from dataclasses import dataclass

from models.grid import Cell, WarehouseGrid
from models.mission import Payload, PayloadType, TaskType
from models.priority import TaskPriority
from simulation.fleet import FleetSimulation
from simulation.robot import RobotAgent
from simulation.scenarios import WarehouseScenario, Zone
from tasks.lifecycle import DeterministicTaskDispatcher, FleetTask


@dataclass(frozen=True, slots=True)
class FullDemoScenario:
    warehouse: WarehouseScenario
    simulation: FleetSimulation


def build_full_demo_scenario() -> FullDemoScenario:
    shelves = {
        Cell(x, y)
        for x0, x1, y0, y1 in (
            (4, 12, 4, 7), (21, 31, 4, 7),
            (4, 12, 16, 19), (21, 31, 16, 19),
        )
        for x in range(x0, x1 + 1)
        for y in range(y0, y1 + 1)
    }
    # This fixed rack edge makes x=16/y=12 a visible choke point.
    shelves.update(Cell(17, y) for y in range(0, 14) if y != 12)
    grid = WarehouseGrid(40, 25, frozenset(shelves))
    robots = (
        RobotAgent("R1", Cell(6, 12)),
        RobotAgent("R2", Cell(16, 0)),
        RobotAgent("R3", Cell(2, 22)),
    )
    dispatcher = DeterministicTaskDispatcher()
    initial_tasks = (
        FleetTask(
            "DEMO-101", Cell(34, 12), priority=TaskPriority.NORMAL,
            task_type=TaskType.PICKUP_DELIVERY, source=Cell(6, 12),
            payload=Payload("TOTE-42", PayloadType.TOTE, 14.0),
        ),
        FleetTask("DEMO-102", Cell(28, 24), priority=TaskPriority.NORMAL),
        FleetTask(
            "DEMO-103", Cell(12, 22), priority=TaskPriority.NORMAL,
            task_type=TaskType.REPLENISHMENT, source=Cell(2, 22),
            payload=Payload("BIN-19", PayloadType.PARTS_BIN, 8.0),
        ),
    )
    for robot, task in zip(robots, initial_tasks, strict=True):
        dispatcher.add_task(task)
        dispatcher.assign_initial(task.task_id, robot)
    warehouse = WarehouseScenario(
        grid=grid,
        zones=(
            Zone("LOADING", 0, 20, 8, 5, (65, 133, 199)),
            Zone("PACKING", 31, 9, 9, 6, (102, 187, 106)),
            Zone("DISPATCH", 32, 20, 8, 5, (255, 167, 38)),
            Zone("STAGING", 0, 9, 7, 6, (128, 112, 190)),
        ),
        robot_colors={"R1": (70, 200, 220), "R2": (238, 182, 92), "R3": (170, 134, 255)},
    )
    return FullDemoScenario(warehouse, FleetSimulation(grid, robots, task_dispatcher=dispatcher))