"""Deterministic conflict scenarios for decentralized behavior integration."""

from __future__ import annotations

from dataclasses import dataclass

from models.grid import Cell, WarehouseGrid
from simulation.fleet import FleetSimulation
from simulation.robot import RobotAgent


@dataclass(frozen=True, slots=True)
class CoordinationScenario:
    name: str
    description: str
    simulation: FleetSimulation


def build_same_cell_scenario() -> CoordinationScenario:
    return _build(
        "same-cell",
        "Two routes predict the same intersection cell at the same future tick.",
        7,
        5,
        (("R1", Cell(1, 2), Cell(5, 2)), ("R2", Cell(3, 4), Cell(3, 0))),
    )


def build_head_on_scenario() -> CoordinationScenario:
    return _build(
        "head-on",
        "Adjacent robots predict an edge swap; the loser uses a local side bay.",
        7,
        3,
        (("R1", Cell(2, 1), Cell(5, 1)), ("R2", Cell(3, 1), Cell(0, 1))),
    )


def build_two_robot_intersection_scenario() -> CoordinationScenario:
    return _build(
        "two-robot-intersection",
        "Two routes cross the same cell at different predicted arrival ticks.",
        7,
        5,
        (("R1", Cell(1, 2), Cell(5, 2)), ("R2", Cell(3, 3), Cell(3, 0))),
    )


def build_three_robot_intersection_scenario() -> CoordinationScenario:
    return _build(
        "three-robot-intersection",
        "Three independent routes initially predict the center cell at the same tick.",
        7,
        7,
        (
            ("R1", Cell(0, 3), Cell(6, 3)),
            ("R2", Cell(3, 0), Cell(3, 6)),
            ("R3", Cell(3, 6), Cell(6, 0)),
        ),
    )


def build_all_coordination_scenarios() -> tuple[CoordinationScenario, ...]:
    return (
        build_same_cell_scenario(),
        build_head_on_scenario(),
        build_two_robot_intersection_scenario(),
        build_three_robot_intersection_scenario(),
    )


def _build(
    name: str,
    description: str,
    width: int,
    height: int,
    robot_specs: tuple[tuple[str, Cell, Cell], ...],
) -> CoordinationScenario:
    grid = WarehouseGrid(width=width, height=height)
    robots = tuple(RobotAgent(robot_id, start) for robot_id, start, _ in robot_specs)
    for robot, (_, _, goal) in zip(robots, robot_specs, strict=True):
        robot.assign_goal(goal)
    return CoordinationScenario(
        name=name,
        description=description,
        simulation=FleetSimulation(grid=grid, robots=robots),
    )
