"""Deterministic narrow-corridor and deadlock test/demo scenarios."""

from __future__ import annotations

from coordination.intent import IntentManager
from coordination.local_world import LocalWorldModel
from models.corridor import CorridorRegion
from models.grid import Cell, WarehouseGrid
from models.messages import PeerStateMessage
from models.priority import TaskPriority
from models.robot_state import RobotState
from simulation.fleet import FleetSimulation
from simulation.robot import RobotAgent


def basic_corridor_region() -> CorridorRegion:
    return CorridorRegion(
        corridor_id="C1",
        cells=frozenset(Cell(x, 2) for x in range(3, 8)),
        entry_cells=frozenset({Cell(2, 2), Cell(8, 2)}),
        passing_bays=frozenset({Cell(2, 1), Cell(8, 1)}),
    )


def build_opposing_corridor_scenario() -> FleetSimulation:
    corridor = basic_corridor_region()
    obstacles = frozenset(
        Cell(x, y)
        for x in range(3, 8)
        for y in range(5)
        if y != 2
    )
    grid = WarehouseGrid(width=11, height=5, obstacles=obstacles)
    robots = (RobotAgent("R1", Cell(1, 2)), RobotAgent("R2", Cell(9, 2)))
    robots[0].assign_goal(Cell(9, 2))
    robots[1].assign_goal(Cell(1, 2))
    return FleetSimulation(grid=grid, robots=robots, corridor_regions=(corridor,))


def build_recoverable_deadlock_scenario() -> FleetSimulation:
    """Restore two mutually blocked robot snapshots, then let agents recover."""
    corridor = basic_corridor_region()
    grid = WarehouseGrid(width=11, height=5)
    robots = (RobotAgent("R1", Cell(2, 2)), RobotAgent("R2", Cell(8, 2)))
    robots[0].assign_goal(Cell(9, 2))
    robots[1].assign_goal(Cell(1, 2))
    simulation = FleetSimulation(grid=grid, robots=robots, corridor_regions=(corridor,))
    for robot in robots:
        robot.decide_next_move()
    robots[0].restore_waiting_snapshot(("R2",), waiting_ticks=6)
    robots[1].restore_waiting_snapshot(("R1",), waiting_ticks=6)
    for robot in robots:
        robot.publish_state(simulation.message_bus, timestamp=0)
    return simulation


def build_intentional_deadlock_worlds(
    waiting_ticks: int = 6,
) -> tuple[LocalWorldModel, LocalWorldModel, CorridorRegion]:
    corridor = basic_corridor_region()
    grid = WarehouseGrid(width=11, height=5)
    hold_manager = IntentManager()
    r1 = PeerStateMessage(
        robot_id="R1",
        timestamp=waiting_ticks,
        pose=Cell(2, 2),
        velocity=(0, 0),
        goal=Cell(9, 2),
        intent=hold_manager.generate_hold("R1", Cell(2, 2), waiting_ticks),
        priority=TaskPriority.NORMAL,
        battery=100.0,
        state=RobotState.WAITING,
        heartbeat=waiting_ticks,
        waiting_ticks=waiting_ticks,
        blocked_by=("R2",),
    )
    r2 = PeerStateMessage(
        robot_id="R2",
        timestamp=waiting_ticks,
        pose=Cell(8, 2),
        velocity=(0, 0),
        goal=Cell(1, 2),
        intent=hold_manager.generate_hold("R2", Cell(8, 2), waiting_ticks),
        priority=TaskPriority.NORMAL,
        battery=100.0,
        state=RobotState.WAITING,
        heartbeat=waiting_ticks,
        waiting_ticks=waiting_ticks,
        blocked_by=("R1",),
    )
    r1_world = LocalWorldModel("R1", static_map=grid, corridor_regions=(corridor,))
    r1_world.update_own_state(r1)
    r1_world.receive_peer_state(r2)
    r2_world = LocalWorldModel("R2", static_map=grid, corridor_regions=(corridor,))
    r2_world.update_own_state(r2)
    r2_world.receive_peer_state(r1)
    return r1_world, r2_world, corridor
