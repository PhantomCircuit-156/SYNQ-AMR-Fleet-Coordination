"""A tick-based world runner that applies robot-selected moves."""

from __future__ import annotations

from dataclasses import dataclass, field

from models.grid import Cell, WarehouseGrid
from models.robot_state import RobotState
from simulation.robot import RobotAgent


@dataclass(slots=True)
class HeadlessSimulation:
    """Advance one robot through a static warehouse without traffic authority."""

    grid: WarehouseGrid
    robot: RobotAgent
    tick_count: int = field(default=0, init=False)
    position_history: list[Cell] = field(default_factory=list, init=False)

    def __post_init__(self) -> None:
        if not self.grid.is_walkable(self.robot.position):
            raise ValueError(f"Robot spawn is not walkable: {self.robot.position}")
        self.robot.attach_static_map(self.grid)
        self.position_history.append(self.robot.position)

    def step(self) -> None:
        """Advance one tick; movement is selected by the robot, not this class."""
        selected_destination = self.robot.decide_next_move()
        if selected_destination is not None:
            self.grid.validate_move(self.robot.position, selected_destination)
            self.robot.confirm_move(selected_destination)
            self.position_history.append(selected_destination)
        self.tick_count += 1

    def run_until_complete(self, max_ticks: int = 1_000) -> None:
        for _ in range(max_ticks):
            if self.robot.state is RobotState.TASK_COMPLETE:
                return
            self.step()
        raise TimeoutError(f"Robot did not complete within {max_ticks} ticks.")
