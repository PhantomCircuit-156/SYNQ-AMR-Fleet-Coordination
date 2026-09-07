"""Robot-local short-horizon predicted occupancy generation."""

from __future__ import annotations

from dataclasses import dataclass

from models.grid import Cell


INTENT_HORIZON = 5


@dataclass(frozen=True, slots=True)
class IntentStep:
    """One predicted future occupancy relative to the message timestamp."""

    timestep: int
    cell: Cell

    def __post_init__(self) -> None:
        if self.timestep < 1:
            raise ValueError("Intent timesteps must start at 1.")


@dataclass(frozen=True, slots=True)
class Intent:
    """A robot's immutable upcoming occupancy, excluding its current cell."""

    robot_id: str
    generated_at: int
    steps: tuple[IntentStep, ...]

    @property
    def cells(self) -> tuple[Cell, ...]:
        return tuple(step.cell for step in self.steps)


@dataclass(frozen=True, slots=True)
class IntentManager:
    """Generate bounded predicted occupancy from the owner's current path."""

    horizon: int = INTENT_HORIZON

    def __post_init__(self) -> None:
        if self.horizon <= 0:
            raise ValueError("Intent horizon must be positive.")

    def generate(
        self,
        robot_id: str,
        planned_path: tuple[Cell, ...],
        current_path_index: int,
        timestamp: int,
    ) -> Intent:
        if not planned_path:
            return Intent(robot_id=robot_id, generated_at=timestamp, steps=())
        if not 0 <= current_path_index < len(planned_path):
            raise ValueError("Current path index is outside the planned path.")
        future_cells = planned_path[
            current_path_index + 1 : current_path_index + 1 + self.horizon
        ]
        steps = tuple(
            IntentStep(timestep=offset, cell=cell)
            for offset, cell in enumerate(future_cells, start=1)
        )
        return Intent(robot_id=robot_id, generated_at=timestamp, steps=steps)

    def generate_hold(self, robot_id: str, cell: Cell, timestamp: int) -> Intent:
        """Advertise that a waiting robot will continue occupying its cell."""
        return Intent(
            robot_id=robot_id,
            generated_at=timestamp,
            steps=tuple(
                IntentStep(timestep=offset, cell=cell)
                for offset in range(1, self.horizon + 1)
            ),
        )
