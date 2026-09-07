"""Immutable grid primitives used by robot-local planners and simulation."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True, order=True, slots=True)
class Cell:
    """A zero-indexed warehouse grid location."""

    x: int
    y: int


@dataclass(frozen=True, slots=True)
class WarehouseGrid:
    """Static warehouse geometry and obstacle occupancy."""

    width: int
    height: int
    obstacles: frozenset[Cell] = field(default_factory=frozenset)

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Warehouse dimensions must be positive.")
        invalid_obstacles = [cell for cell in self.obstacles if not self.in_bounds(cell)]
        if invalid_obstacles:
            raise ValueError(f"Obstacles outside warehouse bounds: {invalid_obstacles}")

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell.x < self.width and 0 <= cell.y < self.height

    def is_walkable(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and cell not in self.obstacles

    def neighbors(self, cell: Cell) -> tuple[Cell, ...]:
        """Return walkable 4-connected neighbors in a stable order."""
        candidates = (
            Cell(cell.x, cell.y - 1),
            Cell(cell.x - 1, cell.y),
            Cell(cell.x + 1, cell.y),
            Cell(cell.x, cell.y + 1),
        )
        return tuple(candidate for candidate in candidates if self.is_walkable(candidate))

    def validate_move(self, source: Cell, destination: Cell) -> None:
        """Validate one robot-selected 4-connected move without choosing it."""
        if not self.is_walkable(source):
            raise ValueError(f"Move source is not walkable: {source}")
        if not self.is_walkable(destination):
            raise ValueError(f"Move destination is not walkable: {destination}")
        distance = abs(source.x - destination.x) + abs(source.y - destination.y)
        if distance != 1:
            raise ValueError(f"Move must be 4-connected: {source} -> {destination}")
