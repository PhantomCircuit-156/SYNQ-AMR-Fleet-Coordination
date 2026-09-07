"""Explicit narrow-corridor geometry and decentralized ownership messages."""

from __future__ import annotations

from dataclasses import dataclass

from models.grid import Cell


@dataclass(frozen=True, slots=True)
class CorridorRegion:
    corridor_id: str
    cells: frozenset[Cell]
    entry_cells: frozenset[Cell]
    passing_bays: frozenset[Cell] = frozenset()

    def __post_init__(self) -> None:
        if not self.corridor_id:
            raise ValueError("Corridor ID cannot be empty.")
        if not self.cells:
            raise ValueError("A corridor must contain at least one cell.")
        if self.cells & self.entry_cells:
            raise ValueError("Corridor entry cells must be outside the corridor region.")


@dataclass(frozen=True, slots=True)
class CorridorRequest:
    corridor_id: str
    robot_id: str
    requested_at: int
    entry_cell: Cell
    exit_cell: Cell


@dataclass(frozen=True, slots=True)
class CorridorToken:
    corridor_id: str
    owner_id: str
    issued_at: int
    expires_at: int

    def is_valid_at(self, timestamp: int) -> bool:
        return timestamp <= self.expires_at
