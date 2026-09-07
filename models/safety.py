"""Environment observations used by each robot's local safety manager."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from models.grid import Cell


class SafetyZone(Enum):
    GREEN = auto()
    YELLOW = auto()
    RED = auto()


@dataclass(frozen=True, slots=True)
class HumanEntity:
    """A simulated person, supplied to robots as perception input."""

    human_id: str
    position: Cell


@dataclass(frozen=True, slots=True)
class SafetyZoneConfig:
    """Manhattan-distance safety envelopes around a perceived human."""

    red_radius: int = 0
    yellow_radius: int = 1
    caution_move_interval_ticks: int = 2

    def __post_init__(self) -> None:
        if self.red_radius < 0 or self.yellow_radius < self.red_radius:
            raise ValueError("Safety radii must satisfy 0 <= red <= yellow.")
        if self.caution_move_interval_ticks < 1:
            raise ValueError("Caution movement interval must be at least one tick.")