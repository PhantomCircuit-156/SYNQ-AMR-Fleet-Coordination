"""Robot-owned environmental safety evaluation."""

from __future__ import annotations

from dataclasses import dataclass

from models.grid import Cell
from models.safety import HumanEntity, SafetyZone, SafetyZoneConfig


@dataclass(slots=True)
class SafetyManager:
    """Classify cells from local human observations; never grants traffic rights."""

    config: SafetyZoneConfig

    def zone_for(self, cell: Cell, humans: tuple[HumanEntity, ...]) -> SafetyZone:
        nearest = min(
            (abs(cell.x - human.position.x) + abs(cell.y - human.position.y) for human in humans),
            default=None,
        )
        if nearest is None or nearest > self.config.yellow_radius:
            return SafetyZone.GREEN
        if nearest <= self.config.red_radius:
            return SafetyZone.RED
        return SafetyZone.YELLOW