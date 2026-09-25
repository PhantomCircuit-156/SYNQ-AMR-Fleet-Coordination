"""Shared, deterministic geometry for the three MiR100 benchmark robots."""
from dataclasses import dataclass

CELL_SIZE_METERS = 1.0
WORLD_ORIGIN = (-3.0, -3.0)
GRID_SIZE = 7
CONFLICT_CELLS = frozenset((x, y) for x in range(2, 5) for y in range(2, 5))
APPROACH_CELLS = frozenset({(1, 2), (1, 4), (3, 1)})
WHEEL_RADIUS = 0.0625
HALF_TRACK = 0.222604
MAX_LINEAR = 0.18
MAX_ANGULAR = 0.40
ARRIVAL_METERS = 0.10
HEADING_TOLERANCE = 0.12
SAFETY_DISTANCE_METERS = 0.65
PEER_TIMEOUT_SECONDS = 0.5
WAYPOINT_TIMEOUT_SECONDS = 45.0

@dataclass(frozen=True)
class Assignment:
    start: tuple[int, int]
    goal: tuple[int, int]
    heading: float
    priority: int

ASSIGNMENTS = {
    "R1": Assignment((0, 2), (6, 2), 0.0, 3),
    "R2": Assignment((0, 4), (6, 4), 0.0, 2),
    "R3": Assignment((3, 0), (3, 6), 1.5707963267948966, 1),
}

def to_world(cell):
    return (WORLD_ORIGIN[0] + cell.x * CELL_SIZE_METERS,
            WORLD_ORIGIN[1] + cell.y * CELL_SIZE_METERS)
