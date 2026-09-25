"""Single-MiR100 Webots geometry and motion parameters (simulated only)."""

from models.grid import Cell, WarehouseGrid

CELL_SIZE_METERS = 1.2
WORLD_ORIGIN_X = -1.8
WORLD_ORIGIN_Y = -1.8
GRID = WarehouseGrid(width=4, height=4)
START_CELL = Cell(0, 0)
GOAL_CELL = Cell(2, 1)

WHEEL_RADIUS_METERS = 0.0625
HALF_WHEEL_TRACK_METERS = 0.222604
MAX_WHEEL_SPEED_RAD_S = 20.0
MAX_LINEAR_SPEED_M_S = 0.22
MAX_ANGULAR_SPEED_RAD_S = 0.5
WAYPOINT_TOLERANCE_METERS = 0.10
HEADING_TOLERANCE_RADIANS = 0.12


def cell_to_world(cell: Cell) -> tuple[float, float]:
    if not GRID.in_bounds(cell):
        raise ValueError(f"Cell outside Webots grid: {cell}")
    return (
        WORLD_ORIGIN_X + cell.x * CELL_SIZE_METERS,
        WORLD_ORIGIN_Y + cell.y * CELL_SIZE_METERS,
    )


def world_to_cell(x: float, y: float) -> Cell:
    cell = Cell(
        round((x - WORLD_ORIGIN_X) / CELL_SIZE_METERS),
        round((y - WORLD_ORIGIN_Y) / CELL_SIZE_METERS),
    )
    if not GRID.in_bounds(cell):
        raise ValueError(f"World point outside Webots grid: ({x}, {y})")
    return cell
