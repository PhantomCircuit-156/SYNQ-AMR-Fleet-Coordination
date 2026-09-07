import pytest

from models.grid import Cell, WarehouseGrid
from planning.a_star import AStarPlanner, InvalidGoalError


def test_simple_a_star_route_is_shortest_and_deterministic() -> None:
    grid = WarehouseGrid(width=4, height=3)
    planner = AStarPlanner()

    first_path = planner.plan(grid, Cell(0, 0), Cell(3, 2))
    second_path = planner.plan(grid, Cell(0, 0), Cell(3, 2))

    assert first_path == second_path
    assert first_path == (
        Cell(0, 0), Cell(1, 0), Cell(2, 0), Cell(3, 0), Cell(3, 1), Cell(3, 2)
    )


def test_a_star_routes_around_obstacles() -> None:
    grid = WarehouseGrid(width=5, height=3, obstacles=frozenset({Cell(1, 1), Cell(2, 1), Cell(3, 1)}))

    path = AStarPlanner().plan(grid, Cell(0, 1), Cell(4, 1))

    assert path is not None
    assert all(cell not in grid.obstacles for cell in path)
    assert len(path) == 7


def test_blocked_destination_is_rejected() -> None:
    grid = WarehouseGrid(width=3, height=3, obstacles=frozenset({Cell(2, 2)}))

    with pytest.raises(InvalidGoalError):
        AStarPlanner().plan(grid, Cell(0, 0), Cell(2, 2))


def test_unreachable_destination_returns_none() -> None:
    grid = WarehouseGrid(width=3, height=3, obstacles=frozenset({Cell(1, 0), Cell(1, 1), Cell(1, 2)}))

    assert AStarPlanner().plan(grid, Cell(0, 1), Cell(2, 1)) is None


def test_route_is_walkable_and_4_connected() -> None:
    grid = WarehouseGrid(width=5, height=4, obstacles=frozenset({Cell(2, 0), Cell(2, 1)}))

    path = AStarPlanner().plan(grid, Cell(0, 0), Cell(4, 0))

    assert path is not None
    assert path[0] == Cell(0, 0)
    assert path[-1] == Cell(4, 0)
    for source, destination in zip(path, path[1:]):
        assert grid.is_walkable(destination)
        assert abs(source.x - destination.x) + abs(source.y - destination.y) == 1
