"""Deterministic A* planner for static 4-connected warehouse grids."""

from __future__ import annotations

from heapq import heappop, heappush

from models.grid import Cell, WarehouseGrid


class PlanningError(ValueError):
    """Base error for invalid planning requests."""


class InvalidStartError(PlanningError):
    """Raised when a start cell is outside the grid or blocked."""


class InvalidGoalError(PlanningError):
    """Raised when a goal cell is outside the grid or blocked."""


class AStarPlanner:
    """Compute reproducible shortest paths using Manhattan-distance A*."""

    def plan(self, grid: WarehouseGrid, start: Cell, goal: Cell) -> tuple[Cell, ...] | None:
        """Return an inclusive shortest path, or ``None`` when no route exists."""
        if not grid.is_walkable(start):
            raise InvalidStartError(f"Start cell is outside the grid or blocked: {start}")
        if not grid.is_walkable(goal):
            raise InvalidGoalError(f"Goal cell is outside the grid or blocked: {goal}")
        if start == goal:
            return (start,)

        frontier: list[tuple[int, int, int, Cell]] = []
        insertion_order = 0
        heappush(frontier, (self._heuristic(start, goal), 0, insertion_order, start))
        came_from: dict[Cell, Cell] = {}
        cost_so_far: dict[Cell, int] = {start: 0}

        while frontier:
            _, current_cost, _, current = heappop(frontier)
            if current_cost != cost_so_far.get(current):
                continue
            if current == goal:
                return self._reconstruct_path(came_from, start, goal)

            for neighbor in grid.neighbors(current):
                next_cost = current_cost + 1
                if next_cost >= cost_so_far.get(neighbor, float("inf")):
                    continue
                cost_so_far[neighbor] = next_cost
                came_from[neighbor] = current
                heuristic = self._heuristic(neighbor, goal)
                insertion_order += 1
                heappush(frontier, (next_cost + heuristic, next_cost, insertion_order, neighbor))

        return None

    @staticmethod
    def _heuristic(source: Cell, destination: Cell) -> int:
        return abs(source.x - destination.x) + abs(source.y - destination.y)

    @staticmethod
    def _reconstruct_path(
        came_from: dict[Cell, Cell], start: Cell, goal: Cell
    ) -> tuple[Cell, ...]:
        path = [goal]
        current = goal
        while current != start:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return tuple(path)
