"""Robot-local intent conflict prediction from LocalWorldModel snapshots."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from coordination.local_world import LocalWorldModel
from models.grid import Cell, WarehouseGrid
from models.messages import PeerStateMessage


class ConflictType(Enum):
    SAME_CELL = auto()
    HEAD_ON = auto()
    INTERSECTION = auto()
    CORRIDOR = auto()


@dataclass(frozen=True, slots=True, order=True)
class DirectedTransition:
    robot_id: str
    timestep: int
    source: Cell
    destination: Cell


@dataclass(frozen=True, slots=True)
class Conflict:
    """A normalized prediction that is identical from either robot's perspective."""

    conflict_type: ConflictType
    robot_ids: tuple[str, str]
    cell: Cell | None
    time_by_robot: tuple[tuple[str, int], tuple[str, int]]
    transitions: tuple[DirectedTransition, ...] = ()

    def __post_init__(self) -> None:
        if self.robot_ids != tuple(sorted(self.robot_ids)):
            raise ValueError("Conflict robot IDs must be sorted for deterministic agreement.")
        if tuple(robot_id for robot_id, _ in self.time_by_robot) != self.robot_ids:
            raise ValueError("Conflict times must be ordered by normalized robot IDs.")


@dataclass(slots=True)
class ConflictDetector:
    """Detect conflicts for one owner using only its LocalWorldModel."""

    owner_robot_id: str

    def detect(self, local_world: LocalWorldModel) -> tuple[Conflict, ...]:
        if local_world.robot_id != self.owner_robot_id:
            raise ValueError("ConflictDetector can only evaluate its owner's world model.")
        if local_world.own_state is None or local_world.own_state.intent is None:
            return ()
        conflicts: set[Conflict] = set()
        for peer_id in sorted(local_world.peer_states):
            peer_state = local_world.peer_states[peer_id]
            if peer_state.intent is None:
                continue
            conflicts.update(
                self.detect_pair(local_world.own_state, peer_state, local_world.static_map)
            )
        return tuple(sorted(conflicts, key=self._sort_key))

    def detect_pair(
        self,
        first: PeerStateMessage,
        second: PeerStateMessage,
        static_map: WarehouseGrid | None = None,
    ) -> tuple[Conflict, ...]:
        """Compare two message snapshots without consulting simulator state."""
        if first.robot_id == second.robot_id or first.intent is None or second.intent is None:
            return ()
        first_steps = {
            first.intent.generated_at + step.timestep: step.cell
            for step in first.intent.steps
        }
        second_steps = {
            second.intent.generated_at + step.timestep: step.cell
            for step in second.intent.steps
        }
        conflicts: set[Conflict] = set()

        for timestep in sorted(first_steps.keys() & second_steps.keys()):
            if first_steps[timestep] == second_steps[timestep]:
                conflicts.add(
                    self._occupancy_conflict(
                        ConflictType.SAME_CELL,
                        first.robot_id,
                        timestep,
                        second.robot_id,
                        timestep,
                        first_steps[timestep],
                    )
                )

        first_trajectory = {first.intent.generated_at: first.pose, **first_steps}
        second_trajectory = {second.intent.generated_at: second.pose, **second_steps}
        current_times = {first.intent.generated_at, second.intent.generated_at}
        for timestep in sorted(
            (first_trajectory.keys() & second_trajectory.keys()) - current_times
        ):
            if timestep - 1 not in first_trajectory or timestep - 1 not in second_trajectory:
                continue
            first_source = first_trajectory[timestep - 1]
            second_source = second_trajectory[timestep - 1]
            first_destination = first_trajectory[timestep]
            second_destination = second_trajectory[timestep]
            if first_source == second_destination and second_source == first_destination:
                conflicts.add(
                    self._transition_conflict(
                        ConflictType.HEAD_ON,
                        first.robot_id,
                        timestep,
                        first_source,
                        first_destination,
                        second.robot_id,
                        timestep,
                        second_source,
                        second_destination,
                    )
                )

        first_times_by_cell = self._times_by_cell(first_steps)
        second_times_by_cell = self._times_by_cell(second_steps)
        for cell in sorted(first_times_by_cell.keys() & second_times_by_cell.keys()):
            for first_time in first_times_by_cell[cell]:
                for second_time in second_times_by_cell[cell]:
                    if first_time != second_time:
                        conflicts.add(
                            self._occupancy_conflict(
                                ConflictType.INTERSECTION,
                                first.robot_id,
                                first_time,
                                second.robot_id,
                                second_time,
                                cell,
                            )
                        )

        if static_map is not None:
            corridor = self._detect_corridor_conflict(
                first, second, first_trajectory, second_trajectory, static_map
            )
            if corridor is not None:
                conflicts.add(corridor)
        return tuple(sorted(conflicts, key=self._sort_key))

    def _detect_corridor_conflict(
        self,
        first: PeerStateMessage,
        second: PeerStateMessage,
        first_trajectory: dict[int, Cell],
        second_trajectory: dict[int, Cell],
        static_map: WarehouseGrid,
    ) -> Conflict | None:
        common_cells = sorted(set(first_trajectory.values()) & set(second_trajectory.values()))
        for cell in common_cells:
            if not self._is_narrow_corridor_cell(static_map, cell):
                continue
            first_time = next(time for time, position in first_trajectory.items() if position == cell)
            second_time = next(time for time, position in second_trajectory.items() if position == cell)
            if first_time - 1 not in first_trajectory or second_time - 1 not in second_trajectory:
                continue
            first_source = first_trajectory[first_time - 1]
            second_source = second_trajectory[second_time - 1]
            first_vector = (cell.x - first_source.x, cell.y - first_source.y)
            second_vector = (cell.x - second_source.x, cell.y - second_source.y)
            if first_vector == (-second_vector[0], -second_vector[1]):
                return self._transition_conflict(
                    ConflictType.CORRIDOR,
                    first.robot_id,
                    first_time,
                    first_source,
                    cell,
                    second.robot_id,
                    second_time,
                    second_source,
                    cell,
                )
        return None

    @staticmethod
    def _is_narrow_corridor_cell(static_map: WarehouseGrid, cell: Cell) -> bool:
        neighbors = static_map.neighbors(cell)
        if len(neighbors) != 2:
            return False
        first, second = neighbors
        return first.x == second.x or first.y == second.y

    @staticmethod
    def _times_by_cell(steps: dict[int, Cell]) -> dict[Cell, tuple[int, ...]]:
        result: dict[Cell, list[int]] = {}
        for timestep, cell in steps.items():
            result.setdefault(cell, []).append(timestep)
        return {cell: tuple(times) for cell, times in result.items()}

    @staticmethod
    def _occupancy_conflict(
        conflict_type: ConflictType,
        first_id: str,
        first_time: int,
        second_id: str,
        second_time: int,
        cell: Cell,
    ) -> Conflict:
        times = tuple(sorted(((first_id, first_time), (second_id, second_time))))
        robot_ids = (times[0][0], times[1][0])
        return Conflict(conflict_type, robot_ids, cell, times)

    @staticmethod
    def _transition_conflict(
        conflict_type: ConflictType,
        first_id: str,
        first_time: int,
        first_source: Cell,
        first_destination: Cell,
        second_id: str,
        second_time: int,
        second_source: Cell,
        second_destination: Cell,
    ) -> Conflict:
        times = tuple(sorted(((first_id, first_time), (second_id, second_time))))
        transitions = tuple(
            sorted(
                (
                    DirectedTransition(first_id, first_time, first_source, first_destination),
                    DirectedTransition(second_id, second_time, second_source, second_destination),
                )
            )
        )
        return Conflict(conflict_type, (times[0][0], times[1][0]), None, times, transitions)

    @staticmethod
    def _sort_key(conflict: Conflict) -> tuple[object, ...]:
        return (
            conflict.conflict_type.value,
            conflict.robot_ids,
            conflict.time_by_robot,
            conflict.cell or Cell(-1, -1),
        )
