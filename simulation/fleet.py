"""Multi-robot headless runner that validates, but never resolves, traffic conflicts."""

from __future__ import annotations

from dataclasses import dataclass, field

from models.corridor import CorridorRegion
from models.grid import Cell, WarehouseGrid
from models.robot_state import RobotState
from networking.peer_bus import InProcessPeerMessageBus
from simulation.robot import RobotAgent
from tasks.lifecycle import DeterministicTaskDispatcher, TaskStatus


class CollisionDetectedError(RuntimeError):
    """Raised when independently selected motions violate physical occupancy."""


@dataclass(slots=True)
class FleetSimulation:
    """Advance independent robots after checking their already-selected moves.

    This class has no traffic policy: it does not yield, prioritize, reroute, or
    otherwise choose between robot actions. A detected collision ends the tick
    and is recorded as a metric for a later decentralized coordination layer.
    """

    grid: WarehouseGrid
    robots: tuple[RobotAgent, ...]
    corridor_regions: tuple[CorridorRegion, ...] = ()
    task_dispatcher: DeterministicTaskDispatcher | None = None
    tick_count: int = field(default=0, init=False)
    collision_count: int = field(default=0, init=False)
    position_history: dict[str, list[Cell]] = field(default_factory=dict, init=False)
    occupancy_history: list[dict[str, Cell]] = field(default_factory=list, init=False)
    state_history: list[dict[str, RobotState]] = field(default_factory=list, init=False)
    message_bus: InProcessPeerMessageBus = field(default_factory=InProcessPeerMessageBus, init=False)

    def __post_init__(self) -> None:
        robot_ids = [robot.robot_id for robot in self.robots]
        if len(robot_ids) != len(set(robot_ids)):
            raise ValueError("Every robot must have a unique ID.")
        positions = [robot.position for robot in self.robots]
        if len(positions) != len(set(positions)):
            raise ValueError("Robots cannot spawn in the same cell.")
        for robot in self.robots:
            if not self.grid.is_walkable(robot.position):
                raise ValueError(f"Robot spawn is not walkable: {robot.position}")
            robot.attach_static_map(self.grid)
            robot.attach_corridor_regions(self.corridor_regions)
            self.message_bus.register(robot.robot_id)
            self.position_history[robot.robot_id] = [robot.position]
        self._record_tick_snapshot()

    def disable_robot(self, robot_id: str) -> None:
        """Environment fault injection only; peers infer failure from heartbeats."""
        robot = next((item for item in self.robots if item.robot_id == robot_id), None)
        if robot is None:
            raise KeyError(robot_id)
        robot.disable()

    def step(self) -> None:
        """Advance one tick using moves selected by independent robot agents."""
        for robot in self.robots:
            if robot.enabled:
                robot.receive_peer_messages(self.message_bus)
                robot.evaluate_peer_liveness(self.tick_count)
        if self.task_dispatcher is not None:
            self.task_dispatcher.update(self.robots, self.tick_count)
        proposals = {robot.robot_id: robot.decide_next_move() for robot in self.robots}
        self._reject_physical_collisions(proposals)
        for robot in self.robots:
            destination = proposals[robot.robot_id]
            if destination is not None:
                self.grid.validate_move(robot.position, destination)
        for robot in self.robots:
            destination = proposals[robot.robot_id]
            if destination is not None:
                robot.confirm_move(destination)
                self.position_history[robot.robot_id].append(destination)
        self.tick_count += 1
        for robot in self.robots:
            if robot.enabled:
                robot.publish_state(self.message_bus, timestamp=self.tick_count)
        if self.task_dispatcher is not None:
            self.task_dispatcher.update(self.robots, self.tick_count)
        self._record_tick_snapshot()
    def run_until_complete(self, max_ticks: int = 1_000) -> None:
        for _ in range(max_ticks):
            active_complete = all(not robot.enabled or robot.state in {RobotState.IDLE, RobotState.TASK_COMPLETE} for robot in self.robots)
            tasks_complete = self.task_dispatcher is None or all(task.status is TaskStatus.COMPLETED for task in self.task_dispatcher.tasks.values())
            if active_complete and tasks_complete:
                return
            self.step()
        raise TimeoutError(f"Fleet did not complete within {max_ticks} ticks.")

    def metrics_snapshot(self) -> dict[str, object]:
        return {
            "ticks": self.tick_count,
            "collisions": self.collision_count,
            "robots": {
                robot.robot_id: robot.conflict_metrics.as_dict()
                for robot in self.robots
            },
        }

    def _record_tick_snapshot(self) -> None:
        self.occupancy_history.append(
            {robot.robot_id: robot.position for robot in self.robots}
        )
        self.state_history.append(
            {robot.robot_id: robot.state for robot in self.robots}
        )

    def _reject_physical_collisions(self, proposals: dict[str, Cell | None]) -> None:
        current_positions = {robot.robot_id: robot.position for robot in self.robots}
        destinations = {
            robot_id: destination if destination is not None else current_positions[robot_id]
            for robot_id, destination in proposals.items()
        }
        if len(destinations.values()) != len(set(destinations.values())):
            self._record_collision("Two robots selected the same occupancy cell.")
        for first_id, first_destination in destinations.items():
            for second_id, second_destination in destinations.items():
                if first_id >= second_id:
                    continue
                if (
                    first_destination == current_positions[second_id]
                    and second_destination == current_positions[first_id]
                ):
                    self._record_collision("Two robots selected an edge swap.")

    def _record_collision(self, message: str) -> None:
        self.collision_count += 1
        raise CollisionDetectedError(message)