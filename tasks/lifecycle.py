"""Deterministic warehouse task lifecycle, separate from traffic coordination."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable, Protocol

from config.simulation import DROPOFF_DWELL_TICKS, PICKUP_DWELL_TICKS
from models.grid import Cell
from models.mission import MissionPhase, Payload, RobotLoadState, TaskType
from models.priority import TaskPriority
from models.robot_state import RobotState


class TaskStatus(str, Enum):
    PENDING = "PENDING"
    ASSIGNED = "ASSIGNED"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    RELEASED = "RELEASED"


class TaskEventType(str, Enum):
    TASK_RELEASED = "TASK_RELEASED"
    TASK_REASSIGNED = "TASK_REASSIGNED"


@dataclass(frozen=True, slots=True)
class TaskEvent:
    event_type: TaskEventType
    task_id: str
    timestamp: int
    robot_id: str


@dataclass(slots=True)
class FleetTask:
    """A deterministic warehouse mission.

    ``goal`` remains the destination field for backwards compatibility. Missions
    without ``source`` retain the original destination-only behavior.
    """

    task_id: str
    goal: Cell
    status: TaskStatus = TaskStatus.PENDING
    assigned_robot_id: str | None = None
    release_count: int = 0
    priority: TaskPriority = TaskPriority.NORMAL
    task_type: TaskType = TaskType.TRANSFER
    source: Cell | None = None
    payload: Payload | None = None
    phase: MissionPhase = field(default=MissionPhase.DIRECT, init=False)
    dwell_remaining_ticks: int = field(default=0, init=False)
    pickup_dwell_ticks: int = PICKUP_DWELL_TICKS
    dropoff_dwell_ticks: int = DROPOFF_DWELL_TICKS
    assigned_at: int | None = field(default=None, init=False)
    completed_at: int | None = field(default=None, init=False)
    phase_started_at: int | None = field(default=None, init=False)
    leg_initial_distance: int = field(default=0, init=False)
    assignment_reason: tuple[str, ...] = field(default=(), init=False)
    allocation_cost: float | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        if not self.task_id.strip():
            raise ValueError("Task ID cannot be empty.")
        if self.source == self.goal and self.source is not None:
            raise ValueError("Mission source and destination must be different cells.")
        if self.source is not None and self.payload is None:
            raise ValueError("Pickup/delivery missions require payload metadata.")
        if self.payload is not None and self.source is None:
            raise ValueError("A payload mission requires a pickup source.")
        if self.pickup_dwell_ticks < 0 or self.dropoff_dwell_ticks < 0:
            raise ValueError("Dwell ticks cannot be negative.")
        self.phase = MissionPhase.TO_PICKUP if self.source is not None else MissionPhase.DIRECT

    @property
    def destination(self) -> Cell:
        return self.goal

    def progress_percent(self, robot: TaskRobot | None) -> int:
        if self.status is TaskStatus.COMPLETED or self.phase is MissionPhase.COMPLETE:
            return 100
        if robot is None or self.status in {TaskStatus.PENDING, TaskStatus.RELEASED}:
            return 0
        if self.phase is MissionPhase.DIRECT:
            return self._leg_progress(robot, 0, 99)
        if self.phase is MissionPhase.TO_PICKUP:
            return self._leg_progress(robot, 0, 40)
        if self.phase is MissionPhase.PICKUP_DWELL:
            return self._dwell_progress(self.pickup_dwell_ticks, 40, 50)
        if self.phase is MissionPhase.TO_DROPOFF:
            return self._leg_progress(robot, 50, 95)
        if self.phase is MissionPhase.DROPOFF_DWELL:
            return self._dwell_progress(self.dropoff_dwell_ticks, 95, 100)
        return 0

    def eta_ticks(self, robot: TaskRobot | None) -> int | None:
        if robot is None or self.status in {TaskStatus.PENDING, TaskStatus.RELEASED}:
            return None
        if self.status is TaskStatus.COMPLETED:
            return 0
        if self.phase is MissionPhase.DIRECT:
            return _distance(robot.position, self.goal)
        if self.phase is MissionPhase.TO_PICKUP and self.source is not None:
            return _distance(robot.position, self.source) + self.pickup_dwell_ticks + _distance(self.source, self.goal) + self.dropoff_dwell_ticks
        if self.phase is MissionPhase.PICKUP_DWELL:
            return self.dwell_remaining_ticks + _distance(robot.position, self.goal) + self.dropoff_dwell_ticks
        if self.phase is MissionPhase.TO_DROPOFF:
            return _distance(robot.position, self.goal) + self.dropoff_dwell_ticks
        if self.phase is MissionPhase.DROPOFF_DWELL:
            return self.dwell_remaining_ticks
        return None

    def _leg_progress(self, robot: TaskRobot, start: int, end: int) -> int:
        target = self.source if self.phase is MissionPhase.TO_PICKUP else self.goal
        total = max(1, self.leg_initial_distance)
        remaining = _distance(robot.position, target)
        fraction = max(0.0, min(1.0, 1.0 - remaining / total))
        return min(end, max(start, round(start + (end - start) * fraction)))

    def _dwell_progress(self, total: int, start: int, end: int) -> int:
        if total == 0:
            return end
        elapsed = total - self.dwell_remaining_ticks
        return min(end, max(start, round(start + (end - start) * elapsed / total)))


class TaskRobot(Protocol):
    robot_id: str
    position: Cell
    state: RobotState
    enabled: bool
    priority: TaskPriority
    battery: float
    max_payload_kg: float
    load_state: RobotLoadState
    current_payload: Payload | None
    local_world_model: object

    def assign_goal(self, goal: Cell) -> None: ...
    def prepare_for_pickup(self) -> None: ...
    def load_payload(self, payload: Payload) -> None: ...
    def unload_payload(self) -> None: ...


@dataclass(slots=True)
class DeterministicTaskDispatcher:
    """Allocate logistics work only; never grant traffic or movement rights."""

    tasks: dict[str, FleetTask] = field(default_factory=dict)
    events: list[TaskEvent] = field(default_factory=list)

    def add_task(self, task: FleetTask) -> None:
        if task.task_id in self.tasks:
            raise ValueError(f"Duplicate task ID: {task.task_id}")
        self.tasks[task.task_id] = task

    def assign_initial(self, task_id: str, robot: TaskRobot, timestamp: int = 0) -> None:
        task = self.tasks[task_id]
        if task.status is not TaskStatus.PENDING:
            raise RuntimeError("Task is not pending.")
        self._ensure_capacity(task, robot)
        distance = _distance(robot.position, task.source or task.goal)
        task.assignment_reason = (
            f"selected robot {robot.robot_id}",
            f"distance to {'pickup' if task.source else 'destination'}: {distance} cells",
            f"battery: {robot.battery:.0f}%",
            self._capacity_reason(task, robot),
            "deterministic robot ID tie-break",
        )
        task.allocation_cost = float(distance)
        self._assign(task, robot, timestamp)

    def assign_best(self, task_id: str, robots: Iterable[TaskRobot], timestamp: int = 0, *, reassignment: bool = False) -> TaskRobot:
        task = self.tasks[task_id]
        expected = {TaskStatus.RELEASED} if reassignment else {TaskStatus.PENDING}
        if task.status not in expected:
            raise RuntimeError("Task is not available for allocation.")
        robot_list = tuple(robots)
        candidates = [r for r in robot_list if self._is_available(r) and self._capacity_eligible(task, r)]
        if not candidates:
            raise RuntimeError("No idle robot with sufficient payload capacity is available.")
        target = task.source or task.goal

        def score(robot: TaskRobot) -> tuple[float, str]:
            distance = _distance(robot.position, target)
            workload = sum(1 for item in self.tasks.values() if item.assigned_robot_id == robot.robot_id and item.status is not TaskStatus.COMPLETED)
            workload_penalty = workload * 20
            battery_penalty = 10 if robot.battery < 50 else 0
            return distance + workload_penalty + battery_penalty, robot.robot_id

        winner = min(candidates, key=score)
        cost, _ = score(winner)
        distance = _distance(winner.position, target)
        workload = sum(1 for item in self.tasks.values() if item.assigned_robot_id == winner.robot_id and item.status is not TaskStatus.COMPLETED)
        battery_penalty = 10 if winner.battery < 50 else 0
        reasons = [
            f"lowest deterministic allocation cost: {cost:.0f}",
            f"distance to {'pickup' if task.source else 'destination'}: {distance} cells",
            f"workload penalty: {workload * 20}",
            f"battery penalty: {battery_penalty}",
            f"battery: {winner.battery:.0f}%",
            self._capacity_reason(task, winner),
            "robot ID is the final tie-break",
        ]
        task.assignment_reason = tuple(reasons)
        task.allocation_cost = cost
        self._assign(task, winner, timestamp)
        if reassignment:
            self.events.append(TaskEvent(TaskEventType.TASK_REASSIGNED, task.task_id, timestamp, winner.robot_id))
        return winner

    def update(self, robots: Iterable[TaskRobot], timestamp: int) -> None:
        robot_by_id = {robot.robot_id: robot for robot in robots}
        for task in sorted(self.tasks.values(), key=lambda item: item.task_id):
            assigned = robot_by_id.get(task.assigned_robot_id) if task.assigned_robot_id else None
            if assigned is not None and assigned.enabled:
                self._advance_task(task, assigned, timestamp)
                continue
            if assigned is not None and not self._reported_offline(assigned.robot_id, robot_by_id.values()):
                continue
            if task.status in {TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS}:
                previous_id = task.assigned_robot_id or ""
                if assigned is not None and assigned.current_payload == task.payload:
                    assigned.unload_payload()
                task.status = TaskStatus.RELEASED
                task.release_count += 1
                task.assigned_robot_id = None
                task.phase = MissionPhase.TO_PICKUP if task.source is not None else MissionPhase.DIRECT
                task.dwell_remaining_ticks = 0
                self.events.append(TaskEvent(TaskEventType.TASK_RELEASED, task.task_id, timestamp, previous_id))
            if task.status is TaskStatus.RELEASED:
                try:
                    self.assign_best(task.task_id, robot_by_id.values(), timestamp, reassignment=True)
                except RuntimeError:
                    pass

    def _advance_task(self, task: FleetTask, robot: TaskRobot, timestamp: int) -> None:
        if task.status is TaskStatus.ASSIGNED:
            task.status = TaskStatus.IN_PROGRESS
        if task.phase is MissionPhase.DIRECT:
            if robot.state is RobotState.TASK_COMPLETE and robot.position == task.goal:
                self._complete(task, robot, timestamp)
            return
        if task.phase is MissionPhase.TO_PICKUP and robot.state is RobotState.TASK_COMPLETE and robot.position == task.source:
            task.phase = MissionPhase.PICKUP_DWELL
            task.phase_started_at = timestamp
            task.dwell_remaining_ticks = task.pickup_dwell_ticks
            if task.pickup_dwell_ticks == 0:
                self._finish_pickup(task, robot, timestamp)
            return
        if task.phase is MissionPhase.PICKUP_DWELL:
            elapsed = max(0, timestamp - (task.phase_started_at if task.phase_started_at is not None else timestamp))
            task.dwell_remaining_ticks = max(0, task.pickup_dwell_ticks - elapsed)
            if task.dwell_remaining_ticks == 0:
                self._finish_pickup(task, robot, timestamp)
            return
        if task.phase is MissionPhase.TO_DROPOFF and robot.state is RobotState.TASK_COMPLETE and robot.position == task.goal:
            task.phase = MissionPhase.DROPOFF_DWELL
            task.phase_started_at = timestamp
            task.dwell_remaining_ticks = task.dropoff_dwell_ticks
            if task.dropoff_dwell_ticks == 0:
                self._complete(task, robot, timestamp)
            return
        if task.phase is MissionPhase.DROPOFF_DWELL:
            elapsed = max(0, timestamp - (task.phase_started_at if task.phase_started_at is not None else timestamp))
            task.dwell_remaining_ticks = max(0, task.dropoff_dwell_ticks - elapsed)
            if task.dwell_remaining_ticks == 0:
                self._complete(task, robot, timestamp)

    def _finish_pickup(self, task: FleetTask, robot: TaskRobot, timestamp: int) -> None:
        if task.payload is None:
            raise RuntimeError("Pickup phase has no payload.")
        robot.load_payload(task.payload)
        task.phase = MissionPhase.TO_DROPOFF
        task.phase_started_at = timestamp
        task.leg_initial_distance = max(1, _distance(robot.position, task.goal))
        robot.assign_goal(task.goal)
        robot.load_state = RobotLoadState.EN_ROUTE_TO_DROPOFF

    def _complete(self, task: FleetTask, robot: TaskRobot, timestamp: int) -> None:
        robot.unload_payload()
        task.phase = MissionPhase.COMPLETE
        task.status = TaskStatus.COMPLETED
        task.completed_at = timestamp
        task.dwell_remaining_ticks = 0

    def _assign(self, task: FleetTask, robot: TaskRobot, timestamp: int) -> None:
        target = task.source or task.goal
        robot.priority = task.priority
        robot.assign_goal(target)
        if task.source is not None:
            robot.prepare_for_pickup()
        else:
            robot.unload_payload()
        task.assigned_robot_id = robot.robot_id
        task.status = TaskStatus.ASSIGNED
        task.assigned_at = timestamp
        task.phase_started_at = timestamp
        task.leg_initial_distance = max(1, _distance(robot.position, target))

    def _is_available(self, robot: TaskRobot) -> bool:
        if not robot.enabled or robot.state not in {RobotState.IDLE, RobotState.TASK_COMPLETE}:
            return False
        return not any(task.assigned_robot_id == robot.robot_id and task.status is not TaskStatus.COMPLETED for task in self.tasks.values())

    @staticmethod
    def _capacity_eligible(task: FleetTask, robot: TaskRobot) -> bool:
        return task.payload is None or task.payload.weight_kg <= robot.max_payload_kg

    def _ensure_capacity(self, task: FleetTask, robot: TaskRobot) -> None:
        if not self._capacity_eligible(task, robot):
            raise ValueError(f"{robot.robot_id} capacity {robot.max_payload_kg:.1f} kg is below payload weight {task.payload.weight_kg:.1f} kg.")

    @staticmethod
    def _capacity_reason(task: FleetTask, robot: TaskRobot) -> str:
        if task.payload is None:
            return f"no payload; robot capacity {robot.max_payload_kg:.1f} kg"
        return f"payload {task.payload.weight_kg:.1f} kg within {robot.max_payload_kg:.1f} kg capacity"

    @staticmethod
    def _reported_offline(robot_id: str, robots: Iterable[TaskRobot]) -> bool:
        return any(robot.enabled and robot_id in robot.local_world_model.offline_peers for robot in robots)


def _distance(first: Cell, second: Cell) -> int:
    return abs(first.x - second.x) + abs(first.y - second.y)