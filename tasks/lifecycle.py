"""Deterministic task lifecycle, deliberately separate from traffic coordination."""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable, Protocol
from models.grid import Cell
from models.robot_state import RobotState
from models.priority import TaskPriority

class TaskStatus(str, Enum):
    PENDING="PENDING"; ASSIGNED="ASSIGNED"; IN_PROGRESS="IN_PROGRESS"; COMPLETED="COMPLETED"; RELEASED="RELEASED"
class TaskEventType(str, Enum):
    TASK_RELEASED="TASK_RELEASED"; TASK_REASSIGNED="TASK_REASSIGNED"
@dataclass(frozen=True, slots=True)
class TaskEvent:
    event_type: TaskEventType
    task_id: str
    timestamp: int
    robot_id: str
@dataclass(slots=True)
class FleetTask:
    task_id: str
    goal: Cell
    status: TaskStatus=TaskStatus.PENDING
    assigned_robot_id: str|None=None
    release_count: int=0
    priority: TaskPriority=TaskPriority.NORMAL
class TaskRobot(Protocol):
    robot_id: str; position: Cell; state: RobotState; enabled: bool
    local_world_model: object
    def assign_goal(self, goal: Cell) -> None: ...
@dataclass(slots=True)
class DeterministicTaskDispatcher:
    """Assign work only; it never grants traffic priority or movement rights."""
    tasks: dict[str,FleetTask]=field(default_factory=dict)
    events: list[TaskEvent]=field(default_factory=list)
    def add_task(self, task: FleetTask) -> None:
        if task.task_id in self.tasks: raise ValueError(f"Duplicate task ID: {task.task_id}")
        self.tasks[task.task_id]=task
    def assign_initial(self, task_id: str, robot: TaskRobot) -> None:
        task=self.tasks[task_id]
        if task.status is not TaskStatus.PENDING: raise RuntimeError("Task is not pending.")
        robot.priority=task.priority; robot.assign_goal(task.goal); task.assigned_robot_id=robot.robot_id; task.status=TaskStatus.ASSIGNED
    def update(self, robots: Iterable[TaskRobot], timestamp: int) -> None:
        robot_by_id={r.robot_id:r for r in robots}
        for task in sorted(self.tasks.values(),key=lambda t:t.task_id):
            assigned=robot_by_id.get(task.assigned_robot_id) if task.assigned_robot_id else None
            if assigned is not None and assigned.enabled:
                if assigned.state is RobotState.TASK_COMPLETE and assigned.position == task.goal: task.status=TaskStatus.COMPLETED
                elif task.status is TaskStatus.ASSIGNED: task.status=TaskStatus.IN_PROGRESS
                continue
            if assigned is not None and not self._reported_offline(assigned.robot_id,robot_by_id.values()): continue
            if task.status in {TaskStatus.ASSIGNED,TaskStatus.IN_PROGRESS}:
                task.status=TaskStatus.RELEASED; task.release_count += 1
                self.events.append(TaskEvent(TaskEventType.TASK_RELEASED,task.task_id,timestamp,task.assigned_robot_id or "")); task.assigned_robot_id=None
            if task.status is TaskStatus.RELEASED:
                candidates=[r for r in robot_by_id.values() if r.enabled and r.state in {RobotState.IDLE,RobotState.TASK_COMPLETE}]
                if candidates:
                    winner=min(candidates,key=lambda r:(abs(r.position.x-task.goal.x)+abs(r.position.y-task.goal.y),r.robot_id))
                    winner.priority=task.priority; winner.assign_goal(task.goal); task.assigned_robot_id=winner.robot_id; task.status=TaskStatus.ASSIGNED
                    self.events.append(TaskEvent(TaskEventType.TASK_REASSIGNED,task.task_id,timestamp,winner.robot_id))
    @staticmethod
    def _reported_offline(robot_id: str, robots: Iterable[TaskRobot]) -> bool:
        return any(r.enabled and robot_id in r.local_world_model.offline_peers for r in robots)