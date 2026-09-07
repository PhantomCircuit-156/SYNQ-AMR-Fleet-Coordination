"""Peer-to-peer state messages transported without coordination policy."""

from __future__ import annotations

from dataclasses import dataclass

from coordination.intent import Intent
from models.corridor import CorridorRequest, CorridorToken
from models.grid import Cell
from models.priority import TaskPriority
from models.robot_state import RobotState


@dataclass(frozen=True, slots=True)
class PeerStateMessage:
    """A robot's self-published state at one deterministic simulation timestamp."""

    robot_id: str
    timestamp: int
    pose: Cell
    velocity: tuple[int, int]
    goal: Cell | None
    intent: Intent | None
    priority: TaskPriority
    battery: float
    state: RobotState
    heartbeat: int
    waiting_ticks: int = 0
    safety_clear: bool = True
    corridor_requests: tuple[CorridorRequest, ...] = ()
    corridor_tokens: tuple[CorridorToken, ...] = ()
    blocked_by: tuple[str, ...] = ()