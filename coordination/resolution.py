"""Symmetric deterministic conflict-resolution policy used locally by robots."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from coordination.conflict import Conflict
from coordination.local_world import LocalWorldModel
from models.priority import TaskPriority


class ResolutionReason(Enum):
    SAFETY = auto()
    TASK_PRIORITY = auto()
    WAITING_FAIRNESS = auto()
    ROBOT_ID = auto()
    BOTH_UNSAFE = auto()


@dataclass(frozen=True, slots=True)
class ResolutionCandidate:
    robot_id: str
    task_priority: TaskPriority
    waiting_ticks: int = 0
    safety_clear: bool = True


@dataclass(frozen=True, slots=True)
class ConflictResolution:
    winner_id: str | None
    yielding_robot_ids: tuple[str, ...]
    reason: ResolutionReason


@dataclass(slots=True)
class ConflictResolver:
    """Compute the same winner regardless of caller or argument order."""

    owner_robot_id: str

    def resolve(
        self,
        first: ResolutionCandidate,
        second: ResolutionCandidate,
    ) -> ConflictResolution:
        if first.robot_id == second.robot_id:
            raise ValueError("Conflict resolution requires two different robots.")
        ordered_ids = tuple(sorted((first.robot_id, second.robot_id)))
        if not first.safety_clear and not second.safety_clear:
            return ConflictResolution(None, ordered_ids, ResolutionReason.BOTH_UNSAFE)
        if first.safety_clear != second.safety_clear:
            winner = first if first.safety_clear else second
            return self._winner(winner.robot_id, ordered_ids, ResolutionReason.SAFETY)
        if first.task_priority != second.task_priority:
            winner = first if first.task_priority > second.task_priority else second
            return self._winner(winner.robot_id, ordered_ids, ResolutionReason.TASK_PRIORITY)
        if first.waiting_ticks != second.waiting_ticks:
            winner = first if first.waiting_ticks > second.waiting_ticks else second
            return self._winner(winner.robot_id, ordered_ids, ResolutionReason.WAITING_FAIRNESS)
        return self._winner(ordered_ids[0], ordered_ids, ResolutionReason.ROBOT_ID)

    def resolve_from_local_world(
        self,
        conflict: Conflict,
        local_world: LocalWorldModel,
    ) -> ConflictResolution:
        """Resolve using only self state and peer state stored in the local model."""
        if local_world.robot_id != self.owner_robot_id or local_world.own_state is None:
            raise ValueError("Resolver can only use its owner's populated local world model.")
        peer_ids = [robot_id for robot_id in conflict.robot_ids if robot_id != self.owner_robot_id]
        if len(peer_ids) != 1 or peer_ids[0] not in local_world.peer_states:
            raise ValueError("Conflict peer is absent from the local world model.")
        own = local_world.own_state
        peer = local_world.peer_states[peer_ids[0]]
        return self.resolve(
            ResolutionCandidate(own.robot_id, own.priority, own.waiting_ticks, own.safety_clear),
            ResolutionCandidate(peer.robot_id, peer.priority, peer.waiting_ticks, peer.safety_clear),
        )

    @staticmethod
    def _winner(
        winner_id: str,
        ordered_ids: tuple[str, str],
        reason: ResolutionReason,
    ) -> ConflictResolution:
        yielding = tuple(robot_id for robot_id in ordered_ids if robot_id != winner_id)
        return ConflictResolution(winner_id, yielding, reason)
