"""Robot-local mutual-wait detection and deterministic backoff selection."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from coordination.local_world import LocalWorldModel
from models.corridor import CorridorRegion
from models.grid import Cell
from models.robot_state import RobotState


class DeadlockEventType(str, Enum):
    DEADLOCK_DETECTED = "DEADLOCK_DETECTED"
    DEADLOCK_RESOLUTION = "DEADLOCK_RESOLUTION"
    DEADLOCK_CLEARED = "DEADLOCK_CLEARED"


@dataclass(frozen=True, slots=True)
class DeadlockConfig:
    threshold_seconds: float = 6.0
    tick_seconds: float = 1.0
    offline_timeout_ticks: int = 6

    @property
    def threshold_ticks(self) -> int:
        return max(1, round(self.threshold_seconds / self.tick_seconds))


@dataclass(frozen=True, slots=True)
class DeadlockEvent:
    event_type: DeadlockEventType
    timestamp: int
    participants: tuple[str, str]
    detail: str


@dataclass(frozen=True, slots=True)
class DeadlockResolution:
    participants: tuple[str, str]
    proceed_robot_id: str
    backoff_robot_id: str
    passing_bay: Cell | None


@dataclass(slots=True)
class DeadlockManager:
    owner_robot_id: str
    config: DeadlockConfig = DeadlockConfig()
    events: list[DeadlockEvent] = field(default_factory=list)
    _active_participants: tuple[str, str] | None = None

    def evaluate(
        self,
        local_world: LocalWorldModel,
        corridors: tuple[CorridorRegion, ...],
        timestamp: int,
    ) -> DeadlockResolution | None:
        deadlock = self._find_mutual_wait(local_world, timestamp)
        if deadlock is None:
            if self._active_participants is not None:
                self.events.append(
                    DeadlockEvent(
                        DeadlockEventType.DEADLOCK_CLEARED,
                        timestamp,
                        self._active_participants,
                        "Mutual wait no longer present.",
                    )
                )
                self._active_participants = None
            return None

        participants = deadlock
        proceed_robot_id = participants[0]
        backoff_robot_id = participants[1]
        passing_bay = self._nearest_passing_bay(local_world, corridors, backoff_robot_id)
        if self._active_participants != participants:
            self._active_participants = participants
            self.events.extend(
                (
                    DeadlockEvent(
                        DeadlockEventType.DEADLOCK_DETECTED,
                        timestamp,
                        participants,
                        "Persistent mutual blocking exceeded the configured threshold.",
                    ),
                    DeadlockEvent(
                        DeadlockEventType.DEADLOCK_RESOLUTION,
                        timestamp,
                        participants,
                        f"{backoff_robot_id} backs off; {proceed_robot_id} proceeds.",
                    ),
                )
            )
        return DeadlockResolution(
            participants, proceed_robot_id, backoff_robot_id, passing_bay
        )

    def _find_mutual_wait(
        self, local_world: LocalWorldModel, timestamp: int
    ) -> tuple[str, str] | None:
        own = local_world.own_state
        if (
            own is None
            or own.state not in {RobotState.WAITING, RobotState.YIELDING}
            or own.waiting_ticks < self.config.threshold_ticks
        ):
            return None
        for peer_id in sorted(own.blocked_by):
            peer = local_world.peer_states.get(peer_id)
            if peer is None or timestamp - peer.heartbeat > self.config.offline_timeout_ticks:
                continue
            if (
                peer.state in {RobotState.WAITING, RobotState.YIELDING}
                and peer.waiting_ticks >= self.config.threshold_ticks
                and self.owner_robot_id in peer.blocked_by
            ):
                return tuple(sorted((self.owner_robot_id, peer_id)))
        return None

    @staticmethod
    def _nearest_passing_bay(
        local_world: LocalWorldModel,
        corridors: tuple[CorridorRegion, ...],
        backoff_robot_id: str,
    ) -> Cell | None:
        state = (
            local_world.own_state
            if backoff_robot_id == local_world.robot_id
            else local_world.peer_states.get(backoff_robot_id)
        )
        if state is None:
            return None
        bays = sorted({bay for corridor in corridors for bay in corridor.passing_bays})
        if not bays:
            return None
        return min(
            bays,
            key=lambda bay: (
                abs(bay.x - state.pose.x) + abs(bay.y - state.pose.y),
                bay,
            ),
        )
