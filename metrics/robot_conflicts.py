"""Metrics recorded by each robot while evaluating local conflicts."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class RobotConflictMetrics:
    predicted_conflicts: int = 0
    conflict_types: dict[str, int] = field(default_factory=dict)
    conflicts_won: int = 0
    conflicts_lost: int = 0
    wait_events: int = 0
    yield_events: int = 0
    blocked_ticks: int = 0
    conflicts_cleared: int = 0
    resume_events: int = 0
    corridor_wait_ticks: int = 0
    corridor_tokens_acquired: int = 0
    deadlocks_detected: int = 0
    deadlocks_resolved: int = 0
    deadlocks_cleared: int = 0

    def record_conflict(self, conflict_type: str) -> None:
        self.predicted_conflicts += 1
        self.conflict_types[conflict_type] = self.conflict_types.get(conflict_type, 0) + 1

    def as_dict(self) -> dict[str, object]:
        return {
            "predicted_conflicts": self.predicted_conflicts,
            "conflict_types": dict(sorted(self.conflict_types.items())),
            "conflicts_won": self.conflicts_won,
            "conflicts_lost": self.conflicts_lost,
            "wait_events": self.wait_events,
            "yield_events": self.yield_events,
            "blocked_ticks": self.blocked_ticks,
            "conflicts_cleared": self.conflicts_cleared,
            "resume_events": self.resume_events,
            "corridor_wait_ticks": self.corridor_wait_ticks,
            "corridor_tokens_acquired": self.corridor_tokens_acquired,
            "deadlocks_detected": self.deadlocks_detected,
            "deadlocks_resolved": self.deadlocks_resolved,
            "deadlocks_cleared": self.deadlocks_cleared,
        }
