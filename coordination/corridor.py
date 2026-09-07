"""Robot-local distributed corridor ownership evaluation."""

from __future__ import annotations

from dataclasses import dataclass

from coordination.local_world import LocalWorldModel
from models.corridor import CorridorRegion, CorridorRequest, CorridorToken
from models.messages import PeerStateMessage


@dataclass(frozen=True, slots=True)
class CorridorProtocolConfig:
    token_lease_ticks: int = 3
    offline_timeout_ticks: int = 6

    def __post_init__(self) -> None:
        if self.token_lease_ticks <= 0 or self.offline_timeout_ticks <= 0:
            raise ValueError("Corridor lease and offline timeout must be positive.")


@dataclass(frozen=True, slots=True)
class CorridorOwnership:
    corridor_id: str
    owner_id: str | None
    evaluated_at: int
    token_expires_at: int | None


@dataclass(slots=True)
class CorridorProtocol:
    """Compute ownership locally from immutable peer messages."""

    owner_robot_id: str
    config: CorridorProtocolConfig = CorridorProtocolConfig()

    def evaluate(
        self,
        corridor: CorridorRegion,
        local_world: LocalWorldModel,
        timestamp: int,
    ) -> CorridorOwnership:
        if local_world.robot_id != self.owner_robot_id:
            raise ValueError("CorridorProtocol can only evaluate its owner's world model.")
        states = self._fresh_participants(corridor, local_world, timestamp)
        if not states:
            return CorridorOwnership(corridor.corridor_id, None, timestamp, None)

        token_owners: set[str] = set()
        token_expiries: dict[str, int] = {}
        for state in states:
            for token in state.corridor_tokens:
                if token.corridor_id == corridor.corridor_id and token.is_valid_at(timestamp):
                    token_owners.add(token.owner_id)
                    token_expiries[token.owner_id] = max(
                        token.expires_at, token_expiries.get(token.owner_id, token.expires_at)
                    )
        eligible_states = [state for state in states if state.robot_id in token_owners]
        if not eligible_states:
            eligible_states = states
        winner = min(eligible_states, key=self._priority_key)
        expiry = token_expiries.get(winner.robot_id)
        return CorridorOwnership(corridor.corridor_id, winner.robot_id, timestamp, expiry)

    def issue_or_refresh_token(self, corridor_id: str, timestamp: int) -> CorridorToken:
        return CorridorToken(
            corridor_id=corridor_id,
            owner_id=self.owner_robot_id,
            issued_at=timestamp,
            expires_at=timestamp + self.config.token_lease_ticks,
        )

    def _fresh_participants(
        self,
        corridor: CorridorRegion,
        local_world: LocalWorldModel,
        timestamp: int,
    ) -> list[PeerStateMessage]:
        candidates: list[PeerStateMessage] = []
        states = [local_world.own_state, *local_world.peer_states.values()]
        for state in states:
            if state is None:
                continue
            if timestamp - state.heartbeat > self.config.offline_timeout_ticks:
                continue
            requests_corridor = any(
                request.corridor_id == corridor.corridor_id
                for request in state.corridor_requests
            )
            owns_token = any(
                token.corridor_id == corridor.corridor_id
                and token.owner_id == state.robot_id
                and token.is_valid_at(timestamp)
                for token in state.corridor_tokens
            )
            if requests_corridor or owns_token or state.pose in corridor.cells:
                candidates.append(state)
        return candidates

    @staticmethod
    def _priority_key(state: PeerStateMessage) -> tuple[object, ...]:
        return (
            -int(state.safety_clear),
            -int(state.priority),
            -state.waiting_ticks,
            state.robot_id,
        )
