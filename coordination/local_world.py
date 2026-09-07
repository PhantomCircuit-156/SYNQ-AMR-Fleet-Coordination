"""Robot-owned local knowledge reconstructed from self state and peer messages."""

from __future__ import annotations

from dataclasses import dataclass, field

from coordination.intent import Intent
from models.corridor import CorridorRegion, CorridorRequest, CorridorToken
from models.grid import Cell, WarehouseGrid
from models.messages import PeerStateMessage
from models.priority import TaskPriority


@dataclass(slots=True)
class LocalWorldModel:
    """A single robot's local, potentially incomplete view of the warehouse."""

    robot_id: str
    static_map: WarehouseGrid | None = None
    own_state: PeerStateMessage | None = None
    peer_states: dict[str, PeerStateMessage] = field(default_factory=dict)
    peer_intents: dict[str, Intent | None] = field(default_factory=dict)
    peer_priorities: dict[str, TaskPriority] = field(default_factory=dict)
    peer_heartbeats: dict[str, int] = field(default_factory=dict)
    known_dynamic_obstacles: set[Cell] = field(default_factory=set)
    corridor_regions: tuple[CorridorRegion, ...] = ()
    peer_corridor_requests: dict[str, tuple[CorridorRequest, ...]] = field(
        default_factory=dict
    )
    peer_corridor_tokens: dict[str, tuple[CorridorToken, ...]] = field(
        default_factory=dict
    )

    def set_static_map(self, static_map: WarehouseGrid) -> None:
        """Store the robot's local copy/reference to immutable static geometry."""
        if self.static_map is not None and self.static_map != static_map:
            raise ValueError("A robot's static map cannot be replaced during this phase.")
        self.static_map = static_map

    def set_corridor_regions(self, regions: tuple[CorridorRegion, ...]) -> None:
        if self.corridor_regions and self.corridor_regions != regions:
            raise ValueError("Corridor knowledge cannot be replaced during a run.")
        self.corridor_regions = regions

    def update_own_state(self, message: PeerStateMessage) -> None:
        if message.robot_id != self.robot_id:
            raise ValueError("Own state must belong to this local world model's robot.")
        self.own_state = message

    def receive_peer_state(self, message: PeerStateMessage) -> bool:
        """Update local peer knowledge from one transport-delivered message."""
        if message.robot_id == self.robot_id:
            return False
        previous = self.peer_states.get(message.robot_id)
        if previous is not None and message.timestamp < previous.timestamp:
            return False
        self.peer_states[message.robot_id] = message
        self.peer_intents[message.robot_id] = message.intent
        self.peer_priorities[message.robot_id] = message.priority
        self.peer_heartbeats[message.robot_id] = message.heartbeat
        self.peer_corridor_requests[message.robot_id] = message.corridor_requests
        self.peer_corridor_tokens[message.robot_id] = message.corridor_tokens
        return True