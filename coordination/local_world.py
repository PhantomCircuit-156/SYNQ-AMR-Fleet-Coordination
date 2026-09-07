"""Robot-owned local knowledge reconstructed from self state and peer messages."""
from __future__ import annotations
from dataclasses import dataclass, field
from coordination.intent import Intent
from models.corridor import CorridorRegion, CorridorRequest, CorridorToken
from models.grid import Cell, WarehouseGrid
from models.liveness import HEARTBEAT_TIMEOUT, LivenessEvent, LivenessEventType
from models.messages import PeerStateMessage
from models.priority import TaskPriority

@dataclass(slots=True)
class LocalWorldModel:
    robot_id: str
    static_map: WarehouseGrid | None = None
    own_state: PeerStateMessage | None = None
    peer_states: dict[str, PeerStateMessage] = field(default_factory=dict)
    peer_intents: dict[str, Intent | None] = field(default_factory=dict)
    peer_priorities: dict[str, TaskPriority] = field(default_factory=dict)
    peer_heartbeats: dict[str, int] = field(default_factory=dict)
    known_dynamic_obstacles: set[Cell] = field(default_factory=set)
    corridor_regions: tuple[CorridorRegion, ...] = ()
    peer_corridor_requests: dict[str, tuple[CorridorRequest, ...]] = field(default_factory=dict)
    peer_corridor_tokens: dict[str, tuple[CorridorToken, ...]] = field(default_factory=dict)
    offline_peers: set[str] = field(default_factory=set)
    liveness_events: list[LivenessEvent] = field(default_factory=list)
    def set_static_map(self, static_map: WarehouseGrid) -> None:
        if self.static_map is not None and self.static_map != static_map: raise ValueError("A robot's static map cannot be replaced during this phase.")
        self.static_map = static_map
    def set_corridor_regions(self, regions: tuple[CorridorRegion, ...]) -> None:
        if self.corridor_regions and self.corridor_regions != regions: raise ValueError("Corridor knowledge cannot be replaced during a run.")
        self.corridor_regions = regions
    def update_own_state(self, message: PeerStateMessage) -> None:
        if message.robot_id != self.robot_id: raise ValueError("Own state must belong to this local world model's robot.")
        self.own_state = message
    def receive_peer_state(self, message: PeerStateMessage) -> bool:
        if message.robot_id == self.robot_id: return False
        previous = self.peer_states.get(message.robot_id)
        if previous is not None and message.timestamp < previous.timestamp: return False
        self.peer_states[message.robot_id] = message; self.peer_intents[message.robot_id] = message.intent
        self.peer_priorities[message.robot_id] = message.priority; self.peer_heartbeats[message.robot_id] = message.heartbeat
        self.peer_corridor_requests[message.robot_id] = message.corridor_requests; self.peer_corridor_tokens[message.robot_id] = message.corridor_tokens
        self.offline_peers.discard(message.robot_id)
        return True
    @property
    def peer_last_seen(self) -> dict[str, int]: return self.peer_heartbeats
    def expire_stale_peers(self, current_time: int, timeout: int = HEARTBEAT_TIMEOUT) -> tuple[str, ...]:
        newly=[]
        for peer_id,last_seen in sorted(self.peer_heartbeats.items()):
            if current_time-last_seen <= timeout or peer_id in self.offline_peers: continue
            self.offline_peers.add(peer_id); newly.append(peer_id)
            self.liveness_events.extend((LivenessEvent(LivenessEventType.HEARTBEAT_TIMEOUT,self.robot_id,peer_id,current_time),LivenessEvent(LivenessEventType.ROBOT_OFFLINE,self.robot_id,peer_id,current_time)))
            self.peer_states.pop(peer_id,None); self.peer_intents.pop(peer_id,None); self.peer_priorities.pop(peer_id,None); self.peer_corridor_requests.pop(peer_id,None); self.peer_corridor_tokens.pop(peer_id,None)
        return tuple(newly)