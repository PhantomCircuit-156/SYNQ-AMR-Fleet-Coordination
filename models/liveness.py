"""Peer-local liveness states and structured events."""
from dataclasses import dataclass
from enum import Enum
HEARTBEAT_TIMEOUT = 6
class LivenessEventType(str, Enum):
    HEARTBEAT_TIMEOUT = "HEARTBEAT_TIMEOUT"
    ROBOT_OFFLINE = "ROBOT_OFFLINE"
@dataclass(frozen=True, slots=True)
class LivenessEvent:
    event_type: LivenessEventType
    observer_robot_id: str
    peer_robot_id: str
    timestamp: int