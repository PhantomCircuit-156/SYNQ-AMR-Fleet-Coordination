"""Robot lifecycle states for the simulation."""

from enum import Enum, auto


class RobotState(Enum):
    IDLE = auto()
    PLANNING = auto()
    REPLANNING = auto()
    MOVING = auto()
    CAUTION = auto()
    SAFETY_STOP = auto()
    YIELDING = auto()
    WAITING = auto()
    TASK_COMPLETE = auto()
    UNREACHABLE = auto()
    OFFLINE = auto()