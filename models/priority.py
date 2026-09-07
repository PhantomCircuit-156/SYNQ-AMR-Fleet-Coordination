"""Deterministic task-priority levels used in peer coordination claims."""

from enum import IntEnum


class TaskPriority(IntEnum):
    LOW = 0
    NORMAL = 1
    HIGH = 2
