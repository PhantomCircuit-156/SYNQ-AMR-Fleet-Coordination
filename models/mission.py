"""Warehouse mission and payload value objects.

These models describe logistics work only. They grant no movement or traffic rights.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from models.priority import TaskPriority


class TaskType(str, Enum):
    TRANSFER = "TRANSFER"
    PICKUP_DELIVERY = "PICKUP_DELIVERY"
    REPLENISHMENT = "REPLENISHMENT"
    EXPEDITE = "EXPEDITE"


class PayloadType(str, Enum):
    TOTE = "TOTE"
    CARTON = "CARTON"
    PALLET = "PALLET"
    PARTS_BIN = "PARTS_BIN"


class RobotLoadState(str, Enum):
    EMPTY = "EMPTY"
    EN_ROUTE_TO_PICKUP = "EN_ROUTE_TO_PICKUP"
    LOADED = "LOADED"
    EN_ROUTE_TO_DROPOFF = "EN_ROUTE_TO_DROPOFF"


class MissionPhase(str, Enum):
    DIRECT = "DIRECT"
    TO_PICKUP = "TO_PICKUP"
    PICKUP_DWELL = "PICKUP_DWELL"
    TO_DROPOFF = "TO_DROPOFF"
    DROPOFF_DWELL = "DROPOFF_DWELL"
    COMPLETE = "COMPLETE"


@dataclass(frozen=True, slots=True)
class Payload:
    payload_id: str
    payload_type: PayloadType
    weight_kg: float
    fragile: bool = False
    priority: TaskPriority = TaskPriority.NORMAL

    def __post_init__(self) -> None:
        if not self.payload_id.strip():
            raise ValueError("Payload ID cannot be empty.")
        if self.weight_kg <= 0 or self.weight_kg > 1_000:
            raise ValueError("Payload weight must be greater than 0 kg and no more than 1000 kg.")