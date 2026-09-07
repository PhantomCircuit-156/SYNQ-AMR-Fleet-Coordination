"""Typed operator-injected environment occupancy metadata."""
from enum import Enum


class EnvironmentObjectType(str, Enum):
    BOX = "BOX"
    PALLET = "PALLET"
    FORKLIFT = "FORKLIFT"
    BLOCKED_AISLE = "BLOCKED_AISLE"