#!/usr/bin/env python3
"""Mission state definitions for the autonomous drone system."""
from enum import Enum


class MissionState(Enum):
    """Defines the operational states of the drone."""
    IDLE = 0
    TAKING_OFF = 1
    ASCENDING = 2
    SEARCHING = 3
    LOCKING_ON = 4
    CENTERING = 5
    APPROACHING = 6
    CAMERA_SWITCHING = 7
    PRECISION_LANDING = 8
    LANDING = 9
    COMPLETING_MISSION = 10
    RESETTING = 11
