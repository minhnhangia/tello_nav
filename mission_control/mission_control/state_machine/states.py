#!/usr/bin/env python3
"""Mission state definitions for the autonomous drone system."""
from enum import Enum


class MissionState(Enum):
    """Defines the operational states of the drone."""
    IDLE = 0
    TAKING_OFF = 1
    ASCENDING = 2
    WAYPOINT_CENTERING = 3
    WAYPOINT_APPROACHING = 4
    WAYPOINT_ACTION = 5
    SEARCHING = 6
    LOCKING_ON = 7
    CENTERING = 8
    APPROACHING = 9
    CAMERA_SWITCHING = 10
    PRECISION_LANDING = 11
    LANDING = 12
    COMPLETING_MISSION = 13
    RESETTING = 14