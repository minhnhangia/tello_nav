#!/usr/bin/env python3
"""Mission state definitions for the autonomous drone system."""
from enum import Enum


class MissionState(Enum):
    """Defines the operational states of the drone."""
    IDLE = 0
    TAKING_OFF = 1
    ASCENDING = 2
    STANDBY = 3
    WAYPOINT_CENTERING = 4
    WAYPOINT_APPROACHING = 5
    WAYPOINT_ACTION = 6
    SEARCHING = 7
    LOCKING_ON = 8
    CENTERING = 9
    APPROACHING = 10
    CAMERA_SWITCHING = 11
    PRECISION_LANDING = 12
    LANDING = 13
    COMPLETING_MISSION = 14
    RESETTING = 15
    PRIORITY_SCANNING = 16