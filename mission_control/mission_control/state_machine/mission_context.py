#!/usr/bin/env python3
"""Mission context containing shared runtime state."""
from dataclasses import dataclass
from typing import Optional
from rclpy.time import Time


@dataclass(slots=True)
class MissionContext:
    """
    Shared runtime state for the mission.
    
    This context object is passed to all states, allowing them to read/write
    shared variables without direct coupling to other states.
    """

    # Standby state variables
    standby_start_time: Optional[Time] = None

    # Waypoint approaching variables
    camera_switched: bool = False
    downward_camera_active: bool = False
    marker_detected_with_down_camera: bool = False

    # Searching state variables
    is_near_exit: bool = False
    
    # Precision landing state variables
    is_blind_landing: bool = False
    precision_landing_initialized: bool = False

    # Drone detection (YOLO) avoidance variables
    drone_avoidance_start_time: Optional[Time] = None
    drone_position: int = -1  # DroneDetection.POSITION_LEFT / POSITION_RIGHT (-1 = unset)
    drone_avoidance_direction: str = ""  # "left" or "right"
    
    def reset(self):
        """Reset all runtime variables to their initial state."""
        self.standby_start_time = None
        self.camera_switched = False
        self.downward_camera_active = False
        self.marker_detected_with_down_camera = False    
        self.is_near_exit = False
        self.is_blind_landing = False
        self.precision_landing_initialized = False
        self.drone_avoidance_start_time = None
        self.drone_position = -1
        self.drone_avoidance_direction = ""
