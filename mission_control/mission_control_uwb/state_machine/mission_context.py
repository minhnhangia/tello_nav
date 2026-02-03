#!/usr/bin/env python3
"""Mission context containing shared runtime state."""
from dataclasses import dataclass
from typing import Optional, Tuple
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

    # Searching state variables
    is_near_exit: bool = False
    
    # Precision landing state variables
    is_blind_landing: bool = False
    precision_landing_initialized: bool = False
    
    # Waypoint navigation state variables
    current_waypoint_target: Optional[Tuple[float, float]] = None  # (x, y) in meters
    current_waypoint_index: int = 0
    
    def reset(self):
        """Reset all runtime variables to their initial state."""
        self.standby_start_time = None        
        self.is_near_exit = False
        self.is_blind_landing = False
        self.precision_landing_initialized = False
        self.current_waypoint_target = None
        self.current_waypoint_index = 0
