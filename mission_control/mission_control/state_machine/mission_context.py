#!/usr/bin/env python3
"""Mission context containing shared runtime state."""
from dataclasses import dataclass


@dataclass(slots=True)
class MissionContext:
    """
    Shared runtime state for the mission.
    
    This context object is passed to all states, allowing them to read/write
    shared variables without direct coupling to other states.
    """
    
    # Searching state variables
    is_near_exit: bool = False
    
    # Precision landing state variables
    is_blind_landing: bool = False
    
    def reset(self):
        """Reset all runtime variables to their initial state."""
        self.is_near_exit = False
        self.is_blind_landing = False
