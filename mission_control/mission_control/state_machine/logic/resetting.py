#!/usr/bin/env python3
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState


class ResettingState(BaseState):
    """Handles cleanup and return to IDLE."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute RESETTING state logic."""
        self.node.get_logger().info("Resetting Mission Control to initial state.")
        
        # Reset all shared runtime state via context
        self.context.reset()
        
        self.marker_handler.reset_marker_state()
        self.drone.execute_action(
            'downvision 0',
            MissionState.IDLE,
            MissionState.IDLE,
            max_retries=2
        )
        return None