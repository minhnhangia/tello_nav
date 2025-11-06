#!/usr/bin/env python3
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState


class CameraSwitchingState(BaseState):
    """Handles camera switch to downward view."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute CAMERA_SWITCHING state logic."""
        self.node.get_logger().info(
            "Switching to downward-facing camera for precision landing."
        )
        self.drone.execute_action(
            'downvision 1',
            MissionState.PRECISION_LANDING,
            MissionState.CAMERA_SWITCHING
        )
        return None