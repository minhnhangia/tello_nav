#!/usr/bin/env python3
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState


class LandingState(BaseState):
    """Handles landing action."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute LANDING state logic."""
        if self.drone.latest_height < self.params.min_takeoff_height:
            self.node.get_logger().info(
                f"LANDING: Already landed at {self.drone.latest_height:.2f}m. "
                "Transitioning to COMPLETING_MISSION."
            )
            return MissionState.COMPLETING_MISSION
        
        self.drone.execute_action(
            "land",
            MissionState.COMPLETING_MISSION,
            MissionState.LANDING
        )
        return None