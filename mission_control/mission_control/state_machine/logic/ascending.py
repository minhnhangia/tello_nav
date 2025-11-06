#!/usr/bin/env python3
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState


class AscendingState(BaseState):
    """Handles ascending to initial search height."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute ASCENDING state logic."""
        if self.drone.latest_height >= self.params.initial_search_height:
            self.node.get_logger().info(
                f"ASCENDING: Height: {self.drone.latest_height:.2f}m. "
                "Transitioning to SEARCHING."
            )
            self.drone.hover()
            return MissionState.SEARCHING
        
        self.node.get_logger().info(
            f"ASCENDING: Current height {self.drone.latest_height:.2f}m. "
            f"Ascending to {self.params.initial_search_height:.2f}m.",
            throttle_duration_sec=2.0
        )
        self.drone.move_up(self.params.ascending_speed)
        return None