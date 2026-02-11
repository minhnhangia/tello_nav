#!/usr/bin/env python3
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState


class TakingOffState(BaseState):
    """Handles takeoff logic."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute TAKING_OFF state logic."""
        if self.drone.latest_height >= self.params.min_takeoff_height:
            self.node.get_logger().info(
                f"TAKING_OFF: Already airborne at {self.drone.latest_height:.2f}m. "
                "Transitioning to ASCENDING."
            )
            return MissionState.ASCENDING
        
        self.node.get_logger().info("TAKING_OFF: attempting takeoff.")
        self.drone.execute_action(
            'takeoff',
            MissionState.ASCENDING,
            MissionState.TAKING_OFF,
            max_retries=1
        )
        return None