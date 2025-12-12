#!/usr/bin/env python3
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState


class AscendingState(BaseState):
    """Handles ascending to initial search height."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute ASCENDING state logic."""        
        if self.drone.latest_height < self.params.initial_search_height:
            self.node.get_logger().info(
                f"ASCENDING: Current height {self.drone.latest_height:.2f}m. "
                f"Ascending to {self.params.initial_search_height:.2f}m.",
                throttle_duration_sec=2.0
            )
            self.drone.move_up(self.params.ascending_speed)
            return None
        
        self.node.get_logger().info(
            f"ASCENDING: Height: {self.drone.latest_height:.2f}m reached."
        )
        self.drone.hover()
        
        # Check if waypoint navigation is enabled
        if self.params.enable_waypoint_navigation:
            if self.waypoint_manager and self.waypoint_manager.get_waypoint_count() > 0:
                self.node.get_logger().info(
                    "ASCENDING: Waypoint navigation enabled. "
                    "Transitioning to WAYPOINT_CENTERING."
                )
                return MissionState.WAYPOINT_CENTERING
            else:
                self.node.get_logger().warning(
                    "ASCENDING: Waypoint navigation enabled but no waypoints configured. "
                    "Transitioning to SEARCHING."
                )
                return MissionState.SEARCHING
        else:
            self.node.get_logger().info(
                "ASCENDING: Waypoint navigation disabled. Transitioning to SEARCHING."
            )
            return MissionState.SEARCHING