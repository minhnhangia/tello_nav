#!/usr/bin/env python3
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState


class SearchingState(BaseState):
    """
    Performs 360° scan to search for markers at each waypoint.
    
    This state is entered after reaching a waypoint.
    During the scan, if a marker is detected (handled in _aruco_callback),
    the drone will lock on to that marker.
    """
    
    def execute(self) -> Optional[MissionState]:
        """
        Execute SEARCHING state logic.
        
        Initiates a 360° yaw rotation to scan for markers.
        ArUco detection during the scan is handled by _aruco_callback in the main node.
        """
        # Initiate 360° scan (only if not already rotating)
        if not self.drone.yaw_controller.is_busy():
            self.node.get_logger().info(
                "SEARCHING: Starting 360° scan for markers."
            )
            self.drone.yaw_right_by_angle(
                angle=360,
                next_state=MissionState.WAYPOINT_NAVIGATION,  # No marker found after full scan
                fallback_state=MissionState.WAYPOINT_NAVIGATION,  # Timeout - proceed with next waypoint anyways
                speed=self.params.scanning_yaw_speed,
                timeout=25.0  # Allow sufficient time for full rotation
            )
        
        return None  # Wait for yaw completion or ArUco interrupt