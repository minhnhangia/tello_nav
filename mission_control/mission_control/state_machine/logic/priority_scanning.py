#!/usr/bin/env python3
"""Priority scanning state for detecting priority markers before landing."""
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState


class PriorityScanningState(BaseState):
    """
    Performs 360° scan to detect priority markers before landing.
    
    This state is entered after APPROACHING when locked onto a non-priority marker.
    During the scan, if a priority marker is detected (handled in _aruco_callback),
    the drone will switch to that marker. Otherwise, it proceeds to land on the
    current marker.
    """
    
    def execute(self) -> Optional[MissionState]:
        """
        Execute PRIORITY_SCANNING state logic.
        
        Initiates a 360° yaw rotation to scan for priority markers.
        ArUco detection during the scan is handled by _aruco_callback in the main node.
        """
        # Guard: If already on priority marker, skip scan
        if self.marker_handler.is_locked_on_priority_marker():
            self.node.get_logger().info(
                "PRIORITY_SCANNING: Already on priority marker. Skipping scan."
            )
            return MissionState.CAMERA_SWITCHING
        
        # Initiate 360° scan (only if not already rotating)
        if not self.drone.yaw_controller.is_busy():
            self.node.get_logger().info(
                "PRIORITY_SCANNING: Starting 360° scan for priority markers."
            )
            self.drone.yaw_right_by_angle(
                angle=360,
                next_state=MissionState.CAMERA_SWITCHING,  # No priority found after full scan
                fallback_state=MissionState.CAMERA_SWITCHING,  # Timeout - proceed with landing
                speed=self.params.yaw_speed,
                timeout=20.0  # Allow sufficient time for full rotation
            )
        
        return None  # Wait for yaw completion or ArUco interrupt
