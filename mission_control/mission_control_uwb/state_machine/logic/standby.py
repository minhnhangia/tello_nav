#!/usr/bin/env python3
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState


class StandbyState(BaseState):
    """Handles standby delay before starting mission."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute STANDBY state logic with configurable delay."""
        now = self.node.get_clock().now()

        # Initialize standby timer on first entry
        if self.context.standby_start_time is None:
            self.context.standby_start_time = now
            self.node.get_logger().info(
                f"STANDBY: Waiting {self.params.standby_delay:.1f}s before starting mission."
            )
        
        self.drone.hover()
        
        # Check if delay period has elapsed
        elapsed_time = (now - self.context.standby_start_time).nanoseconds / 1e9
        if elapsed_time < self.params.standby_delay:
            # Still waiting
            remaining = self.params.standby_delay - elapsed_time
            self.node.get_logger().debug(
                f"STANDBY: {remaining:.1f}s remaining...",
                throttle_duration_sec=1.0
            )
            return None
        
        # Delay complete - reset timer for next entry
        self._reset_internal_state()
        
        # Determine next state based on waypoint configuration
        return self._determine_next_state()
        
    def _reset_internal_state(self):
        self.context.standby_start_time = None
    
    def _determine_next_state(self) -> MissionState:
        """Determine the next state after standby based on configuration."""
        if not self.params.enable_waypoint_navigation:
            self.node.get_logger().info(
                "STANDBY: Delay complete. Waypoint navigation disabled. "
                "Transitioning to SEARCHING."
            )
            return MissionState.SEARCHING
        
        if self.waypoint_manager and self.waypoint_manager.has_waypoints():
            self.node.get_logger().info(
                f"STANDBY: Delay complete. Starting waypoint navigation "
                f"({self.waypoint_manager.total_count} waypoints). "
                "Transitioning to WAYPOINT_NAVIGATION."
            )
            return MissionState.WAYPOINT_NAVIGATION
        else:
            self.node.get_logger().warning(
                "STANDBY: Delay complete. Waypoint navigation enabled but "
                "no waypoints configured. Transitioning to SEARCHING."
            )
            return MissionState.SEARCHING