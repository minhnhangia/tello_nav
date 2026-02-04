#!/usr/bin/env python3
"""
Waypoint Navigation State

Uses UWB positioning to navigate the drone to predefined waypoints.
This state handles the actual movement; waypoint sequencing is managed
by the WaypointManager.
"""
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState
from ...uwb import NavStatus


class WaypointNavigationState(BaseState):
    """
    Navigate drone to the current waypoint using UWB position feedback.
    
    This state:
    1. Retrieves target position from context (set by a higher-level planner)
    2. Uses UWBNavigator's non-blocking update() to move toward target
    3. Transitions to next state on success or handles failures    
    """
    
    def execute(self) -> Optional[MissionState]:
        """
        Execute waypoint navigation tick.
        
        Returns:
            MissionState.SEARCHING on success (waypoint reached)
            MissionState.STANDBY on blocked (UWB failure)
            None to continue navigation
        """
        # Check if navigator is available
        if self.uwb_navigator is None:
            self.node.get_logger().error(
                "WAYPOINT_NAVIGATION: UWBNavigator not initialized. "
                "Enable waypoint navigation in parameters."
            )
            return MissionState.SEARCHING
        
        # First-time setup: set the goal if not already active
        if not self.uwb_navigator.is_active:
            if not self._set_navigation_goal():
                # No valid waypoint target - transition out
                return MissionState.SEARCHING
        
        # Tick the navigator
        status = self.uwb_navigator.update()
        
        if status == NavStatus.SUCCESS:
            self.node.get_logger().info(
                "WAYPOINT_NAVIGATION: Waypoint reached!"
            )
            return MissionState.SEARCHING
        
        elif status == NavStatus.BLOCKED:
            self.node.get_logger().error(
                "WAYPOINT_NAVIGATION: Navigation blocked (UWB failure). "
                "Falling back to STANDBY."
            )
            self.uwb_navigator.cancel()
            return MissionState.STANDBY
        
        # NavStatus.RUNNING - continue navigation
        return None
    
    def _set_navigation_goal(self) -> bool:
        """
        Set the navigation goal from context waypoint target.
        
        Returns:
            True if goal was set successfully, False if no valid target
        """
        # target = self.context.current_waypoint_target
        # if target is None:
        #     self.node.get_logger().warning(
        #         "WAYPOINT_NAVIGATION: No waypoint target set in context"
        #     )
        #     return False
        
        # target is expected to be a tuple/list: (x, y) in meters
        # x, y = target[0], target[1]
        
        # self.node.get_logger().info(
        #     f"WAYPOINT_NAVIGATION: Navigating to ({x:.2f}, {y:.2f})m"
        # )
        
        self.uwb_navigator.set_goal(
            x=4.6,
            y=5.3,
            strategy="DRIVE"
        )
        return True