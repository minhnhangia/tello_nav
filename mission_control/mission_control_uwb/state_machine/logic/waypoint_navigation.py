#!/usr/bin/env python3
"""
Waypoint Navigation State

Uses UWB positioning to navigate the drone to predefined waypoints.
Waypoint sequencing is managed by WaypointManager; this state handles movement.
"""
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState
from ...uwb import NavStatus


class WaypointNavigationState(BaseState):
    """
    Navigate drone to waypoints using UWB position feedback.
    
    This state:
    1. Gets current waypoint from WaypointManager
    2. Sets UWBNavigator goal if not active
    3. Ticks navigator and handles success/timeout/blocked
    4. Advances to next waypoint or transitions to SEARCHING when complete
    """
    
    def execute(self) -> Optional[MissionState]:
        """
        Execute waypoint navigation tick.
        
        Returns:
            MissionState.SEARCHING: All waypoints complete or no waypoints
            MissionState.STANDBY: Navigation blocked (UWB failure)
            None: Continue navigation
        """
        # Validate dependencies
        if self.uwb_navigator is None or self.waypoint_manager is None:
            self.node.get_logger().error(
                "WAYPOINT_NAVIGATION: Navigator or WaypointManager not initialized"
            )
            return MissionState.SEARCHING
        
        # Check if all waypoints completed
        if self.waypoint_manager.is_complete():
            self.node.get_logger().info(
                "WAYPOINT_NAVIGATION: All waypoints complete → SEARCHING"
            )
            self._reset_state()
            return MissionState.SEARCHING
        
        # Get current waypoint
        waypoint = self.waypoint_manager.get_current_waypoint()
        if waypoint is None:
            self.node.get_logger().warning(
                "WAYPOINT_NAVIGATION: No waypoint available → SEARCHING"
            )
            self._reset_state()
            return MissionState.SEARCHING
        
        # Initialize navigation goal if not active
        if not self.uwb_navigator.is_active:
            self._start_navigation(waypoint.x, waypoint.y)
        
        # Check for timeout
        if self._is_timed_out():
            return self._handle_timeout()
        
        # Tick the navigator
        status = self.uwb_navigator.update()
        
        if status == NavStatus.SUCCESS:
            return self._handle_waypoint_reached()
        
        if status == NavStatus.BLOCKED:
            return self._handle_blocked()
        
        # NavStatus.RUNNING - continue
        return None
    
    def _start_navigation(self, x: float, y: float) -> None:
        """Initialize navigation to waypoint and start timeout timer."""
        self.context.waypoint_nav_start_time = self.node.get_clock().now()
        self.uwb_navigator.set_goal(x=x, y=y, strategy="DRIVE")
        
        progress = self.waypoint_manager.get_progress_string()
        self.node.get_logger().info(
            f"WAYPOINT_NAVIGATION: Starting {progress} → ({x:.2f}, {y:.2f})m"
        )
    
    def _is_timed_out(self) -> bool:
        """Check if current waypoint navigation has timed out."""
        if self.context.waypoint_nav_start_time is None:
            return False
        
        elapsed = (
            self.node.get_clock().now() - self.context.waypoint_nav_start_time
        ).nanoseconds / 1e9
        
        return elapsed > self.params.waypoint_timeout
    
    def _handle_timeout(self) -> Optional[MissionState]:
        """Handle waypoint timeout by skipping to next."""
        self.node.get_logger().warning(
            f"WAYPOINT_NAVIGATION: Timeout after {self.params.waypoint_timeout}s"
        )
        self.uwb_navigator.cancel()
        self.context.waypoint_nav_start_time = None
        
        # Skip to next waypoint
        if self.waypoint_manager.skip_current():
            return None  # Stay in WAYPOINT_NAVIGATION for next waypoint
        
        # No more waypoints
        self._reset_state()
        return MissionState.SEARCHING
    
    def _handle_waypoint_reached(self) -> Optional[MissionState]:
        """Handle successful waypoint arrival."""
        progress = self.waypoint_manager.get_progress_string()
        self.node.get_logger().info(
            f"WAYPOINT_NAVIGATION: Waypoint {progress} reached!"
        )
        self.context.waypoint_nav_start_time = None
        
        # Advance to next waypoint
        if self.waypoint_manager.advance():
            return None  # Stay in WAYPOINT_NAVIGATION
        
        # All waypoints complete
        self._reset_state()
        return MissionState.SEARCHING
    
    def _handle_blocked(self) -> MissionState:
        """Handle navigation blocked (UWB failure)."""
        self.node.get_logger().error(
            "WAYPOINT_NAVIGATION: Blocked (UWB failure) → STANDBY"
        )
        self.uwb_navigator.cancel()
        self._reset_state()
        return MissionState.STANDBY
    
    def _reset_state(self) -> None:
        """Reset state variables for clean exit."""
        self.context.waypoint_nav_start_time = None