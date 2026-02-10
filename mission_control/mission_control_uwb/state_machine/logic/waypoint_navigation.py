#!/usr/bin/env python3
"""
Waypoint Navigation State

Uses UWB positioning to navigate the drone to predefined waypoints.
Waypoint sequencing is managed by WaypointManager; this state handles movement.
Waypoint coordination (reservation/heartbeat) is managed by WaypointCoordinator.
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
    2. Checks waypoint availability via WaypointCoordinator
    3. Reserves waypoint before navigation (async)
    4. Sets UWBNavigator goal if not active
    5. Publishes heartbeats to maintain reservation (via timer in main node)
    6. Ticks navigator and handles success/timeout/blocked
    7. Unreserves waypoint after reaching it
    8. Advances to next waypoint or transitions to SEARCHING when complete
    """
    
    __slots__ = ('_retry_unavailable_start_time',)
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._retry_unavailable_start_time = None
    
    def execute(self) -> Optional[MissionState]:
        """
        Execute waypoint navigation tick.
        
        Returns:
            MissionState.SEARCHING: A waypoint has been reached
            MissionState.LANDING: All waypoints complete
            None: Continue navigation
        """
        # Validate dependencies
        if self.uwb_navigator is None or self.waypoint_manager is None:
            self.node.get_logger().error(
                "WAYPOINT_NAVIGATION: Navigator or WaypointManager not initialized"
            )
            return MissionState.LANDING
        
        if self.waypoint_coordinator is None:
            self.node.get_logger().error(
                "WAYPOINT_NAVIGATION: WaypointCoordinator not initialized"
            )
            return MissionState.LANDING
        
        # Check if all waypoints completed
        if self.waypoint_manager.is_complete():
            self.node.get_logger().info(
                "WAYPOINT_NAVIGATION: All waypoints complete → LANDING"
            )
            self._reset_state()
            return MissionState.LANDING
        
        # Get current waypoint
        waypoint = self.waypoint_manager.get_current_waypoint()
        if waypoint is None:
            self.node.get_logger().warning(
                "WAYPOINT_NAVIGATION: No waypoint available → LANDING"
            )
            self._reset_state()
            return MissionState.LANDING
        
        current_wp_index = self.waypoint_manager.current_index
        
        # ====================================================================
        # STEP 1: CHECK RESERVATION STATUS
        # ====================================================================
        
        # Check if waypoint is reserved by this drone
        if not self.waypoint_coordinator.is_waypoint_reserved_by_this_drone(current_wp_index):
            # Not reserved yet - need to request reservation
            return self._handle_reservation_process(waypoint, current_wp_index)
        
        # ====================================================================
        # STEP 2: WAYPOINT IS RESERVED - PROCEED WITH NAVIGATION
        # ====================================================================
        
        # Verify reservation is still valid (prevent race condition)
        if not self._verify_reservation_valid(current_wp_index):
            return None  # Will retry reservation on next tick
        
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
    
    # ========================================================================
    # RESERVATION MANAGEMENT
    # ========================================================================
    
    def _handle_reservation_process(
        self, 
        waypoint, 
        waypoint_index: int
    ) -> Optional[MissionState]:
        """
        Handle waypoint reservation request and wait logic.
        
        Returns:
            MissionState or None to continue waiting
        """
        # Check if waypoint is available (not reserved by another drone)
        if not self.waypoint_coordinator.is_waypoint_available(waypoint_index):
            # Waypoint is unavailable - hover and wait
            return self._handle_unavailable_waypoint(waypoint, waypoint_index)
        
        # Check if reservation request is already pending
        if self.waypoint_coordinator.is_reservation_pending():
            # Hover while waiting for server response
            self.drone.hover()
            self.node.get_logger().debug(
                f"WAYPOINT_NAVIGATION: Awaiting reservation response for waypoint {waypoint_index}",
                throttle_duration_sec=2.0
            )
            return None
        
        # Request reservation (async, non-blocking)
        self.node.get_logger().info(
            f"WAYPOINT_NAVIGATION: Requesting reservation for waypoint {waypoint_index} at ({waypoint.x:.2f}, {waypoint.y:.2f})m"
        )
        
        self.waypoint_coordinator.reserve_waypoint_async(
            waypoint_index=waypoint_index,
            callback=self._on_reservation_response
        )
        
        # Hover while waiting for response
        self.drone.hover()
        return None
    
    def _handle_unavailable_waypoint(
        self,
        waypoint,
        waypoint_index: int
    ) -> Optional[MissionState]:
        """
        Handle case where waypoint is reserved by another drone.
        
        Hover and wait with periodic retry attempts.
        """
        # Initialize retry timer
        if self._retry_unavailable_start_time is None:
            self._retry_unavailable_start_time = self.node.get_clock().now()
            self.node.get_logger().warning(
                f"WAYPOINT_NAVIGATION: Waypoint {waypoint_index} is currently reserved by another drone. "
                f"Hovering and waiting for availability..."
            )
        
        # Hover while waiting
        self.drone.hover()
        
        # Log waiting status periodically
        elapsed = (
            self.node.get_clock().now() - self._retry_unavailable_start_time
        ).nanoseconds / 1e9
        
        self.node.get_logger().info(
            f"WAYPOINT_NAVIGATION: Waiting for waypoint {waypoint_index} "
            f"(elapsed: {elapsed:.1f}s)...",
            throttle_duration_sec=3.0
        )
        
        # Check if waypoint became available
        if self.waypoint_coordinator.is_waypoint_available(waypoint_index):
            self.node.get_logger().info(
                f"WAYPOINT_NAVIGATION: Waypoint {waypoint_index} is now available!"
            )
            self._retry_unavailable_start_time = None
            # Will request reservation on next tick
        
        return None
    
    def _on_reservation_response(self, success: bool, message: str):
        """
        Callback invoked when reservation request completes.
        
        Called by ROS2 executor (async response handler).
        """
        if success:
            self.node.get_logger().info(
                f"WAYPOINT_NAVIGATION: ✓ Reservation granted - starting navigation"
            )
            # Navigation will start on next execute() tick
        else:
            self.node.get_logger().warning(
                f"WAYPOINT_NAVIGATION: ✗ Reservation failed: {message}. "
                f"Will retry on next tick."
            )
            # Will retry on next execute() tick
    
    def _verify_reservation_valid(self, waypoint_index: int) -> bool:
        """
        Verify that reservation is still valid before navigating.
        
        Prevents race condition where heartbeat fails and another drone
        reserves the waypoint while we're processing.
        """
        if not self.waypoint_coordinator.is_waypoint_reserved_by_this_drone(waypoint_index):
            self.node.get_logger().error(
                f"WAYPOINT_NAVIGATION: Lost reservation for waypoint {waypoint_index}! "
                f"Cancelling navigation and re-requesting reservation."
            )
            self.uwb_navigator.cancel()
            self.context.waypoint_nav_start_time = None
            return False
        return True
    
    # ========================================================================
    # NAVIGATION EXECUTION
    # ========================================================================
    
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
        """Handle waypoint timeout by unreserving and skipping to next."""
        self.node.get_logger().warning(
            f"WAYPOINT_NAVIGATION: Timeout after {self.params.waypoint_timeout}s"
        )
        
        # Cancel navigation
        self.uwb_navigator.cancel()
        self.context.waypoint_nav_start_time = None
        
        # Unreserve current waypoint
        current_index = self.waypoint_manager.current_index
        self.waypoint_coordinator.unreserve_waypoint(current_index)
        
        # Skip to next waypoint
        if self.waypoint_manager.skip_current():
            self._retry_unavailable_start_time = None
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
        
        # Reset timers
        self.context.waypoint_nav_start_time = None
        self._retry_unavailable_start_time = None
        
        # Unreserve current waypoint
        current_index = self.waypoint_manager.current_index
        self.waypoint_coordinator.unreserve_waypoint(current_index)
        
        # Advance to next waypoint
        if self.waypoint_manager.advance():
            return MissionState.SEARCHING  # Proceed to SEARCHING after each waypoint
        
        # All waypoints complete
        self._reset_state()
        return MissionState.LANDING
    
    def _handle_blocked(self) -> Optional[MissionState]:
        """Handle navigation blocked (UWB failure)."""
        self.node.get_logger().error(
            "WAYPOINT_NAVIGATION: Blocked (UWB failure) → STANDBY"
        )
        
        # Cancel navigation and unreserve
        self.uwb_navigator.cancel()
        current_index = self.waypoint_manager.current_index
        self.waypoint_coordinator.unreserve_waypoint(current_index)
        
        self._reset_state()
        return MissionState.STANDBY
    
    def _reset_state(self) -> None:
        """Reset state variables for clean exit."""
        self.context.waypoint_nav_start_time = None
        self._retry_unavailable_start_time = None