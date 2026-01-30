#!/usr/bin/env python3
from typing import Optional
from geometry_msgs.msg import Pose

from ..states import MissionState
from ..base_state import BaseState


class WaypointApproachingState(BaseState):
    """Handles forward movement toward waypoint marker."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute WAYPOINT_APPROACHING state logic."""
        # Check if waypoint sequence is complete
        if self.waypoint_manager.is_sequence_complete():  # type: ignore
            self.node.get_logger().error(
                "WAYPOINT_APPROACHING: Waypoint sequence complete. "
                "Transitioning to SEARCHING."
            )
            return MissionState.SEARCHING
        
        # Defensive check: action-only waypoints should not reach this state
        if self.waypoint_manager.is_current_waypoint_action_only():  # type: ignore
            self.node.get_logger().warning(
                "WAYPOINT_APPROACHING: Action-only waypoint detected. "
                "Transitioning to WAYPOINT_ACTION."
            )
            return MissionState.WAYPOINT_ACTION
        
        # Check for marker timeout
        if self.waypoint_manager.check_marker_timeout(self.params.waypoint_timeout):  # type: ignore
            self.node.get_logger().error(
                f"WAYPOINT_APPROACHING: Lost waypoint marker {self.waypoint_manager.get_current_marker_id()} for "  # type: ignore
                f">{self.params.waypoint_timeout}s. Returning to WAYPOINT_CENTERING."
            )
            return MissionState.WAYPOINT_CENTERING
        
        # Check if marker is visible
        if not self.waypoint_manager.is_marker_visible():  # type: ignore
            self._hover_on_temporary_marker_lost()
            return None
        
        self._move_to_marker()
        return None
    
    def _hover_on_temporary_marker_lost(self):
        self.node.get_logger().debug(
            "WAYPOINT_APPROACHING: Marker temporarily lost. Hovering...",
            throttle_duration_sec=2.0
        )
        self.drone.hover()
    
    def _move_to_marker(self):
        """Move forward to waypoint marker position."""
        marker_pose = self.waypoint_manager.get_marker_pose()  # type: ignore
        if marker_pose is None:
            return
        
        x = marker_pose.position.x
        y = marker_pose.position.y
        z = marker_pose.position.z
        forward_dist = z
        
        self.node.get_logger().info(
            f"WAYPOINT_APPROACHING: Moving to waypoint {self.waypoint_manager.get_current_marker_id()}, "  # type: ignore
            f"forward {forward_dist:.2f}m, x={x:.2f}m, y={y:.2f}m"
        )
        
        forward_dist_cmd = self._compute_fwd_dist_cmd(forward_dist)
        self.drone.execute_action(
            f'forward {forward_dist_cmd}',
            MissionState.WAYPOINT_ACTION,
            MissionState.SEARCHING,
            max_retries=2
        )
    
    def _compute_fwd_dist_cmd(self, forward_dist: float) -> int:
        """Compute forward distance command in cm with final approach offset."""
        return max(0, int((forward_dist - self.params.final_approach_offset) * 100))