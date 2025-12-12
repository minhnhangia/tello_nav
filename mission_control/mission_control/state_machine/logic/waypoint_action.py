#!/usr/bin/env python3
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState


class WaypointActionState(BaseState):
    """Handles post-waypoint action execution and sequence advancement."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute WAYPOINT_ACTION state logic."""
        # Check waypoint manager availability
        if self.waypoint_manager is None:
            self.node.get_logger().error(
                "WAYPOINT_ACTION: No waypoint manager. Transitioning to SEARCHING."
            )
            return MissionState.SEARCHING
        
        # Get current waypoint configuration
        current_waypoint = self.waypoint_manager.get_current_waypoint()
        if current_waypoint is None:
            self.node.get_logger().warning(
                "WAYPOINT_ACTION: No current waypoint. Transitioning to SEARCHING."
            )
            return MissionState.SEARCHING
        
        marker_id = current_waypoint.marker_id
        post_action = current_waypoint.post_action
        
        # Check if there are more waypoints after this one
        has_more_waypoints = self.waypoint_manager.has_next_waypoint()
        
        # Determine next state based on whether more waypoints exist
        next_state = MissionState.WAYPOINT_CENTERING if has_more_waypoints else MissionState.SEARCHING
        
        # If post-action is defined, execute it
        if post_action and post_action.lower() not in ['none', 'null', '']:
            self.node.get_logger().info(
                f"WAYPOINT_ACTION: Waypoint {marker_id} reached. "
                f"Executing post-action: '{post_action}'"
            )
            
            # Execute the action, then advance
            # Note: We advance BEFORE executing action so next waypoint is ready
            # when action completes
            if has_more_waypoints:
                self.waypoint_manager.advance_to_next()
                next_waypoint = self.waypoint_manager.get_current_waypoint()
                self.node.get_logger().info(
                    f"Advanced to next waypoint: {next_waypoint.marker_id if next_waypoint else 'unknown'}"
                )
            
            # Execute action command
            self.drone.execute_action(
                post_action,
                next_state,
                next_state  # No fallback differentiation - proceed anyway
            )
            return None
        
        # No post-action defined - advance immediately
        self.node.get_logger().info(
            f"WAYPOINT_ACTION: Waypoint {marker_id} reached. No post-action defined."
        )
        
        if has_more_waypoints:
            self.waypoint_manager.advance_to_next()
            next_waypoint = self.waypoint_manager.get_current_waypoint()
            self.node.get_logger().info(
                f"WAYPOINT_ACTION: Moving to next waypoint: "
                f"{next_waypoint.marker_id if next_waypoint else 'unknown'}"
            )
            return MissionState.WAYPOINT_CENTERING
        else:
            self.node.get_logger().info(
                "WAYPOINT_ACTION: All waypoints completed! Starting search phase."
            )
            return MissionState.SEARCHING