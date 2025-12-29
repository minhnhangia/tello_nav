#!/usr/bin/env python3
from typing import Optional

from ..states import MissionState
from ..base_state import BaseState
from ...aruco import WaypointConfig


class WaypointActionState(BaseState):
    """Handles post-waypoint action execution and sequence advancement."""
    
    def execute(self) -> Optional[MissionState]:
        """Execute WAYPOINT_ACTION state logic."""

        waypoint = self._get_current_waypoint()
        if waypoint is None:
            return MissionState.SEARCHING

        self.node.get_logger().info(
            f"WAYPOINT_ACTION: Reached waypoint {waypoint.marker_id}"
        )

        if self._has_post_action(waypoint.post_action):
            return self._handle_post_action(waypoint)

        return self._handle_no_action(waypoint)

    # ----------------------------------------------------------------------
    # Guard & Utility Helpers
    # ----------------------------------------------------------------------

    def _get_current_waypoint(self) -> Optional[WaypointConfig]:
        """Ensures waypoint manager exists and returns the current waypoint."""
        if self.waypoint_manager is None:
            self.node.get_logger().error(
                "WAYPOINT_ACTION: Missing waypoint manager. Switching to SEARCHING."
            )
            return None

        waypoint = self.waypoint_manager.get_current_waypoint()
        if waypoint is None:
            self.node.get_logger().warning(
                "WAYPOINT_ACTION: No current waypoint. Switching to SEARCHING."
            )
        return waypoint

    @staticmethod
    def _has_post_action(post_action: Optional[str]) -> bool:
        """
        Determines whether a post-action command should be executed.
        Normalizes weird encodings like "", None, "none", "null".
        """
        action = (post_action or "").strip().lower()
        return action not in ("", "none", "null")

    def _is_last_waypoint(self) -> bool:
        """True if no further waypoints remain."""
        if not self.waypoint_manager:
            return True
        return not self.waypoint_manager.has_next_waypoint()

    def _advance_waypoint(self) -> Optional[int]:
        """Moves to next waypoint and returns that waypoint's marker_id."""
        self.waypoint_manager.advance_to_next() # type: ignore
        wp = self.waypoint_manager.get_current_waypoint()   # type: ignore
        return wp.marker_id if wp else None

    # ----------------------------------------------------------------------
    # Post-Action Path
    # ----------------------------------------------------------------------

    def _handle_post_action(self, waypoint: WaypointConfig) -> Optional[MissionState]:
        """Executes waypoint actions and may advance waypoint before execution."""
        action = waypoint.post_action.strip()   # type: ignore
        last_wp = self._is_last_waypoint()

        next_state = MissionState.SEARCHING if last_wp else MissionState.WAYPOINT_CENTERING

        self.node.get_logger().info(
            f"WAYPOINT_ACTION: Executing post-action '{action}' "
            f"({'last waypoint' if last_wp else 'more waypoints ahead'})"
        )

        # Advance early only if there are more waypoints
        if not last_wp:
            next_marker = self._advance_waypoint()
            self.node.get_logger().info(
                f"WAYPOINT_ACTION: Advanced to next waypoint {next_marker}"
            )

        # Trigger async drone command execution
        # NOTE: Caller resumes when drone reports action completion.
        self.drone.execute_action(action, next_state, MissionState.SEARCHING, max_retries=2)

        # Return None to indicate async processing
        return None

    # ----------------------------------------------------------------------
    # No-Action Path
    # ----------------------------------------------------------------------

    def _handle_no_action(self, waypoint: WaypointConfig) -> MissionState:
        """Advances or finishes mission when no post-action is defined."""
        self.node.get_logger().info(
            f"WAYPOINT_ACTION: No post-action defined at waypoint {waypoint.marker_id}"
        )

        if self._is_last_waypoint():
            self.node.get_logger().info(
                "WAYPOINT_ACTION: All waypoints complete — entering SEARCH phase."
            )
            return MissionState.SEARCHING

        next_marker = self._advance_waypoint()
        self.node.get_logger().info(
            f"WAYPOINT_ACTION: Moving to next waypoint {next_marker}"
        )
        return MissionState.WAYPOINT_CENTERING
